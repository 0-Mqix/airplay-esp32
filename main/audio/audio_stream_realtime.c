#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include "audio_buffer.h"
#include "audio_crypto.h"
#include "audio_receiver_internal.h"
#include "config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/ringbuf.h"
#include "network/socket_utils.h"

#define RTP_HEADER_SIZE 12

// Reorder buffer: holds packets for short-term reordering and retransmit recovery
#define REORDER_SLOTS       400
#define REORDER_DELAY       200  // ~1.6s buffer depth before draining starts
#define REORDER_MAX_PAYLOAD 1500

// Decode ring buffer: compressed frames queued from recv task to decode task
// Each item: 6-byte header + up to 1500 bytes payload ≈ 1506 bytes
// 512KB on PSRAM gives ~340 items of headroom
#define DECODE_RING_SIZE (512 * 1024)

typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  uint16_t len;  // 0 = silence marker
} decode_item_header_t;

typedef struct __attribute__((packed)) {
  uint8_t  flags;
  uint8_t  type;
  uint16_t seq;
  uint32_t timestamp;
  uint32_t ssrc;
} rtp_header_t;

typedef struct {
  uint8_t  data[REORDER_MAX_PAYLOAD];
  uint16_t len;
  uint16_t seq;
  uint32_t timestamp;
  bool     occupied;
} reorder_slot_t;

typedef struct {
  reorder_slot_t *slots;
  uint16_t       next_seq;
  uint16_t       highest_seq;
  bool           initialized;
} reorder_buffer_t;

static const char* TAG = "audio_rt";

static reorder_buffer_t reorder = {0};
static int16_t          silence_samples[1024 * 2] = {0};
static uint32_t         reorder_drain_count = 0;
static uint32_t         reorder_silence_count = 0;
static uint32_t         reorder_gap_count = 0;
static uint32_t         reorder_force_drain_count = 0;
static uint32_t         retransmit_recovered_count = 0;

static void reorder_reset(void) {
  if (!reorder.slots) {
    reorder.slots = calloc(REORDER_SLOTS, sizeof(reorder_slot_t));
    if (!reorder.slots) {
      ESP_LOGE(TAG, "Failed to allocate reorder buffer (%u bytes)", (unsigned)(REORDER_SLOTS * sizeof(reorder_slot_t)));
      return;
    }
  } else {
    memset(reorder.slots, 0, REORDER_SLOTS * sizeof(reorder_slot_t));
  }
  reorder.next_seq = 0;
  reorder.highest_seq = 0;
  reorder.initialized = false;
}

// ============================================================================
// RTP parsing
// ============================================================================

static const uint8_t*
parse_rtp(const uint8_t* packet, size_t len, uint16_t* seq, uint32_t* timestamp, size_t* payload_len) {
  if (len < RTP_HEADER_SIZE) { return NULL; }

  const rtp_header_t* hdr = (const rtp_header_t*)packet;
  uint8_t             version = (hdr->flags >> 6) & 0x03;
  if (version != 2) {
    ESP_LOGW(TAG, "Invalid RTP version: %d", version);
    return NULL;
  }

  *seq = ntohs(hdr->seq);
  *timestamp = ntohl(hdr->timestamp);

  size_t header_len = RTP_HEADER_SIZE;
  if (hdr->flags & 0x10) {
    if (len < RTP_HEADER_SIZE + 4) { return NULL; }
    uint16_t ext_len = ntohs(*(uint16_t*)(packet + RTP_HEADER_SIZE + 2));
    header_len += 4 + ext_len * 4;
  }

  uint8_t csrc_count = hdr->flags & 0x0F;
  header_len += csrc_count * 4;

  if (len <= header_len) { return NULL; }

  *payload_len = len - header_len;
  return packet + header_len;
}

// ============================================================================
// Retransmit requests (AirPlay NACK via control port)
// ============================================================================

static void send_retransmit_request(audio_receiver_state_t* state, uint16_t first_seq, uint16_t count) {
  if (state->control_socket < 0 || state->client_control_port == 0) { return; }
  if (count == 0 || count > 128) { return; }

  struct sockaddr_in dest = {0};
  dest.sin_family = AF_INET;
  dest.sin_addr.s_addr = state->client_ip;
  dest.sin_port = htons(state->client_control_port);

  uint8_t  pkt[8];
  uint16_t req_seq = state->retransmit_seq++;
  pkt[0] = 0x80;
  pkt[1] = 0x55 | 0x80;  // AP1 retransmit request type with marker bit
  pkt[2] = 0x00;
  pkt[3] = 0x01;  // seq=1, matches shairport-sync
  pkt[4] = (first_seq >> 8) & 0xFF;
  pkt[5] = first_seq & 0xFF;
  pkt[6] = (count >> 8) & 0xFF;
  pkt[7] = count & 0xFF;

  sendto(state->control_socket, pkt, sizeof(pkt), 0, (struct sockaddr*)&dest, sizeof(dest));
}

// Forward declaration
static void reorder_insert(audio_stream_t* stream, uint16_t seq, uint32_t timestamp, const uint8_t* data, size_t len);

// ============================================================================
// Retransmit response handling (control port)
// ============================================================================

// AirPlay retransmit responses arrive on the control port with type 0x56.
// Format: 4-byte retransmit header + standard RTP packet.
static void process_control_packet(audio_stream_t* stream, uint8_t* packet, int len) {
  if (len < 4) { return; }

  uint8_t type = packet[1] & 0x7F;
  if (type != 0x56) { return; }  // 0x56 (AP1) or 0xD6 (AP2), masked by 0x7F

  // Skip 4-byte retransmit header, rest is a normal RTP packet
  uint8_t*       rtp_pkt = packet + 4;
  size_t         rtp_len = (size_t)(len - 4);

  audio_receiver_state_t* state = audio_stream_state(stream);

  uint16_t       seq = 0;
  uint32_t       timestamp = 0;
  size_t         payload_len = 0;
  const uint8_t* payload = parse_rtp(rtp_pkt, rtp_len, &seq, &timestamp, &payload_len);

  if (!payload || payload_len == 0) { return; }

  state->stats.packets_received++;

  const uint8_t* audio_data = payload;
  size_t         audio_len = payload_len;

  if (stream->encrypt.type != AUDIO_ENCRYPT_NONE && state->decrypt_buffer) {
    int decrypted_len =
      audio_crypto_decrypt_rtp(&stream->encrypt, payload, payload_len, state->decrypt_buffer, MAX_RTP_PACKET_SIZE, rtp_pkt, rtp_len);
    if (decrypted_len < 0) {
      state->stats.decrypt_errors++;
      return;
    }
    audio_data = state->decrypt_buffer;
    audio_len = (size_t)decrypted_len;
  }

  retransmit_recovered_count++;
  reorder_insert(stream, seq, timestamp, audio_data, audio_len);
}

// ============================================================================
// Silence insertion for unrecoverable packet loss
// ============================================================================

static void insert_silence(audio_receiver_state_t* state, uint32_t timestamp) {
  int frame_size = state->stream->format.frame_size > 0 ? state->stream->format.frame_size : 352;
  int channels = state->stream->format.channels > 0 ? state->stream->format.channels : 2;
  if (frame_size > 1024) { frame_size = 1024; }

  audio_buffer_queue_decoded(&state->buffer, NULL, timestamp, silence_samples, (size_t)frame_size, channels);
}

// Send a compressed frame (or silence marker) to the decode ring buffer
static bool decode_ring_send(audio_receiver_state_t* state, uint32_t timestamp, const uint8_t* data, uint16_t len) {
  if (!state->decode_ring) { return false; }

  decode_item_header_t hdr = {.timestamp = timestamp, .len = len};
  size_t               item_size = sizeof(hdr) + len;
  uint8_t              buf[sizeof(decode_item_header_t) + REORDER_MAX_PAYLOAD];

  memcpy(buf, &hdr, sizeof(hdr));
  if (len > 0 && data) { memcpy(buf + sizeof(hdr), data, len); }

  BaseType_t ret = xRingbufferSend(state->decode_ring, buf, item_size, pdMS_TO_TICKS(50));
  if (ret != pdTRUE) {
    ESP_LOGW(TAG, "decode ring full, dropping frame ts=%lu len=%u", (unsigned long)timestamp, len);
    return false;
  }
  return true;
}

// Send a silence marker to the decode ring (len=0)
static bool decode_ring_send_silence(audio_receiver_state_t* state, uint32_t timestamp) {
  return decode_ring_send(state, timestamp, NULL, 0);
}

// ============================================================================
// Reorder buffer operations
// ============================================================================

static void reorder_drain(audio_receiver_state_t* state) {
  while ((int16_t)(reorder.highest_seq - reorder.next_seq) >= REORDER_DELAY) {
    int             idx = reorder.next_seq % REORDER_SLOTS;
    reorder_slot_t* slot = &reorder.slots[idx];

    if (slot->occupied && slot->seq == reorder.next_seq) {
      decode_ring_send(state, slot->timestamp, slot->data, slot->len);
      slot->occupied = false;
      reorder_drain_count++;
      reorder.next_seq++;
    } else {
      // Still missing — retry NACK and stop draining to give it more time.
      // It will become silence only when force-drained (buffer full).
      send_retransmit_request(state, reorder.next_seq, 1);
      break;
    }
  }
}

static void reorder_force_drain_to(audio_receiver_state_t* state, uint16_t target_seq) {
  int      frame_size = state->stream->format.frame_size > 0 ? state->stream->format.frame_size : 352;
  uint32_t est_ts = state->stats.last_timestamp;

  while ((int16_t)(target_seq - reorder.next_seq) > 0) {
    int             idx = reorder.next_seq % REORDER_SLOTS;
    reorder_slot_t* slot = &reorder.slots[idx];

    if (slot->occupied && slot->seq == reorder.next_seq) {
      decode_ring_send(state, slot->timestamp, slot->data, slot->len);
      est_ts = slot->timestamp;
      slot->occupied = false;
    } else {
      est_ts += (uint32_t)frame_size;
      decode_ring_send_silence(state, est_ts);
      state->stats.packets_dropped++;
    }
    reorder.next_seq++;
  }
}

static void
reorder_insert(audio_stream_t* stream, uint16_t seq, uint32_t timestamp, const uint8_t* data, size_t len) {
  audio_receiver_state_t* state = audio_stream_state(stream);

  // On flush, reset reorder buffer and drain decode ring so stale data doesn't linger
  if (atomic_load(&state->flush_pending)) {
    reorder_reset();
    // Drain stale compressed frames from decode ring
    if (state->decode_ring) {
      size_t sz = 0;
      void*  stale = NULL;
      while ((stale = xRingbufferReceive(state->decode_ring, &sz, 0)) != NULL) {
        vRingbufferReturnItem(state->decode_ring, stale);
      }
    }
    audio_buffer_flush(&state->buffer);
    atomic_store(&state->flush_pending, false);
  }

  if (!reorder.slots) { return; }

  if (!reorder.initialized) {
    reorder.next_seq = seq;
    reorder.highest_seq = seq;
    reorder.initialized = true;
  }

  int16_t diff = (int16_t)(seq - reorder.next_seq);

  if (diff < 0) {
    // Late arrival — already moved past this sequence
    // counted in periodic stats
    state->stats.late_frames++;
    return;
  }

  if (diff >= REORDER_SLOTS) {
    // Too far ahead — force drain to make room, inserting silence for gaps
    reorder_force_drain_count++;
    reorder_force_drain_to(state, (uint16_t)(seq - REORDER_SLOTS + 1));
    diff = (int16_t)(seq - reorder.next_seq);
  }

  // Request retransmit only for actual gaps (between highest seen and this packet)
  int16_t gap = (int16_t)(seq - reorder.highest_seq);
  if (gap > 1) {
    uint16_t first_missing = reorder.highest_seq + 1;
    uint16_t count = (uint16_t)(gap - 1);
    reorder_gap_count += count;
    send_retransmit_request(state, first_missing, count);
  }

  // Store in slot
  int             idx = seq % REORDER_SLOTS;
  reorder_slot_t* slot = &reorder.slots[idx];
  size_t          copy_len = len < REORDER_MAX_PAYLOAD ? len : REORDER_MAX_PAYLOAD;
  memcpy(slot->data, data, copy_len);
  slot->len = (uint16_t)copy_len;
  slot->seq = seq;
  slot->timestamp = timestamp;
  slot->occupied = true;

  // Track highest sequence for delay calculation
  if ((int16_t)(seq - reorder.highest_seq) > 0) { reorder.highest_seq = seq; }

  // Drain packets that have waited long enough
  reorder_drain(state);
}

// ============================================================================
// Packet reception
// ============================================================================

static bool
realtime_receive_packet(audio_stream_t* stream, uint8_t* packet, struct sockaddr_in* src_addr, socklen_t* addr_len) {
  audio_receiver_state_t* state = audio_stream_state(stream);

  int len = recvfrom(state->data_socket, packet, MAX_RTP_PACKET_SIZE, 0, (struct sockaddr*)src_addr, addr_len);
  if (len < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) { return true; }
    if (stream->running) { ESP_LOGE(TAG, "recvfrom error: %d", errno); }
    return false;
  }

  if (len == 0) { return true; }

  // Check if this is a retransmit response on the data port (AirPlay 1: type 0x56)
  if (len >= 4 && (packet[1] & 0x7F) == 0x56) {
    process_control_packet(stream, packet, len);
    return true;
  }

  state->stats.packets_received++;

  uint16_t       seq = 0;
  uint32_t       timestamp = 0;
  size_t         payload_len = 0;
  const uint8_t* payload = parse_rtp(packet, (size_t)len, &seq, &timestamp, &payload_len);

  if (!payload || payload_len == 0) {
    state->stats.packets_dropped++;
    return true;
  }

  state->stats.last_seq = seq;
  state->stats.last_timestamp = timestamp;
  state->blocks_read++;

  const uint8_t* audio_data = payload;
  size_t         audio_len = payload_len;

  if (stream->encrypt.type != AUDIO_ENCRYPT_NONE && state->decrypt_buffer) {
    int decrypted_len =
      audio_crypto_decrypt_rtp(&stream->encrypt, payload, payload_len, state->decrypt_buffer, MAX_RTP_PACKET_SIZE, packet, (size_t)len);
    if (decrypted_len < 0) {
      state->stats.decrypt_errors++;
      state->stats.packets_dropped++;
      return true;
    }
    audio_data = state->decrypt_buffer;
    audio_len = (size_t)decrypted_len;
  }

  // Insert into reorder buffer — handles sequencing, retransmit, and silence
  reorder_insert(stream, seq, timestamp, audio_data, audio_len);

  return true;
}

// ============================================================================
// Decode task: pulls compressed frames from decode_ring, decodes, queues PCM
// ============================================================================

static void decode_task(void* pvParameters) {
  audio_stream_t*         stream = (audio_stream_t*)pvParameters;
  audio_receiver_state_t* state = audio_stream_state(stream);

  while (stream->running) {
    size_t item_size = 0;
    void*  item = xRingbufferReceive(state->decode_ring, &item_size, pdMS_TO_TICKS(100));
    if (!item) { continue; }

    if (item_size < sizeof(decode_item_header_t)) {
      vRingbufferReturnItem(state->decode_ring, item);
      continue;
    }

    decode_item_header_t hdr;
    memcpy(&hdr, item, sizeof(hdr));

    if (hdr.len > 0) {
      const uint8_t* payload = (const uint8_t*)item + sizeof(decode_item_header_t);
      audio_stream_process_frame(state, hdr.timestamp, payload, hdr.len);
    } else {
      insert_silence(state, hdr.timestamp);
    }

    vRingbufferReturnItem(state->decode_ring, item);
  }

  state->decode_task_handle = NULL;
  vTaskDelete(NULL);
}

// ============================================================================
// Receive task and lifecycle
// ============================================================================

static void receiver_task(void* pvParameters) {
  audio_stream_t*         stream = (audio_stream_t*)pvParameters;
  audio_receiver_state_t* state = audio_stream_state(stream);

  uint8_t* packet = (uint8_t*)malloc(MAX_RTP_PACKET_SIZE);
  if (!packet) {
    ESP_LOGE(TAG, "Failed to allocate packet buffer");
    vTaskDelete(NULL);
    return;
  }

  struct sockaddr_in src_addr;
  socklen_t          addr_len = sizeof(src_addr);
  bool               has_control = (state->control_socket >= 0);

  while (stream->running) {
    if (has_control) {
      // Use select() to listen on both data and control sockets
      fd_set fds;
      FD_ZERO(&fds);
      FD_SET(state->data_socket, &fds);
      FD_SET(state->control_socket, &fds);
      int maxfd = state->data_socket > state->control_socket ? state->data_socket : state->control_socket;

      struct timeval tv = {.tv_sec = 2, .tv_usec = 0};
      int ready = select(maxfd + 1, &fds, NULL, NULL, &tv);
      if (ready <= 0) { continue; }

      // Process control socket first — retransmits are high priority
      if (FD_ISSET(state->control_socket, &fds)) {
        addr_len = sizeof(src_addr);
        int len = recvfrom(state->control_socket, packet, MAX_RTP_PACKET_SIZE, 0, (struct sockaddr*)&src_addr, &addr_len);
        if (len > 0) { process_control_packet(stream, packet, len); }
      }

      // Then process data socket
      if (FD_ISSET(state->data_socket, &fds)) {
        if (!realtime_receive_packet(stream, packet, &src_addr, &addr_len)) { break; }
      }
    } else {
      if (!realtime_receive_packet(stream, packet, &src_addr, &addr_len)) { break; }
    }
  }

  free(packet);
  state->task_handle = NULL;
  vTaskDelete(NULL);
}

static esp_err_t realtime_start(audio_stream_t* stream, uint16_t port) {
  audio_receiver_state_t* state = audio_stream_state(stream);
  if (stream->running) {
    ESP_LOGI(TAG, "Audio receiver already running on port %u", state->data_port);
    return ESP_OK;
  }

  uint16_t bound_port = port;
  state->data_socket = socket_utils_bind_udp(port, 2, 131072, &bound_port);
  if (state->data_socket < 0) { return ESP_FAIL; }
  state->data_port = bound_port;

  // Bind control socket for sending retransmit requests
  if (state->control_port > 0 && state->client_control_port > 0) {
    uint16_t ctrl_bound = state->control_port;
    state->control_socket = socket_utils_bind_udp(state->control_port, 1, 0, &ctrl_bound);
    if (state->control_socket >= 0) {
      ESP_LOGI(TAG, "Control socket on port %u, retransmit to client port %u", ctrl_bound, state->client_control_port);
    } else {
      ESP_LOGW(TAG, "Failed to bind control socket, retransmit disabled");
    }
  }

  reorder_reset();

  // Free stale decode ring if previous stop didn't fully clean up
  if (state->decode_ring) {
    vRingbufferDelete(state->decode_ring);
    state->decode_ring = NULL;
  }

  // Create decode ring buffer on PSRAM for recv→decode pipeline
  state->decode_ring = xRingbufferCreateWithCaps(DECODE_RING_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!state->decode_ring) {
    ESP_LOGW(TAG, "PSRAM decode ring failed, trying internal RAM");
    state->decode_ring = xRingbufferCreate(DECODE_RING_SIZE / 2, RINGBUF_TYPE_NOSPLIT);
  }
  if (!state->decode_ring) {
    ESP_LOGE(TAG, "Failed to create decode ring buffer");
    close(state->data_socket);
    state->data_socket = 0;
    if (state->control_socket >= 0) {
      close(state->control_socket);
      state->control_socket = -1;
    }
    return ESP_FAIL;
  }

  stream->running = true;

  // Start decode task (CPU-intensive ALAC decode on Core 1)
  BaseType_t ret = xTaskCreatePinnedToCore(
    decode_task,
    "audio_dec",
    TASK_DECODE_STACK,
    stream,
    TASK_DECODE_PRIORITY,
    &state->decode_task_handle,
    TASK_DECODE_CORE);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create decode task");
    vRingbufferDelete(state->decode_ring);
    state->decode_ring = NULL;
    close(state->data_socket);
    state->data_socket = 0;
    if (state->control_socket >= 0) {
      close(state->control_socket);
      state->control_socket = -1;
    }
    stream->running = false;
    return ESP_FAIL;
  }

  // Start receiver task (fast recv + reorder on Core 0)
  ret = xTaskCreatePinnedToCore(
    receiver_task,
    "audio_recv",
    TASK_AUDIO_RECV_STACK,
    stream,
    TASK_AUDIO_RECV_PRIORITY,
    &state->task_handle,
    TASK_AUDIO_RECV_CORE);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create receiver task");
    stream->running = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    state->decode_task_handle = NULL;
    vRingbufferDelete(state->decode_ring);
    state->decode_ring = NULL;
    close(state->data_socket);
    state->data_socket = 0;
    if (state->control_socket >= 0) {
      close(state->control_socket);
      state->control_socket = -1;
    }
    return ESP_FAIL;
  }

  return ESP_OK;
}

static void realtime_stop(audio_stream_t* stream) {
  audio_receiver_state_t* state = audio_stream_state(stream);
  if (!stream->running) { return; }

  stream->running = false;

  if (state->data_socket > 0) {
    close(state->data_socket);
    state->data_socket = 0;
  }

  if (state->control_socket >= 0) {
    close(state->control_socket);
    state->control_socket = -1;
  }

  // Wait for recv task to exit
  if (state->task_handle) {
    vTaskDelay(pdMS_TO_TICKS(200));
    state->task_handle = NULL;
  }

  // Wait for decode task to exit
  if (state->decode_task_handle) {
    vTaskDelay(pdMS_TO_TICKS(200));
    state->decode_task_handle = NULL;
  }

  // Clean up decode ring buffer
  if (state->decode_ring) {
    vRingbufferDelete(state->decode_ring);
    state->decode_ring = NULL;
  }
}

static uint16_t realtime_get_port(audio_stream_t* stream) {
  audio_receiver_state_t* state = audio_stream_state(stream);
  return state->data_port;
}

static bool realtime_is_running(audio_stream_t* stream) { return stream->running; }

static void realtime_destroy(audio_stream_t* stream) {
  if (!stream) { return; }

  realtime_stop(stream);
  free(stream);
}

void audio_realtime_get_reorder_stats(uint32_t* ok, uint32_t* sil, uint32_t* gap, uint32_t* fdrain, uint32_t* recovered, int* buf_depth) {
  if (ok) { *ok = reorder_drain_count; }
  if (sil) { *sil = reorder_silence_count; }
  if (gap) { *gap = reorder_gap_count; }
  if (fdrain) { *fdrain = reorder_force_drain_count; }
  if (recovered) { *recovered = retransmit_recovered_count; }
  if (buf_depth) { *buf_depth = reorder.initialized ? (int16_t)(reorder.highest_seq - reorder.next_seq) : 0; }
}

const audio_stream_ops_t audio_stream_realtime_ops = {
  .start = realtime_start,
  .stop = realtime_stop,
  .receive_packet = NULL,
  .decrypt_payload = NULL,
  .get_port = realtime_get_port,
  .is_running = realtime_is_running,
  .destroy = realtime_destroy};
