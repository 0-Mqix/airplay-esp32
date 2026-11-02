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
#include "esp_timer.h"
#include "freertos/ringbuf.h"
#include "network/socket_utils.h"

#define RTP_HEADER_SIZE 12

// Reorder buffer: in-order packets go straight through, a missing packet holds the ones
// behind it until the retransmit arrives or it is too old, then it becomes silence.
// The wait is timed from when the gap is noticed: after a loss burst the packets behind
// it arrive at once, and the retransmits need a round trip on top. A burst plus the
// wait must stay below the playback buffer (CONFIG_REALTIME_BUFFER_MS).
#define REORDER_SLOTS          128
#define REORDER_MAX_WAIT_US    250000 // Wait for a missing packet's retransmit
#define REORDER_MAX_WAIT       100    // Packets, never hold more than this (slots run out)
#define REORDER_MIN_BUFFER_MS  100    // Stop waiting when playback has less than this left
#define REORDER_RETRY_INTERVAL 8      // Packets between retransmit requests while waiting
#define REORDER_START_HOLD     4  // Packets held at the start so the first ones may arrive swapped
#define REORDER_MAX_PAYLOAD    MAX_RTP_PACKET_SIZE // Never truncate a frame before the decoder

// Decode ring buffer: compressed frames queued from recv task to decode task
// Each item: 10-byte header + payload (ALAC frames are ~1.4 KB at most)
// 512KB on PSRAM gives ~340 items of headroom
#define DECODE_RING_SIZE (512 * 1024)

typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  uint32_t generation;
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
  bool           holding;        // Start of the stream: nothing passed on yet
  uint32_t       next_timestamp; // Expected timestamp of next_seq, for silence in its place
  uint32_t       generation;     // Flush generation the buffered packets belong to
  uint16_t       wait_packets;   // Packets received while next_seq is missing
  int64_t        missing_since_us;
} reorder_buffer_t;

static const char* TAG = "audio_rt";

static reorder_buffer_t reorder = {0};
static int16_t          silence_samples[1024 * 2] = {0};
static uint32_t         reorder_drain_count = 0;
static uint32_t         reorder_silence_count = 0;
static uint32_t         reorder_gap_count = 0;
static uint32_t         reorder_resync_count = 0;
static uint32_t         retransmit_recovered_count = 0;

static void reorder_reset(void) {
  if (!reorder.slots) {
    reorder.slots = heap_caps_calloc(REORDER_SLOTS, sizeof(reorder_slot_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!reorder.slots) { reorder.slots = calloc(REORDER_SLOTS, sizeof(reorder_slot_t)); }
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
  reorder.wait_packets = 0;
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
static void reorder_insert(audio_stream_t* stream, uint16_t seq, uint32_t timestamp, const uint8_t* data, size_t len, bool retransmit);

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

  reorder_insert(stream, seq, timestamp, audio_data, audio_len, true);
}

// ============================================================================
// Silence insertion for unrecoverable packet loss
// ============================================================================

static uint32_t reorder_frame_size(audio_receiver_state_t* state) {
  int frame_size = state->stream->format.frame_size > 0 ? state->stream->format.frame_size : 352;
  if (frame_size > 1024) { frame_size = 1024; }
  return (uint32_t)frame_size;
}

static void insert_silence(audio_receiver_state_t* state, uint32_t generation, uint32_t timestamp) {
  int channels = state->stream->format.channels > 0 ? state->stream->format.channels : 2;

  audio_buffer_queue_decoded(&state->buffer, NULL, generation, timestamp, silence_samples, reorder_frame_size(state), channels);
}

// Send a compressed frame (or silence marker) to the decode ring buffer
static bool
decode_ring_send(audio_receiver_state_t* state, uint32_t generation, uint32_t timestamp, const uint8_t* data, uint16_t len) {
  if (!state->decode_ring) { return false; }

  decode_item_header_t hdr = {.timestamp = timestamp, .generation = generation, .len = len};
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

// ============================================================================
// Reorder buffer operations
// ============================================================================

// Pass on every packet that is ready, in sequence order. A missing packet holds the
// ones behind it until its retransmit arrives or REORDER_MAX_WAIT newer packets came in.
static void reorder_drain(audio_receiver_state_t* state) {
  uint32_t frame_size = reorder_frame_size(state);

  if (reorder.holding) {
    if ((int16_t)(reorder.highest_seq - reorder.next_seq) < REORDER_START_HOLD) { return; }
    reorder.holding = false;
  }

  while ((int16_t)(reorder.highest_seq - reorder.next_seq) >= 0) {
    reorder_slot_t* slot = &reorder.slots[reorder.next_seq % REORDER_SLOTS];

    if (slot->occupied && slot->seq == reorder.next_seq) {
      decode_ring_send(state, reorder.generation, slot->timestamp, slot->data, slot->len);
      reorder.next_timestamp = slot->timestamp + frame_size;
      slot->occupied = false;
      reorder_drain_count++;
    } else {
      int64_t now = esp_timer_get_time();
      if (reorder.wait_packets == 0) { reorder.missing_since_us = now; }

      // While audio plays out, waiting longer than the buffer lasts turns one lost
      // packet into an underrun and a rebuffer
      int  sample_rate = state->stream->format.sample_rate > 0 ? state->stream->format.sample_rate : 44100;
      bool buffer_low = atomic_load(&state->playout_running) &&
                        audio_buffer_get_sample_count(&state->buffer) < sample_rate * REORDER_MIN_BUFFER_MS / 1000;

      if (!buffer_low && now - reorder.missing_since_us < REORDER_MAX_WAIT_US &&
          (int16_t)(reorder.highest_seq - reorder.next_seq) < REORDER_MAX_WAIT) {
        // Still missing: wait, and ask again every few packets
        reorder.wait_packets++;
        if (reorder.wait_packets % REORDER_RETRY_INTERVAL == 0) { send_retransmit_request(state, reorder.next_seq, 1); }
        return;
      }

      // The retransmit is not coming in time: play silence in its place
      decode_ring_send(state, reorder.generation, reorder.next_timestamp, NULL, 0);
      reorder.next_timestamp += frame_size;
      reorder_silence_count++;
      state->stats.packets_dropped++;
    }

    reorder.next_seq++;
    reorder.wait_packets = 0;
  }
}

static void reorder_start(uint16_t seq, uint32_t timestamp) {
  reorder.next_seq = seq;
  reorder.highest_seq = seq;
  reorder.next_timestamp = timestamp;
  reorder.initialized = true;
  reorder.holding = true;
}

static void reorder_insert(
  audio_stream_t* stream, uint16_t seq, uint32_t timestamp, const uint8_t* data, size_t len, bool retransmit) {
  audio_receiver_state_t* state = audio_stream_state(stream);

  if (!reorder.slots) { return; }

  // After a flush, forget the packets of the old stream
  uint32_t generation = audio_receiver_get_generation();
  if (generation != reorder.generation) {
    reorder_reset();
    reorder.generation = generation;
  }

  if (!retransmit && audio_stream_drop_flushed(state, seq, timestamp)) { return; }

  if (!reorder.initialized) {
    // Only a fresh packet starts the sequence, a retransmit may be from before a flush
    if (retransmit) { return; }
    reorder_start(seq, timestamp);
  }

  int16_t diff = (int16_t)(seq - reorder.next_seq);

  if (diff < 0 && reorder.holding && diff > -REORDER_SLOTS / 2 && !retransmit) {
    // Arrived after a later packet at the start of the stream: start from this one
    reorder.next_seq = seq;
    reorder.next_timestamp = timestamp;
    diff = 0;
  }

  if (diff < 0) {
    // Late arrival — already moved past this sequence
    // counted in periodic stats
    state->stats.late_frames++;
    return;
  }

  if (diff >= REORDER_SLOTS) {
    // Sequence jumped (long outage or sender restart). Start over at this packet instead
    // of queueing seconds of silence; playback rebuffers if it ran dry meanwhile.
    if (retransmit) { return; }
    reorder_resync_count++;
    reorder_reset();
    reorder_start(seq, timestamp);
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
  if (slot->occupied && slot->seq == seq) { return; } // Duplicate
  if (retransmit) { retransmit_recovered_count++; }

  size_t copy_len = len < REORDER_MAX_PAYLOAD ? len : REORDER_MAX_PAYLOAD;
  memcpy(slot->data, data, copy_len);
  slot->len = (uint16_t)copy_len;
  slot->seq = seq;
  slot->timestamp = timestamp;
  slot->occupied = true;

  // Track highest sequence for the wait limit
  if ((int16_t)(seq - reorder.highest_seq) > 0) { reorder.highest_seq = seq; }

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

  // Insert into reorder buffer — handles flush filtering, sequencing, retransmit, and silence
  reorder_insert(stream, seq, timestamp, audio_data, audio_len, false);

  return true;
}

// ============================================================================
// Decode task: pulls compressed frames from decode_ring, decodes, queues PCM
// ============================================================================

static void decode_item(audio_receiver_state_t* state, const void* item, size_t item_size) {
  if (item_size < sizeof(decode_item_header_t)) { return; }

  decode_item_header_t hdr;
  memcpy(&hdr, item, sizeof(hdr));

  if (hdr.generation != audio_receiver_get_generation()) {
    // Queued before a flush, skip without decoding
  } else if (hdr.len > 0) {
    const uint8_t* payload = (const uint8_t*)item + sizeof(decode_item_header_t);
    audio_stream_process_frame(state, hdr.generation, hdr.timestamp, payload, hdr.len);
  } else {
    insert_silence(state, hdr.generation, hdr.timestamp);
  }
}

static void decode_task(void* pvParameters) {
  audio_stream_t*         stream = (audio_stream_t*)pvParameters;
  audio_receiver_state_t* state = audio_stream_state(stream);

  while (stream->running) {
    size_t item_size = 0;
    void*  item = xRingbufferReceive(state->decode_ring, &item_size, pdMS_TO_TICKS(100));
    if (!item) { continue; }

    decode_item(state, item, item_size);
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
    state->task_handle = NULL;
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

// A ring from xRingbufferCreateWithCaps() must be freed with vRingbufferDeleteWithCaps():
// vRingbufferDelete() frees nothing for it, which leaked 512 KB of PSRAM per stream stop
static bool decode_ring_with_caps = false;

static void decode_ring_delete(audio_receiver_state_t* state) {
  if (!state->decode_ring) { return; }
  if (decode_ring_with_caps) {
    vRingbufferDeleteWithCaps(state->decode_ring);
  } else {
    vRingbufferDelete(state->decode_ring);
  }
  state->decode_ring = NULL;
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
  decode_ring_delete(state);

  // Create decode ring buffer on PSRAM for recv→decode pipeline
  state->decode_ring = xRingbufferCreateWithCaps(DECODE_RING_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  decode_ring_with_caps = state->decode_ring != NULL;
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
    decode_ring_delete(state);
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
    decode_ring_delete(state);
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

  // Both tasks clear their handle on exit; they must be gone before the decode ring is
  // deleted. The recv task may sit in select() for up to 2 s.
  for (int waited = 0; (state->task_handle || state->decode_task_handle) && waited < 2500; waited += 10) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  if (state->task_handle || state->decode_task_handle) {
    ESP_LOGW(TAG, "Realtime audio tasks did not exit in time");
    state->task_handle = NULL;
    state->decode_task_handle = NULL;
  }

  decode_ring_delete(state);
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

void audio_realtime_get_reorder_stats(uint32_t* ok, uint32_t* sil, uint32_t* gap, uint32_t* resync, uint32_t* recovered, int* buf_depth) {
  if (ok) { *ok = reorder_drain_count; }
  if (sil) { *sil = reorder_silence_count; }
  if (gap) { *gap = reorder_gap_count; }
  if (resync) { *resync = reorder_resync_count; }
  if (recovered) { *recovered = retransmit_recovered_count; }
  if (buf_depth) { *buf_depth = reorder.initialized ? (int16_t)(reorder.highest_seq - reorder.next_seq) + 1 : 0; }
}

const audio_stream_ops_t audio_stream_realtime_ops = {
  .start = realtime_start,
  .stop = realtime_stop,
  .receive_packet = NULL,
  .decrypt_payload = NULL,
  .get_port = realtime_get_port,
  .is_running = realtime_is_running,
  .destroy = realtime_destroy};
