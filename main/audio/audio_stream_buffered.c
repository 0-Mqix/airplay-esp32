#include <errno.h>
#include <netinet/in.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "audio_crypto.h"
#include "audio_receiver_internal.h"
#include "config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "network/socket_utils.h"

// The length prefix is 16 bits, so a packet never exceeds this. Lossless (ALAC)
// packets can be well over 8 KB.
#define BUFFERED_AUDIO_PACKET_SIZE 65536

static const char* TAG = "audio_buf";

static ssize_t read_exact(audio_stream_t* stream, int sock, uint8_t* buf, size_t len) {
  size_t total = 0;
  while (total < len && stream->running) {
    ssize_t n = recv(sock, buf + total, len - total, 0);
    if (n <= 0) {
      if (n == 0) {
        // Clean disconnect
        ESP_LOGI(TAG, "Buffered audio client disconnected");
      } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // Timeout - will retry
        continue;
      } else if (errno == ECONNRESET || errno == ENOTCONN || errno == EPIPE || errno == EBADF || !stream->running) {
        // Expected disconnect scenarios - not an error
        ESP_LOGI(TAG, "Buffered audio connection closed");
      } else {
        ESP_LOGW(TAG, "Buffered audio recv error: %d", errno);
      }
      return -1;
    }
    total += (size_t)n;
  }
  if (!stream->running) {
    return -1; // Shutdown requested
  }
  return (ssize_t)total;
}

// Decrypt, flush-filter, decode and queue one packet (without the 2-byte length prefix)
static void buffered_handle_packet(audio_stream_t* stream, uint8_t* packet, size_t packet_len) {
  audio_receiver_state_t* state = audio_stream_state(stream);

  state->stats.packets_received++;

  // RTP header: the sequence number and timestamp below must exist
  if (packet_len < 12) {
    state->stats.packets_dropped++;
    return;
  }

  uint32_t seq_no = ((uint32_t)packet[1] << 16) | ((uint32_t)packet[2] << 8) | packet[3];
  uint32_t timestamp = ((uint32_t)packet[4] << 24) | ((uint32_t)packet[5] << 16) | ((uint32_t)packet[6] << 8) | packet[7];

  uint8_t* decrypted = state->decrypt_buffer;
  size_t   decrypt_capacity = state->decrypt_buffer_size;
  if (!decrypted) {
    decrypted = packet + 12;
    decrypt_capacity = packet_len > 12 ? packet_len - 12 : 0;
  }

  int decrypted_len = audio_crypto_decrypt_buffered(&stream->encrypt, packet, packet_len, decrypted, decrypt_capacity);
  if (decrypted_len < 0) {
    state->stats.decrypt_errors++;
    state->stats.packets_dropped++;
    return;
  }

  // Stamp the generation before the flush check, so a flush that lands while this
  // packet is decoded still invalidates it
  uint32_t generation = audio_receiver_get_generation();

  if (audio_stream_drop_flushed(state, seq_no, timestamp)) { return; }

  state->stats.last_seq = (uint16_t)(seq_no & 0xFFFF);
  state->stats.last_timestamp = timestamp;

  state->blocks_read++;

  if (!audio_stream_process_frame(state, generation, timestamp, decrypted, (size_t)decrypted_len)) { state->stats.packets_dropped++; }
}

static void buffered_audio_task(void* pvParameters) {
  audio_stream_t*         stream = (audio_stream_t*)pvParameters;
  audio_receiver_state_t* state = audio_stream_state(stream);

  while (stream->running) {
    struct sockaddr_in client_addr;
    socklen_t          addr_len = sizeof(client_addr);

    int client_sock = accept(state->buffered_listen_socket, (struct sockaddr*)&client_addr, &addr_len);
    if (client_sock < 0) {
      if (errno != EAGAIN && errno != EWOULDBLOCK && stream->running) { ESP_LOGE(TAG, "Buffered audio accept error: %d", errno); }
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    state->buffered_client_socket = client_sock;

    struct timeval tv = {.tv_sec = 30, .tv_usec = 0};
    setsockopt(client_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    uint8_t* packet = state->buffered_recv_buffer;
    if (!packet) {
      packet = heap_caps_malloc(BUFFERED_AUDIO_PACKET_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
      if (!packet) { packet = malloc(BUFFERED_AUDIO_PACKET_SIZE); }
      if (!packet) {
        ESP_LOGE(TAG, "Failed to allocate buffered audio packet buffer");
        close(client_sock);
        state->buffered_client_socket = -1;
        continue;
      }
      state->buffered_recv_buffer = packet;
    }

    while (stream->running) {
      uint8_t len_buf[2];
      if (read_exact(stream, client_sock, len_buf, 2) != 2) { break; }

      // Always fits the buffer: a 16-bit length is at most BUFFERED_AUDIO_PACKET_SIZE - 1
      uint16_t data_len = (uint16_t)((len_buf[0] << 8) | len_buf[1]);
      if (data_len < 2) {
        ESP_LOGW(TAG, "Invalid buffered audio packet length: %u", data_len);
        break;
      }

      size_t packet_len = data_len - 2;
      if (read_exact(stream, client_sock, packet, packet_len) != (ssize_t)packet_len) { break; }

      buffered_handle_packet(stream, packet, packet_len);
    }

    close(client_sock);
    state->buffered_client_socket = -1;

    // Keep what is already decoded playing out; a new session's SETUP resets the buffer
    if (stream->running) { ESP_LOGI(TAG, "Buffered audio ready for new connection"); }
  }

  state->buffered_task_handle = NULL;
  vTaskDelete(NULL);
}

static esp_err_t buffered_start(audio_stream_t* stream, uint16_t port) {
  audio_receiver_state_t* state = audio_stream_state(stream);
  if (stream->running) {
    ESP_LOGI(TAG, "Buffered audio already running, continuing");
    return ESP_OK;
  }

  uint16_t bound_port = port;
  state->buffered_listen_socket = socket_utils_bind_tcp_listener(port, 1, true, &bound_port);
  if (state->buffered_listen_socket < 0) { return ESP_FAIL; }
  state->buffered_port = bound_port;

  stream->running = true;
  BaseType_t ret = xTaskCreatePinnedToCore(
    buffered_audio_task,
    "buff_audio",
    TASK_BUFFERED_STACK,
    stream,
    TASK_BUFFERED_PRIORITY,
    &state->buffered_task_handle,
    TASK_BUFFERED_CORE);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create buffered audio task");
    close(state->buffered_listen_socket);
    state->buffered_listen_socket = -1;
    stream->running = false;
    return ESP_FAIL;
  }

  return ESP_OK;
}

static void buffered_stop(audio_stream_t* stream) {
  audio_receiver_state_t* state = audio_stream_state(stream);
  if (!stream->running) { return; }

  stream->running = false;

  if (state->buffered_client_socket > 0) {
    close(state->buffered_client_socket);
    state->buffered_client_socket = -1;
  }

  if (state->buffered_listen_socket > 0) {
    close(state->buffered_listen_socket);
    state->buffered_listen_socket = -1;
  }

  // The task clears its handle on exit; it must be gone before its buffer is freed
  for (int waited = 0; state->buffered_task_handle && waited < 2000; waited += 10) { vTaskDelay(pdMS_TO_TICKS(10)); }
  if (state->buffered_task_handle) {
    ESP_LOGW(TAG, "Buffered audio task did not exit in time");
    state->buffered_task_handle = NULL;
  }

  if (state->buffered_recv_buffer) {
    heap_caps_free(state->buffered_recv_buffer);
    state->buffered_recv_buffer = NULL;
  }

  state->buffered_port = 0;
}

static uint16_t buffered_get_port(audio_stream_t* stream) {
  audio_receiver_state_t* state = audio_stream_state(stream);
  return state->buffered_port;
}

static bool buffered_is_running(audio_stream_t* stream) { return stream->running; }

static void buffered_destroy(audio_stream_t* stream) {
  if (!stream) { return; }

  buffered_stop(stream);
  free(stream);
}

const audio_stream_ops_t audio_stream_buffered_ops = {
  .start = buffered_start,
  .stop = buffered_stop,
  .receive_packet = NULL,
  .decrypt_payload = NULL,
  .get_port = buffered_get_port,
  .is_running = buffered_is_running,
  .destroy = buffered_destroy};
