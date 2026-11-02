#include "rtsp_server.h"

#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "config.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rtsp_conn.h"
#include "rtsp_crypto.h"
#include "rtsp_handlers.h"
#include "rtsp_message.h"

static const char* TAG = "rtsp_server";

#define RTSP_PORT           7000
#define RTSP_BUFFER_INITIAL 4096
#define RTSP_BUFFER_LARGE   (256 * 1024)

static int          server_socket = -1;
static TaskHandle_t server_task_handle = NULL;
static bool         server_running = false;

// Connections served side by side: iOS opens extra ones (GET /info when the AirPlay
// picker opens) and a passenger's phone may connect. A new connection no longer ends
// the playing session; only a SETUP takes it over (see rtsp_handlers.c).
#define MAX_CLIENTS 4

// Dead connections (phone left the WiFi) are found by TCP keepalive after
// KEEPALIVE_IDLE_S + KEEPALIVE_INTERVAL_S * KEEPALIVE_COUNT seconds
#define KEEPALIVE_IDLE_S     30
#define KEEPALIVE_INTERVAL_S 5
#define KEEPALIVE_COUNT      4

// Client slot for tracking connections
typedef struct {
  rtsp_conn_t*  conn;
  TaskHandle_t  task;
  int           socket;
  volatile bool should_stop;
  uint32_t      accepted; // Accept order, the oldest idle connection is evicted first
} client_slot_t;

static client_slot_t clients[MAX_CLIENTS] = {0};
static uint32_t      accept_count = 0;

// Helper to grow buffer
static uint8_t* grow_buffer(uint8_t* old_buf, size_t old_size, size_t new_size, size_t data_len) {
  (void)old_size;
  uint8_t* new_buf = heap_caps_malloc(new_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!new_buf) { new_buf = malloc(new_size); }
  if (!new_buf) { return NULL; }
  if (old_buf && data_len > 0) { memcpy(new_buf, old_buf, data_len); }
  free(old_buf);
  return new_buf;
}

// Process buffered RTSP requests
static void process_rtsp_buffer(client_slot_t* slot, uint8_t* buffer, size_t* buf_len) {
  while (*buf_len > 0 && !slot->should_stop) {
    const uint8_t* header_end = rtsp_find_header_end(buffer, *buf_len);
    if (!header_end) { break; }

    size_t header_len = (size_t)(header_end - buffer) + 4;
    char*  header_str = malloc(header_len + 1);
    if (!header_str) {
      *buf_len = 0;
      break;
    }
    memcpy(header_str, buffer, header_len);
    header_str[header_len] = '\0';

    int content_len = rtsp_parse_content_length(header_str);
    if (content_len < 0) content_len = 0;

    size_t total_len = header_len + (size_t)content_len;
    if (total_len > RTSP_BUFFER_LARGE || *buf_len < total_len) {
      free(header_str);
      if (total_len > RTSP_BUFFER_LARGE) *buf_len = 0;
      break;
    }

    rtsp_dispatch(slot->socket, slot->conn, buffer, total_len);
    free(header_str);

    if (*buf_len > total_len) { memmove(buffer, buffer + total_len, *buf_len - total_len); }
    *buf_len -= total_len;
  }
}

// Client task
static void client_task(void* pvParameters) {
  int            slot_idx = (int)(intptr_t)pvParameters;
  client_slot_t* slot = &clients[slot_idx];

  // Create connection state
  rtsp_conn_t* conn = rtsp_conn_create();
  if (!conn) {
    ESP_LOGE(TAG, "Failed to create connection state");
    close(slot->socket);
    slot->socket = -1;
    slot->task = NULL;
    vTaskDelete(NULL);
    return;
  }
  slot->conn = conn;

  // Get client IP address for timing requests
  struct sockaddr_in peer_addr;
  socklen_t          peer_len = sizeof(peer_addr);
  if (getpeername(slot->socket, (struct sockaddr*)&peer_addr, &peer_len) == 0) {
    conn->client_ip = peer_addr.sin_addr.s_addr;
    ESP_LOGI(
      TAG,
      "Client IP: %d.%d.%d.%d",
      conn->client_ip & 0xFF,
      (conn->client_ip >> 8) & 0xFF,
      (conn->client_ip >> 16) & 0xFF,
      (conn->client_ip >> 24) & 0xFF);
  }

  // Allocate buffer
  size_t   buf_capacity = RTSP_BUFFER_INITIAL;
  uint8_t* buffer = malloc(buf_capacity);
  if (!buffer) {
    ESP_LOGE(TAG, "Failed to allocate buffer");
    rtsp_conn_free(conn);
    slot->conn = NULL;
    close(slot->socket);
    slot->socket = -1;
    slot->task = NULL;
    vTaskDelete(NULL);
    return;
  }

  size_t buf_len = 0;

  // Socket timeout for stop signal responsiveness
  struct timeval tv = {.tv_sec = 1, .tv_usec = 0};
  setsockopt(slot->socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

  while (server_running && !slot->should_stop) {
    if (conn->encrypted_mode) {
      // Encrypted mode
      while (server_running && conn->encrypted_mode && !slot->should_stop) {
        if (buf_len >= buf_capacity - 1024) {
          size_t new_cap = buf_capacity < RTSP_BUFFER_LARGE ? RTSP_BUFFER_LARGE : buf_capacity * 2;
          if (new_cap > RTSP_BUFFER_LARGE) goto cleanup;
          uint8_t* new_buf = grow_buffer(buffer, buf_capacity, new_cap, buf_len);
          if (!new_buf) goto cleanup;
          buffer = new_buf;
          buf_capacity = new_cap;
        }

        int block_len = rtsp_crypto_read_block(slot->socket, conn, buffer + buf_len, buf_capacity - buf_len);
        if (block_len <= 0) {
          if (slot->should_stop) { goto cleanup; }
          if (errno == EAGAIN || errno == EWOULDBLOCK) { continue; }
          // EPROTO means encrypted stream is out of sync - nonces can't be
          // recovered without reconnecting, so close and let client reconnect
          if (errno == EPROTO) { ESP_LOGW(TAG, "Encrypted stream out of sync, closing connection"); }
          goto cleanup;
        }

        buf_len += (size_t)block_len;
        process_rtsp_buffer(slot, buffer, &buf_len);
      }
      goto cleanup;
    }

    // Plain-text mode
    if (buf_len >= buf_capacity - 1024) {
      size_t new_cap = buf_capacity < RTSP_BUFFER_LARGE ? RTSP_BUFFER_LARGE : buf_capacity * 2;
      if (new_cap > RTSP_BUFFER_LARGE) break;
      uint8_t* new_buf = grow_buffer(buffer, buf_capacity, new_cap, buf_len);
      if (!new_buf) break;
      buffer = new_buf;
      buf_capacity = new_cap;
    }

    int recv_len = recv(slot->socket, buffer + buf_len, buf_capacity - buf_len, 0);
    if (recv_len <= 0) {
      if (recv_len < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) { continue; }
      break;
    }
    buf_len += (size_t)recv_len;
    process_rtsp_buffer(slot, buffer, &buf_len);
  }

cleanup:
  ESP_LOGI(TAG, "Client slot %d disconnected", slot_idx);
  free(buffer);
  close(slot->socket);

  // Stops audio and the event task only if this connection had the session (another
  // connection may have taken it over); then this connection's own sockets
  rtsp_release_session(conn);
  rtsp_conn_free(conn);

  slot->conn = NULL;
  slot->socket = -1;
  slot->should_stop = false;
  slot->task = NULL;

  vTaskDelete(NULL);
}

// Ask a client to stop (non-blocking): shutdown unblocks its recv
static void signal_client_stop(client_slot_t* slot) {
  if (slot->task == NULL) { return; }
  slot->should_stop = true;
  if (slot->socket >= 0) { shutdown(slot->socket, SHUT_RDWR); }
}

static int find_free_slot(void) {
  for (int i = 0; i < MAX_CLIENTS; i++) {
    if (clients[i].task == NULL) { return i; }
  }
  return -1;
}

// All slots taken: end the oldest connection that does not have the playing session
static int make_free_slot(void) {
  client_slot_t* oldest = NULL;
  for (int i = 0; i < MAX_CLIENTS; i++) {
    client_slot_t* slot = &clients[i];
    if (slot->task == NULL || rtsp_owns_session(slot->conn)) { continue; }
    if (!oldest || (int32_t)(slot->accepted - oldest->accepted) < 0) { oldest = slot; }
  }
  if (!oldest) { return -1; }

  ESP_LOGI(TAG, "All client slots in use, closing the oldest idle connection");
  signal_client_stop(oldest);
  for (int waited = 0; waited < 1000; waited += 50) {
    int free_slot = find_free_slot();
    if (free_slot >= 0) { return free_slot; }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
  return -1;
}

static void enable_keepalive(int socket) {
  int on = 1;
  int idle = KEEPALIVE_IDLE_S;
  int interval = KEEPALIVE_INTERVAL_S;
  int count = KEEPALIVE_COUNT;
  setsockopt(socket, SOL_SOCKET, SO_KEEPALIVE, &on, sizeof(on));
  setsockopt(socket, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
  setsockopt(socket, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
  setsockopt(socket, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
}

static void server_task(void* pvParameters) {
  (void)pvParameters;

  struct sockaddr_in server_addr, client_addr;
  socklen_t          client_addr_len = sizeof(client_addr);

  // Initialize slots
  for (int i = 0; i < MAX_CLIENTS; i++) {
    clients[i].socket = -1;
    clients[i].conn = NULL;
    clients[i].task = NULL;
    clients[i].should_stop = false;
  }

  server_socket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (server_socket < 0) {
    ESP_LOGE(TAG, "Failed to create socket: %d", errno);
    vTaskDelete(NULL);
    return;
  }

  int opt = 1;
  setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
  server_addr.sin_port = htons(RTSP_PORT);

  if (bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
    ESP_LOGE(TAG, "Failed to bind: %d", errno);
    close(server_socket);
    server_socket = -1;
    vTaskDelete(NULL);
    return;
  }

  if (listen(server_socket, 5) < 0) {
    ESP_LOGE(TAG, "Failed to listen: %d", errno);
    close(server_socket);
    server_socket = -1;
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "RTSP server listening on port %d", RTSP_PORT);
  server_running = true;

  while (server_running) {
    int new_socket = accept(server_socket, (struct sockaddr*)&client_addr, &client_addr_len);
    if (new_socket < 0) {
      if (server_running) { ESP_LOGE(TAG, "Failed to accept: %d", errno); }
      continue;
    }

    ESP_LOGI(TAG, "New client connected");
    enable_keepalive(new_socket);

    int new_slot = find_free_slot();
    if (new_slot < 0) { new_slot = make_free_slot(); }
    if (new_slot < 0) {
      ESP_LOGW(TAG, "No client slot free, refusing connection");
      close(new_socket);
      continue;
    }

    // Setup new slot
    clients[new_slot].conn = NULL;
    clients[new_slot].socket = new_socket;
    clients[new_slot].should_stop = false;
    clients[new_slot].accepted = accept_count++;

    // Start new client task immediately
    BaseType_t ret = xTaskCreatePinnedToCore(
      client_task,
      "rtsp_client",
      TASK_RTSP_CLIENT_STACK,
      (void*)(intptr_t)new_slot,
      TASK_RTSP_CLIENT_PRIORITY,
      &clients[new_slot].task,
      TASK_RTSP_CLIENT_CORE);
    if (ret != pdPASS) {
      ESP_LOGE(TAG, "Failed to create client task");
      close(new_socket);
      clients[new_slot].socket = -1;
    }
  }

  // Stop all clients
  for (int i = 0; i < MAX_CLIENTS; i++) { signal_client_stop(&clients[i]); }

  vTaskDelay(pdMS_TO_TICKS(500));

  if (server_socket >= 0) {
    close(server_socket);
    server_socket = -1;
  }

  vTaskDelete(NULL);
}

esp_err_t rtsp_server_start(void) {
  if (server_task_handle != NULL) { return ESP_ERR_INVALID_STATE; }

  BaseType_t ret = xTaskCreatePinnedToCore(
    server_task,
    "rtsp_server",
    TASK_RTSP_SERVER_STACK,
    NULL,
    TASK_RTSP_SERVER_PRIORITY,
    &server_task_handle,
    TASK_RTSP_SERVER_CORE);
  if (ret != pdPASS) { return ESP_FAIL; }

  return ESP_OK;
}

void rtsp_server_stop(void) {
  server_running = false;

  if (server_socket >= 0) {
    shutdown(server_socket, SHUT_RDWR);
    close(server_socket);
    server_socket = -1;
  }

  if (server_task_handle != NULL) {
    vTaskDelay(pdMS_TO_TICKS(100));
    server_task_handle = NULL;
  }
}
