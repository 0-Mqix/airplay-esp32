// Host implementations of the ESP-IDF / FreeRTOS calls used by the audio code

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

// Owned by the simulator
extern int64_t sim_time_us;
extern bool    sim_verbose;
extern int     sim_warnings;

int64_t esp_timer_get_time(void) { return sim_time_us; }

void sim_log(char level, const char* tag, const char* format, ...) {
  if (level == 'W' || level == 'E') { sim_warnings++; }
  if (!sim_verbose && level != 'E') { return; }

  printf("    [%9.3f] %c %s: ", (double)sim_time_us / 1e6, level, tag);
  va_list args;
  va_start(args, format);
  vprintf(format, args);
  va_end(args);
  printf("\n");
}

BaseType_t xTaskCreatePinnedToCore(
  TaskFunction_t function,
  const char*    name,
  uint32_t       stack,
  void*          parameter,
  UBaseType_t    priority,
  TaskHandle_t*  handle,
  BaseType_t     core) {
  (void)function;
  (void)name;
  (void)stack;
  (void)parameter;
  (void)priority;
  (void)core;
  if (handle) { *handle = NULL; }
  return pdPASS;
}

void vTaskDelay(TickType_t ticks) { (void)ticks; }

void vTaskDelete(TaskHandle_t task) { (void)task; }

// ============================================================================
// Ring buffer
// ============================================================================

// ESP-IDF no-split items carry an 8 byte header and are padded to 4 bytes
#define ITEM_OVERHEAD 8

typedef struct sim_ring_item {
  struct sim_ring_item* next;
  size_t                size;
  size_t                cost;
  bool                  returned;
  _Alignas(8) uint8_t   data[]; // ESP-IDF hands out 4-byte aligned items
} sim_ring_item_t;

struct sim_ring {
  bool             with_caps;
  size_t           capacity;
  size_t           used;
  sim_ring_item_t* head;   // Oldest item still holding space
  sim_ring_item_t* tail;   // Newest item
  sim_ring_item_t* unread; // Next item to hand out
};

static size_t item_cost(size_t size) { return ITEM_OVERHEAD + ((size + 3) & ~(size_t)3); }

RingbufHandle_t xRingbufferCreate(size_t capacity, RingbufferType_t type) {
  (void)type;
  RingbufHandle_t ring = calloc(1, sizeof(*ring));
  if (ring) { ring->capacity = capacity; }
  return ring;
}

RingbufHandle_t xRingbufferCreateWithCaps(size_t capacity, RingbufferType_t type, uint32_t caps) {
  (void)caps;
  RingbufHandle_t ring = xRingbufferCreate(capacity, type);
  if (ring) { ring->with_caps = true; }
  return ring;
}

size_t sim_ring_leaked_bytes = 0;
int    sim_ring_misuse = 0;

static void ring_free(RingbufHandle_t ring) {
  sim_ring_item_t* item = ring->head;
  while (item) {
    sim_ring_item_t* next = item->next;
    free(item);
    item = next;
  }
  free(ring);
}

void vRingbufferDelete(RingbufHandle_t ring) {
  if (!ring) { return; }
  if (ring->with_caps) {
    // ESP-IDF treats it as static memory and frees nothing
    sim_ring_leaked_bytes += ring->capacity;
    return;
  }
  ring_free(ring);
}

void vRingbufferDeleteWithCaps(RingbufHandle_t ring) {
  if (!ring) { return; }
  if (!ring->with_caps) {
    // ESP-IDF would heap_caps_free() memory it did not allocate that way
    sim_ring_misuse++;
    return;
  }
  ring_free(ring);
}

BaseType_t xRingbufferSend(RingbufHandle_t ring, const void* data, size_t size, TickType_t ticks) {
  (void)ticks;
  size_t cost = item_cost(size);
  // No-split items larger than half the buffer never fit in ESP-IDF either
  if (cost > ring->capacity / 2 || ring->used + cost > ring->capacity) { return pdFALSE; }

  sim_ring_item_t* item = malloc(sizeof(*item) + size);
  if (!item) { return pdFALSE; }
  item->next = NULL;
  item->size = size;
  item->cost = cost;
  item->returned = false;
  memcpy(item->data, data, size);

  if (ring->tail) {
    ring->tail->next = item;
  } else {
    ring->head = item;
  }
  ring->tail = item;
  if (!ring->unread) { ring->unread = item; }
  ring->used += cost;
  return pdTRUE;
}

void* xRingbufferReceive(RingbufHandle_t ring, size_t* size, TickType_t ticks) {
  (void)ticks;
  sim_ring_item_t* item = ring->unread;
  if (!item) { return NULL; }
  ring->unread = item->next;
  if (size) { *size = item->size; }
  return item->data;
}

void vRingbufferReturnItem(RingbufHandle_t ring, void* data) {
  sim_ring_item_t* item = (sim_ring_item_t*)((uint8_t*)data - offsetof(sim_ring_item_t, data));
  item->returned = true;

  // Space frees in order, like the real ring buffer
  while (ring->head && ring->head->returned) {
    sim_ring_item_t* oldest = ring->head;
    ring->head = oldest->next;
    if (!ring->head) { ring->tail = NULL; }
    ring->used -= oldest->cost;
    free(oldest);
  }
}

size_t xRingbufferGetCurFreeSize(RingbufHandle_t ring) { return ring->capacity - ring->used; }
