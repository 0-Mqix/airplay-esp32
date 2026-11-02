#pragma once

#include <stddef.h>

#include "freertos/FreeRTOS.h"

// No-split ring buffer: a FIFO of items with ESP-IDF's per-item overhead counted
// against the capacity. Sends fail right away when full (nothing else can run to
// make space in a single threaded simulation).
typedef struct sim_ring* RingbufHandle_t;

typedef enum { RINGBUF_TYPE_NOSPLIT = 0 } RingbufferType_t;

RingbufHandle_t xRingbufferCreate(size_t capacity, RingbufferType_t type);
RingbufHandle_t xRingbufferCreateWithCaps(size_t capacity, RingbufferType_t type, uint32_t caps);
// Like ESP-IDF: vRingbufferDelete() frees nothing of a WithCaps ring (counted in
// sim_ring_leaked_bytes), vRingbufferDeleteWithCaps() only accepts WithCaps rings
void            vRingbufferDelete(RingbufHandle_t ring);
void            vRingbufferDeleteWithCaps(RingbufHandle_t ring);
BaseType_t      xRingbufferSend(RingbufHandle_t ring, const void* data, size_t size, TickType_t ticks);
void*           xRingbufferReceive(RingbufHandle_t ring, size_t* size, TickType_t ticks);
void            vRingbufferReturnItem(RingbufHandle_t ring, void* item);
size_t          xRingbufferGetCurFreeSize(RingbufHandle_t ring);
