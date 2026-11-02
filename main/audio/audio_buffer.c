#include "audio_buffer.h"

#include <stdlib.h>
#include <string.h>

#include "audio_receiver.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

static const char* TAG = "audio_buf";

// How long a producer waits for ring space before dropping a frame. The ring holds ~16 s,
// so it only fills when the buffered sender runs far ahead; waiting gives TCP backpressure.
#define QUEUE_WAIT_SLICE_MS 100
#define QUEUE_WAIT_MAX_MS   2000

static void audio_buffer_adjust(audio_buffer_t* buffer, int frames, int samples) {
  portENTER_CRITICAL(&buffer->lock);
  buffer->buffered_frames += frames;
  buffer->buffered_samples += samples;
  if (buffer->buffered_frames < 0) { buffer->buffered_frames = 0; }
  if (buffer->buffered_samples < 0) { buffer->buffered_samples = 0; }
  portEXIT_CRITICAL(&buffer->lock);
}

// vRingbufferDelete() does not free a ring made by xRingbufferCreateWithCaps()
static void audio_buffer_delete_ring(audio_buffer_t* buffer) {
  if (!buffer->ring) { return; }
  if (buffer->ring_with_caps) {
    vRingbufferDeleteWithCaps(buffer->ring);
  } else {
    vRingbufferDelete(buffer->ring);
  }
  buffer->ring = NULL;
}

static int audio_buffer_item_samples(const void* item, size_t item_size) {
  if (item_size < sizeof(audio_frame_header_t)) { return 0; }
  return ((const audio_frame_header_t*)item)->samples_per_channel;
}

static bool audio_buffer_queue_chunk(
  audio_buffer_t* buffer,
  audio_stats_t*  stats,
  uint32_t        generation,
  uint32_t        timestamp,
  const int16_t*  pcm_data,
  size_t          samples,
  int             channels) {
  if (samples == 0) { return false; }

  audio_frame_header_t* hdr = (audio_frame_header_t*)buffer->frame_buffer;
  hdr->rtp_timestamp = timestamp;
  hdr->samples_per_channel = (uint16_t)samples;
  hdr->channels = (uint8_t)channels;
  hdr->reserved = 0;
  hdr->generation = generation;

  size_t   pcm_bytes = samples * channels * sizeof(int16_t);
  int16_t* dest = (int16_t*)(hdr + 1);
  memmove(dest, pcm_data, pcm_bytes);

  size_t total_bytes = sizeof(*hdr) + pcm_bytes;

  // Count the frame before it becomes visible, so a concurrent take/flush never
  // decrements a frame that was not counted yet
  audio_buffer_adjust(buffer, 1, (int)samples);

  BaseType_t ret = pdFALSE;
  for (int waited = 0; waited < QUEUE_WAIT_MAX_MS; waited += QUEUE_WAIT_SLICE_MS) {
    ret = xRingbufferSend(buffer->ring, buffer->frame_buffer, total_bytes, pdMS_TO_TICKS(QUEUE_WAIT_SLICE_MS));
    if (ret == pdTRUE || generation != audio_receiver_get_generation()) { break; }
  }

  if (ret != pdTRUE) {
    audio_buffer_adjust(buffer, -1, -(int)samples);
    if (generation == audio_receiver_get_generation()) {
      ESP_LOGW(
        TAG, "OVERRUN ring full, dropping frame ts=%lu samples=%u frames_buffered=%d", (unsigned long)timestamp, (unsigned)samples, buffer->buffered_frames);
      if (stats) { stats->buffer_underruns++; }
    }
    return false;
  }

  if (stats) { stats->packets_decoded++; }
  return true;
}

esp_err_t audio_buffer_init(audio_buffer_t* buffer) {
  if (!buffer) { return ESP_ERR_INVALID_ARG; }

  portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;
  buffer->lock = lock;
  buffer->buffered_frames = 0;
  buffer->buffered_samples = 0;

  buffer->ring = xRingbufferCreateWithCaps(AUDIO_BUFFER_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  buffer->ring_with_caps = buffer->ring != NULL;
  if (!buffer->ring) {
    ESP_LOGW(TAG, "PSRAM not available, using smaller buffer");
    buffer->ring = xRingbufferCreate(1024 * 1024, RINGBUF_TYPE_NOSPLIT);
  }
  if (!buffer->ring) {
    ESP_LOGE(TAG, "Failed to create ring buffer");
    return ESP_ERR_NO_MEM;
  }

  size_t max_pcm_bytes = MAX_SAMPLES_PER_FRAME * AUDIO_MAX_CHANNELS * sizeof(int16_t);
  buffer->frame_buffer = (uint8_t*)malloc(sizeof(audio_frame_header_t) + max_pcm_bytes);
  if (!buffer->frame_buffer) {
    ESP_LOGE(TAG, "Failed to allocate frame buffer");
    audio_buffer_delete_ring(buffer);
    return ESP_ERR_NO_MEM;
  }

  buffer->decode_buffer = (int16_t*)(buffer->frame_buffer + sizeof(audio_frame_header_t));
  buffer->decode_capacity_samples = MAX_SAMPLES_PER_FRAME;

  ESP_LOGI(TAG, "Ring buffer created: %zu bytes for %d frames", (size_t)AUDIO_BUFFER_SIZE, MAX_RING_BUFFER_FRAMES);

  return ESP_OK;
}

void audio_buffer_deinit(audio_buffer_t* buffer) {
  if (!buffer) { return; }

  audio_buffer_delete_ring(buffer);

  if (buffer->frame_buffer) {
    free(buffer->frame_buffer);
    buffer->frame_buffer = NULL;
    buffer->decode_buffer = NULL;
    buffer->decode_capacity_samples = 0;
  }

  buffer->buffered_frames = 0;
  buffer->buffered_samples = 0;
}

void audio_buffer_flush(audio_buffer_t* buffer) {
  if (!buffer || !buffer->ring) { return; }

  // Zero-copy drain: receive pointer + return, no data copying. Counts are
  // decremented per item so frames a producer is queueing right now stay counted.
  size_t bytes_read = 0;
  void*  data = NULL;
  while ((data = xRingbufferReceive(buffer->ring, &bytes_read, 0)) != NULL) {
    int samples = audio_buffer_item_samples(data, bytes_read);
    vRingbufferReturnItem(buffer->ring, data);
    audio_buffer_adjust(buffer, -1, -samples);
  }
}

int audio_buffer_get_frame_count(audio_buffer_t* buffer) {
  if (!buffer) { return 0; }

  int frames = 0;
  portENTER_CRITICAL(&buffer->lock);
  frames = buffer->buffered_frames;
  portEXIT_CRITICAL(&buffer->lock);
  return frames;
}

int audio_buffer_get_sample_count(audio_buffer_t* buffer) {
  if (!buffer) { return 0; }

  int samples = 0;
  portENTER_CRITICAL(&buffer->lock);
  samples = buffer->buffered_samples;
  portEXIT_CRITICAL(&buffer->lock);
  return samples;
}

bool audio_buffer_take(audio_buffer_t* buffer, void** item, size_t* item_size, TickType_t ticks) {
  if (!buffer || !buffer->ring || !item || !item_size) { return false; }

  *item = xRingbufferReceive(buffer->ring, item_size, ticks);
  if (!*item) { return false; }

  audio_buffer_adjust(buffer, -1, -audio_buffer_item_samples(*item, *item_size));
  return true;
}

void audio_buffer_return(audio_buffer_t* buffer, void* item) {
  if (!buffer || !buffer->ring || !item) { return; }

  vRingbufferReturnItem(buffer->ring, item);
}

int16_t* audio_buffer_get_decode_buffer(audio_buffer_t* buffer, size_t* capacity_samples) {
  if (!buffer) { return NULL; }

  if (capacity_samples) { *capacity_samples = buffer->decode_capacity_samples; }
  return buffer->decode_buffer;
}

bool audio_buffer_queue_decoded(
  audio_buffer_t* buffer,
  audio_stats_t*  stats,
  uint32_t        generation,
  uint32_t        timestamp,
  const int16_t*  pcm_data,
  size_t          samples,
  int             channels) {
  if (!buffer || !pcm_data || samples == 0) { return false; }

  if (channels <= 0) { channels = 2; }

  size_t   offset = 0;
  uint32_t chunk_timestamp = timestamp;

  while (offset < samples) {
    size_t chunk_samples = samples - offset;
    if (chunk_samples > AAC_FRAMES_PER_PACKET) { chunk_samples = AAC_FRAMES_PER_PACKET; }

    if (!audio_buffer_queue_chunk(buffer, stats, generation, chunk_timestamp, pcm_data + (offset * channels), chunk_samples, channels)) {
      return false;
    }

    offset += chunk_samples;
    chunk_timestamp += chunk_samples;
  }

  return true;
}
