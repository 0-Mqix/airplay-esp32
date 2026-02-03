#include <stdlib.h>
#include <string.h>

#include "audio_timing.h"

#include "freertos/FreeRTOS.h"

#define DEFAULT_BUFFER_LATENCY_US 200000  // 200ms buffer
#define MIN_STARTUP_FRAMES        4

static uint32_t frame_samples_from_format(const audio_format_t *format) {
  if (format->frame_size > 0) {
    return (uint32_t)format->frame_size;
  }
  if (format->max_samples_per_frame > 0) {
    return format->max_samples_per_frame;
  }
  return AAC_FRAMES_PER_PACKET;
}

static void update_timing_targets(audio_timing_t *timing,
                                  const audio_format_t *format) {
  timing->nominal_frame_samples = frame_samples_from_format(format);

  if (format->sample_rate <= 0 || timing->nominal_frame_samples == 0) {
    timing->target_buffer_frames = MIN_STARTUP_FRAMES;
    return;
  }

  uint64_t latency_samples =
      ((uint64_t)timing->output_latency_us * (uint64_t)format->sample_rate) /
      1000000ULL;
  uint32_t target_frames =
      (uint32_t)((latency_samples + timing->nominal_frame_samples - 1) /
                 timing->nominal_frame_samples);
  if (target_frames < MIN_STARTUP_FRAMES) {
    target_frames = MIN_STARTUP_FRAMES;
  }
  timing->target_buffer_frames = target_frames;
}

void audio_timing_init(audio_timing_t *timing, size_t pending_capacity) {
  if (!timing) {
    return;
  }
  (void)pending_capacity;  // Not used in simple mode

  memset(timing, 0, sizeof(*timing));
  timing->output_latency_us = DEFAULT_BUFFER_LATENCY_US;
  timing->playing = true;
}

void audio_timing_reset(audio_timing_t *timing) {
  if (!timing) {
    return;
  }
  timing->playout_started = false;
}

void audio_timing_set_flushing(audio_timing_t *timing, bool flushing) {
  (void)timing;
  (void)flushing;
  // Not used in simple mode
}

void audio_timing_set_format(audio_timing_t *timing,
                             const audio_format_t *format) {
  if (!timing || !format) {
    return;
  }

  update_timing_targets(timing, format);
}

void audio_timing_set_output_latency(audio_timing_t *timing,
                                     const audio_format_t *format,
                                     uint32_t latency_us) {
  if (!timing || !format) {
    return;
  }

  timing->output_latency_us = latency_us;
  update_timing_targets(timing, format);
}

uint32_t audio_timing_get_output_latency(const audio_timing_t *timing) {
  if (!timing) {
    return 0;
  }

  return timing->output_latency_us;
}

void audio_timing_set_anchor(audio_timing_t *timing,
                             const audio_format_t *format, uint64_t clock_id,
                             uint64_t network_time_ns, uint32_t rtp_time) {
  (void)timing;
  (void)format;
  (void)clock_id;
  (void)network_time_ns;
  (void)rtp_time;
  // Not used in simple mode
}

void audio_timing_set_playing(audio_timing_t *timing, bool playing) {
  if (!timing) {
    return;
  }
  timing->playing = playing;
  if (!playing) {
    timing->playout_started = false;
  }
}

size_t audio_timing_read(audio_timing_t *timing, audio_buffer_t *buffer,
                         const audio_stream_t *stream, audio_stats_t *stats,
                         int16_t *out, size_t samples) {
  if (!timing || !buffer || !stream || !out || samples == 0) {
    return 0;
  }

  if (!timing->playing) {
    return 0;
  }

  const audio_format_t *format = &stream->format;

  // Wait for some buffer before starting
  if (!timing->playout_started) {
    int buffered_frames = audio_buffer_get_frame_count(buffer);
    if (buffered_frames < (int)timing->target_buffer_frames) {
      return 0;
    }
  }

  // Get frame from buffer
  size_t item_size = 0;
  void *item = NULL;
  if (!audio_buffer_take(buffer, &item, &item_size, pdMS_TO_TICKS(50))) {
    if (stats) {
      stats->buffer_underruns++;
    }
    return 0;
  }

  if (item_size < sizeof(audio_frame_header_t)) {
    audio_buffer_return(buffer, item);
    return 0;
  }

  audio_frame_header_t *hdr = (audio_frame_header_t *)item;
  size_t frame_samples = hdr->samples_per_channel;
  size_t channels = hdr->channels ? hdr->channels : format->channels;
  int16_t *pcm = (int16_t *)(hdr + 1);

  if (frame_samples == 0 || channels == 0) {
    audio_buffer_return(buffer, item);
    return 0;
  }

  if (frame_samples > samples) {
    frame_samples = samples;
  }

  // Copy PCM data to output
  memcpy(out, pcm, frame_samples * channels * sizeof(int16_t));
  audio_buffer_return(buffer, item);

  timing->playout_started = true;
  return frame_samples;
}
