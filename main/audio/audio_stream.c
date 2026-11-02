#include "audio_stream.h"

#include <stdlib.h>
#include <string.h>

#include "audio_buffer.h"
#include "audio_decoder.h"
#include "audio_receiver_internal.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char* TAG = "audio_stream";

extern const audio_stream_ops_t audio_stream_realtime_ops;
extern const audio_stream_ops_t audio_stream_buffered_ops;

// Largest timestamp step between two packets of one stream (AAC: 1024, ALAC: 352)
#define FLUSH_MAX_PACKET_SAMPLES 4096
// How far behind the newest one a stale packet can arrive (reordering, retransmits)
#define FLUSH_MAX_REORDER_SAMPLES 22050

// Signed distance between two 24-bit sequence numbers
static int32_t sequence_delta(uint32_t a, uint32_t b) { return (int32_t)((a - b) << 8) >> 8; }

bool audio_stream_drop_flushed(audio_receiver_state_t* state, uint32_t sequence, uint32_t timestamp) {
  bool drop = false;

  portENTER_CRITICAL(&state->flush_lock);
  flush_filter_t* filter = &state->flush_filter;

  if (filter->mode == FLUSH_FILTER_UNTIL) {
    int32_t until_delta = 0;
    int32_t from_delta = 0;
    if (filter->use_sequence) {
      until_delta = sequence_delta(sequence, filter->until_sequence);
      from_delta = sequence_delta(sequence, filter->from_sequence);
    } else {
      until_delta = (int32_t)(timestamp - filter->until_timestamp);
      from_delta = (int32_t)(timestamp - filter->from_timestamp);
    }

    // Stays active after the new audio starts: a stale packet can still arrive behind
    // it. The timestamp distance says nothing (a skip can jump any amount), so the
    // filter ends by time.
    if (esp_timer_get_time() >= filter->expires_us) {
      filter->mode = FLUSH_FILTER_NONE;
    } else {
      drop = until_delta < 0 && !(filter->has_from && from_delta < 0);
    }
  } else if (filter->mode == FLUSH_FILTER_CONTINUATION) {
    int32_t delta = (int32_t)(timestamp - filter->last_timestamp);
    if (esp_timer_get_time() < filter->expires_us && delta > -FLUSH_MAX_REORDER_SAMPLES && delta <= FLUSH_MAX_PACKET_SAMPLES) {
      if (delta > 0) { filter->last_timestamp = timestamp; }
      drop = true;
    } else {
      // Timestamp jumped (new track, seek, resume) or the backlog is gone
      filter->mode = FLUSH_FILTER_NONE;
    }
  }

  portEXIT_CRITICAL(&state->flush_lock);
  return drop;
}

bool audio_stream_process_frame(
  audio_receiver_state_t* state, uint32_t generation, uint32_t timestamp, const uint8_t* audio_data, size_t audio_len) {
  if (!state || !state->decoder) { return false; }

  size_t   capacity_samples = 0;
  int16_t* decode_buffer = audio_buffer_get_decode_buffer(&state->buffer, &capacity_samples);
  if (!decode_buffer || capacity_samples == 0) { return false; }

  audio_decode_info_t info = {0};
  int                 decoded_samples = audio_decoder_decode(state->decoder, audio_data, audio_len, decode_buffer, capacity_samples, &info);
  if (decoded_samples <= 0) { return false; }

  int channels = info.channels > 0 ? info.channels : state->stream->format.channels;
  if (channels <= 0) { channels = 2; }

  return audio_buffer_queue_decoded(&state->buffer, &state->stats, generation, timestamp, decode_buffer, (size_t)decoded_samples, channels);
}

audio_stream_t* audio_stream_create_realtime(void) {
  audio_stream_t* stream = calloc(1, sizeof(*stream));
  if (!stream) { return NULL; }

  stream->ops = &audio_stream_realtime_ops;
  stream->type = AUDIO_STREAM_REALTIME;
  return stream;
}

audio_stream_t* audio_stream_create_buffered(void) {
  audio_stream_t* stream = calloc(1, sizeof(*stream));
  if (!stream) { return NULL; }

  stream->ops = &audio_stream_buffered_ops;
  stream->type = AUDIO_STREAM_BUFFERED;
  return stream;
}

bool audio_stream_uses_buffer(audio_stream_type_t type) { return type == AUDIO_STREAM_BUFFERED; }

void audio_stream_destroy(audio_stream_t* stream) {
  if (!stream) { return; }

  if (stream->ops && stream->ops->destroy) {
    stream->ops->destroy(stream);
    return;
  }

  free(stream);
}

