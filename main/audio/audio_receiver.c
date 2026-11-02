#include "audio_receiver.h"

#include <math.h>
#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "audio_buffer.h"
#include "audio_decoder.h"
#include "audio_receiver_internal.h"
#include "audio_stream.h"
#include "config.h"

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"

#define DEFAULT_SAMPLE_RATE     44100
#define DEFAULT_CHANNELS        2
#define DEFAULT_BITS_PER_SAMPLE 16
#define DEFAULT_FRAME_SIZE      352
#define DECRYPT_BUFFER_SIZE     65536 // Largest buffered packet (16-bit length prefix)

// After a pause/stop flush, how long packets continuing the old stream are dropped.
// Realtime packets in flight arrive within ms, a TCP backlog takes longer to drain.
#define CONTINUATION_FILTER_REALTIME_MS 250
#define CONTINUATION_FILTER_BUFFERED_MS 2000
// After a flush with a known restart point: give up waiting for it after this long
#define UNTIL_FILTER_MAX_MS 5000

// A deferred flush (flushFromTS) closer than this to what is playing is done right away
#define DEFERRED_FLUSH_MIN_MS 1000

// Drift correction (realtime only). The phone sends at its clock rate and the I2S clock
// differs slightly, so the buffer slowly drains or fills. Keep it at the target level by
// dropping or inserting one sample per frame when the averaged level wanders off.
#define DRIFT_AVERAGE_FACTOR 0.002f // ~4 s time constant at one read per ~8 ms
#define DRIFT_DEADBAND_MS    20
#define DRIFT_FAST_MS        100 // Beyond this, correct every frame (~2800 ppm)
#define DRIFT_SLOW_INTERVAL  4   // Otherwise every 4th frame (~700 ppm)

static const char* TAG = "audio_recv";

static atomic_uint audio_generation = 0;

static audio_receiver_state_t receiver = {0};

// Playout state, only touched by the playback task (audio_receiver_read)
static struct {
  uint32_t generation;
  bool     started;
  float    depth_average; // Samples
  int      frames_since_correction;
} playout;

// Last timestamp handed to the output, for deciding how to handle a deferred flush
static atomic_uint playing_timestamp = 0;
static atomic_bool playing_valid = false;

// Playback ran out of audio while playing (stats)
static atomic_uint stall_count = 0;

// Deferred flush window: frames in [from, until) are skipped by the reader
static portMUX_TYPE discard_lock = portMUX_INITIALIZER_UNLOCKED;
static struct {
  bool     active;
  uint32_t from_timestamp;
  uint32_t until_timestamp;
} discard;

static void audio_receiver_reset_stats(void) { memset(&receiver.stats, 0, sizeof(receiver.stats)); }

uint32_t audio_receiver_get_generation(void) { return atomic_load(&audio_generation); }

static void audio_receiver_reset_blocks(void) { receiver.blocks_read = 0; }

static int audio_receiver_sample_rate(void) {
  if (receiver.stream && receiver.stream->format.sample_rate > 0) { return receiver.stream->format.sample_rate; }
  return DEFAULT_SAMPLE_RATE;
}

static int ms_to_samples(int ms) { return (int)((int64_t)audio_receiver_sample_rate() * ms / 1000); }

static bool audio_receiver_is_realtime(void) { return !receiver.stream || receiver.stream->type != AUDIO_STREAM_BUFFERED; }

static void set_flush_filter(const flush_filter_t* filter) {
  portENTER_CRITICAL(&receiver.flush_lock);
  receiver.flush_filter = *filter;
  portEXIT_CRITICAL(&receiver.flush_lock);
}

static void set_discard_window(bool active, uint32_t from_timestamp, uint32_t until_timestamp) {
  portENTER_CRITICAL(&discard_lock);
  discard.active = active;
  discard.from_timestamp = from_timestamp;
  discard.until_timestamp = until_timestamp;
  portEXIT_CRITICAL(&discard_lock);
}

// Drop everything decoded so far. Bumping the generation also invalidates frames that
// stream tasks are still decoding, and makes the reader wait for the start buffer again.
static void flush_all(const flush_filter_t* filter) {
  set_flush_filter(filter);
  set_discard_window(false, 0, 0);
  atomic_fetch_add(&audio_generation, 1);
  atomic_store(&playing_valid, false);
  audio_buffer_flush(&receiver.buffer);
}

static void audio_receiver_reset_flush(void) {
  flush_filter_t none = {0};
  flush_all(&none);
}

static void audio_receiver_copy_stream_state(audio_stream_t* dst, const audio_stream_t* src) {
  if (!dst || !src) { return; }

  dst->format = src->format;
  dst->encrypt = src->encrypt;
}

esp_err_t audio_receiver_init(void) {
  if (receiver.buffer.ring) { return ESP_OK; }

  receiver.realtime_stream = audio_stream_create_realtime();
  if (receiver.realtime_stream) { receiver.realtime_stream->ctx = &receiver; }
  receiver.buffered_stream = audio_stream_create_buffered();
  if (receiver.buffered_stream) { receiver.buffered_stream->ctx = &receiver; }
  if (!receiver.realtime_stream || !receiver.buffered_stream) {
    ESP_LOGE(TAG, "Failed to allocate audio streams");
    audio_stream_destroy(receiver.realtime_stream);
    audio_stream_destroy(receiver.buffered_stream);
    receiver.realtime_stream = NULL;
    receiver.buffered_stream = NULL;
    return ESP_ERR_NO_MEM;
  }

  receiver.stream = receiver.realtime_stream;

  audio_format_t default_format = {0};
  strcpy(default_format.codec, "AppleLossless");
  default_format.sample_rate = DEFAULT_SAMPLE_RATE;
  default_format.channels = DEFAULT_CHANNELS;
  default_format.bits_per_sample = DEFAULT_BITS_PER_SAMPLE;
  default_format.frame_size = DEFAULT_FRAME_SIZE;

  receiver.realtime_stream->format = default_format;
  receiver.buffered_stream->format = default_format;

  esp_err_t err = audio_buffer_init(&receiver.buffer);
  if (err != ESP_OK) {
    audio_stream_destroy(receiver.realtime_stream);
    audio_stream_destroy(receiver.buffered_stream);
    receiver.realtime_stream = NULL;
    receiver.buffered_stream = NULL;
    return err;
  }

  receiver.decrypt_buffer_size = DECRYPT_BUFFER_SIZE;
  receiver.decrypt_buffer = heap_caps_malloc(receiver.decrypt_buffer_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!receiver.decrypt_buffer) { receiver.decrypt_buffer = malloc(receiver.decrypt_buffer_size); }
  if (!receiver.decrypt_buffer) {
    ESP_LOGE(TAG, "Failed to allocate decrypt buffer");
    audio_buffer_deinit(&receiver.buffer);
    audio_stream_destroy(receiver.realtime_stream);
    audio_stream_destroy(receiver.buffered_stream);
    receiver.realtime_stream = NULL;
    receiver.buffered_stream = NULL;
    return ESP_ERR_NO_MEM;
  }

  portMUX_TYPE flush_lock = portMUX_INITIALIZER_UNLOCKED;
  receiver.flush_lock = flush_lock;

  receiver.buffered_listen_socket = -1;
  receiver.buffered_client_socket = -1;
  receiver.control_socket = -1;

  audio_receiver_reset_blocks();

  return ESP_OK;
}

static bool same_format(const audio_format_t* a, const audio_format_t* b) {
  return strcmp(a->codec, b->codec) == 0 && a->sample_rate == b->sample_rate && a->channels == b->channels &&
         a->bits_per_sample == b->bits_per_sample && a->frame_size == b->frame_size &&
         a->max_samples_per_frame == b->max_samples_per_frame && a->sample_size == b->sample_size &&
         a->rice_history_mult == b->rice_history_mult && a->rice_initial_history == b->rice_initial_history &&
         a->rice_limit == b->rice_limit && a->num_channels == b->num_channels && a->max_run == b->max_run &&
         a->max_coded_frame_size == b->max_coded_frame_size && a->sample_rate_config == b->sample_rate_config;
}

void audio_receiver_set_format(const audio_format_t* format) {
  if (!format) { return; }
  if (!receiver.realtime_stream || !receiver.buffered_stream) { return; }

  // A new SETUP for the next song often repeats the format: keep the decoder
  if (receiver.decoder && same_format(format, &receiver.stream->format)) { return; }

  // The stream tasks decode with receiver.decoder, so they must be stopped before it
  // is replaced (the SETUP that changes the format starts the stream again)
  if (receiver.realtime_stream->running) { receiver.realtime_stream->ops->stop(receiver.realtime_stream); }
  if (receiver.buffered_stream->running) { receiver.buffered_stream->ops->stop(receiver.buffered_stream); }

  receiver.realtime_stream->format = *format;
  receiver.buffered_stream->format = *format;

  audio_decoder_destroy(receiver.decoder);
  receiver.decoder = NULL;

  audio_decoder_config_t cfg = {.format = *format};
  receiver.decoder = audio_decoder_create(&cfg);
  if (!receiver.decoder) { ESP_LOGW(TAG, "Decoder not initialized for codec: %s", format->codec); }
}

void audio_receiver_set_encryption(const audio_encrypt_t* encrypt) {
  if (!receiver.realtime_stream || !receiver.buffered_stream) { return; }
  if (encrypt) {
    receiver.realtime_stream->encrypt = *encrypt;
    receiver.buffered_stream->encrypt = *encrypt;
  } else {
    memset(&receiver.realtime_stream->encrypt, 0, sizeof(receiver.realtime_stream->encrypt));
    memset(&receiver.buffered_stream->encrypt, 0, sizeof(receiver.buffered_stream->encrypt));
  }
}

void audio_receiver_set_client_addr(uint32_t client_ip, uint16_t client_control_port) {
  receiver.client_ip = client_ip;
  receiver.client_control_port = client_control_port;
}

void audio_receiver_set_stream_type(audio_stream_type_t type) {
  if (!receiver.realtime_stream || !receiver.buffered_stream) { return; }
  audio_stream_t* target = receiver.realtime_stream;
  if (type == AUDIO_STREAM_BUFFERED) { target = receiver.buffered_stream; }

  if (!target) { return; }

  if (receiver.stream != target) {
    if (receiver.stream) {
      audio_receiver_copy_stream_state(target, receiver.stream);
      if (receiver.stream->running && receiver.stream->ops && receiver.stream->ops->stop) { receiver.stream->ops->stop(receiver.stream); }
    }
    receiver.stream = target;
  }

  receiver.stream->type = type;
}

esp_err_t audio_receiver_start(uint16_t data_port, uint16_t control_port) {
  audio_receiver_set_stream_type(AUDIO_STREAM_REALTIME);

  if (!receiver.stream || !receiver.stream->ops || !receiver.stream->ops->start) { return ESP_FAIL; }

  // Always stop and restart fresh
  if (receiver.stream->running) { receiver.stream->ops->stop(receiver.stream); }

  receiver.data_port = data_port;
  receiver.control_port = control_port;

  audio_receiver_reset_stats();
  audio_receiver_reset_flush();
  audio_receiver_reset_blocks();

  return receiver.stream->ops->start(receiver.stream, data_port);
}

esp_err_t audio_receiver_start_buffered(uint16_t tcp_port) {
  audio_receiver_set_stream_type(AUDIO_STREAM_BUFFERED);

  if (!receiver.stream || !receiver.stream->ops || !receiver.stream->ops->start) { return ESP_FAIL; }

  // Always reset state for new session, even if stream is already running
  audio_receiver_reset_stats();
  audio_receiver_reset_flush();
  audio_receiver_reset_blocks();

  // Buffered streams use a fixed port, no need to restart if already running
  if (receiver.stream->running) {
    ESP_LOGI(TAG, "Buffered audio stream ready for new session");
    return ESP_OK;
  }

  return receiver.stream->ops->start(receiver.stream, tcp_port);
}

esp_err_t audio_receiver_start_stream(uint16_t data_port, uint16_t control_port, uint16_t tcp_port) {
  if (!receiver.stream) { return ESP_FAIL; }
  if (receiver.stream->type == AUDIO_STREAM_BUFFERED) { return audio_receiver_start_buffered(tcp_port); }

  return audio_receiver_start(data_port, control_port);
}

uint16_t audio_receiver_get_stream_port(void) {
  if (!receiver.stream || !receiver.stream->ops || !receiver.stream->ops->get_port) { return 0; }

  return receiver.stream->ops->get_port(receiver.stream);
}

void audio_receiver_stop(void) {
  if (receiver.realtime_stream && receiver.realtime_stream->ops && receiver.realtime_stream->ops->stop) {
    receiver.realtime_stream->ops->stop(receiver.realtime_stream);
  }

  if (receiver.buffered_stream && receiver.buffered_stream->ops && receiver.buffered_stream->ops->stop) {
    receiver.buffered_stream->ops->stop(receiver.buffered_stream);
  }

  audio_decoder_destroy(receiver.decoder);
  receiver.decoder = NULL;

  if (receiver.realtime_stream) { memset(&receiver.realtime_stream->encrypt, 0, sizeof(receiver.realtime_stream->encrypt)); }
  if (receiver.buffered_stream) { memset(&receiver.buffered_stream->encrypt, 0, sizeof(receiver.buffered_stream->encrypt)); }

  audio_receiver_flush();
}

void audio_receiver_stop_buffered_only(void) {
  if (receiver.buffered_stream && receiver.buffered_stream->ops && receiver.buffered_stream->ops->stop) {
    receiver.buffered_stream->ops->stop(receiver.buffered_stream);
  }
}

void audio_receiver_get_stats(audio_stats_t* stats) {
  if (!stats) { return; }
  memcpy(stats, &receiver.stats, sizeof(receiver.stats));
}

// Returns -1 to drop a sample from the next frame, +1 to insert one, 0 to leave it as is
static int drift_correction(int depth_samples) {
  playout.depth_average += ((float)depth_samples - playout.depth_average) * DRIFT_AVERAGE_FACTOR;
  playout.frames_since_correction++;

  float error_ms = (playout.depth_average - (float)ms_to_samples(CONFIG_REALTIME_BUFFER_MS)) * 1000.0f / (float)audio_receiver_sample_rate();
  float magnitude = fabsf(error_ms);
  if (magnitude < DRIFT_DEADBAND_MS) { return 0; }

  int interval = magnitude > DRIFT_FAST_MS ? 1 : DRIFT_SLOW_INTERVAL;
  if (playout.frames_since_correction < interval) { return 0; }

  playout.frames_since_correction = 0;
  return error_ms > 0 ? -1 : 1;
}

static void put_sample(int16_t* out, size_t index, const int16_t* pcm, size_t source, size_t channels) {
  out[index * 2] = pcm[source * channels];
  out[index * 2 + 1] = pcm[source * channels + (channels > 1 ? 1 : 0)];
}

static void put_average(int16_t* out, size_t index, const int16_t* pcm, size_t first, size_t second, size_t channels) {
  for (size_t c = 0; c < 2; c++) {
    size_t channel = channels > 1 ? c : 0;
    out[index * 2 + c] = (int16_t)(((int32_t)pcm[first * channels + channel] + pcm[second * channels + channel]) / 2);
  }
}

// Copy one frame to interleaved stereo, dropping or inserting one sample in the middle.
// The dropped/inserted sample is replaced by the average of its neighbours to stay smooth.
static size_t copy_frame(int16_t* out, const int16_t* pcm, size_t frames, size_t channels, int correction) {
  size_t middle = frames / 2;
  size_t written = 0;

  if (correction < 0 && frames >= 4) {
    for (size_t i = 0; i < middle - 1; i++) { put_sample(out, written++, pcm, i, channels); }
    put_average(out, written++, pcm, middle - 1, middle, channels);
    for (size_t i = middle + 1; i < frames; i++) { put_sample(out, written++, pcm, i, channels); }
  } else if (correction > 0 && frames >= 2) {
    for (size_t i = 0; i < middle; i++) { put_sample(out, written++, pcm, i, channels); }
    put_average(out, written++, pcm, middle - 1, middle, channels);
    for (size_t i = middle; i < frames; i++) { put_sample(out, written++, pcm, i, channels); }
  } else {
    for (size_t i = 0; i < frames; i++) { put_sample(out, written++, pcm, i, channels); }
  }

  return written;
}

// True when a frame falls in the window of a deferred flush
static bool in_discard_window(uint32_t timestamp) {
  bool skip = false;

  portENTER_CRITICAL(&discard_lock);
  if (discard.active) {
    if ((int32_t)(timestamp - discard.until_timestamp) >= 0) {
      // Reached the audio that replaces the flushed range
      discard.active = false;
    } else {
      skip = (int32_t)(timestamp - discard.from_timestamp) >= 0;
    }
  }
  portEXIT_CRITICAL(&discard_lock);

  return skip;
}

size_t audio_receiver_read(int16_t* buffer, size_t samples) {
  if (!receiver.buffer.ring || !receiver.stream || !buffer || samples < 2) { return 0; }

  uint32_t generation = audio_receiver_get_generation();
  if (generation != playout.generation) {
    // Flushed: build up the start buffer again before playing
    playout.generation = generation;
    playout.started = false;
  }

  bool realtime = audio_receiver_is_realtime();

  if (!playout.started) {
    atomic_store(&receiver.playout_running, false);
    int depth = audio_buffer_get_sample_count(&receiver.buffer);
    if (depth < ms_to_samples(realtime ? CONFIG_REALTIME_BUFFER_MS : CONFIG_BUFFERED_START_MS)) { return 0; }
    playout.started = true;
    playout.depth_average = (float)depth;
    playout.frames_since_correction = 0;
    atomic_store(&receiver.playout_running, true);
  }

  while (true) {
    size_t item_size = 0;
    void*  item = NULL;
    if (!audio_buffer_take(&receiver.buffer, &item, &item_size, 0)) {
      // Underrun: refill to the start level instead of playing every packet the moment
      // it arrives, which would keep the buffer empty and glitch until the next flush
      playout.started = false;
      atomic_store(&receiver.playout_running, false);
      atomic_fetch_add(&stall_count, 1);
      return 0;
    }

    size_t                result = 0;
    audio_frame_header_t* hdr = (audio_frame_header_t*)item;

    if (item_size >= sizeof(*hdr) && hdr->generation == generation && !in_discard_window(hdr->rtp_timestamp)) {
      size_t frames = hdr->samples_per_channel;
      size_t channels = hdr->channels ? hdr->channels : (size_t)receiver.stream->format.channels;
      // Keep room for one inserted sample
      if (frames > samples - 1) { frames = samples - 1; }

      if (frames > 0 && channels > 0) {
        int correction = realtime ? drift_correction(audio_buffer_get_sample_count(&receiver.buffer)) : 0;
        result = copy_frame(buffer, (const int16_t*)(hdr + 1), frames, channels, correction);
        atomic_store(&playing_timestamp, hdr->rtp_timestamp);
        atomic_store(&playing_valid, true);
      }
    }

    audio_buffer_return(&receiver.buffer, item);
    if (result > 0) { return result; }
    // Stale or flushed frame: skip it without outputting silence
  }
}

bool audio_receiver_has_data(void) {
  int buffered_frames = audio_buffer_get_frame_count(&receiver.buffer);
  return buffered_frames > 0;
}

void audio_receiver_flush(void) {
  flush_filter_t filter = {0};
  if (receiver.stats.packets_received > 0) {
    int filter_ms = audio_receiver_is_realtime() ? CONTINUATION_FILTER_REALTIME_MS : CONTINUATION_FILTER_BUFFERED_MS;
    filter.mode = FLUSH_FILTER_CONTINUATION;
    filter.last_timestamp = receiver.stats.last_timestamp;
    filter.expires_us = esp_timer_get_time() + (int64_t)filter_ms * 1000;
  }
  flush_all(&filter);
}

void audio_receiver_flush_until(uint32_t until_timestamp) {
  flush_filter_t filter = {
    .mode = FLUSH_FILTER_UNTIL,
    .until_timestamp = until_timestamp,
    .expires_us = esp_timer_get_time() + (int64_t)UNTIL_FILTER_MAX_MS * 1000,
  };
  flush_all(&filter);
}

void audio_receiver_flush_buffered(const audio_flush_request_t* request) {
  flush_filter_t filter = {
    .mode = FLUSH_FILTER_UNTIL,
    .use_sequence = true,
    .until_sequence = request->until_sequence,
    .until_timestamp = request->until_timestamp,
    .expires_us = esp_timer_get_time() + (int64_t)UNTIL_FILTER_MAX_MS * 1000,
  };

  bool deferred = false;
  if (request->has_from && (int32_t)(request->until_timestamp - request->from_timestamp) > 0 && atomic_load(&playing_valid)) {
    int32_t ahead = (int32_t)(request->from_timestamp - (uint32_t)atomic_load(&playing_timestamp));
    deferred = ahead > ms_to_samples(DEFERRED_FLUSH_MIN_MS);
  }

  if (!deferred) {
    // Playback is at or near the cut already (skip): cut now instead of playing on
    flush_all(&filter);
    return;
  }

  // Keep playing up to from, drop the range both on receive and on playback
  filter.has_from = true;
  filter.from_sequence = request->from_sequence;
  filter.from_timestamp = request->from_timestamp;
  set_flush_filter(&filter);
  set_discard_window(true, request->from_timestamp, request->until_timestamp);
}

uint16_t audio_receiver_get_buffered_port(void) { return receiver.buffered_port; }

int audio_receiver_get_buffered_frames(void) { return audio_buffer_get_frame_count(&receiver.buffer); }

int audio_receiver_get_sample_rate(void) { return audio_receiver_sample_rate(); }

uint32_t audio_receiver_get_stalls(void) { return atomic_load(&stall_count); }

int audio_receiver_get_buffered_ms(void) {
  return (int)((int64_t)audio_buffer_get_sample_count(&receiver.buffer) * 1000 / audio_receiver_sample_rate());
}
