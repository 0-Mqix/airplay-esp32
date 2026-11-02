#pragma once

#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "audio_buffer.h"
#include "audio_decoder.h"
#include "audio_receiver.h"
#include "audio_stream.h"

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"

#define MAX_RTP_PACKET_SIZE 2048

// What a stream task drops after a flush. A flush only clears what was already
// received; packets still in flight (socket buffers, TCP backlog) must be filtered.
typedef enum {
  FLUSH_FILTER_NONE = 0,
  FLUSH_FILTER_UNTIL,        // Drop packets before the point the sender says new audio starts
  FLUSH_FILTER_CONTINUATION, // Drop packets that continue the flushed stream, until expires_us
} flush_filter_mode_t;

typedef struct {
  flush_filter_mode_t mode;
  // UNTIL: compare 24-bit sequence numbers (buffered) instead of timestamps (realtime).
  // Resent audio may reuse timestamps of stale packets, sequence numbers only go up.
  bool     use_sequence;
  bool     has_from; // UNTIL: packets before from are kept
  uint32_t from_sequence;
  uint32_t until_sequence;
  uint32_t from_timestamp;
  uint32_t until_timestamp;
  uint32_t last_timestamp; // CONTINUATION: last timestamp of the flushed stream
  int64_t  expires_us;     // Stop filtering after this time
} flush_filter_t;

typedef struct {
  audio_stream_t* stream;
  audio_stream_t* realtime_stream;
  audio_stream_t* buffered_stream;

  audio_decoder_t* decoder;
  audio_buffer_t   buffer;

  audio_stats_t stats;

  int             data_socket;
  TaskHandle_t    task_handle;
  RingbufHandle_t decode_ring;
  TaskHandle_t    decode_task_handle;
  uint16_t     data_port;
  uint16_t     control_port;
  int          control_socket;
  uint32_t     client_ip;
  uint16_t     client_control_port;
  uint16_t     retransmit_seq;

  int          buffered_listen_socket;
  int          buffered_client_socket;
  uint16_t     buffered_port;
  TaskHandle_t buffered_task_handle;
  uint8_t*     buffered_recv_buffer;

  uint8_t* decrypt_buffer;
  size_t   decrypt_buffer_size;

  uint64_t blocks_read;

  // Set by the reader while audio plays out (not while building the start buffer)
  atomic_bool playout_running;

  portMUX_TYPE   flush_lock;
  flush_filter_t flush_filter;
} audio_receiver_state_t;

bool audio_stream_process_frame(
  audio_receiver_state_t* state, uint32_t generation, uint32_t timestamp, const uint8_t* audio_data, size_t audio_len);

/**
 * Check a freshly received packet against the flush filter.
 * @param sequence 24-bit sequence number (only used for buffered flushes)
 * @return true when the packet belongs to flushed audio and must be dropped
 */
bool audio_stream_drop_flushed(audio_receiver_state_t* state, uint32_t sequence, uint32_t timestamp);

static inline audio_receiver_state_t* audio_stream_state(audio_stream_t* stream) { return (audio_receiver_state_t*)stream->ctx; }
