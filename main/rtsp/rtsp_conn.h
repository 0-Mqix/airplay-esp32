#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "hap.h"

/**
 * RTSP Connection State Management
 * Consolidates all session state for an AirPlay connection
 */

// Forward declaration
typedef struct rtsp_conn rtsp_conn_t;

/**
 * Connection state struct - consolidates all session state
 */
struct rtsp_conn {
  // HAP session for pairing/encryption
  hap_session_t* hap_session;
  bool           encrypted_mode;

  // Audio streaming state
  bool     stream_active;
  bool     stream_paused;
  int64_t  stream_type;   // 96=UDP realtime, 103=TCP buffered
  uint16_t data_port;     // UDP port for audio data (type 96)
  uint16_t control_port;  // UDP port for control (retransmit requests)
  uint16_t timing_port;   // UDP port for timing (our local port)
  uint16_t event_port;    // TCP port for server->client events
  uint16_t buffered_port; // TCP port for buffered audio (type 103)
  int      data_socket;
  int      control_socket;
  int      event_socket; // TCP listener for event port

  // Client address for AirPlay 1 timing requests
  uint32_t client_ip;           // Client IP (network byte order)
  uint16_t client_timing_port;  // Client's timing port (for sending requests)
  uint16_t client_control_port; // Client's control port

  // Codec info from ANNOUNCE/SETUP
  char codec[32];
  int  sample_rate;
  int  channels;
  int  bits_per_sample;
};

/**
 * Create a new connection state
 * @return Allocated connection state, or NULL on failure
 */
rtsp_conn_t* rtsp_conn_create(void);

/**
 * Free connection state and associated resources
 */
void rtsp_conn_free(rtsp_conn_t* conn);

/**
 * Reset stream-related state (called on stream teardown)
 * Keeps session alive but clears audio stream state
 */
void rtsp_conn_reset_stream(rtsp_conn_t* conn);

/**
 * Full cleanup when connection closes
 * Stops audio, closes sockets, clears PTP
 */
void rtsp_conn_cleanup(rtsp_conn_t* conn);

