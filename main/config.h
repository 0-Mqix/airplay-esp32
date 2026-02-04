#pragma once

// Device name shown in AirPlay
#define CONFIG_DEVICE_NAME "Volvo"

// WiFi password to try on all discovered networks
#define CONFIG_WIFI_PASSWORD "tweaker!"

// I2S pins for DAC (UM S3 Pro)
#define CONFIG_I2S_BCK_PIN  14
#define CONFIG_I2S_DATA_PIN 13
#define CONFIG_I2S_LRCK_PIN 12

// WiFi AP settings
#define CONFIG_WIFI_CHANNEL  1  // Channel 1, 6, or 11 for least overlap
#define CONFIG_WIFI_MAX_CONN 2  // Fewer clients = more bandwidth per client
#define CONFIG_WIFI_TX_POWER 52 // 13 dBm (52 quarter-dBm) - safe for all modules

// Maximum volume in dB (0 = full volume, -6 = half power, -12 = quarter power)
// Use this to protect speakers from being too loud
#define CONFIG_MAX_VOLUME_DB -10

// =============================================================================
// Task Configuration
// =============================================================================
// Core 0: Network/WiFi tasks
// Core 1: Audio playback (isolated for glitch-free audio)
// =============================================================================

// Audio playback - highest priority, isolated on Core 1
#define TASK_PLAYBACK_CORE     1
#define TASK_PLAYBACK_PRIORITY (configMAX_PRIORITIES - 1)
#define TASK_PLAYBACK_STACK    8192

// Audio receive (realtime UDP) - high priority on Core 0
#define TASK_AUDIO_RECV_CORE     0
#define TASK_AUDIO_RECV_PRIORITY 19
#define TASK_AUDIO_RECV_STACK    12288

// Control receive (realtime) - just below audio recv
#define TASK_CTRL_RECV_CORE     0
#define TASK_CTRL_RECV_PRIORITY 18
#define TASK_CTRL_RECV_STACK    4096

// Buffered audio (TCP streaming)
#define TASK_BUFFERED_CORE     0
#define TASK_BUFFERED_PRIORITY 17
#define TASK_BUFFERED_STACK    4096

// RTSP server
#define TASK_RTSP_SERVER_CORE     0
#define TASK_RTSP_SERVER_PRIORITY 5
#define TASK_RTSP_SERVER_STACK    4096

// RTSP client handler
#define TASK_RTSP_CLIENT_CORE     0
#define TASK_RTSP_CLIENT_PRIORITY 5
#define TASK_RTSP_CLIENT_STACK    8192

// RTSP event port
#define TASK_EVENT_PORT_CORE     0
#define TASK_EVENT_PORT_PRIORITY 5
#define TASK_EVENT_PORT_STACK    3072
