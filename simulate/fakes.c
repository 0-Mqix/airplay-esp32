// Stand-ins for the firmware parts the simulator does not exercise: the codec, the
// crypto and the sockets. The fake codec makes every output sample say which source
// sample it is, so the simulator can check exactly what reached the speaker.

#include <stdlib.h>
#include <string.h>

#include "audio_crypto.h"
#include "audio_decoder.h"
#include "network/socket_utils.h"
#include "simulate.h"

struct audio_decoder {
  int unused;
};

audio_decoder_t* audio_decoder_create(const audio_decoder_config_t* config) {
  (void)config;
  return calloc(1, sizeof(audio_decoder_t));
}

void audio_decoder_destroy(audio_decoder_t* decoder) { free(decoder); }

bool audio_decoder_is_aac(const audio_decoder_t* decoder) {
  (void)decoder;
  return false;
}

bool audio_decoder_is_alac(const audio_decoder_t* decoder) {
  (void)decoder;
  return true;
}

void sim_payload_write(uint8_t* payload, uint32_t timestamp, int track, int samples) {
  payload[0] = (uint8_t)(timestamp >> 24);
  payload[1] = (uint8_t)(timestamp >> 16);
  payload[2] = (uint8_t)(timestamp >> 8);
  payload[3] = (uint8_t)timestamp;
  payload[4] = (uint8_t)track;
  payload[5] = (uint8_t)(samples >> 8);
  payload[6] = (uint8_t)samples;
}

int16_t sim_sample_left(uint32_t source) { return (int16_t)(source & 0xFFFF); }

int16_t sim_sample_right(uint32_t source, int track) { return (int16_t)(((track & 7) << 12) | ((source >> 16) & 0xFFF)); }

int audio_decoder_decode(
  audio_decoder_t*     decoder,
  const uint8_t*       input,
  size_t               input_len,
  int16_t*             output,
  size_t               output_capacity_samples,
  audio_decode_info_t* info) {
  (void)decoder;
  if (input_len < SIM_PAYLOAD_SIZE) { return -1; }

  uint32_t timestamp = ((uint32_t)input[0] << 24) | ((uint32_t)input[1] << 16) | ((uint32_t)input[2] << 8) | input[3];
  int      track = input[4];
  size_t   samples = ((size_t)input[5] << 8) | input[6];
  if (samples > output_capacity_samples) { return -1; }

  for (size_t i = 0; i < samples; i++) {
    uint32_t source = timestamp + (uint32_t)i;
    output[i * 2] = sim_sample_left(source);
    output[i * 2 + 1] = sim_sample_right(source, track);
  }

  if (info) { info->channels = 2; }
  return (int)samples;
}

// Unencrypted: the payload follows the 12 byte RTP header
int audio_crypto_decrypt_rtp(
  const audio_encrypt_t* encrypt,
  const uint8_t*         input,
  size_t                 input_len,
  uint8_t*               output,
  size_t                 output_capacity,
  const uint8_t*         full_packet,
  size_t                 full_packet_len) {
  (void)encrypt;
  (void)full_packet;
  (void)full_packet_len;
  if (input_len > output_capacity) { return -1; }
  memcpy(output, input, input_len);
  return (int)input_len;
}

int audio_crypto_decrypt_buffered(const audio_encrypt_t* encrypt, const uint8_t* packet, size_t packet_len, uint8_t* output, size_t output_capacity) {
  (void)encrypt;
  if (packet_len < 12 || packet_len - 12 > output_capacity) { return -1; }
  memcpy(output, packet + 12, packet_len - 12);
  return (int)(packet_len - 12);
}

// Sockets are never opened; the simulator delivers packets itself. The numbers are
// high so the close() calls in the stop paths cannot hit a real descriptor.
int socket_utils_bind_udp(uint16_t port, int recv_timeout_sec, int recvbuf_size, uint16_t* bound_port) {
  (void)recv_timeout_sec;
  (void)recvbuf_size;
  static int next_socket = 10000;
  if (bound_port) { *bound_port = port; }
  return next_socket++;
}

int socket_utils_bind_tcp_listener(uint16_t port, int backlog, bool nonblocking, uint16_t* bound_port) {
  (void)backlog;
  (void)nonblocking;
  static int next_socket = 20000;
  if (bound_port) { *bound_port = port; }
  return next_socket++;
}
