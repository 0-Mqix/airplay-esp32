// Host simulation of the audio pipeline. A fake phone sends audio over a simulated
// WiFi link into the real receiver code (reorder, flush filtering, ring buffer, drift
// correction) and a fake I2S output reads it at its own clock rate. Every output sample
// says which source sample it is, so gaps, stale audio after a skip and the phone to
// car delay can be measured exactly. Each scenario runs in its own process.

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>

#include "simulate.h"

static ssize_t
sim_sendto(int socket, const void* data, size_t length, int flags, const struct sockaddr* address, socklen_t address_length);
static ssize_t sim_recvfrom(int socket, void* buffer, size_t length, int flags, struct sockaddr* address, socklen_t* address_length);

// The firmware sources are compiled into this file so the simulator can drive their
// internal functions; the realtime stream's socket calls go to the simulated network
#define sendto   sim_sendto
#define recvfrom sim_recvfrom
#define TAG      TAG_receiver
#include "audio_receiver.c"
#undef TAG
#define TAG TAG_realtime
#include "audio_stream_realtime.c"
#undef TAG
#undef sendto
#undef recvfrom
#define TAG TAG_buffered
#include "audio_stream_buffered.c"
#undef TAG

int64_t sim_time_us = 0;
bool    sim_verbose = false;
int     sim_warnings = 0;

extern size_t sim_ring_leaked_bytes; // esp_shim.c
extern int    sim_ring_misuse;

#define SAMPLE_RATE   44100
#define ALAC_SAMPLES  352
#define AAC_SAMPLES   1024
#define READ_SAMPLES  512 // FRAME_SAMPLES in audio_output.c
#define OUTPUT_DMA_MS 186 // I2S DMA queue after the ring buffer (audio_output.c)
#define STEP_US       250
#define TRACK_JUMP    ((uint32_t)SAMPLE_RATE * 600) // Timestamp jump to the next track
#define SOURCE_MASK   0x0FFFFFFFu                   // Source bits the fake samples carry
#define SETTLE_US     3000000                       // Ignore delay/buffer right after a start
#define HISTORY_SIZE  4096
#define MAX_TRACKS    8 // Track ids 1..7 (3 bits in the fake samples)

// Realtime play/skip/resume to sound: the start buffer, the output DMA and some margin
#define REALTIME_START_MS (CONFIG_REALTIME_BUFFER_MS + OUTPUT_DMA_MS + 100)

typedef struct {
  const char* name;
  const char* description;
  bool        buffered;
  double      duration_s;
  double      sender_ppm; // Phone clock error
  double      output_ppm; // I2S clock error
  double      base_delay_ms;
  double      jitter_ms; // Uniform extra delay per packet
  double      loss;      // Realtime packet (and retransmit) loss probability
  double      stall_every_s;
  double      stall_ms; // WiFi stall: packets held back, then delivered in a burst
  double      outage_at_s;
  double      outage_ms;      // Packets lost completely
  double      outage_every_s; // Repeat the outage (phone radio away for AirDrop/Handoff, scans)
  double      skip_every_s;
  double      skip_until_s; // Stop skipping after this, 0 = never
  double      pause_every_s;
  double      pause_ms;
  double      up_next_at_s;   // Buffered: replace the next track while the current one plays
  double      track_length_s; // Buffered: length of the first track (up-next scenario)
  double      duplicate;      // Probability a packet arrives twice
  double      burst_loss;     // Probability a loss burst (10-40 packets) starts at a packet
  double      garbage;        // Probability a malformed packet is sent along with a packet
  double      restart_every_s;       // TEARDOWN, then a new session a second later
  double      format_change_every_s; // Buffered: new SETUP with another frame size while playing
  int         samples_per_packet;    // 0 = codec default (ALAC 352, AAC 1024)
  uint32_t    start_timestamp;       // 0 = default
  uint32_t    start_sequence;        // 0 = default
  // Pass limits
  int    max_gaps;
  double max_start_ms;        // From play/skip/resume to sound at the speaker
  double max_delay_spread_ms; // Realtime: drift of the phone to speaker delay, 0 = unchecked
} scenario_t;

static const scenario_t scenarios[] = {
  {.name = "realtime-steady",
   .description = "Spotify, 30 min, 20 ms jitter, 0.2% loss",
   .duration_s = 1800,
   .base_delay_ms = 3,
   .jitter_ms = 20,
   .loss = 0.002,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 60},
  {.name = "realtime-drift-fast",
   .description = "Spotify, 2 h, I2S clock +300 ppm, 30 ms jitter, 0.3% loss, 400 ms stall every 90 s",
   .duration_s = 7200,
   .output_ppm = 300,
   .base_delay_ms = 3,
   .jitter_ms = 30,
   .loss = 0.003,
   .stall_every_s = 90,
   .stall_ms = 400,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 80},
  {.name = "realtime-drift-slow",
   .description = "Spotify, 2 h, I2S clock -300 ppm, 30 ms jitter, 0.3% loss, 400 ms stall every 90 s",
   .duration_s = 7200,
   .output_ppm = -300,
   .base_delay_ms = 3,
   .jitter_ms = 30,
   .loss = 0.003,
   .stall_every_s = 90,
   .stall_ms = 400,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 80},
  {.name = "realtime-skip",
   .description = "Spotify, 10 min, skip every 15 s",
   .duration_s = 600,
   .base_delay_ms = 3,
   .jitter_ms = 20,
   .loss = 0.002,
   .skip_every_s = 15,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 60},
  {.name = "realtime-pause",
   .description = "Spotify, 10 min, 3 s pause every 30 s",
   .duration_s = 600,
   .base_delay_ms = 3,
   .jitter_ms = 20,
   .loss = 0.002,
   .pause_every_s = 30,
   .pause_ms = 3000,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 60},
  {.name = "realtime-radio-gaps",
   .description = "Spotify, 30 min, phone radio away for 0.7 s every 3 min (AirDrop/Handoff)",
   .duration_s = 1800,
   .base_delay_ms = 3,
   .jitter_ms = 20,
   .loss = 0.002,
   .outage_at_s = 60,
   .outage_ms = 700,
   .outage_every_s = 180,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 60},
  {.name = "realtime-outage",
   .description = "Spotify, 5 min, WiFi gone for 3 s",
   .duration_s = 300,
   .base_delay_ms = 3,
   .jitter_ms = 20,
   .loss = 0.002,
   .outage_at_s = 120,
   .outage_ms = 3000,
   .max_gaps = 1,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 60},
  {.name = "buffered-steady",
   .description = "Apple Music, 30 min, 800 ms stall every 60 s",
   .buffered = true,
   .duration_s = 1800,
   .output_ppm = 300,
   .base_delay_ms = 5,
   .jitter_ms = 10,
   .stall_every_s = 60,
   .stall_ms = 800,
   .max_start_ms = 700},
  {.name = "buffered-skip",
   .description = "Apple Music, 10 min, skip every 20 s, 150 ms of audio in flight",
   .buffered = true,
   .duration_s = 600,
   .base_delay_ms = 150,
   .jitter_ms = 20,
   .skip_every_s = 20,
   .max_start_ms = 800},
  {.name = "buffered-pause",
   .description = "Apple Music, 10 min, 2 s pause every 25 s, resent from the pause point",
   .buffered = true,
   .duration_s = 600,
   .base_delay_ms = 150,
   .jitter_ms = 20,
   .pause_every_s = 25,
   .pause_ms = 2000,
   .max_start_ms = 800},
  {.name = "buffered-up-next",
   .description = "Apple Music, next track replaced while the 20 s current track plays",
   .buffered = true,
   .duration_s = 60,
   .base_delay_ms = 5,
   .jitter_ms = 10,
   .up_next_at_s = 12,
   .track_length_s = 20,
   .max_start_ms = 700},
  {.name = "realtime-skip-spam",
   .description = "Spotify, next pressed every 0.4 s for 20 s, then plays",
   .duration_s = 40,
   .base_delay_ms = 3,
   .jitter_ms = 20,
   .loss = 0.002,
   .skip_every_s = 0.4,
   .skip_until_s = 20,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 60},
  {.name = "buffered-skip-spam",
   .description = "Apple Music, next pressed every 0.4 s for 20 s, then plays",
   .buffered = true,
   .duration_s = 40,
   .base_delay_ms = 150,
   .jitter_ms = 20,
   .skip_every_s = 0.4,
   .skip_until_s = 20,
   .max_start_ms = 800},
  {.name = "realtime-hostile",
   .description = "Spotify, 30 min, 80 ms jitter, 1% loss, loss bursts, duplicates, malformed packets",
   .duration_s = 1800,
   .output_ppm = 150,
   .base_delay_ms = 3,
   .jitter_ms = 80,
   .loss = 0.01,
   .burst_loss = 0.0005,
   .duplicate = 0.01,
   .garbage = 0.01,
   .max_gaps = 60,
   .max_start_ms = REALTIME_START_MS + 400, // A loss burst at the start delays it
   .max_delay_spread_ms = 150},
  {.name = "buffered-hostile",
   .description = "Apple Music, 30 min, malformed packets, 2 s stall every 2 min",
   .buffered = true,
   .duration_s = 1800,
   .base_delay_ms = 20,
   .jitter_ms = 40,
   .garbage = 0.01,
   .stall_every_s = 120,
   .stall_ms = 2000,
   .max_start_ms = 800},
  {.name = "realtime-chaos",
   .description = "Spotify, 20 min, skips, pauses and session restarts mixed, lossy link",
   .duration_s = 1200,
   .output_ppm = -200,
   .base_delay_ms = 3,
   .jitter_ms = 40,
   .loss = 0.005,
   .duplicate = 0.005,
   .garbage = 0.005,
   .skip_every_s = 11,
   .pause_every_s = 17,
   .pause_ms = 1500,
   .restart_every_s = 53,
   .max_gaps = 5,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 120},
  {.name = "buffered-chaos",
   .description = "Apple Music, 20 min, skips, pauses, restarts and format changes mixed",
   .buffered = true,
   .duration_s = 1200,
   .base_delay_ms = 100,
   .jitter_ms = 40,
   .garbage = 0.005,
   .skip_every_s = 11,
   .pause_every_s = 17,
   .pause_ms = 1500,
   .restart_every_s = 53,
   .format_change_every_s = 23,
   .max_start_ms = 800},
  {.name = "buffered-lossless",
   .description = "Apple Music lossless-size packets (4096 samples), 10 min, skips",
   .buffered = true,
   .duration_s = 600,
   .base_delay_ms = 20,
   .jitter_ms = 20,
   .skip_every_s = 30,
   .samples_per_packet = 4096,
   .max_start_ms = 800},
  {.name = "realtime-wrap",
   .description = "Spotify, timestamps wrap past 2^32 and sequence numbers past 2^16, with skips",
   .duration_s = 300,
   .base_delay_ms = 3,
   .jitter_ms = 20,
   .loss = 0.002,
   .skip_every_s = 70,
   .start_timestamp = 0xFFFFFFFFu - SAMPLE_RATE * 20,
   .start_sequence = 0xFF00,
   .max_start_ms = REALTIME_START_MS,
   .max_delay_spread_ms = 60},
  {.name = "buffered-wrap",
   .description = "Apple Music, timestamps wrap past 2^32 and sequence numbers past 2^24, with skips",
   .buffered = true,
   .duration_s = 300,
   .base_delay_ms = 50,
   .jitter_ms = 20,
   .skip_every_s = 20,
   .start_timestamp = 0xFFFFFFFFu - SAMPLE_RATE * 30,
   .start_sequence = 0xFFFF00,
   .max_start_ms = 800},
};

#define SCENARIO_COUNT (sizeof(scenarios) / sizeof(scenarios[0]))

static const scenario_t* scenario;

static double random_unit(void) { return (double)rand() / ((double)RAND_MAX + 1.0); }

static int next_track_id(int track) { return track % (MAX_TRACKS - 1) + 1; }

// ============================================================================
// Results
// ============================================================================

static struct {
  long   gaps;     // Audio stopped while the phone was playing
  double gap_ms;   // Total length of those gaps
  long   stale;    // Samples of flushed audio that reached the speaker
  long   jumps;    // Unexplained jumps in the source audio
  long   inserted; // Drift correction: samples inserted
  long   dropped;  // Drift correction: samples dropped
  long   concealed;        // Lost packets played as silence (samples)
  double audio_lost_ms;    // Audio skipped over after a gap (outage)
  double resume_lost_ms;   // Audio missing at the start of a resume
  double worst_start_ms;   // Longest play/skip/resume to sound
  double delay_min_ms;     // Phone send to speaker (realtime)
  double delay_max_ms;
  double buffer_min_ms;    // Ring buffer level during steady playback
  double buffer_max_ms;
  bool   up_next_checked;
  bool   up_next_ok;       // Current track played to its end, straight into the new next track
} results;

// ============================================================================
// Network
// ============================================================================

// GARBAGE goes to the data path, CONTROL_GARBAGE to the realtime control port
typedef enum { PACKET_DATA, PACKET_RETRANSMIT, PACKET_GARBAGE, PACKET_CONTROL_GARBAGE } packet_kind_t;

typedef struct {
  int64_t       time_us;
  uint64_t      order;
  packet_kind_t kind;
  uint16_t      length;
  uint8_t       data[32];
} packet_t;

static struct {
  packet_t* heap;
  size_t    count;
  size_t    capacity;
  uint64_t  order;
  int       data_in_flight;
  int       burst_remaining; // Packets still to lose in a loss burst
  int64_t   last_arrival_us; // TCP delivers in order
  uint8_t   pending[64];     // Packet handed to recvfrom
  size_t    pending_length;
} network;

static bool packet_before(const packet_t* a, const packet_t* b) {
  return a->time_us < b->time_us || (a->time_us == b->time_us && a->order < b->order);
}

static void network_push(const packet_t* packet) {
  if (network.count == network.capacity) {
    network.capacity = network.capacity ? network.capacity * 2 : 1024;
    network.heap = realloc(network.heap, network.capacity * sizeof(packet_t));
  }
  size_t i = network.count++;
  network.heap[i] = *packet;
  network.heap[i].order = network.order++;
  while (i > 0) {
    size_t parent = (i - 1) / 2;
    if (!packet_before(&network.heap[i], &network.heap[parent])) { break; }
    packet_t swap = network.heap[i];
    network.heap[i] = network.heap[parent];
    network.heap[parent] = swap;
    i = parent;
  }
  if (packet->kind == PACKET_DATA) { network.data_in_flight++; }
}

static packet_t network_pop(void) {
  packet_t top = network.heap[0];
  network.heap[0] = network.heap[--network.count];
  size_t i = 0;
  while (true) {
    size_t left = i * 2 + 1;
    size_t right = left + 1;
    size_t smallest = i;
    if (left < network.count && packet_before(&network.heap[left], &network.heap[smallest])) { smallest = left; }
    if (right < network.count && packet_before(&network.heap[right], &network.heap[smallest])) { smallest = right; }
    if (smallest == i) { break; }
    packet_t swap = network.heap[i];
    network.heap[i] = network.heap[smallest];
    network.heap[smallest] = swap;
    i = smallest;
  }
  if (top.kind == PACKET_DATA) { network.data_in_flight--; }
  return top;
}

static bool in_outage(int64_t time_us) {
  double start_us = scenario->outage_at_s * 1e6;
  if (scenario->outage_ms <= 0 || time_us < start_us) { return false; }
  double since_us = (double)time_us - start_us;
  if (scenario->outage_every_s > 0) { since_us = fmod(since_us, scenario->outage_every_s * 1e6); }
  return since_us < scenario->outage_ms * 1000;
}

// The connection is gone (TEARDOWN, new SETUP): nothing in flight arrives any more
static void network_clear(void) {
  network.count = 0;
  network.data_in_flight = 0;
  network.burst_remaining = 0;
  network.last_arrival_us = sim_time_us;
}

static void network_deliver_at(packet_kind_t kind, const uint8_t* data, size_t length, int64_t arrival);

// Send a packet over the simulated link: delay, jitter, stalls, loss, duplicates
static void network_send(packet_kind_t kind, const uint8_t* data, size_t length) {
  if (in_outage(sim_time_us)) { return; }
  if (!scenario->buffered) {
    if (network.burst_remaining > 0) {
      network.burst_remaining--;
      return;
    }
    if (random_unit() < scenario->burst_loss) {
      network.burst_remaining = 10 + rand() % 31;
      return;
    }
    if (random_unit() < scenario->loss) { return; }
  }

  int64_t arrival = sim_time_us + (int64_t)((scenario->base_delay_ms + random_unit() * scenario->jitter_ms) * 1000);
  network_deliver_at(kind, data, length, arrival);
  // UDP can duplicate, TCP cannot
  if (!scenario->buffered && random_unit() < scenario->duplicate) {
    network_deliver_at(kind, data, length, arrival + 1000 + rand() % 20000);
  }
}

static void network_deliver_at(packet_kind_t kind, const uint8_t* data, size_t length, int64_t arrival) {
  if (kind == PACKET_RETRANSMIT) { arrival += (int64_t)(scenario->base_delay_ms * 1000); } // Request travels too

  if (scenario->stall_every_s > 0) {
    int64_t every_us = (int64_t)(scenario->stall_every_s * 1e6);
    int64_t window_start = arrival / every_us * every_us;
    if (window_start > 0 && arrival < window_start + (int64_t)(scenario->stall_ms * 1000)) {
      arrival = window_start + (int64_t)(scenario->stall_ms * 1000);
    }
  }

  if (scenario->buffered) {
    if (arrival < network.last_arrival_us) { arrival = network.last_arrival_us; }
    network.last_arrival_us = arrival;
  }

  packet_t packet = {.time_us = arrival, .kind = kind, .length = (uint16_t)length};
  memcpy(packet.data, data, length);
  network_push(&packet);
}

// ============================================================================
// Phone
// ============================================================================

typedef struct {
  bool     valid;
  uint16_t sequence;
  uint32_t timestamp;
  int      track;
} history_t;

typedef struct {
  uint32_t start_timestamp;
  int64_t  start_us;
} segment_t;

static struct {
  bool      playing;
  double    rate; // Samples per simulated second
  int       samples_per_packet;
  uint32_t  sequence;
  uint32_t  timestamp;
  int       track;
  double    next_send_us;
  uint32_t  track_end; // Buffered up-next: where the current track ends (0 = never)
  int       next_track;
  uint32_t  next_track_sequence; // First sequence number of next_track
  history_t history[HISTORY_SIZE];
  segment_t segments[MAX_TRACKS];
} phone;

// What the speaker is expected to play
static struct {
  bool     audio;         // Sound should be coming out (gaps count)
  bool     allowed[MAX_TRACKS];
  bool     waiting;       // Waiting for the first sample of track after a start
  int      track;
  int64_t  start_us;
  bool     check_resume;  // The first sample must be resume_source (nothing lost)
  uint32_t resume_source;
  int64_t  steady_after_us;
  uint32_t first_track_end; // Up-next: the first track must play up to here
} expected;

static void build_rtp(uint8_t* packet, uint16_t sequence, uint32_t timestamp, int track, int samples) {
  packet[0] = 0x80;
  packet[1] = 0x60;
  packet[2] = (uint8_t)(sequence >> 8);
  packet[3] = (uint8_t)sequence;
  packet[4] = (uint8_t)(timestamp >> 24);
  packet[5] = (uint8_t)(timestamp >> 16);
  packet[6] = (uint8_t)(timestamp >> 8);
  packet[7] = (uint8_t)timestamp;
  memset(packet + 8, 0x11, 4); // SSRC
  sim_payload_write(packet + 12, timestamp, track, samples);
}

// Buffered packets: 24-bit sequence number in bytes 1-3
static void build_buffered(uint8_t* packet, uint32_t sequence, uint32_t timestamp, int track, int samples) {
  packet[0] = 0x80;
  packet[1] = (uint8_t)(sequence >> 16);
  packet[2] = (uint8_t)(sequence >> 8);
  packet[3] = (uint8_t)sequence;
  packet[4] = (uint8_t)(timestamp >> 24);
  packet[5] = (uint8_t)(timestamp >> 16);
  packet[6] = (uint8_t)(timestamp >> 8);
  packet[7] = (uint8_t)timestamp;
  memset(packet + 8, 0x11, 4);
  sim_payload_write(packet + 12, timestamp, track, samples);
}

// The phone starts sending a new piece of audio (play, skip, resume)
static void phone_start_segment(int track, uint32_t timestamp) {
  phone.playing = true;
  phone.track = track;
  phone.timestamp = timestamp;
  phone.next_send_us = (double)sim_time_us;
  phone.segments[track] = (segment_t){.start_timestamp = timestamp, .start_us = sim_time_us};

  memset(expected.allowed, 0, sizeof(expected.allowed));
  expected.allowed[track] = true;
  expected.audio = false;
  expected.waiting = true;
  expected.track = track;
  expected.start_us = sim_time_us;
  expected.check_resume = false;
}

// Malformed packets the receiver must reject without harm: too short, wrong RTP
// version, header extensions or CSRC lists running past the end, payloads the codec
// refuses, stale sequence numbers, junk on the control port
static void send_garbage(void) {
  uint8_t packet[32];
  size_t  length = 0;
  for (size_t i = 0; i < sizeof(packet); i++) { packet[i] = (uint8_t)rand(); }

  if (scenario->buffered) {
    // Shorter than the header, or a payload too short to decode
    length = (size_t)(rand() % (12 + SIM_PAYLOAD_SIZE));
    network_send(PACKET_GARBAGE, packet, length);
    return;
  }

  switch (rand() % 6) {
    case 0: length = (size_t)(rand() % 12); break;
    case 1:
      packet[0] = (uint8_t)(rand() % 0x80); // Version 0 or 1
      length = 12 + SIM_PAYLOAD_SIZE;
      break;
    case 2:
      packet[0] = 0x90; // Extension header with a huge length
      packet[1] = 0x60;
      length = 12 + (size_t)(rand() % 8);
      break;
    case 3:
      packet[0] = 0x8F; // 15 CSRCs that are not there
      packet[1] = 0x60;
      length = 12 + (size_t)(rand() % 20);
      break;
    case 4: {
      // Valid header far behind the stream, short payload
      uint16_t sequence = (uint16_t)(phone.sequence - 2000);
      build_rtp(packet, sequence, phone.timestamp - 2000 * ALAC_SAMPLES, phone.track, ALAC_SAMPLES);
      length = 12 + (size_t)(rand() % SIM_PAYLOAD_SIZE);
      break;
    }
    default:
      packet[1] = 0xD6; // Retransmit marker with junk after it
      length = (size_t)(rand() % sizeof(packet));
      network_send(PACKET_CONTROL_GARBAGE, packet, length);
      return;
  }
  network_send(PACKET_GARBAGE, packet, length);
}

static void phone_send_realtime(void) {
  double packet_us = phone.samples_per_packet / phone.rate * 1e6;
  while (phone.playing && phone.next_send_us <= (double)sim_time_us) {
    uint8_t   packet[12 + SIM_PAYLOAD_SIZE];
    uint16_t  sequence = (uint16_t)phone.sequence;
    history_t entry = {.valid = true, .sequence = sequence, .timestamp = phone.timestamp, .track = phone.track};
    phone.history[sequence % HISTORY_SIZE] = entry;

    build_rtp(packet, sequence, phone.timestamp, phone.track, phone.samples_per_packet);
    network_send(PACKET_DATA, packet, sizeof(packet));
    if (random_unit() < scenario->garbage) { send_garbage(); }

    phone.sequence = (phone.sequence + 1) & 0xFFFF;
    phone.timestamp += (uint32_t)phone.samples_per_packet;
    phone.next_send_us += packet_us;
  }
}

// TCP: the phone sends as fast as the receiver takes it, a few packets per step
static void phone_send_buffered(void) {
  for (int sent = 0; phone.playing && sent < 2; sent++) {
    int frames_per_packet = (phone.samples_per_packet + AAC_FRAMES_PER_PACKET - 1) / AAC_FRAMES_PER_PACKET;
    if (audio_receiver_get_buffered_frames() + (network.data_in_flight + 1) * frames_per_packet >= MAX_RING_BUFFER_FRAMES - 16) { return; }

    if (phone.track_end && phone.timestamp == phone.track_end) {
      // Playlist continues into the next track without a timestamp jump
      phone.track = phone.next_track;
      phone.segments[phone.track] = (segment_t){.start_timestamp = phone.timestamp, .start_us = sim_time_us};
      phone.next_track_sequence = phone.sequence;
      phone.track_end = 0;
    }

    uint8_t packet[12 + SIM_PAYLOAD_SIZE];
    build_buffered(packet, phone.sequence, phone.timestamp, phone.track, phone.samples_per_packet);
    network_send(PACKET_DATA, packet, sizeof(packet));
    if (random_unit() < scenario->garbage) { send_garbage(); }

    phone.sequence = (phone.sequence + 1) & 0xFFFFFF;
    phone.timestamp += (uint32_t)phone.samples_per_packet;
  }
}

// Retransmit request from the receiver (realtime control port)
static ssize_t
sim_sendto(int socket, const void* data, size_t length, int flags, const struct sockaddr* address, socklen_t address_length) {
  (void)socket;
  (void)flags;
  (void)address;
  (void)address_length;
  const uint8_t* request = data;
  if (length < 8) { return (ssize_t)length; }

  uint16_t first = (uint16_t)((request[4] << 8) | request[5]);
  uint16_t count = (uint16_t)((request[6] << 8) | request[7]);
  for (uint16_t i = 0; i < count; i++) {
    uint16_t   sequence = (uint16_t)(first + i);
    history_t* entry = &phone.history[sequence % HISTORY_SIZE];
    if (!entry->valid || entry->sequence != sequence) { continue; }

    uint8_t packet[4 + 12 + SIM_PAYLOAD_SIZE] = {0x80, 0xD6, 0x00, 0x01};
    build_rtp(packet + 4, sequence, entry->timestamp, entry->track, ALAC_SAMPLES);
    network_send(PACKET_RETRANSMIT, packet, sizeof(packet));
  }
  return (ssize_t)length;
}

static ssize_t sim_recvfrom(int socket, void* buffer, size_t length, int flags, struct sockaddr* address, socklen_t* address_length) {
  (void)socket;
  (void)flags;
  (void)address;
  (void)address_length;
  size_t copy = network.pending_length < length ? network.pending_length : length;
  memcpy(buffer, network.pending, copy);
  return (ssize_t)copy;
}

// Run the realtime decode task until the decode ring is empty
static void pump_decoder(void) {
  size_t item_size = 0;
  void*  item = NULL;
  while (receiver.decode_ring && (item = xRingbufferReceive(receiver.decode_ring, &item_size, 0)) != NULL) {
    decode_item(&receiver, item, item_size);
    vRingbufferReturnItem(receiver.decode_ring, item);
  }
}

static void deliver_packets(void) {
  while (network.count > 0 && network.heap[0].time_us <= sim_time_us) {
    packet_t packet = network_pop();

    // A stopped stream has closed its sockets
    audio_stream_t* stream = scenario->buffered ? receiver.buffered_stream : receiver.realtime_stream;
    if (!stream->running) { continue; }

    // Exactly sized copy, so AddressSanitizer catches reads past the packet
    uint8_t* exact = malloc(packet.length ? packet.length : 1);
    memcpy(exact, packet.data, packet.length);

    if (scenario->buffered) {
      buffered_handle_packet(stream, exact, packet.length);
    } else if (packet.kind == PACKET_RETRANSMIT || packet.kind == PACKET_CONTROL_GARBAGE) {
      process_control_packet(stream, exact, packet.length);
    } else {
      memcpy(network.pending, packet.data, packet.length);
      network.pending_length = packet.length;
      uint8_t            buffer[MAX_RTP_PACKET_SIZE];
      struct sockaddr_in address;
      socklen_t          address_length = sizeof(address);
      realtime_receive_packet(stream, buffer, &address, &address_length);
    }
    free(exact);
    pump_decoder();
  }
}

// ============================================================================
// Speaker
// ============================================================================

static struct {
  double   next_read_us;
  double   sample_us;
  bool     sounding; // Last read had audio
  bool     gap_since_previous;
  double   gap_start_us;
  bool     has_previous;
  uint32_t previous_source;
  int      previous_track;
  long     zeros_since_previous;
  bool     previous_stale;
} speaker;

static void note_delay(uint32_t source, int track, double out_us) {
  if (scenario->buffered || sim_time_us < expected.steady_after_us) { return; }

  // The samples carry 28 bits of the timestamp (~100 min): unwrap with the time passed
  segment_t* segment = &phone.segments[track];
  double     wrap = (double)SOURCE_MASK + 1;
  double     offset = (double)((source - segment->start_timestamp) & SOURCE_MASK);
  double     elapsed = (out_us - (double)segment->start_us) / 1e6 * phone.rate;
  offset += round((elapsed - offset) / wrap) * wrap;
  double sent_us = (double)segment->start_us + offset / phone.rate * 1e6;
  double     delay_ms = (out_us - sent_us) / 1000;
  if (delay_ms < results.delay_min_ms) { results.delay_min_ms = delay_ms; }
  if (delay_ms > results.delay_max_ms) { results.delay_max_ms = delay_ms; }
}

static void check_sample(int16_t left, int16_t right, double out_us) {
  if (left == 0 && right == 0) {
    // Silence the reorder buffer put in place of a lost packet
    results.concealed++;
    speaker.zeros_since_previous++;
    return;
  }

  int      track = (right >> 12) & 7;
  uint32_t source = ((uint32_t)(right & 0xFFF) << 16) | (uint16_t)left;

  bool stale = !expected.allowed[track];
  if (stale) {
    if (sim_verbose && !speaker.previous_stale) {
      printf("    [%9.3f] stale audio from track %d (source %u)\n", (double)sim_time_us / 1e6, track, (unsigned)source);
    }
    results.stale++;
  }
  speaker.previous_stale = stale;

  if (expected.waiting && track == expected.track) {
    double start_ms = (out_us - (double)expected.start_us) / 1000;
    if (start_ms > results.worst_start_ms) { results.worst_start_ms = start_ms; }
    if (expected.check_resume) {
      double lost_ms = (double)((source - expected.resume_source) & SOURCE_MASK) * 1000 / SAMPLE_RATE;
      if (lost_ms > results.resume_lost_ms) { results.resume_lost_ms = lost_ms; }
    }
    expected.waiting = false;
    expected.audio = true;
    expected.steady_after_us = sim_time_us + SETTLE_US;
  }

  if (scenario->up_next_at_s > 0 && !results.up_next_checked && speaker.has_previous && track != speaker.previous_track) {
    // First sample after the first track: that track must have played to its very end
    results.up_next_checked = true;
    results.up_next_ok =
      speaker.previous_track == 1 && ((speaker.previous_source + 1) & SOURCE_MASK) == (expected.first_track_end & SOURCE_MASK);
  }

  if (speaker.has_previous && track == speaker.previous_track) {
    int32_t step = (int32_t)((source - speaker.previous_source) & SOURCE_MASK);
    int32_t error = step - 1 - (int32_t)speaker.zeros_since_previous;
    if (speaker.gap_since_previous) {
      if (error > 0) { results.audio_lost_ms += (double)error * 1000 / SAMPLE_RATE; }
    } else if (speaker.zeros_since_previous > 0 && labs(error) <= speaker.zeros_since_previous / ALAC_SAMPLES + 2) {
      // Drift correction also inserts/drops samples inside concealment silence (up to
      // one per packet)
    } else if (error == -1) {
      results.inserted++;
    } else if (error == 1) {
      results.dropped++;
    } else if (error != 0) {
      results.jumps++;
      if (sim_verbose) {
        printf(
          "    [%9.3f] jump of %d samples in track %d (%u -> %u)\n",
          (double)sim_time_us / 1e6,
          error,
          track,
          (unsigned)speaker.previous_source,
          (unsigned)source);
      }
    }
  }

  note_delay(source, track, out_us);

  speaker.has_previous = true;
  speaker.previous_source = source;
  speaker.previous_track = track;
  speaker.zeros_since_previous = 0;
  speaker.gap_since_previous = false;
}

// The playback task: read from the receiver at the I2S rate
static void run_speaker(void) {
  int16_t pcm[READ_SAMPLES * 2];
  while (speaker.next_read_us <= (double)sim_time_us) {
    size_t samples = audio_receiver_read(pcm, READ_SAMPLES);

    if (samples > 0) {
      if (!speaker.sounding && speaker.gap_start_us > 0) {
        results.gap_ms += (speaker.next_read_us - speaker.gap_start_us) / 1000;
        speaker.gap_start_us = 0;
        expected.steady_after_us = sim_time_us + SETTLE_US;
      }
      for (size_t i = 0; i < samples; i++) {
        double out_us = speaker.next_read_us + (double)i * speaker.sample_us + OUTPUT_DMA_MS * 1000.0;
        check_sample(pcm[i * 2], pcm[i * 2 + 1], out_us);
      }
      speaker.sounding = true;
      speaker.next_read_us += (double)samples * speaker.sample_us;
    } else {
      if (speaker.sounding && expected.audio) {
        results.gaps++;
        speaker.gap_start_us = speaker.next_read_us;
        speaker.gap_since_previous = true;
        if (sim_verbose) { printf("    [%9.3f] gap\n", (double)sim_time_us / 1e6); }
      }
      speaker.sounding = false;
      speaker.next_read_us += READ_SAMPLES * speaker.sample_us;
    }
  }
}

// ============================================================================
// Scenario events
// ============================================================================

static int64_t resume_at_us = -1;  // Pending resume after a pause
static int64_t session_at_us = -1; // Pending new session after a TEARDOWN

// SETUP (with ANNOUNCE-like format) and RECORD
static void start_session(void) {
  audio_format_t format = {.sample_rate = SAMPLE_RATE, .channels = 2, .bits_per_sample = 16, .frame_size = phone.samples_per_packet};
  strcpy(format.codec, scenario->buffered ? "AAC" : "ALAC");
  audio_receiver_set_format(&format);

  if (scenario->buffered) {
    audio_receiver_set_stream_type(AUDIO_STREAM_BUFFERED);
    audio_receiver_start_buffered(0);
  } else {
    audio_receiver_set_client_addr(0x0100007F, 6001);
    audio_receiver_set_stream_type(AUDIO_STREAM_REALTIME);
    audio_receiver_start(6000, 6002);
  }
}

static void stop_expecting_audio(void) {
  phone.playing = false;
  expected.audio = false;
  expected.waiting = false;
  memset(expected.allowed, 0, sizeof(expected.allowed));
}

static void teardown(void) {
  audio_receiver_stop();
  network_clear();
  stop_expecting_audio();
  resume_at_us = -1;
}

static void new_session(void) {
  phone.sequence = (uint32_t)rand() & 0xFFFF;
  start_session();
  phone_start_segment(next_track_id(phone.track), phone.timestamp + TRACK_JUMP);
}

// Buffered: the next song has another frame size, the phone sets the stream up again
static void change_format(void) {
  phone.samples_per_packet = phone.samples_per_packet == AAC_SAMPLES ? 4096 : AAC_SAMPLES;
  network_clear();
  start_session();
  phone_start_segment(next_track_id(phone.track), phone.timestamp + TRACK_JUMP);
  resume_at_us = -1;
}

static void skip(void) {
  uint32_t timestamp = phone.timestamp + TRACK_JUMP;
  int      track = next_track_id(phone.track);
  resume_at_us = -1; // Skipping while paused starts playing

  if (scenario->buffered) {
    // FLUSHBUFFERED without from: drop everything up to the next sequence number
    audio_flush_request_t request = {.until_sequence = phone.sequence, .until_timestamp = timestamp};
    audio_receiver_flush_buffered(&request);
  } else {
    // FLUSH with RTP-Info rtptime of the new audio
    audio_receiver_flush_until(timestamp);
  }
  phone_start_segment(track, timestamp);
}

static void pause_playback(void) {
  // SETRATEANCHORTIME rate 0
  audio_receiver_flush();
  stop_expecting_audio();
}

static void resume_playback(void) {
  int track = next_track_id(phone.track);

  if (scenario->buffered) {
    // The phone resends from where the speaker was when paused, with new sequence
    // numbers (FLUSHBUFFERED until the next sequence number, then rate 1)
    uint32_t position = speaker.previous_source;
    audio_flush_request_t request = {.until_sequence = phone.sequence, .until_timestamp = position};
    audio_receiver_flush_buffered(&request);
    phone_start_segment(track, position);
    expected.resume_source = position;
  } else {
    // Realtime carries on where it stopped (rate 1, no flush)
    phone_start_segment(track, phone.timestamp);
    expected.resume_source = phone.timestamp;
  }
  expected.check_resume = true;
}

static void change_up_next(void) {
  // The current track (all sent) keeps playing; the next track, already partly sent,
  // is replaced by another one
  if (phone.track_end != 0) { printf("     (scenario error: the phone has not reached the next track yet)\n"); }
  uint32_t              timestamp = phone.timestamp + TRACK_JUMP;
  int                   track = next_track_id(phone.track);
  audio_flush_request_t request = {
    .has_from = true,
    .from_sequence = phone.next_track_sequence,
    .from_timestamp = phone.segments[phone.track].start_timestamp,
    .until_sequence = phone.sequence,
    .until_timestamp = timestamp,
  };
  audio_receiver_flush_buffered(&request);

  int current = expected.track;
  phone.track = track;
  phone.timestamp = timestamp;
  phone.segments[track] = (segment_t){.start_timestamp = timestamp, .start_us = sim_time_us};
  memset(expected.allowed, 0, sizeof(expected.allowed));
  expected.allowed[current] = true;
  expected.allowed[track] = true;
}

// ============================================================================
// Running
// ============================================================================

static bool run_scenario(void) {
  srand(1);
  memset(&results, 0, sizeof(results));
  results.delay_min_ms = 1e9;
  results.delay_max_ms = -1e9;
  results.buffer_min_ms = 1e9;
  results.buffer_max_ms = -1e9;

  phone.rate = SAMPLE_RATE * (1 + scenario->sender_ppm / 1e6);
  speaker.sample_us = 1e6 / (SAMPLE_RATE * (1 + scenario->output_ppm / 1e6));

  phone.samples_per_packet = scenario->samples_per_packet ? scenario->samples_per_packet : (scenario->buffered ? AAC_SAMPLES : ALAC_SAMPLES);
  uint32_t start_timestamp = scenario->start_timestamp ? scenario->start_timestamp : 100000;

  audio_receiver_init();
  start_session();

  phone.sequence = scenario->start_sequence ? scenario->start_sequence : 1000;
  phone_start_segment(1, start_timestamp);
  if (scenario->track_length_s > 0) {
    phone.track_end =
      start_timestamp + (uint32_t)(scenario->track_length_s * SAMPLE_RATE / phone.samples_per_packet) * (uint32_t)phone.samples_per_packet;
    phone.next_track = next_track_id(phone.track);
    expected.first_track_end = phone.track_end;
  }

  int64_t end_us = (int64_t)(scenario->duration_s * 1e6);
  int64_t skip_until_us = scenario->skip_until_s > 0 ? (int64_t)(scenario->skip_until_s * 1e6) : end_us;
  int64_t next_skip_us = scenario->skip_every_s > 0 ? (int64_t)(scenario->skip_every_s * 1e6) : -1;
  int64_t next_pause_us = scenario->pause_every_s > 0 ? (int64_t)(scenario->pause_every_s * 1e6) : -1;
  int64_t next_restart_us = scenario->restart_every_s > 0 ? (int64_t)(scenario->restart_every_s * 1e6) : -1;
  int64_t next_format_us = scenario->format_change_every_s > 0 ? (int64_t)(scenario->format_change_every_s * 1e6) : -1;
  int64_t up_next_us = scenario->up_next_at_s > 0 ? (int64_t)(scenario->up_next_at_s * 1e6) : -1;
  int64_t next_sample_us = 0;

  for (sim_time_us = 0; sim_time_us < end_us; sim_time_us += STEP_US) {
    bool in_session = session_at_us < 0;
    if (next_skip_us >= 0 && sim_time_us >= next_skip_us) {
      if (in_session && sim_time_us < skip_until_us) { skip(); }
      next_skip_us += (int64_t)(scenario->skip_every_s * 1e6);
    }
    if (next_pause_us >= 0 && sim_time_us >= next_pause_us) {
      if (in_session && phone.playing) {
        pause_playback();
        resume_at_us = sim_time_us + (int64_t)(scenario->pause_ms * 1000);
      }
      next_pause_us += (int64_t)(scenario->pause_every_s * 1e6);
    }
    if (resume_at_us >= 0 && sim_time_us >= resume_at_us) {
      resume_playback();
      resume_at_us = -1;
    }
    if (up_next_us >= 0 && sim_time_us >= up_next_us) {
      change_up_next();
      up_next_us = -1;
    }
    if (next_restart_us >= 0 && sim_time_us >= next_restart_us) {
      if (in_session) {
        teardown();
        session_at_us = sim_time_us + 1000000;
      }
      next_restart_us += (int64_t)(scenario->restart_every_s * 1e6);
    }
    if (session_at_us >= 0 && sim_time_us >= session_at_us) {
      new_session();
      session_at_us = -1;
    }
    if (next_format_us >= 0 && sim_time_us >= next_format_us) {
      if (in_session) { change_format(); }
      next_format_us += (int64_t)(scenario->format_change_every_s * 1e6);
    }

    if (scenario->buffered) {
      phone_send_buffered();
    } else {
      phone_send_realtime();
    }
    deliver_packets();
    run_speaker();

    if (sim_time_us >= next_sample_us) {
      next_sample_us += 100000;
      if (expected.audio && speaker.sounding && sim_time_us >= expected.steady_after_us) {
        double buffer_ms = audio_receiver_get_buffered_ms();
        if (buffer_ms < results.buffer_min_ms) { results.buffer_min_ms = buffer_ms; }
        if (buffer_ms > results.buffer_max_ms) { results.buffer_max_ms = buffer_ms; }
      }
    }
  }

  // Never started, unless the last start was too close to the end to finish
  if (expected.waiting && (double)(end_us - expected.start_us) / 1000 > scenario->max_start_ms) { results.worst_start_ms = 1e9; }

  // Verdict
  char problems[512] = "";
  size_t used = 0;
#define PROBLEM(...) used += (size_t)snprintf(problems + used, sizeof(problems) - used, __VA_ARGS__)
  if (results.gaps > scenario->max_gaps) { PROBLEM(" %ld gaps (max %d);", results.gaps, scenario->max_gaps); }
  if (results.stale > 0) { PROBLEM(" %ld stale samples;", results.stale); }
  if (results.jumps > 0) { PROBLEM(" %ld jumps;", results.jumps); }
  if (results.worst_start_ms > scenario->max_start_ms) { PROBLEM(" start %.0f ms (max %.0f);", results.worst_start_ms, scenario->max_start_ms); }
  if (results.resume_lost_ms > 0) { PROBLEM(" %.0f ms lost on resume;", results.resume_lost_ms); }
  double spread_ms = results.delay_max_ms - results.delay_min_ms;
  if (scenario->max_delay_spread_ms > 0 && spread_ms > scenario->max_delay_spread_ms) {
    PROBLEM(" delay drifts %.0f ms (max %.0f);", spread_ms, scenario->max_delay_spread_ms);
  }
  if (scenario->up_next_at_s > 0 && !results.up_next_ok) { PROBLEM(" current track cut short;"); }

  // TEARDOWN at the end, then everything the stream allocated must be released
  audio_receiver_stop();
  if (sim_ring_leaked_bytes > 0) { PROBLEM(" %zu KB of ring buffers leaked;", sim_ring_leaked_bytes / 1024); }
  if (sim_ring_misuse > 0) { PROBLEM(" %d ring buffers freed the wrong way;", sim_ring_misuse); }
#undef PROBLEM

  bool passed = problems[0] == '\0';
  printf("%s %-20s %s\n", passed ? "PASS" : "FAIL", scenario->name, scenario->description);
  printf("     gaps %ld (%.0f ms) | start %.0f ms", results.gaps, results.gap_ms, results.worst_start_ms);
  if (!scenario->buffered) { printf(" | delay %.0f-%.0f ms", results.delay_min_ms, results.delay_max_ms); }
  if (results.buffer_max_ms >= 0) { printf(" | buffer %.0f-%.0f ms", results.buffer_min_ms, results.buffer_max_ms); }
  printf(" | drift fix +%ld/-%ld", results.inserted, results.dropped);
  if (results.concealed) { printf(" | concealed %.0f ms", (double)results.concealed * 1000 / SAMPLE_RATE); }
  if (results.audio_lost_ms > 0) { printf(" | skipped %.0f ms", results.audio_lost_ms); }
  printf(" | warnings %d\n", sim_warnings);
  if (!passed) { printf("     problems:%s\n", problems); }
  fflush(stdout);
  return passed;
}

static void usage(void) {
  printf("usage: simulate [-v] [-l] [scenario...]\n");
  printf("  -v  print firmware log lines, gaps and jumps\n");
  printf("  -l  list scenarios\n");
}

int main(int argc, char** argv) {
  const char* selected[SCENARIO_COUNT + 1];
  size_t      selected_count = 0;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-v") == 0) {
      sim_verbose = true;
    } else if (strcmp(argv[i], "-l") == 0) {
      for (size_t s = 0; s < SCENARIO_COUNT; s++) { printf("%-20s %s\n", scenarios[s].name, scenarios[s].description); }
      return 0;
    } else if (argv[i][0] == '-') {
      usage();
      return 2;
    } else if (selected_count < SCENARIO_COUNT) {
      selected[selected_count++] = argv[i];
    }
  }

  int run = 0;
  int failed = 0;
  for (size_t s = 0; s < SCENARIO_COUNT; s++) {
    bool wanted = selected_count == 0;
    for (size_t i = 0; i < selected_count; i++) { wanted = wanted || strcmp(selected[i], scenarios[s].name) == 0; }
    if (!wanted) { continue; }

    // Fresh process per scenario: the receiver keeps its state in statics
    fflush(stdout);
    pid_t child = fork();
    if (child == 0) {
      scenario = &scenarios[s];
      exit(run_scenario() ? 0 : 1);
    }
    int status = 0;
    waitpid(child, &status, 0);
    run++;
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
      failed++;
      if (!WIFEXITED(status)) { printf("FAIL %-20s crashed\n", scenarios[s].name); }
    }
  }

  if (run == 0) {
    printf("No scenario with that name, -l lists them\n");
    return 2;
  }
  printf("\n%d of %d scenarios passed\n", run - failed, run);
  return failed ? 1 : 0;
}
