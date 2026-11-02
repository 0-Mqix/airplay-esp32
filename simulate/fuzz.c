// Fuzzer for the parsers that read what the phone sends: binary plists (SETUP,
// FLUSHBUFFERED, SETRATEANCHORTIME, /command, /feedback), RTSP requests and TLV8.
// Inputs are generated plists and requests with random corruption, every buffer is
// allocated at its exact size so AddressSanitizer catches any read past the end.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "plist.h"
#include "rtsp_message.h"
#include "tlv8.h"

// rtsp_message.c sends responses through these, the fuzzer never does
int rtsp_crypto_write_frame(int socket, rtsp_conn_t* conn, const uint8_t* data, size_t data_len) {
  (void)socket;
  (void)conn;
  (void)data;
  return (int)data_len;
}

int rtsp_crypto_read_block(int socket, rtsp_conn_t* conn, uint8_t* buffer, size_t buffer_size) {
  (void)socket;
  (void)conn;
  (void)buffer;
  (void)buffer_size;
  return -1;
}

int64_t sim_time_us = 0;
bool    sim_verbose = false;
int     sim_warnings = 0;

static uint32_t random_state = 1;

static uint32_t random_next(void) {
  random_state ^= random_state << 13;
  random_state ^= random_state >> 17;
  random_state ^= random_state << 5;
  return random_state;
}

static uint32_t random_below(uint32_t limit) { return limit ? random_next() % limit : 0; }

// ============================================================================
// Binary plist generator
// ============================================================================

#define MAX_OBJECTS 256

static const char* keys[] = {
  "streams",       "type",         "ct",           "sr",          "spf",         "controlPort", "dataPort",
  "ekey",          "eiv",          "shk",          "rate",        "rtpTime",     "networkTimeSecs",
  "flushUntilSeq", "flushUntilTS", "flushFromSeq", "flushFromTS", "audioFormat", "timingProtocol"};
#define KEY_COUNT (sizeof(keys) / sizeof(keys[0]))

typedef struct {
  uint8_t data[64 * 1024];
  size_t  length;
  size_t  offsets[MAX_OBJECTS];
  int     count;
} plist_writer_t;

static void put(plist_writer_t* w, const void* data, size_t length) {
  if (w->length + length > sizeof(w->data)) { return; }
  memcpy(w->data + w->length, data, length);
  w->length += length;
}

static void put_byte(plist_writer_t* w, uint8_t byte) { put(w, &byte, 1); }

static void put_be(plist_writer_t* w, uint64_t value, int bytes) {
  for (int i = bytes - 1; i >= 0; i--) { put_byte(w, (uint8_t)(value >> (i * 8))); }
}

static int begin_object(plist_writer_t* w) {
  if (w->count >= MAX_OBJECTS) { return -1; }
  w->offsets[w->count] = w->length;
  return w->count++;
}

static void put_marker(plist_writer_t* w, uint8_t type, size_t length) {
  if (length < 15) {
    put_byte(w, (uint8_t)(type | length));
  } else {
    put_byte(w, (uint8_t)(type | 0x0F));
    put_byte(w, 0x12);
    put_be(w, length, 4);
  }
}

static int write_int(plist_writer_t* w, int64_t value) {
  int index = begin_object(w);
  if (index < 0) { return -1; }
  int sizes[] = {1, 2, 4, 8};
  int power = (int)random_below(4);
  put_byte(w, (uint8_t)(0x10 | power));
  put_be(w, (uint64_t)value, sizes[power]);
  return index;
}

static int write_string(plist_writer_t* w, const char* text) {
  int index = begin_object(w);
  if (index < 0) { return -1; }
  size_t length = strlen(text);
  put_marker(w, 0x50, length);
  put(w, text, length);
  return index;
}

static int write_data(plist_writer_t* w, size_t length) {
  int index = begin_object(w);
  if (index < 0) { return -1; }
  put_marker(w, 0x40, length);
  for (size_t i = 0; i < length; i++) { put_byte(w, (uint8_t)random_next()); }
  return index;
}

static int write_value(plist_writer_t* w, int depth, int ref_size);

static int write_container(plist_writer_t* w, int depth, int ref_size, bool dict) {
  int count = (int)random_below(dict ? 8 : 4);
  int key_refs[16];
  int value_refs[16];
  for (int i = 0; i < count; i++) {
    key_refs[i] = dict ? write_string(w, keys[random_below(KEY_COUNT)]) : 0;
    value_refs[i] = write_value(w, depth + 1, ref_size);
  }

  int index = begin_object(w);
  if (index < 0) { return -1; }
  put_marker(w, dict ? 0xD0 : 0xA0, (size_t)count);
  if (dict) {
    for (int i = 0; i < count; i++) { put_be(w, (uint64_t)(key_refs[i] < 0 ? 0 : key_refs[i]), ref_size); }
  }
  for (int i = 0; i < count; i++) { put_be(w, (uint64_t)(value_refs[i] < 0 ? 0 : value_refs[i]), ref_size); }
  return index;
}

static int write_value(plist_writer_t* w, int depth, int ref_size) {
  switch (random_below(depth > 3 ? 4 : 7)) {
    case 0: return write_int(w, (int64_t)random_next() - (int64_t)random_below(3) * 0x7FFFFFFF);
    case 1: return write_string(w, keys[random_below(KEY_COUNT)]);
    case 2: return write_data(w, random_below(48));
    case 3: {
      int index = begin_object(w);
      if (index >= 0) {
        put_byte(w, 0x23);
        double value = (double)random_below(4) * 0.5;
        uint64_t bits;
        memcpy(&bits, &value, sizeof(bits));
        put_be(w, bits, 8);
      }
      return index;
    }
    case 4:
    case 5: return write_container(w, depth, ref_size, true);
    default: return write_container(w, depth, ref_size, false);
  }
}

// A plist shaped like the SETUP/FLUSHBUFFERED bodies the handlers parse
static void generate_plist(plist_writer_t* w) {
  w->length = 0;
  w->count = 0;
  int ref_size = random_below(4) == 0 ? 2 : 1;
  put(w, "bplist00", 8);

  int top = write_container(w, 0, ref_size, true);
  if (top < 0) { top = 0; }

  uint64_t table_offset = w->length;
  int      offset_size = w->length > 0xFFFF ? 4 : (w->length > 0xFF ? 2 : 1);
  for (int i = 0; i < w->count; i++) { put_be(w, w->offsets[i], offset_size); }

  uint8_t trailer[32] = {0};
  trailer[6] = (uint8_t)offset_size;
  trailer[7] = (uint8_t)ref_size;
  for (int i = 0; i < 8; i++) {
    trailer[8 + i] = (uint8_t)((uint64_t)w->count >> ((7 - i) * 8));
    trailer[16 + i] = (uint8_t)((uint64_t)top >> ((7 - i) * 8));
    trailer[24 + i] = (uint8_t)(table_offset >> ((7 - i) * 8));
  }
  put(w, trailer, sizeof(trailer));
}

static void corrupt(uint8_t* data, size_t* length, size_t capacity) {
  int changes = (int)random_below(4);
  for (int i = 0; i < changes && *length > 0; i++) {
    size_t position = random_below((uint32_t)*length);
    switch (random_below(6)) {
      case 0: data[position] = (uint8_t)random_next(); break;
      case 1: data[position] ^= (uint8_t)(1 << random_below(8)); break;
      case 2: data[position] = (uint8_t[]){0x00, 0x0F, 0x7F, 0x80, 0xFF}[random_below(5)]; break;
      case 3: *length = position; break; // Truncate
      case 4:
        // Trailer fields: sizes, object count, top object, offset table
        if (*length >= 32) { data[*length - 32 + 6 + random_below(26)] = (uint8_t)random_next(); }
        break;
      default:
        if (*length < capacity) { data[(*length)++] = (uint8_t)random_next(); }
        break;
    }
  }
}

static void fuzz_plist(const uint8_t* plist, size_t length) {
  size_t count = 0;
  if (bplist_get_streams_count(plist, length, &count)) {
    for (size_t i = 0; i < count + 2 && i < 10; i++) {
      int64_t  type = 0;
      uint16_t control_port = 0;
      size_t   ekey_length = 0, eiv_length = 0, shk_length = 0;
      bplist_get_stream_info(plist, length, i, &type, &control_port, &ekey_length, &eiv_length, &shk_length);

      bplist_kv_info_t kv[16];
      size_t           kv_count = 0;
      bplist_get_stream_kv_info(plist, length, i, kv, 16, &kv_count);
    }
  }

  uint8_t ekey[64], eiv[16], shk[32];
  size_t  ekey_length = 0, eiv_length = 0, shk_length = 0;
  bplist_find_stream_crypto(plist, length, 96, ekey, sizeof(ekey), &ekey_length, eiv, sizeof(eiv), &eiv_length, shk, sizeof(shk), &shk_length);
  bplist_find_stream_crypto(plist, length, 103, ekey, sizeof(ekey), &ekey_length, eiv, sizeof(eiv), &eiv_length, shk, sizeof(shk), &shk_length);
  bplist_find_data_deep(plist, length, "ekey", ekey, sizeof(ekey), &ekey_length);
  bplist_find_data(plist, length, "shk", shk, sizeof(shk), &shk_length);

  for (size_t k = 0; k < KEY_COUNT; k++) {
    int64_t integer = 0;
    double  real = 0;
    bplist_find_int(plist, length, keys[k], &integer);
    bplist_find_real(plist, length, keys[k], &real);
  }
}

// ============================================================================
// RTSP requests
// ============================================================================

static const char* methods[] = {"SETUP", "FLUSH", "FLUSHBUFFERED", "SET_PARAMETER", "GET_PARAMETER", "POST", "GET", "TEARDOWN", "RECORD"};

static size_t generate_request(char* out, size_t capacity) {
  int length = snprintf(
    out,
    capacity,
    "%s rtsp://192.168.4.1/%u RTSP/1.0\r\nCSeq: %u\r\nContent-Type: application/x-apple-binary-plist\r\n"
    "Transport: RTP/AVP/UDP;unicast;control_port=%u;timing_port=%u\r\nRTP-Info: seq=%u;rtptime=%u\r\n"
    "Content-Length: %u\r\n\r\n",
    methods[random_below(sizeof(methods) / sizeof(methods[0]))],
    random_next(),
    random_next(),
    random_below(70000),
    random_below(70000),
    random_below(70000),
    random_next(),
    random_below(64));
  size_t used = (size_t)length < capacity ? (size_t)length : capacity;
  size_t body = random_below(64);
  for (size_t i = 0; i < body && used < capacity; i++) { out[used++] = (char)random_next(); }
  return used;
}

static void fuzz_request(const uint8_t* data, size_t length) {
  rtsp_request_t request;
  rtsp_request_parse(data, length, &request);
  rtsp_find_header_end(data, length);

  char* text = malloc(length + 1);
  memcpy(text, data, length);
  text[length] = '\0';
  uint16_t control_port = 0, timing_port = 0;
  rtsp_parse_transport(text, &control_port, &timing_port);
  rtsp_parse_cseq(text);
  rtsp_parse_content_length(text);
  free(text);

  size_t value_length = 0;
  tlv8_find(data, length, (uint8_t)random_below(16), &value_length);
}

// Copy into an exactly sized heap block so reads past the end are caught
static void run_exact(void (*target)(const uint8_t*, size_t), const uint8_t* data, size_t length) {
  uint8_t* exact = malloc(length ? length : 1);
  memcpy(exact, data, length);
  target(exact, length);
  free(exact);
}

int main(int argc, char** argv) {
  long iterations = argc > 1 ? atol(argv[1]) : 200000;

  static plist_writer_t writer;
  for (long i = 0; i < iterations; i++) {
    random_state = (uint32_t)(i * 2654435761u + 1);
    generate_plist(&writer);
    if (random_below(4) != 0) { corrupt(writer.data, &writer.length, sizeof(writer.data)); }
    run_exact(fuzz_plist, writer.data, writer.length);

    char   request[512];
    size_t request_length = generate_request(request, sizeof(request));
    corrupt((uint8_t*)request, &request_length, sizeof(request));
    run_exact(fuzz_request, (const uint8_t*)request, request_length);
  }

  printf("fuzz: %ld plists and requests, no memory errors\n", iterations);
  return 0;
}
