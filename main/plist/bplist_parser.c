#include <inttypes.h>
#include <string.h>

#include "plist.h"

// Binary plist object types (high nibble of marker byte)
#define BPLIST_INT     0x10
#define BPLIST_REAL    0x20
#define BPLIST_DATA    0x40
#define BPLIST_STRING  0x50
#define BPLIST_UNICODE 0x60
#define BPLIST_UID     0x80
#define BPLIST_ARRAY   0xA0
#define BPLIST_SET     0xC0
#define BPLIST_DICT    0xD0

#define MAX_DEPTH  10
#define MAX_VISITS 512 // Objects a deep search may look at (a SETUP has ~50)

// A plist whose trailer and offset table have been checked to lie inside the data.
// Every object reference and length read from the data is checked again before use:
// the phone controls all of it.
typedef struct {
  const uint8_t* data;
  size_t         length;
  uint8_t        offset_size;
  uint8_t        ref_size;
  uint64_t       object_count;
  uint64_t       top_object;
  uint64_t       table_offset;
} bplist_t;

static uint64_t read_be_int(const uint8_t* data, size_t bytes) {
  uint64_t val = 0;
  for (size_t i = 0; i < bytes; i++) { val = (val << 8) | data[i]; }
  return val;
}

static bool bplist_open(bplist_t* p, const uint8_t* data, size_t length) {
  if (!data || length < 40 || memcmp(data, "bplist00", 8) != 0) { return false; }

  const uint8_t* trailer = data + length - 32;
  p->data = data;
  p->length = length;
  p->offset_size = trailer[6];
  p->ref_size = trailer[7];
  p->object_count = read_be_int(trailer + 8, 8);
  p->top_object = read_be_int(trailer + 16, 8);
  p->table_offset = read_be_int(trailer + 24, 8);

  if (p->offset_size == 0 || p->offset_size > 8 || p->ref_size == 0 || p->ref_size > 8) { return false; }

  // The offset table lies between the header and the trailer
  size_t table_end = length - 32;
  if (p->table_offset < 8 || p->table_offset > table_end) { return false; }
  if (p->object_count > (table_end - p->table_offset) / p->offset_size) { return false; }
  return p->top_object < p->object_count;
}

// Offset of object number index
static bool object_offset(const bplist_t* p, uint64_t index, size_t* offset) {
  if (index >= p->object_count) { return false; }
  uint64_t value = read_be_int(p->data + p->table_offset + index * p->offset_size, p->offset_size);
  if (value >= p->length) { return false; }
  *offset = (size_t)value;
  return true;
}

// Marker of a variable length object: type, element or byte count, and where the
// contents start
static bool object_header(const bplist_t* p, size_t offset, uint8_t* type, uint64_t* count, size_t* contents) {
  if (offset >= p->length) { return false; }

  uint8_t marker = p->data[offset];
  *type = marker & 0xF0;
  *count = marker & 0x0F;
  *contents = offset + 1;

  if (*count == 0x0F) {
    // Count follows as an int object
    if (*contents >= p->length) { return false; }
    uint8_t size_marker = p->data[(*contents)++];
    if ((size_marker & 0xF0) != BPLIST_INT || (size_marker & 0x0F) > 3) { return false; }
    size_t bytes = (size_t)1 << (size_marker & 0x0F);
    if (bytes > p->length - *contents) { return false; }
    *count = read_be_int(p->data + *contents, bytes);
    *contents += bytes;
  }
  return true;
}

// Array, set or dict at offset: element count and where its references start
static bool container(const bplist_t* p, size_t offset, uint8_t want_type, uint64_t* count, size_t* refs) {
  uint8_t type = 0;
  if (!object_header(p, offset, &type, count, refs)) { return false; }
  if (type != want_type && !(want_type == BPLIST_ARRAY && type == BPLIST_SET)) { return false; }

  uint64_t ref_count = type == BPLIST_DICT ? *count * 2 : *count;
  if (*count > p->length || ref_count > (p->length - *refs) / p->ref_size) { return false; }
  return true;
}

// Offset of the object that reference number index (counted from refs) points to
static bool container_ref(const bplist_t* p, size_t refs, uint64_t index, size_t* offset) {
  uint64_t position = refs + index * p->ref_size;
  if (position + p->ref_size > p->length) { return false; }
  return object_offset(p, read_be_int(p->data + position, p->ref_size), offset);
}

static bool read_string(const bplist_t* p, size_t offset, char* out, size_t out_capacity) {
  uint8_t  type = 0;
  uint64_t length = 0;
  size_t   contents = 0;
  if (!object_header(p, offset, &type, &length, &contents) || length >= out_capacity) { return false; }

  if (type == BPLIST_STRING) {
    if (length > p->length - contents) { return false; }
    memcpy(out, p->data + contents, length);
    out[length] = '\0';
    return true;
  }

  if (type == BPLIST_UNICODE) {
    // Only ASCII keys are of interest
    if (length > (p->length - contents) / 2) { return false; }
    for (size_t i = 0; i < length; i++) {
      uint16_t code = (uint16_t)(p->data[contents + i * 2] << 8) | p->data[contents + i * 2 + 1];
      if (code > 0x7F) { return false; }
      out[i] = (char)code;
    }
    out[length] = '\0';
    return true;
  }

  return false;
}

static bool read_string_length(const bplist_t* p, size_t offset, size_t* out_len) {
  uint8_t  type = 0;
  uint64_t length = 0;
  size_t   contents = 0;
  if (!object_header(p, offset, &type, &length, &contents)) { return false; }

  uint64_t bytes = type == BPLIST_UNICODE ? length * 2 : length;
  if ((type != BPLIST_STRING && type != BPLIST_UNICODE) || length > p->length || bytes > p->length - contents) { return false; }
  *out_len = (size_t)length;
  return true;
}

// Data object; out may be NULL to only get the length
static bool read_data(const bplist_t* p, size_t offset, uint8_t* out, size_t out_capacity, size_t* out_len) {
  uint8_t  type = 0;
  uint64_t length = 0;
  size_t   contents = 0;
  if (!object_header(p, offset, &type, &length, &contents)) { return false; }
  if (type != BPLIST_DATA || length > p->length - contents) { return false; }

  if (out) {
    if (length > out_capacity) { return false; }
    memcpy(out, p->data + contents, length);
  }
  *out_len = (size_t)length;
  return true;
}

static bool read_int(const bplist_t* p, size_t offset, int64_t* out) {
  if (offset >= p->length) { return false; }

  uint8_t marker = p->data[offset];
  if ((marker & 0xF0) != BPLIST_INT || (marker & 0x0F) > 3) { return false; }

  size_t bytes = (size_t)1 << (marker & 0x0F);
  if (bytes > p->length - offset - 1) { return false; }
  *out = (int64_t)read_be_int(p->data + offset + 1, bytes);
  return true;
}

static bool read_real(const bplist_t* p, size_t offset, double* out) {
  if (offset >= p->length) { return false; }

  uint8_t marker = p->data[offset];
  if ((marker & 0xF0) != BPLIST_REAL) { return false; }

  size_t bytes = (size_t)1 << (marker & 0x0F);
  if ((bytes != 4 && bytes != 8) || bytes > p->length - offset - 1) { return false; }

  if (bytes == 4) {
    uint32_t bits = (uint32_t)read_be_int(p->data + offset + 1, 4);
    float    f;
    memcpy(&f, &bits, sizeof(f));
    *out = (double)f;
  } else {
    uint64_t bits = read_be_int(p->data + offset + 1, 8);
    memcpy(out, &bits, sizeof(*out));
  }
  return true;
}

// Offset of the value stored under key in the dict at dict_offset
static bool dict_lookup(const bplist_t* p, size_t dict_offset, const char* key, size_t* value_offset) {
  uint64_t count = 0;
  size_t   refs = 0;
  if (!container(p, dict_offset, BPLIST_DICT, &count, &refs)) { return false; }

  for (uint64_t i = 0; i < count; i++) {
    size_t key_offset = 0;
    char   name[64];
    if (container_ref(p, refs, i, &key_offset) && read_string(p, key_offset, name, sizeof(name)) && strcmp(name, key) == 0) {
      return container_ref(p, refs, count + i, value_offset);
    }
  }
  return false;
}

static bool top_dict_lookup(const bplist_t* p, const char* key, size_t* value_offset) {
  size_t top = 0;
  return object_offset(p, p->top_object, &top) && dict_lookup(p, top, key, value_offset);
}

// The "streams" array of the top dict: element count and where its references start
static bool streams_array(const bplist_t* p, uint64_t* count, size_t* refs) {
  size_t streams = 0;
  return top_dict_lookup(p, "streams", &streams) && container(p, streams, BPLIST_ARRAY, count, refs);
}

// Dict of stream number index, with its key count and references
static bool stream_dict(const bplist_t* p, size_t index, size_t* offset, uint64_t* count, size_t* refs) {
  uint64_t stream_count = 0;
  size_t   stream_refs = 0;
  if (!streams_array(p, &stream_count, &stream_refs) || index >= stream_count) { return false; }
  return container_ref(p, stream_refs, index, offset) && container(p, *offset, BPLIST_DICT, count, refs);
}

// budget limits the objects visited: references can be shared, so without it a
// small plist can make the search exponential
static bool find_data_recursive(
  const bplist_t* p, size_t offset, const char* key, uint8_t* out_data, size_t out_capacity, size_t* out_len, int depth, int* budget) {
  if (depth > MAX_DEPTH || --(*budget) < 0) { return false; }

  uint64_t count = 0;
  size_t   refs = 0;

  if (container(p, offset, BPLIST_DICT, &count, &refs)) {
    size_t value_offset = 0;
    if (dict_lookup(p, offset, key, &value_offset)) { return read_data(p, value_offset, out_data, out_capacity, out_len); }

    for (uint64_t i = 0; i < count && *budget > 0; i++) {
      size_t child = 0;
      if (container_ref(p, refs, count + i, &child) && find_data_recursive(p, child, key, out_data, out_capacity, out_len, depth + 1, budget)) {
        return true;
      }
    }
  } else if (container(p, offset, BPLIST_ARRAY, &count, &refs)) {
    for (uint64_t i = 0; i < count && *budget > 0; i++) {
      size_t child = 0;
      if (container_ref(p, refs, i, &child) && find_data_recursive(p, child, key, out_data, out_capacity, out_len, depth + 1, budget)) {
        return true;
      }
    }
  }

  return false;
}

bool bplist_find_data(const uint8_t* plist, size_t plist_len, const char* key, uint8_t* out_data, size_t out_capacity, size_t* out_len) {
  bplist_t p;
  size_t   value_offset = 0;
  if (!bplist_open(&p, plist, plist_len) || !top_dict_lookup(&p, key, &value_offset)) { return false; }
  return read_data(&p, value_offset, out_data, out_capacity, out_len);
}

bool bplist_find_data_deep(const uint8_t* plist, size_t plist_len, const char* key, uint8_t* out_data, size_t out_capacity, size_t* out_len) {
  bplist_t p;
  size_t   top = 0;
  if (!bplist_open(&p, plist, plist_len) || !object_offset(&p, p.top_object, &top)) { return false; }
  int budget = MAX_VISITS;
  return find_data_recursive(&p, top, key, out_data, out_capacity, out_len, 0, &budget);
}

bool bplist_find_int(const uint8_t* plist, size_t plist_len, const char* key, int64_t* out_value) {
  bplist_t p;
  size_t   value_offset = 0;
  if (!bplist_open(&p, plist, plist_len) || !top_dict_lookup(&p, key, &value_offset)) { return false; }
  return read_int(&p, value_offset, out_value);
}

bool bplist_find_real(const uint8_t* plist, size_t plist_len, const char* key, double* out_value) {
  bplist_t p;
  size_t   value_offset = 0;
  if (!bplist_open(&p, plist, plist_len) || !top_dict_lookup(&p, key, &value_offset)) { return false; }

  if (read_real(&p, value_offset, out_value)) { return true; }
  int64_t int_val = 0;
  if (read_int(&p, value_offset, &int_val)) {
    *out_value = (double)int_val;
    return true;
  }
  return false;
}

bool bplist_get_streams_count(const uint8_t* plist, size_t plist_len, size_t* count) {
  if (!count) { return false; }
  *count = 0;

  bplist_t p;
  uint64_t stream_count = 0;
  size_t   refs = 0;
  if (!bplist_open(&p, plist, plist_len) || !streams_array(&p, &stream_count, &refs)) { return false; }

  *count = (size_t)stream_count;
  return true;
}

bool bplist_get_stream_info(const uint8_t* plist, size_t plist_len, size_t index, int64_t* type, uint16_t* control_port, size_t* ekey_len, size_t* eiv_len, size_t* shk_len) {
  if (!type) { return false; }

  *type = -1;
  if (control_port) { *control_port = 0; }
  if (ekey_len) { *ekey_len = 0; }
  if (eiv_len) { *eiv_len = 0; }
  if (shk_len) { *shk_len = 0; }

  bplist_t p;
  size_t   stream = 0;
  uint64_t count = 0;
  size_t   refs = 0;
  if (!bplist_open(&p, plist, plist_len) || !stream_dict(&p, index, &stream, &count, &refs)) { return false; }

  for (uint64_t i = 0; i < count; i++) {
    size_t key_offset = 0;
    size_t value_offset = 0;
    char   key[32];
    if (!container_ref(&p, refs, i, &key_offset) || !read_string(&p, key_offset, key, sizeof(key))) { continue; }
    if (!container_ref(&p, refs, count + i, &value_offset)) { continue; }

    if (strcmp(key, "type") == 0) {
      int64_t type_val = 0;
      if (read_int(&p, value_offset, &type_val)) { *type = type_val; }
    } else if (strcmp(key, "controlPort") == 0 && control_port) {
      int64_t port_val = 0;
      if (read_int(&p, value_offset, &port_val)) { *control_port = (uint16_t)port_val; }
    } else if (strcmp(key, "ekey") == 0 && ekey_len) {
      read_data(&p, value_offset, NULL, 0, ekey_len);
    } else if (strcmp(key, "eiv") == 0 && eiv_len) {
      read_data(&p, value_offset, NULL, 0, eiv_len);
    } else if (strcmp(key, "shk") == 0 && shk_len) {
      read_data(&p, value_offset, NULL, 0, shk_len);
    }
  }

  return (*type != -1);
}

bool bplist_get_stream_kv_info(const uint8_t* plist, size_t plist_len, size_t index, bplist_kv_info_t* out, size_t out_capacity, size_t* out_count) {
  if (!out || out_capacity == 0 || !out_count) { return false; }
  *out_count = 0;

  bplist_t p;
  size_t   stream = 0;
  uint64_t count = 0;
  size_t   refs = 0;
  if (!bplist_open(&p, plist, plist_len) || !stream_dict(&p, index, &stream, &count, &refs)) { return false; }

  for (uint64_t i = 0; i < count && *out_count < out_capacity; i++) {
    size_t key_offset = 0;
    size_t value_offset = 0;
    char   key[64];
    if (!container_ref(&p, refs, i, &key_offset) || !read_string(&p, key_offset, key, sizeof(key))) { continue; }
    if (!container_ref(&p, refs, count + i, &value_offset)) { continue; }

    bplist_kv_info_t* info = &out[*out_count];
    memset(info, 0, sizeof(*info));
    strncpy(info->key, key, sizeof(info->key) - 1);

    uint8_t value_type = p.data[value_offset] & 0xF0;
    if (value_type == BPLIST_INT) {
      info->value_type = BPLIST_VALUE_INT;
      int64_t int_val = 0;
      if (read_int(&p, value_offset, &int_val)) { info->int_value = int_val; }
    } else if (value_type == BPLIST_DATA) {
      info->value_type = BPLIST_VALUE_DATA;
      size_t len = 0;
      if (read_data(&p, value_offset, NULL, 0, &len)) { info->value_len = len; }
    } else if (value_type == BPLIST_STRING || value_type == BPLIST_UNICODE) {
      info->value_type = BPLIST_VALUE_STRING;
      size_t len = 0;
      if (read_string_length(&p, value_offset, &len)) { info->value_len = len; }
    } else if (value_type == BPLIST_UID) {
      info->value_type = BPLIST_VALUE_UID;
    } else if (value_type == BPLIST_ARRAY) {
      info->value_type = BPLIST_VALUE_ARRAY;
    } else if (value_type == BPLIST_DICT) {
      info->value_type = BPLIST_VALUE_DICT;
    }

    (*out_count)++;
  }

  return (*out_count > 0);
}

// Copy a data value from the stream dict, or else the top dict, truncated to capacity
static bool copy_stream_data(const bplist_t* p, size_t stream, const char* key, uint8_t* out, size_t capacity, size_t* out_len) {
  size_t value_offset = 0;
  if (!dict_lookup(p, stream, key, &value_offset) && !top_dict_lookup(p, key, &value_offset)) { return false; }

  size_t length = 0;
  if (!read_data(p, value_offset, NULL, 0, &length)) { return false; }

  // Data starts after the marker (and the length int for 15 bytes or more)
  uint8_t  type = 0;
  uint64_t count = 0;
  size_t   contents = 0;
  object_header(p, value_offset, &type, &count, &contents);

  size_t copy_len = length < capacity ? length : capacity;
  memcpy(out, p->data + contents, copy_len);
  *out_len = copy_len;
  return true;
}

bool bplist_find_stream_crypto(
  const uint8_t* plist,
  size_t         plist_len,
  int64_t        stream_type,
  uint8_t*       ekey,
  size_t         ekey_capacity,
  size_t*        ekey_len,
  uint8_t*       eiv,
  size_t         eiv_capacity,
  size_t*        eiv_len,
  uint8_t*       shk,
  size_t         shk_capacity,
  size_t*        shk_len) {
  if (ekey_len) { *ekey_len = 0; }
  if (eiv_len) { *eiv_len = 0; }
  if (shk_len) { *shk_len = 0; }

  bplist_t p;
  uint64_t stream_count = 0;
  size_t   refs = 0;
  if (!bplist_open(&p, plist, plist_len) || !streams_array(&p, &stream_count, &refs)) { return false; }

  for (uint64_t i = 0; i < stream_count; i++) {
    size_t   stream = 0;
    uint64_t count = 0;
    size_t   stream_refs = 0;
    size_t   type_offset = 0;
    int64_t  type = -1;
    if (!container_ref(&p, refs, i, &stream) || !container(&p, stream, BPLIST_DICT, &count, &stream_refs)) { continue; }
    if (!dict_lookup(&p, stream, "type", &type_offset) || !read_int(&p, type_offset, &type) || type != stream_type) { continue; }

    bool found = false;
    if (ekey && ekey_len) { found |= copy_stream_data(&p, stream, "ekey", ekey, ekey_capacity, ekey_len); }
    if (eiv && eiv_len) { found |= copy_stream_data(&p, stream, "eiv", eiv, eiv_capacity, eiv_len); }
    if (shk && shk_len) { found |= copy_stream_data(&p, stream, "shk", shk, shk_capacity, shk_len); }
    return found;
  }

  return false;
}
