#pragma once

#include <stdint.h>

// Fake codec payload: timestamp (4), track (1), samples (2)
#define SIM_PAYLOAD_SIZE 7

void sim_payload_write(uint8_t* payload, uint32_t timestamp, int track, int samples);

// Decoded samples carry their source: left = timestamp bits 0-15,
// right = track (3 bits) << 12 | timestamp bits 16-27
int16_t sim_sample_left(uint32_t source);
int16_t sim_sample_right(uint32_t source, int track);
