#pragma once

#include <stdint.h>

// Reorder stats (from audio_stream_realtime.c)
void audio_realtime_get_reorder_stats(uint32_t* ok, uint32_t* sil, uint32_t* gap, uint32_t* fdrain, uint32_t* recovered, int* buf_depth);

// Start the stats monitor task (Core 0, low priority, prints every 10s)
void audio_stats_start(void);
