#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

esp_err_t audio_output_init(void);
void      audio_output_start(void);
void      audio_output_get_playback_stats(uint32_t* frames, uint32_t* underruns, bool* active);
