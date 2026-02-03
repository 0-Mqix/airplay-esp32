#pragma once

#include "esp_err.h"

/**
 * Initialize I2S audio output
 */
esp_err_t audio_output_init(void);

/**
 * Start the audio playback task
 */
void audio_output_start(void);

/**
 * Flush the I2S DMA buffer (call on skip/seek)
 */
void audio_output_flush(void);
