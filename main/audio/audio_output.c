#include "audio_output.h"

#include <stdlib.h>

#include "audio_receiver.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

#define TAG          "audio_output"
#define I2S_BCK_PIN  CONFIG_I2S_BCK_PIN
#define I2S_LRCK_PIN CONFIG_I2S_LRCK_PIN
#define I2S_DOUT_PIN CONFIG_I2S_DATA_PIN
#define SAMPLE_RATE  44100
// Small reads keep decode pressure low; DMA depth handles jitter
#define FRAME_SAMPLES 512

static i2s_chan_handle_t tx_handle;
static TaskHandle_t      playback_task_handle = NULL;

// Playback stats — reset on each playout start
static volatile uint32_t play_frames = 0;
static volatile uint32_t play_underruns = 0;
static volatile bool     play_active = false;

void audio_output_get_playback_stats(uint32_t* frames, uint32_t* underruns, bool* active) {
  if (frames) { *frames = play_frames; }
  if (underruns) { *underruns = play_underruns; }
  if (active) { *active = play_active; }
}

// Fixed -12 dB attenuation: >>2 = quarter power, ~0.53V RMS into AUX
static void apply_volume(int16_t* buf, size_t n) {
  for (size_t i = 0; i < n; i++) { buf[i] >>= 2; }
}

static void playback_task(void* arg) {
  // Use DMA-capable internal RAM for lowest latency transfers
  size_t   pcm_size = (size_t)FRAME_SAMPLES * 2 * sizeof(int16_t);
  int16_t* pcm = heap_caps_malloc(pcm_size, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  int16_t* silence = heap_caps_calloc(FRAME_SAMPLES * 2, sizeof(int16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
  if (!pcm || !silence) {
    ESP_LOGE(TAG, "Failed to allocate DMA buffers");
    heap_caps_free(pcm);
    heap_caps_free(silence);
    vTaskDelete(NULL);
    return;
  }

  size_t written;
  while (true) {
    size_t samples = audio_receiver_read(pcm, FRAME_SAMPLES);

    if (samples > 0) {
      if (!play_active) {
        play_frames = 0;
        play_underruns = 0;
        play_active = true;
      }
      apply_volume(pcm, samples * 2);
      i2s_channel_write(tx_handle, pcm, samples * 4, &written, portMAX_DELAY);
      play_frames++;
    } else {
      play_active = false;
      play_underruns++;
      i2s_channel_write(tx_handle, silence, (size_t)FRAME_SAMPLES * 4, &written, portMAX_DELAY);
      taskYIELD();
    }
  }
}

esp_err_t audio_output_init(void) {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // DMA buffer: 8 descriptors * 1024 frames = ~186ms at 44.1kHz stereo
  // Larger descriptors = longer i2s_channel_write sleep = more CPU for decode on Core 1
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = 1024;

  ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &tx_handle, NULL), TAG, "channel create failed");

  i2s_std_config_t std_cfg = {
    .clk_cfg =
      {
        .sample_rate_hz = SAMPLE_RATE,
        .clk_src = I2S_CLK_SRC_DEFAULT, // PLL_160M on ESP32-S3
        .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      },
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
    .gpio_cfg =
      {
        .mclk = I2S_GPIO_UNUSED,
        .bclk = I2S_BCK_PIN,
        .ws = I2S_LRCK_PIN,
        .dout = I2S_DOUT_PIN,
        .din = I2S_GPIO_UNUSED,
      },
  };

  ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(tx_handle, &std_cfg), TAG, "std mode init failed");
  ESP_RETURN_ON_ERROR(i2s_channel_enable(tx_handle), TAG, "channel enable failed");

  return ESP_OK;
}

void audio_output_start(void) {
  xTaskCreatePinnedToCore(playback_task, "audio_play", TASK_PLAYBACK_STACK, NULL, TASK_PLAYBACK_PRIORITY, &playback_task_handle, TASK_PLAYBACK_CORE);
}
