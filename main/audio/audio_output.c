#include "audio_output.h"

#include <stdlib.h>

#include "audio_receiver.h"
#include "config.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rtsp_server.h"
#include "soc/soc_caps.h"

#define TAG          "audio_output"
#define I2S_BCK_PIN  CONFIG_I2S_BCK_PIN
#define I2S_LRCK_PIN CONFIG_I2S_LRCK_PIN
#define I2S_DOUT_PIN CONFIG_I2S_DATA_PIN
#define SAMPLE_RATE  44100
// Read multiple frames at once to reduce task wake-ups and improve efficiency
#define FRAME_SAMPLES 512

// Flush notification bit for task notification
#define FLUSH_NOTIFY_BIT (1 << 0)

static i2s_chan_handle_t tx_handle;
static TaskHandle_t      playback_task_handle = NULL;
static SemaphoreHandle_t flush_complete_sem = NULL;
static volatile bool     flush_pending = false;

void audio_output_flush(void) {
  if (!playback_task_handle) { return; }

  // Signal flush and wait for playback task to acknowledge
  flush_pending = true;
  xTaskNotify(playback_task_handle, FLUSH_NOTIFY_BIT, eSetBits);

  // Wait for flush to complete (max 100ms to avoid blocking forever)
  if (flush_complete_sem) { xSemaphoreTake(flush_complete_sem, pdMS_TO_TICKS(100)); }
}

static void apply_volume(int16_t* buf, size_t n) {
  int32_t vol = airplay_get_volume_q15();
  for (size_t i = 0; i < n; i++) { buf[i] = (int16_t)(((int32_t)buf[i] * vol) >> 15); }
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

  size_t   written;
  uint32_t notify_value;
  while (true) {
    // Check for flush notification (skip/seek) - wait briefly to yield CPU
    if (xTaskNotifyWait(0, FLUSH_NOTIFY_BIT, &notify_value, pdMS_TO_TICKS(1)) == pdTRUE) {
      if (notify_value & FLUSH_NOTIFY_BIT) {
        // Disable I2S to immediately stop DMA and clear buffers
        i2s_channel_disable(tx_handle);
        // Small delay to ensure DMA is fully stopped
        vTaskDelay(pdMS_TO_TICKS(5));
        i2s_channel_enable(tx_handle);
        // Clear the pending flag and signal completion
        flush_pending = false;
        if (flush_complete_sem) { xSemaphoreGive(flush_complete_sem); }
        continue;
      }
    }

    // Double-check flush wasn't requested between notification check and read
    if (flush_pending) {
      vTaskDelay(1);
      continue;
    }

    size_t samples = audio_receiver_read(pcm, FRAME_SAMPLES);

    // Check again after read - if flush happened, discard the data
    if (flush_pending) { continue; }

    if (samples > 0) {
      apply_volume(pcm, samples * 2);
      // I2S write blocks until DMA has space
      i2s_channel_write(tx_handle, pcm, samples * 4, &written, portMAX_DELAY);
    } else {
      // No data available - write silence to keep I2S clock running
      // portMAX_DELAY ensures we block until DMA has space, yielding CPU
      i2s_channel_write(tx_handle, silence, (size_t)FRAME_SAMPLES * 4, &written, portMAX_DELAY);
      // Extra yield when idle to ensure watchdog doesn't trigger
      taskYIELD();
    }
  }
}

esp_err_t audio_output_init(void) {
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  // DMA buffer: 8 descriptors * 512 frames = ~93ms at 44.1kHz stereo
  chan_cfg.dma_desc_num = 8;
  chan_cfg.dma_frame_num = 512;

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
  // Create binary semaphore for flush synchronization
  if (!flush_complete_sem) { flush_complete_sem = xSemaphoreCreateBinary(); }

  xTaskCreatePinnedToCore(playback_task, "audio_play", TASK_PLAYBACK_STACK, NULL, TASK_PLAYBACK_PRIORITY, &playback_task_handle, TASK_PLAYBACK_CORE);
}
