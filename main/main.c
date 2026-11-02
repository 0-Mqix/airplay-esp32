#include "audio_output.h"
#include "audio_receiver.h"
#include "audio_stats.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hap.h"
#include "mdns_airplay.h"
#include "nvs_flash.h"
#include "rtsp_server.h"
#include "wifi.h"

static const char* TAG = "main";

// Why the board (re)started: a crash, watchdog or brownout shows up here after the fact
static void log_reset_reason(void) {
  esp_reset_reason_t reason = esp_reset_reason();
  const char*        name = "other";
  bool               abnormal = true;
  switch (reason) {
    case ESP_RST_POWERON:
      name = "power on";
      abnormal = false;
      break;
    case ESP_RST_SW:
      name = "software restart";
      abnormal = false;
      break;
    case ESP_RST_USB:
      name = "USB (flashing)";
      abnormal = false;
      break;
    case ESP_RST_EXT: name = "reset pin"; break;
    case ESP_RST_PANIC: name = "crash (panic)"; break;
    case ESP_RST_INT_WDT: name = "interrupt watchdog"; break;
    case ESP_RST_TASK_WDT: name = "task watchdog"; break;
    case ESP_RST_WDT: name = "watchdog"; break;
    case ESP_RST_BROWNOUT: name = "brownout (power dip)"; break;
    default: break;
  }
  if (abnormal) {
    ESP_LOGW(TAG, "Reset reason: %s (%d)", name, (int)reason);
  } else {
    ESP_LOGI(TAG, "Reset reason: %s", name);
  }
}

void app_main(void) {
  log_reset_reason();

  // Initialize NVS (needed for WiFi driver)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Start WiFi AP
  wifi_init_ap();

  // Start AirPlay services
  ESP_LOGI(TAG, "Starting AirPlay services...");

  ESP_ERROR_CHECK(hap_init());
  ESP_ERROR_CHECK(audio_receiver_init());
  ESP_ERROR_CHECK(audio_output_init());
  audio_output_start();
  audio_stats_start();
  mdns_airplay_init();
  ESP_ERROR_CHECK(rtsp_server_start());

  char ip_str[16];
  wifi_get_ip_str(ip_str, sizeof(ip_str));
  ESP_LOGI(TAG, "AirPlay ready at %s", ip_str);

  while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
