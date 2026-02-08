#include "audio_output.h"
#include "audio_receiver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hap.h"
#include "mdns_airplay.h"
#include "nvs_flash.h"
#include "rtsp_server.h"
#include "wifi.h"

static const char* TAG = "main";

void app_main(void) {
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
  mdns_airplay_init();
  ESP_ERROR_CHECK(rtsp_server_start());

  char ip_str[16];
  wifi_get_ip_str(ip_str, sizeof(ip_str));
  ESP_LOGI(TAG, "AirPlay ready at %s", ip_str);

  while (1) { vTaskDelay(pdMS_TO_TICKS(10000)); }
}
