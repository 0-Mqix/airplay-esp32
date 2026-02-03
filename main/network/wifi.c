#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "config.h"
#include "wifi.h"

static const char *TAG = "wifi";

static esp_netif_t *s_ap_netif = NULL;
static bool s_wifi_initialized = false;

// AP settings
#define AP_SSID       CONFIG_DEVICE_NAME
#define AP_PASS       CONFIG_WIFI_PASSWORD
#define AP_CHANNEL    6
#define AP_MAX_CONN   4

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
    case WIFI_EVENT_AP_START:
      ESP_LOGI(TAG, "AP started");
      break;
    case WIFI_EVENT_AP_STOP:
      ESP_LOGI(TAG, "AP stopped");
      break;
    case WIFI_EVENT_AP_STACONNECTED: {
      wifi_event_ap_staconnected_t *event =
          (wifi_event_ap_staconnected_t *)event_data;
      ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d",
               MAC2STR(event->mac), event->aid);
      break;
    }
    case WIFI_EVENT_AP_STADISCONNECTED: {
      wifi_event_ap_stadisconnected_t *event =
          (wifi_event_ap_stadisconnected_t *)event_data;
      ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d",
               MAC2STR(event->mac), event->aid);
      break;
    }
    }
  }
}

void wifi_init_ap(void) {
  if (s_wifi_initialized) {
    return;
  }

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Create AP netif with custom DHCP settings (no gateway = iOS won't expect internet)
  esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();

  esp_netif_ip_info_t ip_info = {
      .ip = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
      .gw = {.addr = ESP_IP4TOADDR(0, 0, 0, 0)},  // No gateway - tells iOS no internet
      .netmask = {.addr = ESP_IP4TOADDR(255, 255, 255, 0)},
  };
  base_cfg.ip_info = &ip_info;

  esp_netif_config_t cfg = {
      .base = &base_cfg,
      .driver = NULL,
      .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_AP,
  };

  s_ap_netif = esp_netif_new(&cfg);
  assert(s_ap_netif);
  esp_netif_attach_wifi_ap(s_ap_netif);
  esp_wifi_set_default_wifi_ap_handlers();

  wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

  wifi_config_t wifi_config = {
      .ap =
          {
              .ssid = AP_SSID,
              .ssid_len = strlen(AP_SSID),
              .channel = AP_CHANNEL,
              .password = AP_PASS,
              .max_connection = AP_MAX_CONN,
              .authmode = WIFI_AUTH_WPA2_PSK,
              .pmf_cfg =
                  {
                      .required = false,
                  },
          },
  };

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));

  // Set 40MHz bandwidth for better throughput (audio streaming)
  ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT40));

  // Set 802.11n mode for better performance
  ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));

  ESP_ERROR_CHECK(esp_wifi_start());

  // Disable power saving for low latency audio
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  // TX power (8.5 dBm = 34 quarter-dBm units) - short range only
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(34));

  s_wifi_initialized = true;

  ESP_LOGI(TAG, "WiFi AP started. SSID: %s, Channel: %d, BW: 40MHz", AP_SSID, AP_CHANNEL);
}

void wifi_get_mac_str(char *mac_str, size_t len) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  snprintf(mac_str, len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1],
           mac[2], mac[3], mac[4], mac[5]);
}

esp_err_t wifi_get_ip_str(char *ip_str, size_t len) {
  if (!s_ap_netif || !ip_str || len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_netif_ip_info_t ip_info;
  esp_err_t err = esp_netif_get_ip_info(s_ap_netif, &ip_info);
  if (err == ESP_OK) {
    snprintf(ip_str, len, IPSTR, IP2STR(&ip_info.ip));
  }
  return err;
}

void wifi_stop(void) {
  if (s_wifi_initialized) {
    esp_wifi_stop();
    esp_wifi_deinit();
    s_wifi_initialized = false;
  }
}
