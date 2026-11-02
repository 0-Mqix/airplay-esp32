#include "wifi.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"

static const char* TAG = "wifi";

static esp_netif_t* s_ap_netif = NULL;
static bool         s_wifi_initialized = false;

// AP settings
#define AP_SSID     CONFIG_DEVICE_NAME
#define AP_PASS     CONFIG_WIFI_PASSWORD
#define AP_CHANNEL  CONFIG_WIFI_CHANNEL
#define AP_MAX_CONN CONFIG_WIFI_MAX_CONN

// Channel selection: the three channels that don't overlap on 2.4 GHz
static const uint8_t candidate_channels[] = {1, 6, 11};
#define CANDIDATE_COUNT  (sizeof(candidate_channels) / sizeof(candidate_channels[0]))
#define MAX_SCAN_RECORDS 64

// Interference a network on `channel` causes on `candidate`: its received power,
// weighted by how much the two 20 MHz channels overlap (5 channels apart: none)
static float channel_interference(int channel, int rssi, int candidate) {
  int distance = abs(channel - candidate);
  if (distance >= 5) { return 0; }
  return powf(10.0f, (float)rssi / 10.0f) * (float)(5 - distance) / 5.0f;
}

// Scan the surroundings and return the least busy channel. Runs before the AP starts:
// around a car the networks change every trip (other cars, roadside, the car's own).
static uint8_t pick_channel(void) {
  uint8_t best = candidate_channels[0];

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());

  wifi_scan_config_t scan_config = {.show_hidden = true, .scan_type = WIFI_SCAN_TYPE_ACTIVE};
  if (esp_wifi_scan_start(&scan_config, true) != ESP_OK) {
    ESP_LOGW(TAG, "Channel scan failed, using channel %d", best);
    ESP_ERROR_CHECK(esp_wifi_stop());
    return best;
  }

  uint16_t count = 0;
  esp_wifi_scan_get_ap_num(&count);
  if (count > MAX_SCAN_RECORDS) { count = MAX_SCAN_RECORDS; }

  // Reading the records also frees the driver's scan list, so always read them
  wifi_ap_record_t* records = calloc(count ? count : 1, sizeof(wifi_ap_record_t));
  if (!records) {
    esp_wifi_clear_ap_list();
    ESP_ERROR_CHECK(esp_wifi_stop());
    return best;
  }
  esp_wifi_scan_get_ap_records(&count, records);

  float scores[CANDIDATE_COUNT] = {0};
  for (uint16_t i = 0; i < count; i++) {
    for (size_t c = 0; c < CANDIDATE_COUNT; c++) {
      scores[c] += channel_interference(records[i].primary, records[i].rssi, candidate_channels[c]);
      // A 40 MHz network also occupies the channel 4 above or below its primary
      if (records[i].second == WIFI_SECOND_CHAN_ABOVE) {
        scores[c] += channel_interference(records[i].primary + 4, records[i].rssi, candidate_channels[c]);
      } else if (records[i].second == WIFI_SECOND_CHAN_BELOW) {
        scores[c] += channel_interference(records[i].primary - 4, records[i].rssi, candidate_channels[c]);
      }
    }
  }
  free(records);

  size_t best_index = 0;
  for (size_t c = 1; c < CANDIDATE_COUNT; c++) {
    if (scores[c] < scores[best_index]) { best_index = c; }
  }
  best = candidate_channels[best_index];

  // Scores as dBm of interfering power, for the log
  ESP_LOGI(
    TAG,
    "Channel scan: %u networks, interference ch1 %.0f dBm, ch6 %.0f dBm, ch11 %.0f dBm -> channel %d",
    count,
    scores[0] > 0 ? 10.0f * log10f(scores[0]) : -100.0f,
    scores[1] > 0 ? 10.0f * log10f(scores[1]) : -100.0f,
    scores[2] > 0 ? 10.0f * log10f(scores[2]) : -100.0f,
    best);

  ESP_ERROR_CHECK(esp_wifi_stop());
  return best;
}

bool wifi_get_station_rssi(int* rssi) {
  wifi_sta_list_t list;
  if (!rssi || esp_wifi_ap_get_sta_list(&list) != ESP_OK || list.num == 0) { return false; }

  int strongest = -127;
  for (int i = 0; i < list.num; i++) {
    if (list.sta[i].rssi > strongest) { strongest = list.sta[i].rssi; }
  }
  *rssi = strongest;
  return true;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  if (event_base == WIFI_EVENT) {
    switch (event_id) {
      case WIFI_EVENT_AP_START: ESP_LOGI(TAG, "AP started"); break;
      case WIFI_EVENT_AP_STOP: ESP_LOGI(TAG, "AP stopped"); break;
      case WIFI_EVENT_AP_STACONNECTED: {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*)event_data;
        ESP_LOGI(TAG, "Station " MACSTR " joined, AID=%d", MAC2STR(event->mac), event->aid);
        break;
      }
      case WIFI_EVENT_AP_STADISCONNECTED: {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*)event_data;
        ESP_LOGI(TAG, "Station " MACSTR " left, AID=%d", MAC2STR(event->mac), event->aid);
        break;
      }
    }
  }
}

void wifi_init_ap(void) {
  if (s_wifi_initialized) { return; }

  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());

  // Create AP netif with custom DHCP settings (no gateway = iOS won't expect internet)
  esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();

  esp_netif_ip_info_t ip_info = {
    .ip = {.addr = ESP_IP4TOADDR(192, 168, 4, 1)},
    .gw = {.addr = ESP_IP4TOADDR(0, 0, 0, 0)}, // No gateway - tells iOS no internet
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
  // Increase AMPDU TX/RX buffer for better streaming throughput
  wifi_cfg.ampdu_tx_enable = 1;
  wifi_cfg.ampdu_rx_enable = 1;
  wifi_cfg.nvs_enable = 0; // Don't use NVS - saves flash wear
  ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

  // Store config in RAM only
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

  uint8_t channel = AP_CHANNEL > 0 ? AP_CHANNEL : pick_channel();

  wifi_config_t wifi_config = {
    .ap =
      {
        .ssid = AP_SSID,
        .ssid_len = strlen(AP_SSID),
        .channel = channel,
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

  // 20 MHz: iPhones don't use 40 MHz on 2.4 GHz, and a 40 MHz channel picks up twice the
  // interference (other cars, roadside WiFi). Audio needs ~1.5 Mbit/s at most.
  ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));

  // Set 802.11n mode for better performance
  ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));

  ESP_ERROR_CHECK(esp_wifi_start());

  // The AP drops a station after 300 s without a packet by default. A phone that is
  // paused or locked can be that quiet, and the speaker disappears from it.
  // Only accepted once the AP runs; not worth a boot loop if it fails.
  esp_err_t err = esp_wifi_set_inactive_time(WIFI_IF_AP, CONFIG_WIFI_INACTIVE_TIME);
  if (err != ESP_OK) { ESP_LOGW(TAG, "Failed to set station inactive time: %s", esp_err_to_name(err)); }

  // Disable power saving for low latency audio
  ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

  // TX power from config (quarter-dBm units)
  ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(CONFIG_WIFI_TX_POWER));

  s_wifi_initialized = true;

  ESP_LOGI(TAG, "WiFi AP started. SSID: %s, Channel: %d, BW: 20MHz, TX: %.1f dBm", AP_SSID, channel, CONFIG_WIFI_TX_POWER / 4.0f);
}

void wifi_get_mac_str(char* mac_str, size_t len) {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  snprintf(mac_str, len, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

esp_err_t wifi_get_ip_str(char* ip_str, size_t len) {
  if (!s_ap_netif || !ip_str || len == 0) { return ESP_ERR_INVALID_ARG; }

  esp_netif_ip_info_t ip_info;
  esp_err_t           err = esp_netif_get_ip_info(s_ap_netif, &ip_info);
  if (err == ESP_OK) { snprintf(ip_str, len, IPSTR, IP2STR(&ip_info.ip)); }
  return err;
}

void wifi_stop(void) {
  if (s_wifi_initialized) {
    esp_wifi_stop();
    esp_wifi_deinit();
    s_wifi_initialized = false;
  }
}
