#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
 * Initialize WiFi in AP mode.
 * Creates an open network with no gateway (iOS won't expect internet).
 */
void wifi_init_ap(void);

/**
 * Get MAC address as string (XX:XX:XX:XX:XX:XX)
 */
void wifi_get_mac_str(char *mac_str, size_t len);

/**
 * Get IP address as string
 */
esp_err_t wifi_get_ip_str(char *ip_str, size_t len);

/**
 * Stop WiFi
 */
void wifi_stop(void);
