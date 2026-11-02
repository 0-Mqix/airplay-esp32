#pragma once

// Firmware log lines go to sim_log, which counts warnings and prints them with -v
void sim_log(char level, const char* tag, const char* format, ...) __attribute__((format(printf, 3, 4)));

#define ESP_LOGE(tag, format, ...) sim_log('E', tag, format, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) sim_log('W', tag, format, ##__VA_ARGS__)
#define ESP_LOGI(tag, format, ...) sim_log('I', tag, format, ##__VA_ARGS__)
#define ESP_LOGD(tag, format, ...) sim_log('D', tag, format, ##__VA_ARGS__)
