#include "audio_stats.h"

#include <stdlib.h>
#include <string.h>

#include "audio_output.h"
#include "audio_receiver.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "wifi.h"

static const char* TAG = "stats";

#define STATS_PERIOD_MS 10000
#define HEALTH_EVERY    6 // Stats periods between health lines (1 minute)
#define LOW_STACK_TASKS 4 // Tasks with the least free stack listed in the health line

// Counters restart from zero on a new stream: then the delta is the new count
static uint32_t counter_delta(uint32_t now, uint32_t before) { return now >= before ? now - before : now; }

// Once a minute, playing or not: memory (free, lowest ever, largest block), CPU load per
// core, task count and the tasks closest to a stack overflow. A leak shows as free and
// lowest memory falling in the same state over time, fragmentation as a shrinking
// largest block, a task that doesn't exit as a growing task count.
static void log_health(void) {
  static configRUN_TIME_COUNTER_TYPE previous_total = 0;
  static configRUN_TIME_COUNTER_TYPE previous_idle[2] = {0};

  UBaseType_t   capacity = uxTaskGetNumberOfTasks() + 4;
  TaskStatus_t* tasks = malloc(capacity * sizeof(TaskStatus_t));
  if (!tasks) { return; }

  configRUN_TIME_COUNTER_TYPE total = 0;
  UBaseType_t                 count = uxTaskGetSystemState(tasks, capacity, &total);

  // CPU load from how much of the time each core's idle task ran
  configRUN_TIME_COUNTER_TYPE idle[2] = {0};
  for (UBaseType_t i = 0; i < count; i++) {
    const char* name = tasks[i].pcTaskName;
    if (strncmp(name, "IDLE", 4) == 0 && (name[4] == '0' || name[4] == '1')) { idle[name[4] - '0'] = tasks[i].ulRunTimeCounter; }
  }
  configRUN_TIME_COUNTER_TYPE elapsed = total - previous_total;
  int                         load[2] = {0, 0};
  for (int core = 0; core < 2; core++) {
    if (elapsed > 0 && previous_total > 0) {
      uint64_t idle_part = (uint64_t)(configRUN_TIME_COUNTER_TYPE)(idle[core] - previous_idle[core]) * 100 / elapsed;
      load[core] = idle_part > 100 ? 0 : 100 - (int)idle_part;
    }
    previous_idle[core] = idle[core];
  }
  previous_total = total;

  // Tasks with the least stack left (high water mark, bytes)
  char   low_stack[160] = "";
  size_t used = 0;
  for (int listed = 0; listed < LOW_STACK_TASKS && listed < (int)count; listed++) {
    UBaseType_t lowest = 0;
    for (UBaseType_t i = 1; i < count; i++) {
      if (tasks[i].usStackHighWaterMark < tasks[lowest].usStackHighWaterMark) { lowest = i; }
    }
    used += (size_t)snprintf(
      low_stack + used, sizeof(low_stack) - used, "%s%s %u", listed ? ", " : "", tasks[lowest].pcTaskName, (unsigned)tasks[lowest].usStackHighWaterMark);
    if (used >= sizeof(low_stack)) { break; }
    tasks[lowest].usStackHighWaterMark = (configSTACK_DEPTH_TYPE)~0; // Exclude from the next round
  }
  free(tasks);

  ESP_LOGI(
    TAG,
    "health up=%llus | psram %uK (min %uK, block %uK) | internal %uK (min %uK, block %uK) | cpu %d%%/%d%% | tasks %u | stack left: %s",
    (unsigned long long)(esp_timer_get_time() / 1000000),
    (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024),
    (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM) / 1024),
    (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024),
    (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024),
    (unsigned)(heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL) / 1024),
    (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024),
    load[0],
    load[1],
    (unsigned)count,
    low_stack);
}

static void stats_task(void* arg) {
  // Previous snapshot for delta calculation
  uint32_t      prev_frames = 0, prev_stalls = 0;
  audio_stats_t prev_recv = {0};
  uint32_t      prev_ok = 0, prev_sil = 0, prev_gap = 0, prev_resync = 0, prev_recovered = 0;
  int           periods = 0;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(STATS_PERIOD_MS));

    if (++periods % HEALTH_EVERY == 0) { log_health(); }

    // Playback; a stall is the buffer running dry while playing (an audible gap)
    uint32_t frames = 0;
    bool     active = false;
    audio_output_get_playback_stats(&frames, &active);
    uint32_t stalls = audio_receiver_get_stalls();

    // Only print when there's been activity
    if (!active && frames == prev_frames) {
      prev_stalls = stalls;
      continue;
    }

    // Receiver pipeline
    audio_stats_t recv;
    audio_receiver_get_stats(&recv);
    int pcm_ms = audio_receiver_get_buffered_ms();

    // Reorder buffer
    uint32_t ok = 0, sil = 0, gap = 0, resync = 0, recovered = 0;
    int      reorder_depth = 0;
    audio_realtime_get_reorder_stats(&ok, &sil, &gap, &resync, &recovered, &reorder_depth);

    // Phone signal: above -60 dBm is good, below -75 dBm expect loss
    int rssi = 0;
    if (!wifi_get_station_rssi(&rssi)) { rssi = 0; }

    ESP_LOGI(TAG, "%s play=+%lu stall=+%lu | recv=+%lu dec=+%lu drop=+%lu late=+%lu | ok=+%lu sil=+%lu gap=+%lu resync=+%lu re=+%lu depth=%d | pcm=%dms | rssi=%ddBm | ps=%uK i=%uK",
             active ? "PLAY" : "STOP",
             (unsigned long)counter_delta(frames, prev_frames), (unsigned long)counter_delta(stalls, prev_stalls),
             (unsigned long)counter_delta(recv.packets_received, prev_recv.packets_received),
             (unsigned long)counter_delta(recv.packets_decoded, prev_recv.packets_decoded),
             (unsigned long)counter_delta(recv.packets_dropped, prev_recv.packets_dropped),
             (unsigned long)counter_delta(recv.late_frames, prev_recv.late_frames),
             (unsigned long)counter_delta(ok, prev_ok), (unsigned long)counter_delta(sil, prev_sil),
             (unsigned long)counter_delta(gap, prev_gap), (unsigned long)counter_delta(resync, prev_resync),
             (unsigned long)counter_delta(recovered, prev_recovered), reorder_depth,
             pcm_ms, rssi,
             (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024), (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024));

    // Save snapshot
    prev_frames = frames;
    prev_stalls = stalls;
    prev_recv = recv;
    prev_ok = ok;
    prev_sil = sil;
    prev_gap = gap;
    prev_resync = resync;
    prev_recovered = recovered;
  }
}

void audio_stats_start(void) {
  xTaskCreatePinnedToCore(stats_task, "stats", 4096, NULL, 2, NULL, 0);
}
