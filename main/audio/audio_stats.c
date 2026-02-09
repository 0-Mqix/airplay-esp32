#include "audio_stats.h"

#include "audio_output.h"
#include "audio_receiver.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "stats";

static void stats_task(void* arg) {
  // Previous snapshot for delta calculation
  uint32_t      prev_frames = 0, prev_underruns = 0;
  audio_stats_t prev_recv = {0};
  uint32_t      prev_ok = 0, prev_sil = 0, prev_gap = 0, prev_fdrain = 0, prev_recovered = 0;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(10000));

    // Playback
    uint32_t frames = 0, underruns = 0;
    bool     active = false;
    audio_output_get_playback_stats(&frames, &underruns, &active);

    // Only print when there's been activity
    if (!active && frames == 0 && prev_frames == 0) { continue; }

    // Receiver pipeline
    audio_stats_t recv;
    audio_receiver_get_stats(&recv);
    int pcm_buf = audio_receiver_get_buffered_frames();

    // Reorder buffer
    uint32_t ok = 0, sil = 0, gap = 0, fdrain = 0, recovered = 0;
    int      reorder_depth = 0;
    audio_realtime_get_reorder_stats(&ok, &sil, &gap, &fdrain, &recovered, &reorder_depth);

    // Deltas (handle counter reset on playout start)
    uint32_t d_frames = frames >= prev_frames ? frames - prev_frames : frames;
    uint32_t d_underruns = underruns >= prev_underruns ? underruns - prev_underruns : underruns;
    uint32_t d_recv = recv.packets_received - prev_recv.packets_received;
    uint32_t d_dec = recv.packets_decoded - prev_recv.packets_decoded;
    uint32_t d_drop = recv.packets_dropped - prev_recv.packets_dropped;
    uint32_t d_late = recv.late_frames - prev_recv.late_frames;
    uint32_t d_ok = ok - prev_ok;
    uint32_t d_sil = sil - prev_sil;
    uint32_t d_gap = gap - prev_gap;
    uint32_t d_fdrain = fdrain - prev_fdrain;
    uint32_t d_recovered = recovered - prev_recovered;

    // Memory
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    size_t iram_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);

    ESP_LOGI(TAG, "%s play=+%lu ur=+%lu | recv=+%lu dec=+%lu drop=+%lu late=+%lu | ok=+%lu sil=+%lu gap=+%lu fd=+%lu re=+%lu depth=%d | pcm=%d | ps=%uK i=%uK",
             active ? "PLAY" : "STOP",
             (unsigned long)d_frames, (unsigned long)d_underruns,
             (unsigned long)d_recv, (unsigned long)d_dec,
             (unsigned long)d_drop, (unsigned long)d_late,
             (unsigned long)d_ok, (unsigned long)d_sil, (unsigned long)d_gap, (unsigned long)d_fdrain, (unsigned long)d_recovered, reorder_depth,
             pcm_buf,
             (unsigned)(psram_free / 1024), (unsigned)(iram_free / 1024));

    // Save snapshot
    prev_frames = frames;
    prev_underruns = underruns;
    prev_recv = recv;
    prev_ok = ok;
    prev_sil = sil;
    prev_gap = gap;
    prev_fdrain = fdrain;
    prev_recovered = recovered;
  }
}

void audio_stats_start(void) {
  xTaskCreatePinnedToCore(stats_task, "stats", 3072, NULL, 2, NULL, 0);
}
