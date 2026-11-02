#pragma once

// Host stand-in for the FreeRTOS pieces the audio code uses. The simulator is single
// threaded, so critical sections are no-ops and nothing ever blocks.

#include <stdint.h>

typedef int      BaseType_t;
typedef unsigned UBaseType_t;
typedef uint32_t TickType_t;

#define pdTRUE  1
#define pdFALSE 0
#define pdPASS  1

#define portMAX_DELAY        0xFFFFFFFFu
#define pdMS_TO_TICKS(ms)    ((TickType_t)(ms))
#define configMAX_PRIORITIES 25

typedef struct {
  int unused;
} portMUX_TYPE;

#define portMUX_INITIALIZER_UNLOCKED {0}
#define portENTER_CRITICAL(mux)      ((void)(mux))
#define portEXIT_CRITICAL(mux)       ((void)(mux))
