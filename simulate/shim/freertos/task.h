#pragma once

#include "freertos/FreeRTOS.h"

// Tasks are never started: the simulator drives the stream code itself
typedef void* TaskHandle_t;
typedef void (*TaskFunction_t)(void*);

BaseType_t xTaskCreatePinnedToCore(
  TaskFunction_t function,
  const char*    name,
  uint32_t       stack,
  void*          parameter,
  UBaseType_t    priority,
  TaskHandle_t*  handle,
  BaseType_t     core);
void vTaskDelay(TickType_t ticks);
void vTaskDelete(TaskHandle_t task);

#define taskYIELD()
#define uxTaskGetStackHighWaterMark(task) ((UBaseType_t)0)
