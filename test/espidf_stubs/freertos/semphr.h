#pragma once
#include "FreeRTOS.h"
using SemaphoreHandle_t = int;
inline SemaphoreHandle_t xSemaphoreCreateMutex() { return 1; }
inline int xSemaphoreTake(SemaphoreHandle_t, TickType_t) { return pdTRUE; }
inline int xSemaphoreGive(SemaphoreHandle_t) { return pdTRUE; }
