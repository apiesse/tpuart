#pragma once
#include "FreeRTOS.h"
using QueueHandle_t = void *;
unsigned uxQueueSpacesAvailable(QueueHandle_t queue);
int xQueueReceive(QueueHandle_t queue, void *event, TickType_t wait);
void xQueueReset(QueueHandle_t queue);
