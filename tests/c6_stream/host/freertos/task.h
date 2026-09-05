#pragma once
#include "FreeRTOS.h"
struct HostTask;
using TaskHandle_t = HostTask *;
BaseType_t xTaskCreate(void (*entry)(void *), const char *, uint32_t, void *argument, UBaseType_t,
                       TaskHandle_t *handle);
void vTaskDelete(TaskHandle_t task);
void vTaskSuspend(TaskHandle_t task);
void xTaskNotifyGive(TaskHandle_t task);
void vTaskNotifyGiveFromISR(TaskHandle_t task, BaseType_t *woken);
uint32_t ulTaskNotifyTake(BaseType_t clear, TickType_t ticks);
TaskHandle_t xTaskGetCurrentTaskHandle();
void vTaskDelay(TickType_t ticks);
