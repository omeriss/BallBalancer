#pragma once

#include <stdint.h>
#include "stm32h5xx_hal.h"

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(x) ((x) < 0 ? -(x) : (x))

static void delay_using_timer(TIM_HandleTypeDef *htim, uint32_t delay_ms)
{
#define TIMER_DELAY_CHUNK 10000

    uint16_t loops = delay_ms / TIMER_DELAY_CHUNK;
    uint16_t remainder = delay_ms % TIMER_DELAY_CHUNK;

    for (int i = 0; i < loops; i++)
    {
        uint32_t start = __HAL_TIM_GET_COUNTER(htim); // Get the current timer value
        while ((__HAL_TIM_GET_COUNTER(htim) - start) < TIMER_DELAY_CHUNK)
        {
        }
    }

    if (remainder > 0)
    {
        uint32_t start = __HAL_TIM_GET_COUNTER(htim); // Get the current timer value
        while ((__HAL_TIM_GET_COUNTER(htim) - start) < remainder)
        {
        }
    }
}