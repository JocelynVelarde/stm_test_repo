#include "esc.h"
#include <stdio.h>
#include <string.h>

extern TIM_HandleTypeDef htim2;

#define ESC_PERIOD_US   20000.0f
#define ESC_MIN_US      1000.0f
#define ESC_NEUTRAL_US  1500.0f
#define ESC_MAX_US      2000.0f

static inline uint32_t us_to_ccr(TIM_HandleTypeDef *htim, float us)
{
    if (us < ESC_MIN_US) us = ESC_MIN_US;
    if (us > ESC_MAX_US) us = ESC_MAX_US;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    float ticks = (us / ESC_PERIOD_US) * (float)(arr + 1U);
    if (ticks < 0.0f) ticks = 0.0f;
    if (ticks > (float)(arr + 1U)) ticks = (float)(arr + 1U);
    return (uint32_t)(ticks + 0.5f);
}

void setEscSpeed_us(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, us_to_ccr(&htim2, (float)pulse_us));
}

void stopCarEsc(void)
{
    setEscSpeed_us(1500);
    HAL_Delay(1000);
}
