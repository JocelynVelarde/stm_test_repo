#include "servo.h"

extern TIM_HandleTypeDef htim13;

#define SERVO_PERIOD_US   20000.0f
#define SERVO_MIN_US      1000.0f
#define SERVO_MAX_US      2000.0f

static uint32_t last_move_tick = 0U;

static inline uint32_t us_to_ccr(TIM_HandleTypeDef *htim, float us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);
    float ticks = (us / SERVO_PERIOD_US) * (float)(arr + 1U);
    if (ticks < 0.0f) ticks = 0.0f;
    if (ticks > (float)(arr + 1U)) ticks = (float)(arr + 1U);
    return (uint32_t)(ticks + 0.5f);
}

void Servo_SetPulse_us(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, us_to_ccr(&htim13, (float)pulse_us));
}

void Servo_SetAngleDegrees(float degrees)
{
    if (degrees < 0.0f) degrees = 0.0f;
    if (degrees > 180.0f) degrees = 180.0f;

    float us = SERVO_MIN_US + (degrees / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);
    Servo_SetPulse_us((uint16_t)us);
}

void Servo_MoveToDegrees(float degrees)
{
    uint32_t now = HAL_GetTick();

    if ((now - last_move_tick) >= (uint32_t)SERVO_MOVE_MIN_MS) {
        Servo_SetAngleDegrees(degrees);
        last_move_tick = now;
    }
}

void Servo_MoveToDegreesForce(float degrees)
{
    Servo_SetAngleDegrees(degrees);
    last_move_tick = HAL_GetTick();
}
