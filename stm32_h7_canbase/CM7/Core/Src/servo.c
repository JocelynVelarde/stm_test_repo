#include "servo.h"
#include "main.h"

extern TIM_HandleTypeDef htim13;

#define SERVO_PERIOD_US   20000.0f
#define SERVO_MIN_US       544.0f
#define SERVO_MAX_US      2400.0f

#define SERVO_MOVE_MIN_MS   10

#define SERVO_LIMIT_LEFT   0.0f
#define SERVO_LIMIT_RIGHT  165.0f

static uint32_t last_move_tick = 0U;

static inline uint32_t us_to_ccr(float us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;

    return (uint32_t)(us + 0.5f);
}

void Servo_SetPulse_us(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, us_to_ccr((float)pulse_us));
}

void Servo_SetAngleDegrees(float degrees)
{
	if (degrees < SERVO_LIMIT_LEFT) degrees = SERVO_LIMIT_LEFT;
	if (degrees > SERVO_LIMIT_RIGHT) degrees = SERVO_LIMIT_RIGHT;

	float us = SERVO_MIN_US +
	               (degrees / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

    Servo_SetPulse_us((uint16_t)us);
}

void servo_write_deg(uint16_t angle_deg)
{
    if (angle_deg > 180) angle_deg = 180;

    float us = SERVO_MIN_US +
               ((float)angle_deg / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US);

    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, (uint16_t)us);
}