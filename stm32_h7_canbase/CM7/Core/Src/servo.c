#include "servo.h"
#include "main.h"
#include "constants.h"

extern TIM_HandleTypeDef htim13;

static inline uint32_t us_to_ccr(float us)
{
    if (us < SERVO_MIN_PWM) us = SERVO_MIN_PWM;
    if (us > SERVO_MAX_PWM) us = SERVO_MAX_PWM;

    return (uint32_t)(us + 0.5f);
}

void Servo_SetPulse_us(uint16_t pulse_us)
{
    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, us_to_ccr((float)pulse_us));
}

void Servo_SetAngleDegrees(float degrees)
{
	if (degrees < SERVO_LIMIT_LEFT_DEG) degrees = SERVO_LIMIT_LEFT_DEG;
	if (degrees > SERVO_LIMIT_RIGHT_DEG) degrees = SERVO_LIMIT_RIGHT_DEG;

	float us = SERVO_MIN_PWM +
	               (degrees / 180.0f) * (SERVO_MAX_PWM - SERVO_MIN_PWM);

    Servo_SetPulse_us((uint16_t)us);
}

void servo_write_deg(uint16_t angle_deg)
{
    if (angle_deg > 180) angle_deg = 180;

    float us = SERVO_MIN_PWM +
               ((float)angle_deg / 180.0f) * (SERVO_MAX_PWM - SERVO_MIN_PWM);

    __HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, (uint16_t)us);
}