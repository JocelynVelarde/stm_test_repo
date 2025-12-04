#ifndef SERVO_H
#define SERVO_H

#include "main.h"

void Servo_SetPulse_us(uint16_t pulse_us);

void Servo_SetAngleDegrees(float degrees);

void servo_write_deg(uint16_t angle_deg);

#endif