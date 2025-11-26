#include "motion.h"
#include "esc.h"
#include "servo.h"
#include <math.h>
#include "main.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define SERVO_MAX_DEG 160.0f
#define SERVO_MIN_DEG 0.0f

void Motion_Init(Motion_t *m)
{
    if (!m) return;

    m->target_x = 0; 
    m->target_y = 0; 
    m->has_target_point = 0;
    
    m->Kp = 1.2f;  
    m->Ki = 0.0f;  
    m->Kd = 0.1f;  

    m->prev_error = 0;
    m->integral_error = 0;
    m->last_time_ms = HAL_GetTick();

    stopCarEsc();
    m->servo_center_deg = 115.0f;
    Servo_SetAngleDegrees(m->servo_center_deg);
}

void Motion_AcceptCoords(Motion_t *m, float x, float y)
{
    m->target_x = x;
    m->target_y = y;
    m->has_target_point = 1;
    m->prev_error = 0.0f;
    m->integral_error = 0.0f;
    m->last_time_ms = HAL_GetTick();
}

void Motion_Stop(Motion_t *m)
{
    m->has_target_point = 0;
    setEscSpeed_us(1500); 
    Servo_SetAngleDegrees(m->servo_center_deg);
}

void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw)
{
    if (!m || !m->has_target_point) {
        setEscSpeed_us(1500);
    }

    float dx = m->target_x - current_x;
    float dy = m->target_y - current_y;
    float dist = sqrtf(dx*dx + dy*dy);

    if (dist <= 0.10f) {
        Motion_Stop(m);
        return;
    }

    float desired_yaw = atan2f(dy, dx) * (180.0f / M_PI);
    
    float error = desired_yaw - current_yaw;

    if (error > 180.0f)  error -= 360.0f;
    if (error < -180.0f) error += 360.0f;

    uint32_t now = HAL_GetTick();
    float dt = (now - m->last_time_ms) / 1000.0f; 
    if (dt <= 0.0f) dt = 0.001f; 

    float P = m->Kp * error;

    m->integral_error += error * dt;
    float I = m->Ki * m->integral_error;

    float D = m->Kd * (error - m->prev_error) / dt;

    float pid_output = P + I + D;

    m->prev_error = error;
    m->last_time_ms = now;

    float final_servo_angle = m->servo_center_deg + pid_output;

    if (final_servo_angle > SERVO_MAX_DEG) final_servo_angle = SERVO_MAX_DEG;
    if (final_servo_angle < SERVO_MIN_DEG) final_servo_angle = SERVO_MIN_DEG;

    Servo_SetAngleDegrees(final_servo_angle);

    if (fabsf(error) > 30.0f) {
         setEscSpeed_us(1600); 
    } else {
         setEscSpeed_us(1750);
    }
}