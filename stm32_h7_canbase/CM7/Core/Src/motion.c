#include "motion.h"
#include "esc.h"
#include "servo.h"
#include <math.h>
#include "main.h" /* for HAL_GetTick */

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

static float normalize360(float a)
{
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f) a += 360.0f;
    return a;
}

void Motion_Init(Motion_t *m)
{
    if (!m) return;
    m->throttle_percent = 0;
    m->steering_deg = 0;
    m->target_x = 0.0f;
    m->target_y = 0.0f;
    m->has_target_point = 0;
    m->target_yaw_deg = 0.0f;

    /* PID*/
    m->pid_kp = 20.0f;
    m->pid_ki = 0.01f;
    m->pid_kd = 0.1f;
    m->pid_integrator = 0.0f;
    m->pid_last_error = 0.0f;
    m->pid_last_tick_ms = HAL_GetTick();
    m->servo_invert = 0;
    m->const_throttle_enable = 1;
    m->const_throttle_percent = 60;
    
    stopCarEsc();
    m->servo_center_deg = 17.0f;
    Servo_SetAngleDegrees(m->servo_center_deg);
}

void Motion_SetThrottlePercent(Motion_t *m, int8_t percent)
{
    if (!m) return;
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    if (m->throttle_percent == percent) return;
    m->throttle_percent = percent;
    movePercent(percent);
}

void Motion_MoveForward(Motion_t *m, int8_t percent)
{
    if (!m) return;
    if (percent < 0) percent = -percent;
    Motion_SetThrottlePercent(m, percent);
}

void Motion_MoveBackward(Motion_t *m, int8_t percent)
{
    if (!m) return;
    if (percent > 0) percent = -percent;
    Motion_SetThrottlePercent(m, percent);
}

void Motion_Stop(Motion_t *m)
{
    if (!m) return;
    if (m->throttle_percent == 0) return;
    m->throttle_percent = 0;
    stopCarEsc();
}

void Motion_SetSteeringDeg(Motion_t *m, int16_t deg)
{
    if (!m) return;
    if (deg > 90) deg = 90;
    if (deg < -90) deg = -90;
    if (m->steering_deg == deg) return;
    m->steering_deg = deg;
    float mapped = (float)deg;
    if (m->servo_invert) mapped = -mapped;
    float servo_deg = m->servo_center_deg + mapped;
    Servo_SetAngleDegrees(servo_deg);
}

void Motion_SetServoCenter(Motion_t *m, float center_deg)
{
    if (!m) return;
    m->servo_center_deg = center_deg;
    Servo_SetAngleDegrees(center_deg);
}

void Motion_SetServoInvert(Motion_t *m, uint8_t invert)
{
    if (!m) return;
    m->servo_invert = invert ? 1 : 0;
}

void Motion_EnableConstantThrottle(Motion_t *m, uint8_t enable, int8_t percent)
{
    if (!m) return;
    m->const_throttle_enable = enable ? 1 : 0;
    if (percent > 100) percent = 100;
    if (percent < -100) percent = -100;
    m->const_throttle_percent = percent;
}

void Motion_SetTargetPoint(Motion_t *m, float tx, float ty)
{
    if (!m) return;
    m->target_x = tx;
    m->target_y = ty;
    m->has_target_point = 1;
}

void Motion_ClearTargetPoint(Motion_t *m)
{
    if (!m) return;
    m->has_target_point = 0;
}

void Motion_SetTargetYaw(Motion_t *m, float yaw_deg)
{
    if (!m) return;
    m->target_yaw_deg = yaw_deg;
    m->has_target_point = 0;
}

void Motion_SetPIDParams(Motion_t *m, float kp, float ki, float kd)
{
    if (!m) return;
    m->pid_kp = kp;
    m->pid_ki = ki;
    m->pid_kd = kd;
}

static float wrap180(float a)
{
    while (a > 180.0f) a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw_deg)
{
    if (!m) return;

    float desired_yaw_deg = m->target_yaw_deg;
    if (m->has_target_point) {
        float dx = m->target_x - current_x;
        float dy = m->target_y - current_y;
        if (fabsf(dx) < 1e-6f && fabsf(dy) < 1e-6f) {
            desired_yaw_deg = current_yaw_deg;
        } else {
            desired_yaw_deg = atan2f(dy, dx) * (180.0f / M_PI);
            if (desired_yaw_deg < 0.0f) desired_yaw_deg += 360.0f;
        }
    }

    float cur360 = normalize360(current_yaw_deg);
    float des360 = normalize360(desired_yaw_deg);
    float error = wrap180(des360 - cur360);

    uint32_t now = HAL_GetTick();
    float dt = (now - m->pid_last_tick_ms) * 0.001f; /* seconds */
    if (dt <= 0.0f) dt = 0.001f;

    m->pid_integrator += error * dt;
    if (m->pid_integrator > 100.0f) m->pid_integrator = 100.0f;
    if (m->pid_integrator < -100.0f) m->pid_integrator = -100.0f;

    float derivative = (error - m->pid_last_error) / dt;
    float out = m->pid_kp * error + m->pid_ki * m->pid_integrator + m->pid_kd * derivative;

    if (out > 90.0f) out = 90.0f;
    if (out < -90.0f) out = -90.0f;

    Motion_SetSteeringDeg(m, (int16_t)out);

    m->pid_last_error = error;
    m->pid_last_tick_ms = now;
}

void Motion_AcceptCoords(Motion_t *m, float x_cm, float y_cm)
{
    if (!m) return;
    m->target_x = x_cm;
    m->target_y = y_cm;
    m->has_target_point = 1;
    m->const_throttle_enable = 1;
    m->const_throttle_percent = 100;
}

void Motion_UpdateWithThrottle(Motion_t *m, float current_x, float current_y, float current_yaw_deg)
{
    if (!m) return;

    float desired_yaw_deg = m->target_yaw_deg;
    float dist_cm = 0.0f;
    if (m->has_target_point) {
        float dx = m->target_x - current_x;
        float dy = m->target_y - current_y;
        dist_cm = sqrtf(dx*dx + dy*dy);
        if (dist_cm < 1e-6f) {
            desired_yaw_deg = current_yaw_deg;
        } else {
            desired_yaw_deg = atan2f(dy, dx) * (180.0f / M_PI);
            if (desired_yaw_deg < 0.0f) desired_yaw_deg += 360.0f;
        }
    }

    float cur360 = normalize360(current_yaw_deg);
    float des360 = normalize360(desired_yaw_deg);
    float error = wrap180(des360 - cur360);

    /* PID for steering (same as before) */
    uint32_t now = HAL_GetTick();
    float dt = (now - m->pid_last_tick_ms) * 0.001f;
    if (dt <= 0.0f) dt = 0.001f;

    m->pid_integrator += error * dt;
    if (m->pid_integrator > 100.0f) m->pid_integrator = 100.0f;
    if (m->pid_integrator < -100.0f) m->pid_integrator = -100.0f;

    float derivative = (error - m->pid_last_error) / dt;
    float out = m->pid_kp * error + m->pid_ki * m->pid_integrator + m->pid_kd * derivative;

    if (out > 90.0f) out = 90.0f;
    if (out < -90.0f) out = -90.0f;

    Motion_SetSteeringDeg(m, (int16_t)out);

    m->pid_last_error = error;
    m->pid_last_tick_ms = now;

    if (m->has_target_point) {
        float dx = m->target_x - current_x;
        float dy = m->target_y - current_y;
        float remaining_dist = sqrtf(dx*dx + dy*dy);
        
        if (remaining_dist <= 2.0f) {
            Motion_Stop(m);
            m->has_target_point = 0;
            return;
        }

        float speed = 0.0f;
        if (m->const_throttle_enable) {
            speed = (float)m->const_throttle_percent;
            
            if (remaining_dist < 5.0f) {
                float decel_factor = remaining_dist / 5.0f;  
                if (decel_factor < 0.3f) decel_factor = 0.3f; 
                speed *= decel_factor;
            }
            
            if (speed > 100.0f) speed = 100.0f;
            if (speed < -100.0f) speed = -100.0f;
        } else {
            float k_dist = 0.5f;
            speed = dist_cm * k_dist;
            if (speed > 100.0f) speed = 100.0f;
            if (speed < 15.0f) speed = 15.0f;

            float abs_err = fabsf(error);
            if (abs_err > 45.0f) {
                speed = 10.0f;
            } else if (abs_err > 20.0f) {
                speed *= 0.5f;
                if (speed < 10.0f) speed = 10.0f;
            }
        }

        Motion_SetThrottlePercent(m, (int8_t)speed);
    }
}
