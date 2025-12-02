#include "main.h"
#include "motion.h"
#include "esc.h"
#include "servo.h"
#include "constants.h"
#include <math.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// Forward declaration
static float pure_pursuit_compute_delta(float current_x, float current_y, float current_yaw,
                                        Waypoint_t path[], int path_size, int current_wp);

void Motion_Init(Motion_t *m)
{
    if (!m) return;

    m->target_x = 0; 
    m->target_y = 0; 
    m->has_target_point = 0;
    
    m->path = NULL;
    m->path_size = 0;
    m->current_wp_idx = 0;
    m->path_finished = 0;

    m->Kp = 1.5f;
    m->Ki = 0.0f;
    m->Kd = 0.01f;

    m->prev_error = 0;
    m->integral_error = 0;

    stopCarEsc();
    m->servo_center_deg = 110.0f;
    Servo_SetAngleDegrees(m->servo_center_deg);
}

void Motion_SetPath(Motion_t *m, Waypoint_t path[], int path_size)
{
    if (!m || !path || path_size <= 0) return;
    
    m->path = path;
    m->path_size = path_size;
    m->current_wp_idx = 0;
    m->path_finished = 0;
    
    m->target_x = path[0].x;
    m->target_y = path[0].y;
    m->has_target_point = 1;
    
    m->prev_error = 0.0f;
    m->integral_error = 0.0f;
}

void Motion_Stop(Motion_t *m)
{
    m->prev_error = 0.0f;
    m->integral_error = 0.0f;
    
    setEscSpeed_us(STOP_SPEED_US);
    Servo_SetAngleDegrees(m->servo_center_deg);
}

void Motion_NextWaypoint(Motion_t *motion)
{
    if (motion == NULL) return;
    
    motion->current_wp_idx++;
    
    if (motion->current_wp_idx >= motion->path_size) {
        motion->path_finished = 1;
        Motion_Stop(motion);
        return;
    }
    
    motion->target_x = motion->path[motion->current_wp_idx].x;
    motion->target_y = motion->path[motion->current_wp_idx].y;
    
    motion->integral_error = 0.0f;
    motion->prev_error = 0.0f;
}

void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw, float current_speed_mps, float dt)
{
    if (!m || !m->has_target_point || m->path_finished) {
        Motion_Stop(m);
        return;
    }

    float dx = m->target_x - current_x;
    float dy = m->target_y - current_y;
    float dist = sqrtf(dx*dx + dy*dy);

    float delta_rad;
    if (m->path && m->path_size > 1) {
        delta_rad = pure_pursuit_compute_delta(current_x, current_y, current_yaw, 
                                              m->path, m->path_size, m->current_wp_idx);
    } else {
        Waypoint_t temp_path[1] = { {m->target_x, m->target_y} };
        delta_rad = pure_pursuit_compute_delta(current_x, current_y, current_yaw, temp_path, 1, 0);
    }
    
    float delta_deg = delta_rad * RAD_TO_DEG;
    float error = delta_deg;
    
    m->integral_error += error * dt;
    if (m->integral_error > INTEGRAL_MAX_LIMIT)  m->integral_error = INTEGRAL_MAX_LIMIT;
    if (m->integral_error < INTEGRAL_MIN_LIMIT)  m->integral_error = INTEGRAL_MIN_LIMIT;
    
    float P = m->Kp * error;
    float I = m->Ki * m->integral_error;
    float D = m->Kd * (error - m->prev_error) / dt;
    m->prev_error = error;

    float final_servo_angle = m->servo_center_deg + (P + I + D);
    if (final_servo_angle > SERVO_LIMIT_RIGHT_DEG) final_servo_angle = SERVO_LIMIT_RIGHT_DEG;
    if (final_servo_angle < SERVO_LIMIT_LEFT_DEG) final_servo_angle = SERVO_LIMIT_LEFT_DEG;
    Servo_SetAngleDegrees(final_servo_angle);

    uint16_t motor_speed;

    #define ESC_NEUTRAL 1500
    #define ESC_BRAKE   1350  
    #define ESC_CRAWL   1650
    
    if (dist < FINAL_APPROACH_DIST) {
        
        if (current_speed_mps > 0.1f) {
            motor_speed = ESC_BRAKE; 
        } 
        else if (current_speed_mps < -0.05f) {
             motor_speed = 1650; 
        }
        else {
            motor_speed = ESC_CRAWL; 
        }
    } 
    else if (dist < APPROACH_DISTANCE) {
        float blend = dist / APPROACH_DISTANCE;
        motor_speed = (uint16_t)(APPROACH_SPEED_US + blend * (CRUISE_SPEED_US - APPROACH_SPEED_US));
    } 
    else {
        motor_speed = CRUISE_SPEED_US;
    }
    
    if (fabsf(delta_deg) > 25.0f) {
        if (current_speed_mps > 1.0f) motor_speed = ESC_BRAKE;
        else motor_speed = (uint16_t)(motor_speed * 0.8f);
    }
    
    setEscSpeed_us(motor_speed);
}

static float pure_pursuit_compute_delta(float current_x, float current_y, float current_yaw, Waypoint_t path[], int path_size, int current_wp)
{
    if (current_wp >= path_size) current_wp = path_size - 1;
    
    Waypoint_t goal = path[current_wp];
    
    for (int i = current_wp; i < path_size; i++) {
        float dx = path[i].x - current_x;
        float dy = path[i].y - current_y;
        float dist = sqrtf(dx*dx + dy*dy);
        
        if (dist >= LOOKAHEAD_DIST) {
            goal = path[i];
            break;
        }
    }
    
    float dx = goal.x - current_x;
    float dy = goal.y - current_y;
    
    float yaw_rad = current_yaw * DEG_TO_RAD;
    float cos_t = cosf(yaw_rad);
    float sin_t = sinf(yaw_rad);
    
    float x_cg = cos_t * dx + sin_t * dy;
    float y_cg = -sin_t * dx + cos_t * dy;
    
    float Ld = sqrtf(x_cg*x_cg + y_cg*y_cg);
    if (Ld < 0.001f) Ld = 0.001f;
    
    float wheel_base_cm = WHEEL_BASE * 100.0f;
    float delta = atan2f(2.0f * wheel_base_cm * y_cg, (Ld * Ld));
    
    if (delta > MAX_STEER_ANGLE)  delta = MAX_STEER_ANGLE;
    if (delta < -MAX_STEER_ANGLE) delta = -MAX_STEER_ANGLE;
    
    return delta;
}