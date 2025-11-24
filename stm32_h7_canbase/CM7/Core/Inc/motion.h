#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include "main.h"

typedef struct {
    int16_t steering_deg;    

    /* Targeting */
    float target_x;
    float target_y;
    uint8_t has_target_point;
    float target_yaw_deg; 

    /* PID */
    float pid_kp;
    float pid_ki;
    float pid_kd;
    float pid_integrator;
    float pid_last_error;
    uint32_t pid_last_tick_ms;

    float servo_center_deg;
} Motion_t;

void Motion_Init(Motion_t *m);

void Motion_SetSteeringDeg(Motion_t *m, int16_t deg);

void Motion_SetPIDParams(Motion_t *m, float kp, float ki, float kd);
void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw_deg);

void Motion_AcceptCoords(Motion_t *m, float x_cm, float y_cm);

void Motion_UpdateWithThrottle(Motion_t *m, float current_x, float current_y, float current_yaw_deg);

void Motion_Stop(Motion_t *m);

void Motion_SetServoCenter(Motion_t *m, float center_deg);

#endif /* MOTION_H */
