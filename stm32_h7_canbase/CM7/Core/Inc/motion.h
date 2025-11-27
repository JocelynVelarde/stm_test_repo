#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include "main.h"

typedef struct {
    // Target Control
    float target_x;
    float target_y;
    uint8_t has_target_point;

    // PID Constants
    float Kp;
    float Ki;
    float Kd;

    // PID State
    float prev_error;
    float integral_error;
    // float last_time_ms;  <-- REMOVED: Not needed in fixed-frequency RTOS

    // Hardware Config
    float servo_center_deg;

} Motion_t;

void Motion_Init(Motion_t *m);
void Motion_AcceptCoords(Motion_t *m, float x, float y);
void Motion_Stop(Motion_t *m);

// UPDATED: Added 'dt' argument
void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw, float dt);

#endif /* INC_MOTION_H_ */