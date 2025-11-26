#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>

typedef struct {
    float target_x;
    float target_y;
    uint8_t has_target_point;

    float servo_center_deg;
    
    float Kp;
    float Ki;
    float Kd;
    float prev_error;
    float integral_error;
    
    uint32_t last_time_ms;

} Motion_t;

void Motion_Init(Motion_t *m);
void Motion_AcceptCoords(Motion_t *m, float x, float y);
void Motion_Stop(Motion_t *m);
void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw);

#endif