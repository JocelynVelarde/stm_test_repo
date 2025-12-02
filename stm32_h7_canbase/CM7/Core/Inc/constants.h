#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>
#include <stdint.h>

typedef struct {
    float wheel_v_mps;
    float yaw; 		  
    int32_t last_ticks;
} OdomData_t;

typedef struct {
    float x;		
    float y;		
    float theta;	
} Pose2D_t;

typedef struct {
    float x;
    float y;
} Waypoint_t;

typedef struct {
    float target_x;
    float target_y;
    uint8_t has_target_point;

    float Kp;
    float Ki;
    float Kd;

    float prev_error;
    float integral_error;

    float servo_center_deg;
    
    Waypoint_t *path;
    int path_size;
    int current_wp_idx;
    uint8_t path_finished;

} Motion_t;

typedef struct {
    float yaw;
    int32_t ticks;
} SensorData_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t angle;
} CameraData_t;

// Servo Constants
#define SERVO_PERIOD_US   20000.0f
#define SERVO_MIN_US       544.0f
#define SERVO_MAX_US      2400.0f
#define SERVO_MOVE_MIN_MS   10
#define SERVO_LIMIT_LEFT_DEG   30.0f
#define SERVO_CENTER_DEG  110.0f
#define SERVO_LIMIT_RIGHT_DEG  140.0f

// Robot Constants
#define TICKS_PER_REV      230
#define WHEEL_RADIUS_M     0.0314f
#define WHEEL_BASE         0.1365f

#define SERVO_MAX_DEG 160.0f
#define SERVO_MIN_DEG 30.0f

#define APPROACH_DISTANCE    5.0f      
#define FINAL_APPROACH_DIST  2.5f    
#define CRUISE_SPEED_US      1750      
#define APPROACH_SPEED_US    1650     
#define FINAL_SPEED_US       1550      
#define STOP_SPEED_US        1500

#define INTEGRAL_MAX_LIMIT   50.0f
#define INTEGRAL_MIN_LIMIT  -50.0f
#define MIN_DT              0.001f      
#define MAX_DT              0.1f      

#define CAM_THRESHOLD    20.0f
#define TARGET_THRESHOLD 25.0f
#define CAN_ID_SENSOR    0x30
#define CAN_ID_CAMERA    0x35

#define LOOKAHEAD_DIST   0.5f          
#define MAX_STEER_ANGLE  0.4f          
#define DEG_TO_RAD       (M_PI / 180.0f)
#define RAD_TO_DEG       (180.0f / M_PI)

#endif /* CONSTANTS_H */