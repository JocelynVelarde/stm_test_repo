#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#include "main.h"
#include "constants.h"

void Motion_Init(Motion_t *m);
void Motion_SetPath(Motion_t *m, Waypoint_t path[], int path_size);
void Motion_Stop(Motion_t *m);
void Motion_Update(Motion_t *m, float current_x, float current_y, float current_yaw, float dt);
void Motion_NextWaypoint(Motion_t *motion);

#endif