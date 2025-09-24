#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "motor_types.h"

void control_loop_init(void);
robot_motors_t* control_loop_get_motors(void);

#endif // CONTROL_LOOP_H
