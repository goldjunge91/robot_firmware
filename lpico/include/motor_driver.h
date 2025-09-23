#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "motor_types.h"

void motor_driver_init(void);
void motor_driver_set_speed(motor_state_t* motor, int16_t speed);
void motor_driver_update(robot_motors_t* motors);

#endif // MOTOR_DRIVER_H
