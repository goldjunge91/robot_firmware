#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "motor_types.h"

void pwm_control_init(void);
void pwm_control_update(robot_motors_t* motors);

#endif // PWM_CONTROL_H
