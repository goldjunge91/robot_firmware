#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include "motor_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void pwm_control_init(void);
    void pwm_control_update(robot_motors_t *motors);

#ifdef __cplusplus
}
#endif

#endif // PWM_CONTROL_H
