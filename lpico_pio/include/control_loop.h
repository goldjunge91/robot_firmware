#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "motor_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void control_loop_init(void);
    void control_loop_update(void);
    robot_motors_t *control_loop_get_motors(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_LOOP_H
