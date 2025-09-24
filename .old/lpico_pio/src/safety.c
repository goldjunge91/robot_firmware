#include "safety.h"
#include "control_loop.h"
#include "motor_driver.h"

void safety_emergency_stop(void) {
    robot_motors_t* motors = control_loop_get_motors();
    motor_driver_set_speed(&motors->front_left, 0);
    motor_driver_set_speed(&motors->front_right, 0);
    motor_driver_set_speed(&motors->rear_left, 0);
    motor_driver_set_speed(&motors->rear_right, 0);
}