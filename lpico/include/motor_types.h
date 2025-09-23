#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdint.h>

typedef struct {
    int8_t direction;
    uint8_t speed;
    int8_t trim;
    uint8_t enabled;
} motor_state_t;

typedef struct {
    motor_state_t front_left;
    motor_state_t front_right;
    motor_state_t rear_left;
    motor_state_t rear_right;
} robot_motors_t;

#endif // MOTOR_TYPES_H
