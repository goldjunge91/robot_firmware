#include "pwm_control.h"
#include <Arduino.h>
#include "config.h"

extern "C"
{

    void pwm_control_init(void)
    {
        // Initialize PWM pins as outputs (basic implementation)
        pinMode(FL_PWM, OUTPUT);
        pinMode(FR_PWM, OUTPUT);
        pinMode(RL_PWM, OUTPUT);
        pinMode(RR_PWM, OUTPUT);
    }

    void pwm_control_update(robot_motors_t *motors)
    {
        // Simple PWM implementation using analogWrite
        analogWrite(FL_PWM, motors->front_left.speed);
        analogWrite(FR_PWM, motors->front_right.speed);
        analogWrite(RL_PWM, motors->rear_left.speed);
        analogWrite(RR_PWM, motors->rear_right.speed);
    }

} // extern "C"