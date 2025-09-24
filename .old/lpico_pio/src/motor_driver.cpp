#include "motor_driver.h"
#include <Arduino.h>
#include "config.h"

extern "C"
{

    void motor_driver_init(void)
    {
        pinMode(STBY, OUTPUT);
        digitalWrite(STBY, HIGH);

        pinMode(FL_IN1, OUTPUT);
        pinMode(FL_IN2, OUTPUT);

        pinMode(FR_IN1, OUTPUT);
        pinMode(FR_IN2, OUTPUT);

        pinMode(RL_IN1, OUTPUT);
        pinMode(RL_IN2, OUTPUT);

        pinMode(RR_IN1, OUTPUT);
        pinMode(RR_IN2, OUTPUT);
    }

    void motor_driver_set_speed(motor_state_t *motor, int16_t speed)
    {
        if (speed > 0)
        {
            motor->direction = 1;
            motor->speed = speed;
        }
        else if (speed < 0)
        {
            motor->direction = -1;
            motor->speed = -speed;
        }
        else
        {
            motor->direction = 0;
            motor->speed = 0;
        }
    }

    void motor_driver_update(robot_motors_t *motors)
    {
        // Front left
        if (motors->front_left.direction == 1)
        {
            digitalWrite(FL_IN1, HIGH);
            digitalWrite(FL_IN2, LOW);
        }
        else if (motors->front_left.direction == -1)
        {
            digitalWrite(FL_IN1, LOW);
            digitalWrite(FL_IN2, HIGH);
        }
        else
        {
            digitalWrite(FL_IN1, LOW);
            digitalWrite(FL_IN2, LOW);
        }

        // Front right
        if (motors->front_right.direction == 1)
        {
            digitalWrite(FR_IN1, HIGH);
            digitalWrite(FR_IN2, LOW);
        }
        else if (motors->front_right.direction == -1)
        {
            digitalWrite(FR_IN1, LOW);
            digitalWrite(FR_IN2, HIGH);
        }
        else
        {
            digitalWrite(FR_IN1, LOW);
            digitalWrite(FR_IN2, LOW);
        }

        // Rear left
        if (motors->rear_left.direction == 1)
        {
            digitalWrite(RL_IN1, HIGH);
            digitalWrite(RL_IN2, LOW);
        }
        else if (motors->rear_left.direction == -1)
        {
            digitalWrite(RL_IN1, LOW);
            digitalWrite(RL_IN2, HIGH);
        }
        else
        {
            digitalWrite(RL_IN1, LOW);
            digitalWrite(RL_IN2, LOW);
        }

        // Rear right
        if (motors->rear_right.direction == 1)
        {
            digitalWrite(RR_IN1, HIGH);
            digitalWrite(RR_IN2, LOW);
        }
        else if (motors->rear_right.direction == -1)
        {
            digitalWrite(RR_IN1, LOW);
            digitalWrite(RR_IN2, HIGH);
        }
        else
        {
            digitalWrite(RR_IN1, LOW);
            digitalWrite(RR_IN2, LOW);
        }
    }

} // extern "C"