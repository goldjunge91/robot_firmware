#include "motor_driver.h"
#include "hardware/gpio.h"
#include "config.h"

void motor_driver_init(void) {
    gpio_init(STBY);
    gpio_set_dir(STBY, GPIO_OUT);
    gpio_put(STBY, 1);

    gpio_init(FL_IN1);
    gpio_set_dir(FL_IN1, GPIO_OUT);
    gpio_init(FL_IN2);
    gpio_set_dir(FL_IN2, GPIO_OUT);

    gpio_init(FR_IN1);
    gpio_set_dir(FR_IN1, GPIO_OUT);
    gpio_init(FR_IN2);
    gpio_set_dir(FR_IN2, GPIO_OUT);

    gpio_init(RL_IN1);
    gpio_set_dir(RL_IN1, GPIO_OUT);
    gpio_init(RL_IN2);
    gpio_set_dir(RL_IN2, GPIO_OUT);

    gpio_init(RR_IN1);
    gpio_set_dir(RR_IN1, GPIO_OUT);
    gpio_init(RR_IN2);
    gpio_set_dir(RR_IN2, GPIO_OUT);
}

void motor_driver_set_speed(motor_state_t* motor, int16_t speed) {
    if (speed > 0) {
        motor->direction = 1;
        motor->speed = speed;
    } else if (speed < 0) {
        motor->direction = -1;
        motor->speed = -speed;
    } else {
        motor->direction = 0;
        motor->speed = 0;
    }
}

void motor_driver_update(robot_motors_t* motors) {
    // Front left
    if (motors->front_left.direction == 1) {
        gpio_put(FL_IN1, 1);
        gpio_put(FL_IN2, 0);
    } else if (motors->front_left.direction == -1) {
        gpio_put(FL_IN1, 0);
        gpio_put(FL_IN2, 1);
    } else {
        gpio_put(FL_IN1, 0);
        gpio_put(FL_IN2, 0);
    }

    // Front right
    if (motors->front_right.direction == 1) {
        gpio_put(FR_IN1, 1);
        gpio_put(FR_IN2, 0);
    } else if (motors->front_right.direction == -1) {
        gpio_put(FR_IN1, 0);
        gpio_put(FR_IN2, 1);
    } else {
        gpio_put(FR_IN1, 0);
        gpio_put(FR_IN2, 0);
    }

    // Rear left
    if (motors->rear_left.direction == 1) {
        gpio_put(RL_IN1, 1);
        gpio_put(RL_IN2, 0);
    } else if (motors->rear_left.direction == -1) {
        gpio_put(RL_IN1, 0);
        gpio_put(RL_IN2, 1);
    } else {
        gpio_put(RL_IN1, 0);
        gpio_put(RL_IN2, 0);
    }

    // Rear right
    if (motors->rear_right.direction == 1) {
        gpio_put(RR_IN1, 1);
        gpio_put(RR_IN2, 0);
    } else if (motors->rear_right.direction == -1) {
        gpio_put(RR_IN1, 0);
        gpio_put(RR_IN2, 1);
    } else {
        gpio_put(RR_IN1, 0);
        gpio_put(RR_IN2, 0);
    }
}