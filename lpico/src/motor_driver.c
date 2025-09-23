#include "config.h"
#include "hardware/gpio.h"

void motor_init(void) {
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

void motor_set_speed(uint8_t motor, int16_t speed) {
    // This function will be implemented in a later task.
}
