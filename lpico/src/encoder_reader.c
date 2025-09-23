#include "encoder_reader.h"
#include "hardware/gpio.h"
#include "config.h"

void encoder_reader_init(void) {
    gpio_init(ENC_FL_A);
    gpio_set_dir(ENC_FL_A, GPIO_IN);
    gpio_pull_up(ENC_FL_A);
    gpio_init(ENC_FL_B);
    gpio_set_dir(ENC_FL_B, GPIO_IN);
    gpio_pull_up(ENC_FL_B);

    gpio_init(ENC_FR_A);
    gpio_set_dir(ENC_FR_A, GPIO_IN);
    gpio_pull_up(ENC_FR_A);
    gpio_init(ENC_FR_B);
    gpio_set_dir(ENC_FR_B, GPIO_IN);
    gpio_pull_up(ENC_FR_B);

    gpio_init(ENC_RL_A);
    gpio_set_dir(ENC_RL_A, GPIO_IN);
    gpio_pull_up(ENC_RL_A);
    gpio_init(ENC_RL_B);
    gpio_set_dir(ENC_RL_B, GPIO_IN);
    gpio_pull_up(ENC_RL_B);

    gpio_init(ENC_RR_A);
    gpio_set_dir(ENC_RR_A, GPIO_IN);
    gpio_pull_up(ENC_RR_A);
    gpio_init(ENC_RR_B);
    gpio_set_dir(ENC_RR_B, GPIO_IN);
    gpio_pull_up(ENC_RR_B);
}

void encoder_reader_update(robot_encoders_t* encoders) {
    // This is a simplified implementation. A real implementation would use interrupts.
    encoders->front_left.count++;
    encoders->front_right.count++;
    encoders->rear_left.count++;
    encoders->rear_right.count++;
}