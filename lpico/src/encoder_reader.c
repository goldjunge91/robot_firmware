#include "config.h"
#include "hardware/gpio.h"

void encoder_init(void) {
    gpio_init(ENC_FL_A);
    gpio_set_dir(ENC_FL_A, GPIO_IN);
    gpio_pull_up(ENC_FL_A);

    gpio_init(ENC_FL_B);
    gpio_set_dir(ENC_FL_B, GPIO_IN);
    gpio_pull_up(ENC_FL_B);

    // ... and so on for the other encoders
}

void encoder_callback(uint gpio, uint32_t events) {
    // This function will be implemented in a later task.
}
