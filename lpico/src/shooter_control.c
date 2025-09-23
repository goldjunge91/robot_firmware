#include "config.h"
#include "hardware/gpio.h"

void shooter_init(void) {
    gpio_init(ESC1_PIN);
    gpio_set_dir(ESC1_PIN, GPIO_OUT);

    gpio_init(ESC2_PIN);
    gpio_set_dir(ESC2_PIN, GPIO_OUT);

    gpio_init(GEAR_PIN);
    gpio_set_dir(GEAR_PIN, GPIO_OUT);
}

void shooter_set_esc(uint8_t esc, uint8_t state) {
    // This function will be implemented in a later task.
}

void shooter_set_gear(uint8_t state) {
    // This function will be implemented in a later task.
}
