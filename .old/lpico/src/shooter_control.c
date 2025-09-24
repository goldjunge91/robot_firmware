#include "shooter_control.h"
#include "hardware/gpio.h"
#include "config.h"

void shooter_control_init(void) {
    gpio_init(ESC1_PIN);
    gpio_set_dir(ESC1_PIN, GPIO_OUT);

    gpio_init(ESC2_PIN);
    gpio_set_dir(ESC2_PIN, GPIO_OUT);

    gpio_init(GEAR_PIN);
    gpio_set_dir(GEAR_PIN, GPIO_OUT);
}

void shooter_control_update(shooter_state_t* shooter) {
    gpio_put(ESC1_PIN, shooter->esc1_enabled);
    gpio_put(ESC2_PIN, shooter->esc2_enabled);
    gpio_put(GEAR_PIN, shooter->gear_state);
}