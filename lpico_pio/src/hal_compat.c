#include <Arduino.h>

#include "hardware/pwm.h"
#include "pico/time.h"
#include "hal_compat.h"

void pico_pwm_set_duty(uint8_t gpio_pin, uint16_t duty_cycle)
{
    pwm_set_gpio_level(gpio_pin, duty_cycle);
}

void pico_pwm_enable(uint8_t gpio_pin, bool enabled)
{
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_enabled(slice_num, enabled);
}

void pico_system_init(void)
{
    // stdio_init_all() is called in main.c
}

void pico_gpio_init_all(void)
{
    // GPIOs are initialized as needed
}

void pico_pwm_init_all(void)
{
    // PWMs are initialized as needed
}

void pico_error_handler(const char *error_msg)
{
    while (1)
        ;
}
