#include "config.h"
#include "hardware/pwm.h"

void pwm_init(void) {
    gpio_set_function(FL_PWM, GPIO_FUNC_PWM);
    gpio_set_function(FR_PWM, GPIO_FUNC_PWM);
    gpio_set_function(RL_PWM, GPIO_FUNC_PWM);
    gpio_set_function(RR_PWM, GPIO_FUNC_PWM);

    uint slice_num_fl = pwm_gpio_to_slice_num(FL_PWM);
    uint slice_num_fr = pwm_gpio_to_slice_num(FR_PWM);
    uint slice_num_rl = pwm_gpio_to_slice_num(RL_PWM);
    uint slice_num_rr = pwm_gpio_to_slice_num(RR_PWM);

    pwm_set_wrap(slice_num_fl, 255);
    pwm_set_wrap(slice_num_fr, 255);
    pwm_set_wrap(slice_num_rl, 255);
    pwm_set_wrap(slice_num_rr, 255);

    pwm_set_enabled(slice_num_fl, true);
    pwm_set_enabled(slice_num_fr, true);
    pwm_set_enabled(slice_num_rl, true);
    pwm_set_enabled(slice_num_rr, true);
}

void pwm_set_duty(uint8_t gpio_pin, uint16_t duty_cycle) {
    uint slice_num = pwm_gpio_to_slice_num(gpio_pin);
    pwm_set_chan_level(slice_num, pwm_gpio_to_channel(gpio_pin), duty_cycle);
}
