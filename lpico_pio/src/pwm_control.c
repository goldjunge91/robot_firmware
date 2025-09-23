#include "pwm_control.h"
#include "hardware/pwm.h"
#include "config.h"

void pwm_control_init(void) {
    // Front left
    gpio_set_function(FL_PWM, GPIO_FUNC_PWM);
    uint slice_num_fl = pwm_gpio_to_slice_num(FL_PWM);
    pwm_set_wrap(slice_num_fl, PWM_MAX);
    pwm_set_enabled(slice_num_fl, true);

    // Front right
    gpio_set_function(FR_PWM, GPIO_FUNC_PWM);
    uint slice_num_fr = pwm_gpio_to_slice_num(FR_PWM);
    pwm_set_wrap(slice_num_fr, PWM_MAX);
    pwm_set_enabled(slice_num_fr, true);

    // Rear left
    gpio_set_function(RL_PWM, GPIO_FUNC_PWM);
    uint slice_num_rl = pwm_gpio_to_slice_num(RL_PWM);
    pwm_set_wrap(slice_num_rl, PWM_MAX);
    pwm_set_enabled(slice_num_rl, true);

    // Rear right
    gpio_set_function(RR_PWM, GPIO_FUNC_PWM);
    uint slice_num_rr = pwm_gpio_to_slice_num(RR_PWM);
    pwm_set_wrap(slice_num_rr, PWM_MAX);
    pwm_set_enabled(slice_num_rr, true);
}

void pwm_control_update(robot_motors_t* motors) {
    pwm_set_gpio_level(FL_PWM, motors->front_left.speed);
    pwm_set_gpio_level(FR_PWM, motors->front_right.speed);
    pwm_set_gpio_level(RL_PWM, motors->rear_left.speed);
    pwm_set_gpio_level(RR_PWM, motors->rear_right.speed);
}