#include "control_loop.h"
#include "hardware/timer.h"
#include "motor_driver.h"
#include "pwm_control.h"
#include "encoder_reader.h"
#include "watchdog_manager.h"

static robot_motors_t motors;
static robot_encoders_t encoders;

bool control_loop_callback(struct repeating_timer *t) {
    motor_driver_update(&motors);
    pwm_control_update(&motors);
    encoder_reader_update(&encoders);
    watchdog_manager_update();
    return true;
}

void control_loop_init(void) {
    motor_driver_init();
    pwm_control_init();
    encoder_reader_init();

    struct repeating_timer timer;
    add_repeating_timer_ms(LOOP_MS, control_loop_callback, NULL, &timer);
}

robot_motors_t* control_loop_get_motors(void) {
    return &motors;
}