#include "control_loop.h"
#include <Arduino.h>
#include "motor_driver.h"
#include "pwm_control.h"
#include "encoder_reader.h"
#include "watchdog_manager.h"

extern "C"
{

    static robot_motors_t motors;
    static robot_encoders_t encoders;

    static unsigned long last_loop_time = 0;

    void control_loop_update(void)
    {
        unsigned long current_time = millis();
        if (current_time - last_loop_time >= LOOP_MS)
        {
            motor_driver_update(&motors);
            pwm_control_update(&motors);
            encoder_reader_update(&encoders);
            watchdog_manager_update();
            last_loop_time = current_time;
        }
    }

    void control_loop_init(void)
    {
        motor_driver_init();
        pwm_control_init();
        encoder_reader_init();
        last_loop_time = millis();
    }

    robot_motors_t *control_loop_get_motors(void)
    {
        return &motors;
    }

} // extern "C"