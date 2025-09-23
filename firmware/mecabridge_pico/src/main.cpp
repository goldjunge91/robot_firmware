#include "encoder_reader.hpp"
#include "micro_ros_support.hpp"
#include "motor_controller.hpp"

#include "pico/stdlib.h"

int main() {
    MotorController motor_controller;
    EncoderReader encoder_reader;

    motor_controller.init();
    encoder_reader.init();

    MicroRosSupport micro_ros(motor_controller, encoder_reader);
    if (!micro_ros.init()) {
        while (true) {
            sleep_ms(1000);
        }
    }

    while (true) {
        micro_ros.spin_some();
        tight_loop_contents();
    }

    return 0;
}
