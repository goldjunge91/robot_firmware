#include "pico/stdlib.h"

// Forward declarations
void usb_init(void);
void motor_init(void);
void pwm_init(void);
void encoder_init(void);
void shooter_init(void);
void watchdog_init(void);
void control_loop_init(void);
void process_command(char* command);
void usb_get_string(char* buf, int len);

int main() {
    usb_init();
    motor_init();
    pwm_init();
    encoder_init();
    shooter_init();
    watchdog_init();
    control_loop_init();

    char command[128];

    while (1) {
        usb_get_string(command, 128);
        process_command(command);
    }

    return 0;
}
