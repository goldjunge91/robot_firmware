#include "command_processor.h"
#include <string.h>
#include <stdio.h>
#include "usb_interface.h"
#include "control_loop.h"
#include "motor_driver.h"

void command_processor_task(void) {
    char command[128];
    if (usb_interface_get_command(command, sizeof(command))) {
        if (strncmp(command, "M", 1) == 0) {
            robot_motors_t* motors = control_loop_get_motors();
            int fl, fr, rl, rr;
            sscanf(command, "M FL:%d FR:%d RL:%d RR:%d", &fl, &fr, &rl, &rr);
            motor_driver_set_speed(&motors->front_left, fl);
            motor_driver_set_speed(&motors->front_right, fr);
            motor_driver_set_speed(&motors->rear_left, rl);
            motor_driver_set_speed(&motors->rear_right, rr);
            usb_interface_send_response("OK");
        } else if (strcmp(command, "STOP") == 0) {
            robot_motors_t* motors = control_loop_get_motors();
            motor_driver_set_speed(&motors->front_left, 0);
            motor_driver_set_speed(&motors->front_right, 0);
            motor_driver_set_speed(&motors->rear_left, 0);
            motor_driver_set_speed(&motors->rear_right, 0);
            usb_interface_send_response("STOPPED");
        } else {
            usb_interface_send_response("ERROR: INVALID_CMD");
        }
    }
}