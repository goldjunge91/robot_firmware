#include "pico/stdlib.h"
#include "usb_interface.h"
#include "control_loop.h"
#include "command_processor.h"
#include "watchdog_manager.h"

int main() {
    usb_interface_init();
    control_loop_init();
    watchdog_manager_init();

    while (true) {
        command_processor_task();
    }

    return 0;
}