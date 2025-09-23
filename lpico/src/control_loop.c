#include "hardware/timer.h"

bool control_loop_callback(struct repeating_timer *t) {
    // This function will be implemented in a later task.
    return true;
}

void control_loop_init(void) {
    struct repeating_timer timer;
    add_repeating_timer_us(1000, control_loop_callback, NULL, &timer);
}
