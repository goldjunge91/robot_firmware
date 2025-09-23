#include "hardware/watchdog.h"

void watchdog_init(void) {
    watchdog_enable(300, 1);
}

void watchdog_feed(void) {
    watchdog_update();
}
