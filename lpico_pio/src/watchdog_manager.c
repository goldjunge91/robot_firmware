#include "watchdog_manager.h"
#include "hardware/watchdog.h"

void watchdog_manager_init(void) {
    watchdog_enable(WATCHDOG_MS, 1);
}

void watchdog_manager_update(void) {
    watchdog_update();
}

bool watchdog_manager_was_rebooted(void) {
    return watchdog_enable_caused_reboot();
}