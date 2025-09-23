#include "watchdog_manager.h"
#include <Arduino.h>

extern "C"
{

    void watchdog_manager_init(void)
    {
        // Arduino framework doesn't have built-in watchdog for all boards
        // This is a stub implementation
    }

    void watchdog_manager_update(void)
    {
        // Arduino framework doesn't have built-in watchdog for all boards
        // This is a stub implementation
    }

    bool watchdog_manager_was_rebooted(void)
    {
        // Arduino framework doesn't have built-in watchdog for all boards
        // This is a stub implementation
        return false;
    }

} // extern "C"