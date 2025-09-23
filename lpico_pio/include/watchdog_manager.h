#ifndef WATCHDOG_MANAGER_H
#define WATCHDOG_MANAGER_H

#include <stdbool.h>
#include "system_config.h"

#ifdef __cplusplus
extern "C"
{
#endif

    void watchdog_manager_init(void);
    void watchdog_manager_update(void);
    bool watchdog_manager_was_rebooted(void);

#ifdef __cplusplus
}
#endif

#endif // WATCHDOG_MANAGER_H
