#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <stdint.h>

#define PWM_MAX            255
#define WATCHDOG_MS        300
#define LOOP_MS            10
#define DEADZONE_DEFAULT   30
#define SLEW_DEFAULT       8

typedef struct {
    uint8_t pwm_max;
    uint16_t watchdog_ms;
    uint8_t loop_ms;
    uint8_t deadzone;
    uint8_t slew_rate;
} system_config_t;

#endif // SYSTEM_CONFIG_H
