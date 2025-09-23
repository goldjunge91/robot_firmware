#ifndef SHOOTER_TYPES_H
#define SHOOTER_TYPES_H

#include <stdint.h>

typedef struct {
    uint8_t esc1_enabled;
    uint8_t esc2_enabled;
    uint8_t gear_state;
    uint32_t last_fire_ms;
} shooter_state_t;

#endif // SHOOTER_TYPES_H
