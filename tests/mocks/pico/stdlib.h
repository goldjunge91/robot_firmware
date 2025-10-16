/**
 * @file pico/stdlib.h
 * @brief Mock Pico SDK stdlib.h for host-based unit testing
 */

#ifndef MOCK_PICO_STDLIB_H_
#define MOCK_PICO_STDLIB_H_

#include <cstdint>
#include <cstdbool>

typedef uint8_t uint;

typedef struct {
    int64_t _private_us_since_boot;
} absolute_time_t;

#define GPIO_OUT true
#define GPIO_IN false

// GPIO mocks
inline void gpio_init(uint gpio) { (void)gpio; }
inline void gpio_set_dir(uint gpio, bool out) { (void)gpio; (void)out; }
inline void gpio_put(uint gpio, bool value) { (void)gpio; (void)value; }
inline bool gpio_get(uint gpio) { (void)gpio; return false; }

// Time mocks
inline void sleep_ms(uint32_t ms) { (void)ms; }
inline void sleep_us(uint64_t us) { (void)us; }
inline absolute_time_t get_absolute_time() { absolute_time_t t; t._private_us_since_boot = 0; return t; }
inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return to._private_us_since_boot - from._private_us_since_boot;
}
inline absolute_time_t make_timeout_time_ms(uint32_t ms) { (void)ms; absolute_time_t t; t._private_us_since_boot = 0; return t; }
inline bool time_reached(absolute_time_t t) { (void)t; return true; }

#endif
