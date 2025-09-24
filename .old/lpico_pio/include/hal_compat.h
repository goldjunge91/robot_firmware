#ifndef HAL_COMPAT_H
#define HAL_COMPAT_H

#include <stdint.h>
#include <stdbool.h>

// GPIO
typedef enum {
    GPIO_PIN_RESET = 0,
    GPIO_PIN_SET = 1
} GPIO_PinState;

typedef void GPIO_TypeDef;
#define GPIOA ((GPIO_TypeDef*)0)
#define GPIOB ((GPIO_TypeDef*)0)
#define GPIOC ((GPIO_TypeDef*)0)

#define HAL_GPIO_WritePin(port, pin, state) gpio_put(pin, state)
#define HAL_GPIO_ReadPin(port, pin) gpio_get(pin)
#define HAL_GPIO_TogglePin(port, pin) gpio_xor_mask(1u << pin)

// PWM
typedef struct {
    uint8_t gpio_pin;
    uint8_t slice_num;
    uint8_t channel;
} TIM_HandleTypeDef;

extern TIM_HandleTypeDef htim_fl;
extern TIM_HandleTypeDef htim_fr;
extern TIM_HandleTypeDef htim_rl;
extern TIM_HandleTypeDef htim_rr;

void pico_pwm_set_duty(uint8_t gpio_pin, uint16_t duty_cycle);
void pico_pwm_enable(uint8_t gpio_pin, bool enabled);

#define __HAL_TIM_SET_COMPARE(htim, channel, value) \
    pico_pwm_set_duty((htim)->gpio_pin, value)

#define HAL_TIM_PWM_Start(htim, channel) \
    pico_pwm_enable((htim)->gpio_pin, true)

#define HAL_TIM_PWM_Stop(htim, channel) \
    pico_pwm_enable((htim)->gpio_pin, false)

// System
#define HAL_Delay(ms) sleep_ms(ms)
#define HAL_GetTick() to_ms_since_boot(get_absolute_time())

void pico_system_init(void);
void pico_gpio_init_all(void);
void pico_pwm_init_all(void);

#define SystemClock_Config() pico_system_init()
#define MX_GPIO_Init() pico_gpio_init_all()
#define MX_TIM_Init() pico_pwm_init_all()

// Error
typedef enum {
    HAL_OK       = 0x00,
    HAL_ERROR    = 0x01,
    HAL_BUSY     = 0x02,
    HAL_TIMEOUT  = 0x03
} HAL_StatusTypeDef;

void pico_error_handler(const char* error_msg);
#define Error_Handler() pico_error_handler(__func__)

#endif // HAL_COMPAT_H
