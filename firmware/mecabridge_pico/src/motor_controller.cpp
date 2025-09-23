#include "motor_controller.hpp"

#include <array>
#include <cmath>

#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

#include "config.h"

namespace {
constexpr int kDeadzone = DEADZONE_DEFAULT;
constexpr int kPwmMax = PWM_MAX;

static inline int clamp_pwm(int value) {
    if (value > kPwmMax) {
        return kPwmMax;
    }
    if (value < -kPwmMax) {
        return -kPwmMax;
    }
    return value;
}
}

void MotorController::init() {
    if (initialized_) {
        return;
    }

    gpio_init(STBY);
    gpio_set_dir(STBY, GPIO_OUT);
    gpio_put(STBY, 1);

    configure_single_motor(FRONT_LEFT, FL_IN1, FL_IN2, FL_PWM, OFFSET_FL, TRIM_FL);
    configure_single_motor(FRONT_RIGHT, FR_IN1, FR_IN2, FR_PWM, OFFSET_FR, TRIM_FR);
    configure_single_motor(REAR_LEFT, RL_IN1, RL_IN2, RL_PWM, OFFSET_RL, TRIM_RL);
    configure_single_motor(REAR_RIGHT, RR_IN1, RR_IN2, RR_PWM, OFFSET_RR, TRIM_RR);

    initialized_ = true;
}

void MotorController::stop_all() {
    if (!initialized_) {
        return;
    }
    for (auto& motor : motors_) {
        gpio_put(motor.in1, 0);
        gpio_put(motor.in2, 0);
        pwm_set_chan_level(motor.slice, motor.channel, 0);
    }
}

void MotorController::apply_pwm(const std::array<int16_t, MOTOR_COUNT>& pwm_values) {
    if (!initialized_) {
        return;
    }

    set_motor_pwm(FRONT_LEFT, pwm_values[FRONT_LEFT]);
    set_motor_pwm(FRONT_RIGHT, pwm_values[FRONT_RIGHT]);
    set_motor_pwm(REAR_LEFT, pwm_values[REAR_LEFT]);
    set_motor_pwm(REAR_RIGHT, pwm_values[REAR_RIGHT]);
}

void MotorController::set_motor_pwm(MotorId motor_index, int16_t pwm_command) {
    if (!initialized_) {
        return;
    }

    auto& motor = motors_[motor_index];

    int commanded = static_cast<int>(pwm_command);
    commanded = commanded * motor.offset + motor.trim;

    if (std::abs(commanded) < kDeadzone) {
        commanded = 0;
    }

    commanded = clamp_pwm(commanded);

    const bool forward = commanded >= 0;
    gpio_put(motor.in1, forward ? 1 : 0);
    gpio_put(motor.in2, forward ? 0 : 1);

    const int level = std::abs(commanded);
    pwm_set_chan_level(motor.slice, motor.channel, static_cast<uint16_t>(level));
}

void MotorController::configure_single_motor(std::size_t index,
                                             std::uint8_t in1,
                                             std::uint8_t in2,
                                             std::uint8_t pwm,
                                             int offset,
                                             int trim) {
    gpio_init(in1);
    gpio_set_dir(in1, GPIO_OUT);
    gpio_put(in1, 0);

    gpio_init(in2);
    gpio_set_dir(in2, GPIO_OUT);
    gpio_put(in2, 0);

    gpio_set_function(pwm, GPIO_FUNC_PWM);

    const uint slice = pwm_gpio_to_slice_num(pwm);
    const uint channel = pwm_gpio_to_channel(pwm);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, kPwmMax);
    pwm_init(slice, &config, true);
    pwm_set_chan_level(slice, channel, 0);

    motors_[index] = MotorConfig{
        .in1 = in1,
        .in2 = in2,
        .pwm = pwm,
        .offset = offset,
        .trim = trim,
        .slice = static_cast<std::uint8_t>(slice),
        .channel = static_cast<std::uint8_t>(channel),
    };
}
