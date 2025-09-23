#pragma once

#include <array>
#include <cstdint>

class MotorController {
public:
    enum MotorId : std::uint8_t {
        FRONT_LEFT = 0,
        FRONT_RIGHT = 1,
        REAR_LEFT = 2,
        REAR_RIGHT = 3,
        MOTOR_COUNT = 4
    };

    void init();
    void stop_all();
    void apply_pwm(const std::array<int16_t, MOTOR_COUNT>& pwm_values);
    void set_motor_pwm(MotorId motor, int16_t pwm_command);

private:
    struct MotorConfig {
        std::uint8_t in1;
        std::uint8_t in2;
        std::uint8_t pwm;
        int offset;
        int trim;
        std::uint8_t slice;
        std::uint8_t channel;
    };

    std::array<MotorConfig, MOTOR_COUNT> motors_{};
    bool initialized_ = false;

    void configure_single_motor(std::size_t index, std::uint8_t in1, std::uint8_t in2, std::uint8_t pwm,
                                 int offset, int trim);
};
