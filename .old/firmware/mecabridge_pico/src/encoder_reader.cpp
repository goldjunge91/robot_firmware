#include "encoder_reader.hpp"

#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include "config.h"

namespace {
constexpr std::uint32_t kEncoderIrqEvents = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
constexpr int8_t kQuadratureDelta[16] = {
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};
}

EncoderReader* EncoderReader::instance_ = nullptr;

void EncoderReader::init() {
    if (initialized_) {
        return;
    }

    configs_ = {
        EncoderConfig{ENC_FL_A, ENC_FL_B},
        EncoderConfig{ENC_FR_A, ENC_FR_B},
        EncoderConfig{ENC_RL_A, ENC_RL_B},
        EncoderConfig{ENC_RR_A, ENC_RR_B},
    };

    for (auto& state : states_) {
        state.counts = 0;
        state.last_state = 0;
    }

    instance_ = this;

    bool callback_registered = false;

    for (std::size_t i = 0; i < configs_.size(); ++i) {
        const auto& cfg = configs_[i];

        gpio_init(cfg.channel_a);
        gpio_set_dir(cfg.channel_a, GPIO_IN);
        gpio_pull_up(cfg.channel_a);

        gpio_init(cfg.channel_b);
        gpio_set_dir(cfg.channel_b, GPIO_IN);
        gpio_pull_up(cfg.channel_b);

        const std::uint8_t initial_state = (gpio_get(cfg.channel_a) << 1) | gpio_get(cfg.channel_b);
        states_[i].last_state = initial_state;

        if (!callback_registered) {
            gpio_set_irq_enabled_with_callback(cfg.channel_a, kEncoderIrqEvents, true, &EncoderReader::gpio_irq_trampoline);
            callback_registered = true;
        } else {
            gpio_set_irq_enabled(cfg.channel_a, kEncoderIrqEvents, true);
        }
        gpio_set_irq_enabled(cfg.channel_b, kEncoderIrqEvents, true);
    }

    initialized_ = true;
}

void EncoderReader::reset() {
    for (auto& state : states_) {
        state.counts = 0;
    }
}

std::array<int32_t, EncoderReader::ENCODER_COUNT> EncoderReader::snapshot_counts() const {
    std::array<int32_t, ENCODER_COUNT> values{};
    for (std::size_t i = 0; i < states_.size(); ++i) {
        values[i] = states_[i].counts;
    }
    return values;
}

void EncoderReader::gpio_irq_trampoline(uint gpio, uint32_t events) {
    if (instance_ != nullptr) {
        instance_->handle_gpio_irq(gpio, events);
    }
}

void EncoderReader::handle_gpio_irq(uint gpio, uint32_t /*events*/) {
    const int index = find_encoder_index(gpio);
    if (index < 0) {
        return;
    }

    const auto& cfg = configs_[index];
    const std::uint8_t new_state = (gpio_get(cfg.channel_a) << 1) | gpio_get(cfg.channel_b);
    auto& encoder_state = states_[index];

    const std::uint8_t transition = static_cast<std::uint8_t>((encoder_state.last_state << 2) | new_state);
    const int8_t delta = kQuadratureDelta[transition];

    encoder_state.counts += delta;
    encoder_state.last_state = new_state;
}

int EncoderReader::find_encoder_index(uint gpio) const {
    for (std::size_t i = 0; i < configs_.size(); ++i) {
        const auto& cfg = configs_[i];
        if (gpio == cfg.channel_a || gpio == cfg.channel_b) {
            return static_cast<int>(i);
        }
    }
    return -1;
}
