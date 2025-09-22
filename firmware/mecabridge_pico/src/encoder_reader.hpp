#pragma once

#include <array>
#include <cstdint>

class EncoderReader {
public:
    static constexpr std::size_t ENCODER_COUNT = 4;

    void init();
    void reset();
    std::array<int32_t, ENCODER_COUNT> snapshot_counts() const;

private:
    struct EncoderConfig {
        std::uint8_t channel_a;
        std::uint8_t channel_b;
    };

    struct EncoderState {
        volatile int32_t counts;
        std::uint8_t last_state;
    };

    static EncoderReader* instance_;

    static void gpio_irq_trampoline(uint gpio, uint32_t events);
    void handle_gpio_irq(uint gpio, uint32_t events);
    int find_encoder_index(uint gpio) const;

    std::array<EncoderConfig, ENCODER_COUNT> configs_{};
    std::array<EncoderState, ENCODER_COUNT> states_{};
    bool initialized_ = false;
};
