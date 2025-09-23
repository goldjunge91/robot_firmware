#pragma once

#include <array>
#include <cstdint>

#include "encoder_reader.hpp"
#include "motor_controller.hpp"

extern "C" {
#include "rclc/executor.h"
#include "rclc/rclc.h"
#include "std_msgs/msg/int32_multi_array.h"
}

// Wraps the micro-ROS client entities used by the Pico firmware.
class MicroRosSupport {
public:
    MicroRosSupport(MotorController& motor_controller, EncoderReader& encoder_reader);
    bool init();
    void spin_some();

private:
    MotorController& motor_controller_;
    EncoderReader& encoder_reader_;

    rcl_allocator_t allocator_{};
    rclc_support_t support_{};
    rcl_node_t node_{};
    rcl_publisher_t encoder_publisher_{};
    rcl_subscription_t command_subscription_{};
    rcl_timer_t encoder_timer_{};
    rclc_executor_t executor_{};

    std_msgs__msg__Int32MultiArray encoder_msg_{};
    std_msgs__msg__Int32MultiArray command_msg_{};
    std::array<int32_t, 4> encoder_buffer_{};
    std::array<int32_t, 4> command_buffer_{};

    absolute_time_t last_command_time_{};

    static MicroRosSupport* instance_;

    bool init_transport();
    bool init_messages();
    bool init_entities();
    void destroy_messages();

    static void subscription_callback(const void* msgin);
    static void timer_callback(rcl_timer_t* timer, int64_t last_call_time);

    void handle_command_message(const std_msgs__msg__Int32MultiArray* msg);
    void publish_encoder_snapshot();
    void enforce_watchdog();
};
