#include "micro_ros_support.hpp"

#include <cstdio>

#include "pico/stdlib.h"
#include "pico/time.h"

extern "C" {
#include "micro_ros_pico/transport.h"
#include "rmw_microros/rmw_microros.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "std_msgs/msg/int32_multi_array.h"
}

#include "config.h"

namespace {
constexpr int64_t kWatchdogTimeoutUs = static_cast<int64_t>(WATCHDOG_MS) * 1000;
}

MicroRosSupport* MicroRosSupport::instance_ = nullptr;

MicroRosSupport::MicroRosSupport(MotorController& motor_controller, EncoderReader& encoder_reader)
    : motor_controller_(motor_controller), encoder_reader_(encoder_reader) {}

bool MicroRosSupport::init() {
    allocator_ = rcl_get_default_allocator();

    if (!init_transport()) {
        return false;
    }

    if (!init_messages()) {
        return false;
    }

    if (!init_entities()) {
        destroy_messages();
        return false;
    }

    instance_ = this;
    last_command_time_ = get_absolute_time();

    return true;
}

void MicroRosSupport::spin_some() {
    if (!instance_) {
        return;
    }

    const rcl_ret_t rc = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));
    if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
        printf("[micro-ROS] executor_spin_some failed: %d\n", rc);
    }
}

bool MicroRosSupport::init_transport() {
    stdio_init_all();
    sleep_ms(2000);  // allow USB CDC enumeration

#if defined(MICRO_ROS_USE_UART_TRANSPORT)
    set_microros_transports();
#else
    set_microros_usb_transports();
#endif

    const rmw_ret_t ping = rmw_uros_ping_agent(1000, 5);
    if (ping != RMW_RET_OK) {
        printf("[micro-ROS] agent not reachable (ret=%d)\n", ping);
        return false;
    }

    return true;
}

bool MicroRosSupport::init_messages() {
    encoder_buffer_.fill(0);
    command_buffer_.fill(0);

    if (!std_msgs__msg__Int32MultiArray__init(&encoder_msg_)) {
        printf("[micro-ROS] failed to init encoder_msg\n");
        return false;
    }
    if (!std_msgs__msg__Int32MultiArray__init(&command_msg_)) {
        printf("[micro-ROS] failed to init command_msg\n");
        return false;
    }

    encoder_msg_.data.data = encoder_buffer_.data();
    encoder_msg_.data.capacity = encoder_buffer_.size();
    encoder_msg_.data.size = encoder_buffer_.size();

    command_msg_.data.data = command_buffer_.data();
    command_msg_.data.capacity = command_buffer_.size();
    command_msg_.data.size = 0;

    return true;
}

bool MicroRosSupport::init_entities() {
    const rosidl_message_type_support_t* msg_type = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray);

    rcl_ret_t rc = rclc_support_init(&support_, 0, nullptr, &allocator_);
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] support_init failed: %d\n", rc);
        return false;
    }

    rc = rclc_node_init_default(&node_, "mecabridge_pico", "", &support_);
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] node_init failed: %d\n", rc);
        return false;
    }

    rc = rclc_publisher_init_default(&encoder_publisher_, &node_, msg_type, "encoder_ticks");
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] publisher_init failed: %d\n", rc);
        return false;
    }

    rc = rclc_subscription_init_default(&command_subscription_, &node_, msg_type, "wheel_pwm");
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] subscription_init failed: %d\n", rc);
        return false;
    }

    rc = rclc_timer_init_default(&encoder_timer_, &support_, RCL_MS_TO_NS(LOOP_MS), MicroRosSupport::timer_callback);
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] timer_init failed: %d\n", rc);
        return false;
    }

    rc = rclc_executor_init(&executor_, &support_.context, 2, &allocator_);
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] executor_init failed: %d\n", rc);
        return false;
    }

    rc = rclc_executor_add_timer(&executor_, &encoder_timer_);
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] add_timer failed: %d\n", rc);
        return false;
    }

    rc = rclc_executor_add_subscription(&executor_, &command_subscription_, &command_msg_, MicroRosSupport::subscription_callback, ON_NEW_DATA);
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] add_subscription failed: %d\n", rc);
        return false;
    }

    return true;
}

void MicroRosSupport::destroy_messages() {
    std_msgs__msg__Int32MultiArray__fini(&encoder_msg_);
    std_msgs__msg__Int32MultiArray__fini(&command_msg_);
}

void MicroRosSupport::subscription_callback(const void* msgin) {
    if (instance_ == nullptr) {
        return;
    }
    const auto* msg = static_cast<const std_msgs__msg__Int32MultiArray*>(msgin);
    instance_->handle_command_message(msg);
}

void MicroRosSupport::handle_command_message(const std_msgs__msg__Int32MultiArray* msg) {
    if (msg->data.size < command_buffer_.size()) {
        printf("[micro-ROS] wheel_pwm message ignored (size=%zu)\n", static_cast<std::size_t>(msg->data.size));
        return;
    }

    std::array<int16_t, 4> pwm{};
    for (std::size_t i = 0; i < pwm.size(); ++i) {
        const int32_t raw = msg->data.data[i];
        if (raw > PWM_MAX) {
            pwm[i] = PWM_MAX;
        } else if (raw < -PWM_MAX) {
            pwm[i] = -PWM_MAX;
        } else {
            pwm[i] = static_cast<int16_t>(raw);
        }
    }

    motor_controller_.apply_pwm(pwm);
    last_command_time_ = get_absolute_time();
}

void MicroRosSupport::timer_callback(rcl_timer_t* /*timer*/, int64_t /*last_call_time*/) {
    if (instance_ == nullptr) {
        return;
    }
    instance_->publish_encoder_snapshot();
    instance_->enforce_watchdog();
}

void MicroRosSupport::publish_encoder_snapshot() {
    const auto counts = encoder_reader_.snapshot_counts();
    for (std::size_t i = 0; i < counts.size(); ++i) {
        encoder_buffer_[i] = counts[i];
    }
    encoder_msg_.data.size = encoder_buffer_.size();

    const rcl_ret_t rc = rcl_publish(&encoder_publisher_, &encoder_msg_, nullptr);
    if (rc != RCL_RET_OK) {
        printf("[micro-ROS] publish failed: %d\n", rc);
    }
}

void MicroRosSupport::enforce_watchdog() {
    const int64_t elapsed = absolute_time_diff_us(last_command_time_, get_absolute_time());
    if (elapsed > kWatchdogTimeoutUs) {
        motor_controller_.stop_all();
    }
}
