/**
 * @file FirmwareConfig.h
 * @brief Firmware configuration constants for my_steel robot
 * 
 * This header provides centralized configuration constants for the firmware.
 * All constants use modern C++ features (inline constexpr) to replace
 * legacy #define macros. Constants are organized into logical namespaces for
 * better code organization and type safety.
 * 
 * @note This file uses C++14 features (inline constexpr variables)
 */

#ifndef FIRMWARE_CONFIG_H
#define FIRMWARE_CONFIG_H

#include <cstdint>
#include "hardware/spi.h"
#include "hardware/i2c.h"

/**
 * @namespace config
 * @brief Root namespace for all firmware configuration constants
 */
namespace config {

/**
 * @namespace config::pins
 * @brief Hardware pin assignments for Raspberry Pi Pico
 * 
 * Pin assignments are based on the physical wiring of the my_steel robot.
 * These values should not be changed unless the physical hardware wiring is modified.
 * 
 * @warning Changing these values without corresponding hardware changes will
 * cause the robot to malfunction.
 */
namespace pins {
    /**
     * @defgroup led_pins LED Pin Assignments
     * @{
     */
    inline constexpr uint8_t kBlinkLed = 25u;       ///< Status LED (onboard Pi Pico LED)
    inline constexpr uint8_t kConnectionLed = 26u;  ///< micro-ROS connection indicator LED
    /** @} */

    /**
     * @defgroup motor0_pins Front Left Motor (Motor 0) Pin Assignments
     * @{
     */
    inline constexpr uint8_t kMotor0In1 = 2u;      ///< Direction control pin 1
    inline constexpr uint8_t kMotor0In2 = 3u;      ///< Direction control pin 2
    inline constexpr uint8_t kMotor0Pwm = 4u;      ///< PWM speed control pin
    inline constexpr uint8_t kMotor0EncA = 5u;     ///< Encoder channel A
    inline constexpr uint8_t kMotor0EncB = 6u;     ///< Encoder channel B
    /** @} */

    /**
     * @defgroup motor1_pins Front Right Motor (Motor 1) Pin Assignments
     * @{
     */
    inline constexpr uint8_t kMotor1In1 = 7u;      ///< Direction control pin 1
    inline constexpr uint8_t kMotor1In2 = 8u;      ///< Direction control pin 2
    inline constexpr uint8_t kMotor1Pwm = 9u;      ///< PWM speed control pin
    inline constexpr uint8_t kMotor1EncA = 10u;    ///< Encoder channel A
    inline constexpr uint8_t kMotor1EncB = 11u;    ///< Encoder channel B
    /** @} */

    /**
     * @defgroup motor2_pins Rear Left Motor (Motor 2) Pin Assignments
     * @{
     */
    inline constexpr uint8_t kMotor2In1 = 12u;     ///< Direction control pin 1
    inline constexpr uint8_t kMotor2In2 = 13u;     ///< Direction control pin 2
    inline constexpr uint8_t kMotor2Pwm = 14u;     ///< PWM speed control pin
    inline constexpr uint8_t kMotor2EncA = 15u;    ///< Encoder channel A
    inline constexpr uint8_t kMotor2EncB = 20u;    ///< Encoder channel B
    /** @} */

    /**
     * @defgroup motor3_pins Rear Right Motor (Motor 3) Pin Assignments
     * @{
     */
    inline constexpr uint8_t kMotor3In1 = 21u;     ///< Direction control pin 1
    inline constexpr uint8_t kMotor3In2 = 22u;     ///< Direction control pin 2
    inline constexpr uint8_t kMotor3Pwm = 23u;     ///< PWM speed control pin
    inline constexpr uint8_t kMotor3EncA = 24u;    ///< Encoder channel A
    inline constexpr uint8_t kMotor3EncB = 26u;    ///< Encoder channel B
    /** @} */

    /**
     * @defgroup imu_pins IMU (ICM-20948) SPI Pin Assignments
     * @{
     */
    inline constexpr uint8_t kImuMiso = 16u;       ///< SPI MISO (Master In Slave Out)
    inline constexpr uint8_t kImuCs = 17u;         ///< SPI Chip Select (active low)
    inline constexpr uint8_t kImuSck = 18u;        ///< SPI Clock
    inline constexpr uint8_t kImuMosi = 19u;       ///< SPI MOSI (Master Out Slave In)
    /** @} */

    /**
     * @defgroup uart_pins UART Pin Assignments
     * @{
     */
    inline constexpr uint8_t kUart0Tx = 0u;        ///< UART0 transmit pin
    inline constexpr uint8_t kUart0Rx = 1u;        ///< UART0 receive pin
    /** @} */
}  // namespace pins

/**
 * @namespace config::robot
 * @brief Robot-specific parameters and identification
 */
namespace robot {
    /**
     * @brief Robot identification name
     * 
     * This name is used in debug logging and output to identify the robot.
     */
    inline constexpr const char* kName = "robot_xl";

    /**
     * @brief Number of motors in the mecanum drive system
     * 
     * The my_steel robot uses a 4-motor mecanum wheel configuration.
     * This constant is used for array sizing and bounds checking.
     */
    inline constexpr uint8_t kNumMotors = 4u;

    /**
     * @brief Standard FreeRTOS task priority for agent tasks
     * 
     * Most agent tasks run at this priority level, which is one level above
     * the idle task priority.
     */
    inline constexpr uint32_t kTaskPriority = tskIDLE_PRIORITY + 1UL;

    /**
     * @brief Main task stack size in words (4 bytes per word on ARM Cortex-M0+)
     * 
     * The main task needs a larger stack to accommodate object creation and
     * initialization of all agents.
     */
    inline constexpr uint32_t kMainTaskStackSize = 2048u;
}  // namespace robot

/**
 * @namespace config::pid
 * @brief PID controller tuning parameters for motor control
 * 
 * These parameters are tuned for the TB6612 motor driver and the specific motors
 * used in the my_steel robot.
 * 
 * @note Tuning these values requires careful testing. Incorrect values can cause
 * oscillation, overshoot, or poor tracking performance.
 */
namespace pid {
    /**
     * @brief Proportional gain (Kp)
     * 
     * Controls the immediate response to error. Higher values provide faster
     * response but may cause overshoot and oscillation.
     */
    inline constexpr float kProportional = 1.0f;

    /**
     * @brief Integral gain (Ki)
     * 
     * Eliminates steady-state error by accumulating error over time. Too high
     * values can cause overshoot and instability.
     */
    inline constexpr float kIntegral = 0.1f;

    /**
     * @brief Derivative gain (Kd)
     * 
     * Dampens oscillations and improves stability by responding to rate of
     * change of error. Can amplify noise if set too high.
     */
    inline constexpr float kDerivative = 0.5f;
}  // namespace pid

/**
 * @namespace config::imu
 * @brief IMU (Inertial Measurement Unit) configuration
 */
namespace imu {
    /**
     * @brief SPI bus instance for IMU communication
     * 
     * The ICM-20948 IMU is connected via SPI0.
     * 
     * @warning Do not change this value unless hardware is modified.
     * @note Cannot be constexpr because spi0 is a pointer defined at runtime
     */
    inline spi_inst_t* const kSpiBus = spi0;

    /**
     * @brief SPI baudrate for IMU communication in Hz
     * 
     * The ICM-20948 supports up to 7 MHz SPI clock, but 1 MHz is used
     * for improved reliability and noise immunity.
     */
    inline constexpr uint32_t kSpiBaudrate = 1000u * 1000u;  // 1 MHz

    /**
     * @brief ROS frame ID for IMU data messages
     * 
     * This frame ID is used in sensor_msgs/Imu messages and should match
     * the robot's URDF/TF tree.
     */
    inline constexpr const char* kFrameId = "imu_link";
}  // namespace imu

/**
 * @namespace config::pwm
 * @brief PWM (Pulse Width Modulation) configuration for motor control
 */
namespace pwm {
    /**
     * @brief Target PWM frequency in Hz
     * 
     * 20 kHz is above the audible range (reduces motor whine) and provides
     * smooth motor control with the TB6612 driver.
     */
    inline constexpr uint32_t kTargetFrequency = 20000u;  // 20 kHz
}  // namespace pwm

/**
 * @namespace config::debug
 * @brief Debug and diagnostic configuration
 */
namespace debug {
    /**
     * @defgroup debug_heartbeat Debug Heartbeat Configuration
     * @{
     */
    
    /**
     * @brief Enable/disable debug heartbeat task
     * 
     * When enabled, a periodic heartbeat message is printed to show system
     * uptime and verify the scheduler is running.
     * 
     * @note Can be overridden by defining ENABLE_DEBUG_HEARTBEAT before
     * including this header.
     */
#ifdef ENABLE_DEBUG_HEARTBEAT
    inline constexpr bool kEnableHeartbeat = ENABLE_DEBUG_HEARTBEAT;
#else
    inline constexpr bool kEnableHeartbeat = true;
#endif

    /**
     * @brief Heartbeat message interval in milliseconds
     * 
     * Controls how frequently the heartbeat task prints status messages.
     * 
     * @note Can be overridden by defining DEBUG_HEARTBEAT_INTERVAL_MS before
     * including this header.
     */
#ifdef DEBUG_HEARTBEAT_INTERVAL_MS
    inline constexpr uint32_t kHeartbeatIntervalMs = DEBUG_HEARTBEAT_INTERVAL_MS;
#else
    inline constexpr uint32_t kHeartbeatIntervalMs = 5000u;  // 5 seconds
#endif

    /** @} */
}  // namespace debug

}  // namespace config

/**
 * @section compile_time_validation Compile-Time Configuration Validation
 * 
 * These static_assert statements validate configuration constants at compile time,
 * catching configuration errors before the firmware is built. This prevents runtime
 * failures due to invalid configuration values.
 * 
 * Configuration Constraints Enforced:
 * 
 * **Motor Configuration:**
 * - Number of motors must be exactly 4 (mecanum drive requirement)
 * 
 * **Task Priorities:**
 * - Task priority must be greater than tskIDLE_PRIORITY (to ensure execution)
 * - Task priority must be less than configMAX_PRIORITIES (FreeRTOS limit)
 * 
 * **PID Controller:**
 * - All PID gains (Kp, Ki, Kd) must be non-negative
 * 
 * **PWM Configuration:**
 * - PWM frequency must be between 1 kHz and 100 kHz (hardware limits)
 * 
 * **IMU Configuration:**
 * - SPI baudrate must be between 100 kHz and 10 MHz (sensor limits)
 * 
 * **Debug Configuration:**
 * - Heartbeat interval must be at least 100ms (to avoid log spam)
 * 
 * @note If any assertion fails, the build will stop with a descriptive error message.
 * @note Additional robot physical constraints are validated in RobotController.h
 */

// Validate motor configuration
static_assert(config::robot::kNumMotors == 4u, 
    "Mecanum drive requires exactly 4 motors (front-left, front-right, rear-left, rear-right)");

// Validate task priority is above idle
static_assert(config::robot::kTaskPriority > tskIDLE_PRIORITY,
    "Task priority must be greater than idle priority to ensure tasks execute");

// Validate task priority is within valid range (FreeRTOS max priority is configMAX_PRIORITIES - 1)
static_assert(config::robot::kTaskPriority < configMAX_PRIORITIES,
    "Task priority must be less than configMAX_PRIORITIES");

// Validate PID gains are non-negative
static_assert(config::pid::kProportional >= 0.0f,
    "PID proportional gain (Kp) must be non-negative");

static_assert(config::pid::kIntegral >= 0.0f,
    "PID integral gain (Ki) must be non-negative");

static_assert(config::pid::kDerivative >= 0.0f,
    "PID derivative gain (Kd) must be non-negative");

// Validate PWM frequency is reasonable (1 kHz to 100 kHz range)
static_assert(config::pwm::kTargetFrequency >= 1000u && config::pwm::kTargetFrequency <= 100000u,
    "PWM frequency must be between 1 kHz and 100 kHz for proper motor control");

// Validate IMU SPI baudrate is reasonable (100 kHz to 10 MHz range)
static_assert(config::imu::kSpiBaudrate >= 100000u && config::imu::kSpiBaudrate <= 10000000u,
    "IMU SPI baudrate must be between 100 kHz and 10 MHz");

// Validate debug heartbeat interval is reasonable (at least 100ms)
static_assert(config::debug::kHeartbeatIntervalMs >= 100u,
    "Heartbeat interval must be at least 100ms to avoid excessive logging");

#endif  // FIRMWARE_CONFIG_H
