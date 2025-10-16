# my_steel Robot Firmware

[![Build Firmware](https://github.com/goldjunge91/robot_firmware/actions/workflows/build.yml/badge.svg)](https://github.com/goldjunge91/robot_firmware/actions/workflows/build.yml)
[![Run Tests](https://github.com/goldjunge91/robot_firmware/actions/workflows/test.yml/badge.svg)](https://github.com/goldjunge91/robot_firmware/actions/workflows/test.yml)
[![Create Release](https://github.com/goldjunge91/robot_firmware/actions/workflows/firmware_release.yml/badge.svg)](https://github.com/goldjunge91/robot_firmware/actions/workflows/firmware_release.yml)

Raspberry Pi Pico firmware for the my_steel robot, implementing motor control, sensor integration, and micro-ROS communication.

## Recent Modernization (2025)

The firmware has been modernized to use modern C++ practices while maintaining full backward compatibility:

### Key Improvements

- **Modern C++ Configuration**: Replaced `#define` macros with type-safe `inline constexpr` constants organized in namespaces
- **Enhanced Type Safety**: Added explicit type annotations, bounds checking, and null pointer validation
- **Improved Documentation**: Added comprehensive Doxygen comments for all public APIs
- **Simplified Codebase**: Removed unused HCSR04 ultrasonic sensor support
- **Better Error Handling**: Added runtime validation with clear error messages

### C++ Requirements

- **Minimum Standard**: C++14 (C++17 recommended)
- **Compiler**: GCC ARM toolchain with C++14+ support
- **Features Used**: `constexpr`, scoped enums, default member initializers, explicit type conversions

### Configuration System

All firmware configuration is now centralized in `src/config/FirmwareConfig.h` using namespace-based constants:

```cpp
// Pin configuration
config::pins::kMotor0In1
config::pins::kMotor0Pwm
config::pins::kImuCs

// Robot parameters
config::robot::kRobotName
config::robot::kTaskPriority

// PID tuning
config::pid::kProportional
config::pid::kIntegral
config::pid::kDerivative

// Debug settings
config::debug::kEnableHeartbeat
config::debug::kHeartbeatIntervalMs
```

This replaces the old `#define` macro system and provides better type safety and IDE support.

## Architecture

The firmware uses a modular agent-based architecture built on FreeRTOS:

- **uRosBridge**: Singleton managing micro-ROS communication over USB
- **TB6612MotorsAgent**: PID-controlled motor management with encoder feedback for 4 mecanum wheels
- **ImuAgent**: IMU (ICM20948) data acquisition via SPI
- **RobotController**: Main robot control agent handling odometry, cmd_vel, and mecanum kinematics
- **BlinkAgent**: LED status indicator

### Removed Components

The following sensors have been removed as part of the modernization:
- **HCSR04Agent**: Ultrasonic distance sensor (removed in 2025)
- **distance_sensor library**: PIO-based distance sensor support (removed)

## Hardware Configuration

### Pin Mapping

| Component           | Pin  | Function             |
| ------------------- | ---- | -------------------- |
| Left Motor PWM CW   | GP9  | Motor control        |
| Left Motor PWM CCW  | GP8  | Motor control        |
| Left Encoder A      | GP14 | Encoder feedback     |
| Left Encoder B      | GP15 | Encoder feedback     |
| Right Motor PWM CW  | GP6  | Motor control        |
| Right Motor PWM CCW | GP7  | Motor control        |
| Right Encoder A     | GP12 | Encoder feedback     |
| Right Encoder B     | GP13 | Encoder feedback     |
| IMU CS              | GP17 | SPI chip select      |
| IMU SCK             | GP18 | SPI clock            |
| IMU MOSI            | GP19 | SPI data out         |
| IMU MISO            | GP16 | SPI data in          |
| VL6180X SDA         | GP2  | I2C data             |
| VL6180X SCL         | GP3  | I2C clock            |
| Status LED          | GP2  | Connection indicator |
| Debug LED           | GP3  | Activity indicator   |

### Communication

- **USB**: micro-ROS transport (primary communication with ROS2)
- **UART0 (GP0/GP1)**: Debug output at 115200 baud

## Building

### Prerequisites

- Raspberry Pi Pico SDK
- CMake 3.13+
- GCC ARM toolchain with C++14 or C++17 support
- FreeRTOS (included as submodule)
- micro-ROS for Pico (included as submodule)
- Eigen3 (included as submodule for linear algebra)

### Build Commands

```bash
# Debug build
make build

# Release build  
make build_release

# Clean build artifacts
make clean
```



## Flashing

### Method 1: USB Mass Storage (BOOTSEL)

1. Hold BOOTSEL button while connecting Pico to USB
2. Pico appears as USB drive "RPI-RP2"
3. Copy `build/src/my_firmware.uf2` to the drive
4. Pico automatically reboots with new firmware

```bash
# Automated flashing (if mounted)
make flash
```

### Method 2: picotool

```bash
# Install picotool first
sudo apt install picotool

# Flash and reboot
picotool load -f build/src/my_firmware.uf2
picotool reboot
```

## Testing

### Unit Tests

Run the test suite locally:

```bash
# Build and run all tests
make test

# Run with verbose output
make test-verbose

# Run specific test suites
make test-pid        # Motor PID tests
make test-odometry   # Odometry tests
make test-math       # Math utility tests
```

The test suite includes:
- **Unit Tests**: Motor PID, TB6612 driver, math utilities
- **Integration Tests**: Mecanum kinematics, odometry, ROS topic names
- **HAL Tests**: Hardware abstraction layer mocks

All 70 tests should pass before committing changes.

## Debugging

### Monitor Debug Output

```bash
# Monitor UART debug output
screen /dev/ttyAMA0 115200
# or
minicom -D /dev/ttyAMA0 -b 115200
```

### Test micro-ROS Connection

```bash
# Start micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v
```

### Expected ROS Topics

When firmware is running and micro-ROS agent is connected:

```bash
# Published by Pico
/joint_states      # sensor_msgs/JointState - motor encoder states
/odometry/wheels   # nav_msgs/Odometry - wheel odometry
/imu/data_raw      # sensor_msgs/Imu - raw IMU data

# Subscribed by Pico  
/cmd_vel           # geometry_msgs/Twist - velocity commands (mecanum kinematics)
```

**Note**: HCSR04 ultrasonic sensor topics (`/range_front`, `/range_back`) have been removed as of 2025 modernization.

## Configuration

### Centralized Configuration

All firmware configuration is now centralized in `src/config/FirmwareConfig.h`. This provides type-safe, namespace-organized constants with full IDE support.

### Robot Parameters

Edit `src/config/FirmwareConfig.h` for robot-specific parameters:

```cpp
namespace config {
namespace robot {
    inline constexpr float kWheelRadius = 0.065f;      // meters
    inline constexpr float kWheelDepth = 0.055f;       // meters  
    inline constexpr float kWheelsSeparation = 0.204f; // meters
    inline constexpr float kWheelsOffset = 0.010f;     // meters
    inline constexpr size_t kNumMotors = 4;            // mecanum drive
}
}
```

### PID Tuning

Edit `src/config/FirmwareConfig.h` for motor PID parameters:

```cpp
namespace config {
namespace pid {
    inline constexpr float kProportional = 0.55f;  // P gain
    inline constexpr float kIntegral = 0.019f;     // I gain  
    inline constexpr float kDerivative = 0.24f;    // D gain
}
}
```

### Pin Configuration

All pin assignments are defined in `src/config/FirmwareConfig.h`:

```cpp
namespace config {
namespace pins {
    // Motor 0 (Front Left)
    inline constexpr uint8_t kMotor0In1 = 2;
    inline constexpr uint8_t kMotor0In2 = 3;
    inline constexpr uint8_t kMotor0Pwm = 4;
    inline constexpr uint8_t kMotor0EncA = 5;
    inline constexpr uint8_t kMotor0EncB = 6;
    
    // IMU (SPI)
    inline constexpr uint8_t kImuMiso = 16;
    inline constexpr uint8_t kImuCs = 17;
    inline constexpr uint8_t kImuSck = 18;
    inline constexpr uint8_t kImuMosi = 19;
    
    // LEDs
    inline constexpr uint8_t kBlinkLed = 25;
    inline constexpr uint8_t kConnectionLed = 26;
}
}
```

### Debug Configuration

Enable/disable debug features in `src/config/FirmwareConfig.h`:

```cpp
namespace config {
namespace debug {
    inline constexpr bool kEnableHeartbeat = true;
    inline constexpr uint32_t kHeartbeatIntervalMs = 5000u;
    inline constexpr bool kEnableStackMonitoring = true;
}
}
```

## Migration Guide (Pre-2025 to Modernized)

If you're updating from the pre-modernization firmware:

### Configuration Changes

**Old approach (macros):**
```cpp
#define MOTOR_0_IN1 2
#define KP 0.55
```

**New approach (namespaced constants):**
```cpp
config::pins::kMotor0In1
config::pid::kProportional
```

### Removed Features

- **HCSR04 ultrasonic sensor**: Completely removed including distance_sensor library
- **Range topics**: `/range_front` and `/range_back` topics no longer published
- **distance_sensor.cmake**: Removed as it's no longer needed

### Build System Changes

The CMakeLists.txt now requires C++14 minimum:

```cmake
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
```

Ensure your toolchain supports C++14 or later (most modern ARM GCC toolchains do).

### Testing After Migration

1. **Build verification**: `make build` should complete without warnings
2. **Flash test**: Upload firmware and verify boot sequence via UART
3. **Connection test**: Verify micro-ROS agent connection
4. **Topic verification**: Check that `/joint_states`, `/imu/data_raw`, `/odometry/wheels` are published
5. **Motor test**: Send `/cmd_vel` commands and verify motor response
6. **Integration test**: Run 30-minute stress test to verify stability

See `INTEGRATION_TEST_SUMMARY.md` for detailed testing procedures.

## Troubleshooting

### Common Issues

1. **No USB device detected**
   - Check USB cable (data cable, not power-only)
   - Verify Pico is not in BOOTSEL mode
   - Try different USB port

2. **micro-ROS agent connection fails**
   - Check serial port permissions: `sudo usermod -a -G dialout $USER`
   - Verify correct baud rate (115200)
   - Ensure no other processes using the port

3. **No debug output on UART**
   - Check UART wiring (GP0=TX, GP1=RX)
   - Verify baud rate (115200)
   - Try different terminal program

4. **Motors not responding**
   - Check motor driver connections
   - Verify power supply (motors need separate power)
   - Check encoder wiring

5. **Build errors after updating**
   - Verify C++14 or C++17 support in toolchain
   - Check that `CMAKE_CXX_STANDARD` is set to 14 or higher
   - Ensure all submodules are updated: `git submodule update --init --recursive`
   - Clean build directory: `make clean && make build`

6. **Missing sensor topics after update**
   - HCSR04 ultrasonic sensor has been removed (expected behavior)
   - Check that expected topics are `/joint_states`, `/imu/data_raw`, `/odometry/wheels`
   - VL6180X ToF sensor is optional and can be disabled if not needed

### Debug Logging

The firmware provides extensive UART logging:

```
[HEARTBEAT] Uptime: 5000 ms
[STATE] Changed from 0 to 1
[DIAG] State=CONNECTED, Queue: 0/128, SessionReady=1
[PUB] Attempting publish...
[TIMER PUB] pico_count=42 publish_ret=0 count=5
```

## Development

### Code Quality Standards

The modernized codebase follows these standards:

- **Type Safety**: Use explicit types with suffix notation (`0.5f` for float, `100u` for unsigned)
- **Constants**: Use `inline constexpr` in namespaces instead of `#define` macros
- **Enums**: Use scoped enums (`enum class`) for type safety
- **Error Handling**: Add null checks and bounds validation with clear error messages
- **Documentation**: Use Doxygen comments for all public APIs

### Adding New Sensors

1. Create new agent class inheriting from `Agent`
2. Implement hardware initialization in constructor
3. Add configuration constants to `src/config/FirmwareConfig.h`
4. Add to main task initialization in `main.cpp`
5. Register with `RobotController` agent for ROS integration if needed
6. Add comprehensive Doxygen documentation

### Modifying ROS Interface

1. Edit `RobotController.h` and `RobotController.cpp` for new topics
2. Update entity counts in `getCount()` and `getHandles()`
3. Add publishers/subscribers in `createEntities()`
4. Handle messages in `handleSubscriptionMsg()`
5. Update documentation to reflect new topics

### Type Safety Guidelines

```cpp
// ✅ Good: Explicit types with suffix notation
inline constexpr float kWheelRadius = 0.065f;
inline constexpr uint32_t kTimeout = 1000u;
inline constexpr double kPreciseValue = 3.14159265359;

// ❌ Avoid: Implicit types
#define WHEEL_RADIUS 0.065
#define TIMEOUT 1000

// ✅ Good: Scoped enum
enum class MotorDirection : uint8_t {
    Clockwise,
    CounterClockwise
};

// ❌ Avoid: Unscoped enum
enum MotorDirection {
    CLOCKWISE,
    COUNTER_CLOCKWISE
};

// ✅ Good: Bounds checking
void setMotorSpeed(uint index, float speed) {
    if (index >= config::robot::kNumMotors) {
        printf("[ERROR] Motor index out of bounds\n");
        return;
    }
    // ... safe to proceed
}

// ❌ Avoid: No validation
void setMotorSpeed(uint index, float speed) {
    motors[index]->setSpeed(speed);  // potential crash
}
```

## Performance Notes

- Main loop runs at ~100Hz
- micro-ROS publishing is queued (128 message buffer)
- IMU sampling at 100Hz
- Motor PID control at 50Hz (per motor)
- Odometry calculation at 10Hz
- Stack monitoring available via `getStakHighWater()` for each agent

### Performance Impact of Modernization

The modernization maintains or improves performance:

- **Compile-time optimization**: `constexpr` enables compile-time evaluation
- **Zero-cost abstractions**: Scoped enums and namespaces have no runtime overhead
- **Minimal validation overhead**: Bounds checking adds single comparison per call
- **Reduced binary size**: Removed HCSR04 sensor code reduces flash usage

## Dependencies

- **FreeRTOS**: Real-time task scheduling (v10.6.2)
- **micro-ROS**: ROS2 communication over USB (humble branch)
- **Pico SDK**: Hardware abstraction (v1.5.1)
- **Eigen**: Linear algebra for odometry calculations (v3.4.0)









## CI/CD Workflows

### Automatic Build Checks

Every push to any branch triggers a build workflow that:
- ✅ Verifies firmware builds successfully
- ✅ Generates checksums and size information
- ✅ Uploads build artifacts (retained for 30 days)
- ✅ Shows build summary in GitHub Actions

This ensures code changes don't break the build.

### Creating Releases

Releases are created **manually** when you're ready:

1. Go to GitHub Actions → "Create Release" workflow
2. Click "Run workflow"
3. Enter version (e.g., `v1.0.0`)
4. Click "Run workflow"

The workflow will:
- ✅ Build firmware
- ✅ Create git tag
- ✅ Generate all artifacts with German date format (dd.mm.yyyy)
- ✅ Create GitHub Release with:
  - `my_firmware.uf2` (flashable binary)
  - `my_firmware.elf` (debug symbols)
  - `my_firmware.uf2.sha256` (checksum)
  - `my_firmware.elf.sha256` (checksum)
  - `firmware_size.txt` (memory usage)
  - `build_info.txt` (complete build information)

### Local Release Build

```bash
# Build release version locally
make release VERSION=v1.0.0
```

This creates `releases/my_firmware_v1.0.0.uf2` locally.

### Versioning

**Format:** `vMAJOR.MINOR.PATCH`

- `v1.0.0` - First stable release
- `v1.1.0` - New features
- `v1.1.1` - Bugfixes

**Note:** Binary files (.uf2) should NOT be committed to the repository. They are uploaded to GitHub Releases instead.