# Quick Reference Guide

## Configuration Constants

All configuration is centralized in `src/config/FirmwareConfig.h`.

### Pin Configuration

```cpp
// Motor pins (4 motors for mecanum drive)
config::pins::kMotor0In1, kMotor0In2, kMotor0Pwm
config::pins::kMotor0EncA, kMotor0EncB
// ... similar for motors 1-3

// IMU (SPI)
config::pins::kImuMiso, kImuCs, kImuSck, kImuMosi

// LEDs
config::pins::kBlinkLed, kConnectionLed
```

### Robot Parameters

```cpp
config::robot::kWheelRadius        // 0.065f meters
config::robot::kWheelDepth         // 0.055f meters
config::robot::kWheelsSeparation   // 0.204f meters
config::robot::kWheelsOffset       // 0.010f meters
config::robot::kNumMotors          // 4
config::robot::kRobotName          // "robot_xl"
```

### PID Tuning

```cpp
config::pid::kProportional  // 0.55f
config::pid::kIntegral      // 0.019f
config::pid::kDerivative    // 0.24f
```

### Debug Settings

```cpp
config::debug::kEnableHeartbeat
config::debug::kHeartbeatIntervalMs
config::debug::kEnableStackMonitoring
```

## Common Build Commands

```bash
# Build debug version
make build

# Build release version
make build_release

# Clean and rebuild
make clean && make build

# Flash to Pico
make flash

# View build errors (last 50 lines)
make build 2>&1 | tail -50
```

## ROS Topics

### Published by Pico

```bash
/joint_states      # sensor_msgs/JointState - motor encoder states
/odometry/wheels   # nav_msgs/Odometry - wheel odometry
/imu/data_raw      # sensor_msgs/Imu - raw IMU data
```

### Subscribed by Pico

```bash
/cmd_vel           # geometry_msgs/Twist - velocity commands
```

## Testing Commands

```bash
# Test micro-ROS connection
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0

# List topics
ros2 topic list

# Monitor IMU data
ros2 topic hz /imu/data_raw
ros2 topic echo /imu/data_raw

# Monitor odometry
ros2 topic echo /odometry/wheels

# Monitor joint states
ros2 topic echo /joint_states

# Send velocity command (forward)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Send velocity command (strafe left)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once

# Send velocity command (rotate)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

## Code Style Guidelines

### Constants

```cpp
// ✅ Good
inline constexpr float kWheelRadius = 0.065f;
inline constexpr uint32_t kTimeout = 1000u;

// ❌ Avoid
#define WHEEL_RADIUS 0.065
#define TIMEOUT 1000
```

### Enumerations

```cpp
// ✅ Good
enum class MotorDirection : uint8_t {
    Clockwise,
    CounterClockwise
};

// ❌ Avoid
enum MotorDirection {
    CLOCKWISE,
    COUNTER_CLOCKWISE
};
```

### Error Handling

```cpp
// ✅ Good
if (index >= config::robot::kNumMotors) {
    printf("[ERROR] Motor index out of bounds\n");
    return;
}

// ✅ Good
if (ptr == nullptr) {
    printf("[ERROR] Null pointer\n");
    return;
}
```

### Documentation

```cpp
/**
 * @brief Brief description
 * 
 * Detailed description explaining the purpose and behavior.
 * 
 * @param param1 Description of parameter
 * @param param2 Description of parameter
 * @return Description of return value
 * 
 * @note Important notes
 * @warning Important warnings
 */
void myFunction(int param1, float param2);
```

## Agent Architecture

### Base Agent Class

All agents inherit from `Agent`:

```cpp
class MyAgent : public Agent {
public:
    void run() override;  // Main task loop
    uint32_t getMaxStackSize() override;
};
```

### ROS Entity Interface

Agents that publish/subscribe implement `uRosEntities`:

```cpp
class MyAgent : public Agent, public uRosEntities {
public:
    void createEntities() override;
    void destroyEntities() override;
    uint32_t getCount() override;
    uint32_t getHandles() override;
    void addToExecutor(rclc_executor_t* executor) override;
};
```

## Debugging

### Serial Monitor

```bash
# Using screen
screen /dev/ttyACM0 115200

# Exit screen: Ctrl+A, then K, then Y
```

### Stack Monitoring

Each agent reports stack usage:

```cpp
uint32_t highWater = getStakHighWater();
printf("[Agent] Stack high water: %lu bytes\n", highWater);
```

### Common Log Messages

```
[HEARTBEAT] Uptime: 5000 ms
[STATE] Changed from 0 to 1
[DIAG] State=CONNECTED, Queue: 0/128, SessionReady=1
[PUB] Attempting publish...
[ERROR] Motor index out of bounds
[ERROR] Null pointer detected
```

## Performance Metrics

- **Main loop**: ~100Hz
- **IMU sampling**: 100Hz
- **Motor PID**: 50Hz per motor
- **Odometry**: 10Hz
- **micro-ROS queue**: 128 messages

## File Locations

### Configuration
- `src/config/FirmwareConfig.h` - All configuration constants

### Core Components
- `src/main.cpp` - Entry point and initialization
- `src/Agent.{h,cpp}` - Base agent class
- `src/uRosBridge.{h,cpp}` - micro-ROS singleton

### Motor Control
- `src/TB6612MotorsAgent.{h,cpp}` - Motor agent
- `src/TB6612MotorPID.{h,cpp}` - PID controller
- `src/PWMManager.{h,cpp}` - PWM generation

### Robot Control
- `src/RobotController.{h,cpp}` - Main controller (DDD)

### Sensors
- `src/application/ImuAgent.{h,cpp}` - IMU integration

### Documentation
- `README.md` - General documentation
- `MODERNIZATION.md` - Modernization details
- `CHANGELOG.md` - Version history
- `INTEGRATION_TEST_SUMMARY.md` - Test results

## Troubleshooting Quick Fixes

### Build fails
```bash
make clean
git submodule update --init --recursive
make build
```

### Can't flash
```bash
# Hold BOOTSEL button while connecting USB
# Pico should appear as USB drive
cp build/src/my_firmware.uf2 /media/RPI-RP2/
```

### No micro-ROS connection
```bash
# Check permissions
sudo usermod -a -G dialout $USER
# Log out and back in

# Try different port
ls /dev/ttyACM*
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### Motors not responding
```bash
# Check power supply (motors need separate power)
# Verify motor driver connections
# Test with simple velocity command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

## Additional Resources

- [FreeRTOS Documentation](https://www.freertos.org/Documentation/RTOS_book.html)
- [micro-ROS Documentation](https://micro.ros.org/docs/)
- [Pico SDK Documentation](https://raspberrypi.github.io/pico-sdk-doxygen/)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
