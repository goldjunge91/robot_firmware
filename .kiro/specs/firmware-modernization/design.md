# Design Document

## Overview

This design document outlines the approach for modernizing the my_steel robot firmware codebase. The firmware is a FreeRTOS-based embedded system running on Raspberry Pi Pico, interfacing with ROS2 via micro-ROS. The modernization will introduce modern C++ features (C++14/17), remove unused sensor code, and improve code quality while maintaining 100% functional compatibility with the existing system.

### Design Goals

1. **Zero Functional Regression**: All motor control, IMU, odometry, and micro-ROS functionality must work identically
2. **Improved Maintainability**: Modern C++ features make code more readable and type-safe
3. **Reduced Complexity**: Remove unused HCSR04 and VL6180X sensor code
4. **Better Documentation**: Maintain or improve existing documentation standards
5. **Incremental Approach**: Changes can be tested and validated step-by-step

### Scope

**In Scope:**
- Modern C++ language features (constexpr, scoped enums, smart pointers where beneficial)
- Removal of HCSR04Agent and VL6180X sensor code
- Type safety improvements
- Documentation enhancements
- Code consistency improvements

**Out of Scope:**
- Algorithmic changes to PID control, odometry, or kinematics
- Changes to micro-ROS communication protocol
- Hardware pin assignments
- FreeRTOS task scheduling or priorities
- Build system overhaul (minimal CMake changes only)

## Architecture

### Current System Architecture

The firmware follows an Agent-based architecture where each major subsystem is encapsulated in an Agent class that runs as a FreeRTOS task:

```
┌─────────────────────────────────────────────────────────────┐
│                         main.cpp                             │
│                    (Initialization & Boot)                   │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ├─> BlinkAgent (LED status)
                       │
                       ├─> TB6612MotorsAgent (4 mecanum motors)
                       │   └─> TB6612MotorPID (per-motor PID)
                       │
                       ├─> ImuAgent (ICM20948 sensor)
                       │
                       ├─> DDD Agent (odometry & cmd_vel)
                       │   ├─> Subscribes: /cmd_vel
                       │   └─> Publishes: /odom
                       │
                       └─> uRosBridge (micro-ROS communication)
                           ├─> Entity management
                           ├─> Publisher queue
                           └─> Executor management
```

### Agent Base Class Pattern

All agents inherit from the `Agent` base class which provides:
- FreeRTOS task lifecycle management (`start()`, `stop()`)
- Pure virtual `run()` method for task loop
- Stack size configuration via `getMaxStackSize()`
- Stack monitoring via `getStakHighWater()`

### uRosEntities Interface

Agents that publish/subscribe to ROS topics implement the `uRosEntities` interface:
- `createEntities()` - Initialize ROS publishers/subscribers
- `destroyEntities()` - Clean up ROS entities
- `getCount()` - Return number of entities
- `getHandles()` - Return executor handle count
- `addToExecutor()` - Register with micro-ROS executor

### Refactoring Strategy

The refactoring will maintain this architecture while modernizing implementation details within each component.

## Components and Interfaces

### 1. Configuration Constants (main.cpp)

**Current State:**
- Uses `#define` macros for all constants
- Pin assignments embedded in main.cpp
- Magic numbers scattered throughout

**Modernized Design:**

```cpp
// Pin configuration namespace
namespace config {
namespace pins {
    // LED pins
    inline constexpr uint8_t kBlinkLed = 25;
    inline constexpr uint8_t kConnectionLed = 26;
    
    // Motor 0 (Front Left)
    inline constexpr uint8_t kMotor0In1 = 2;
    inline constexpr uint8_t kMotor0In2 = 3;
    inline constexpr uint8_t kMotor0Pwm = 4;
    inline constexpr uint8_t kMotor0EncA = 5;
    inline constexpr uint8_t kMotor0EncB = 6;
    
    // ... (similar for motors 1-3)
    
    // IMU (SPI)
    inline constexpr uint8_t kImuMiso = 16;
    inline constexpr uint8_t kImuCs = 17;
    inline constexpr uint8_t kImuSck = 18;
    inline constexpr uint8_t kImuMosi = 19;
}  // namespace pins

namespace robot {
    inline constexpr const char* kRobotName = "robot_xl";
    inline constexpr uint32_t kTaskPriority = tskIDLE_PRIORITY + 1UL;
}  // namespace robot

namespace pid {
    inline constexpr float kProportional = 1.0f;
    inline constexpr float kIntegral = 0.1f;
    inline constexpr float kDerivative = 0.5f;
}  // namespace pid

namespace debug {
    inline constexpr bool kEnableHeartbeat = true;
    inline constexpr uint32_t kHeartbeatIntervalMs = 5000u;
}  // namespace debug
}  // namespace config
```

**Rationale:**
- `inline constexpr` allows header-only constants without ODR violations
- Namespaces provide logical grouping and prevent naming conflicts
- Explicit types (uint8_t, float, etc.) improve type safety
- Suffix notation (f, u) makes types explicit

### 2. Sensor Code Removal

**Phase 1: Comment Out (Initial Implementation)**

```cpp
// In main.cpp:
// TODO: delete after successful micro-ROS-Agent connection-Test
// #include "HCSR04Agent.h"
// #include "application/vl6180xAgent.hpp"

// In mainTask():
// TODO: delete after successful micro-ROS-Agent connection-Test
// static HCSR04Agent range;
// range.addSensor(0, "range_front");
// range.addSensor(18, "range_back");
// range.start("Range", TASK_PRIORITY);

// TODO: delete after successful micro-ROS-Agent connection-Test
// hal::hardware::Vl6180x::Config tof_cfg{};
// tof_cfg.bus = VL6180X_I2C_PORT;
// ...
// static application::Vl6180xAgent tof(tof_cfg);
// tof.start("VL6180X", TASK_PRIORITY);
```

```cpp
// In DDD.h:
class DDD : public Agent, public uRosEntities {
public:
    // ...
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // void setHCSR04Agent(HCSR04Agent *p);
    // void setVl6180xAgent(application::Vl6180xAgent *p);
    
private:
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // HCSR04Agent *pHCSR04Agent = NULL;
    // application::Vl6180xAgent *pVl6180xAgent = NULL;
};
```

**Phase 2: Complete Removal (After Testing)**

After successful micro-ROS connection tests, completely remove:
- `src/HCSR04Agent.h` and `src/HCSR04Agent.cpp`
- `src/application/vl6180xAgent.hpp` and `src/application/vl6180xAgent.cpp`
- `src/hal/hardware/vl6180x/` directory
- All commented-out references in DDD.cpp
- Distance sensor CMake configuration

### 3. Type Safety Improvements

**Enumerations:**

```cpp
// Current: unscoped enum (pollutes namespace)
enum MotorDirection {
    CLOCKWISE,
    COUNTER_CLOCKWISE
};

// Modernized: scoped enum
enum class MotorDirection : uint8_t {
    Clockwise,
    CounterClockwise
};

// Usage:
motor.setDirection(MotorDirection::Clockwise);
```

**Numeric Literals:**

```cpp
// Current: implicit types
#define WHEEL_RADIUS 0.065
#define WHEELS_SEP 0.204

// Modernized: explicit types
inline constexpr float kWheelRadius = 0.065f;  // meters
inline constexpr float kWheelSeparation = 0.204f;  // meters
inline constexpr double kWheelOffset = 0.010;  // meters (double for precision)
```

**Array Bounds:**

```cpp
// Current: C-style array with macro
#define NUM_MOTORS 4
TB6612MotorPID *pMotors[NUM_MOTORS];

// Modernized: std::array (if beneficial) or keep as-is with constexpr
inline constexpr size_t kNumMotors = 4;
TB6612MotorPID *pMotors[kNumMotors];  // Keep raw array for embedded compatibility

// Alternative (if std::array is available and beneficial):
std::array<TB6612MotorPID*, kNumMotors> pMotors;
```

### 4. Smart Pointers (Selective Use)

**Analysis:**
- Most pointers in the firmware are non-owning (e.g., `pMotorsAgent`, `pImuAgent`)
- Static objects are created in `mainTask()` and live for the program lifetime
- FreeRTOS tasks manage their own lifecycle

**Decision:**
- **Do NOT use smart pointers** for Agent references (they're non-owning pointers)
- **Consider std::unique_ptr** only for dynamically allocated resources with clear ownership
- Keep raw pointers for FreeRTOS task handles and micro-ROS entities (C API compatibility)

**Rationale:**
- Embedded systems benefit from predictable memory allocation
- Static allocation is preferred over dynamic allocation
- Smart pointers add overhead and complexity without significant benefit here
- C API interop (micro-ROS, FreeRTOS) requires raw pointers

### 5. Documentation Standards

**Doxygen Style for Public APIs:**

```cpp
/**
 * @brief Set the target speed for a motor in radians per second
 * 
 * This method configures the PID controller to maintain the specified
 * angular velocity. The actual motor speed will converge to the target
 * over several PID iterations.
 * 
 * @param index Motor index (0-3 for mecanum drive)
 * @param rps Target angular velocity in radians per second
 * @param cw Direction: true for clockwise, false for counter-clockwise
 * 
 * @note The motor must be initialized before calling this method
 * @warning Index out of bounds will be silently ignored
 */
virtual void setSpeedRadPS(uint index, float rps, bool cw) = 0;
```

**Inline Comments for Implementation:**

```cpp
// Calculate wheel velocities using mecanum inverse kinematics
// Formula: v_wheel = (v_x ± v_y ± ω*L) / r
// where L is the distance from robot center to wheel
double vfl = (vx - vy - omega * L) / WHEEL_RADIUS;  // Front left
double vfr = (vx + vy + omega * L) / WHEEL_RADIUS;  // Front right
```

### 6. Error Handling

**Null Pointer Checks:**

```cpp
// Current: implicit assumption
void DDD::setMotorsAgent(BaseMotorsAgent *p) {
    pMotorsAgent = p;
}

// Modernized: explicit validation
void DDD::setMotorsAgent(BaseMotorsAgent *p) {
    if (p == nullptr) {
        printf("[DDD] ERROR: Attempted to set null MotorsAgent\n");
        return;
    }
    pMotorsAgent = p;
}
```

**Bounds Checking:**

```cpp
// Current: no bounds check
void TB6612MotorsAgent::setSpeedRadPS(uint index, float rps, bool cw) {
    if (pMotors[index] != NULL) {
        pMotors[index]->setSpeedRadPS(rps, cw);
    }
}

// Modernized: explicit bounds check
void TB6612MotorsAgent::setSpeedRadPS(uint index, float rps, bool cw) {
    if (index >= kNumMotors) {
        printf("[Motors] ERROR: Motor index %u out of bounds (max: %zu)\n", 
               index, kNumMotors - 1);
        return;
    }
    if (pMotors[index] != nullptr) {
        pMotors[index]->setSpeedRadPS(rps, cw);
    }
}
```

## Data Models

### Odometry State

```cpp
// Current: C-style struct with typedef
struct DDDOdom {
    double x;
    double y;
    double a;
};
typedef struct DDDOdom DDDOdom_t;

// Modernized: C++ struct with constructor and methods
struct OdometryState {
    double x{0.0};  // Position X in meters
    double y{0.0};  // Position Y in meters
    double a{0.0};  // Angle in radians
    
    // Constructor with default values
    constexpr OdometryState(double x_pos = 0.0, double y_pos = 0.0, double angle = 0.0)
        : x(x_pos), y(y_pos), a(angle) {}
    
    // Reset to origin
    void reset() {
        x = 0.0;
        y = 0.0;
        a = 0.0;
    }
};
```

### Motor Configuration

```cpp
// Encapsulate motor pin configuration
struct MotorPinConfig {
    uint8_t in1_pin;
    uint8_t in2_pin;
    uint8_t pwm_pin;
    uint8_t encoder_a_pin;
    uint8_t encoder_b_pin;
    
    constexpr MotorPinConfig(uint8_t in1, uint8_t in2, uint8_t pwm,
                             uint8_t enc_a, uint8_t enc_b)
        : in1_pin(in1), in2_pin(in2), pwm_pin(pwm),
          encoder_a_pin(enc_a), encoder_b_pin(enc_b) {}
};

// Usage in main.cpp:
constexpr MotorPinConfig kMotorConfigs[4] = {
    {2, 3, 4, 5, 6},      // Motor 0: Front Left
    {7, 8, 9, 10, 11},    // Motor 1: Front Right
    {12, 13, 14, 15, 20}, // Motor 2: Rear Left
    {21, 22, 23, 24, 26}  // Motor 3: Rear Right
};
```

## Error Handling

### Strategy

1. **Compile-Time Checks**: Use `static_assert` for configuration validation
2. **Runtime Validation**: Check pointers, bounds, and state before operations
3. **Logging**: Use printf for error messages (micro-ROS logging not available)
4. **Graceful Degradation**: Continue operation when possible, stop only on critical errors

### Examples

```cpp
// Compile-time validation
static_assert(kNumMotors == 4, "Mecanum drive requires exactly 4 motors");
static_assert(kWheelRadius > 0.0f, "Wheel radius must be positive");

// Runtime validation with early return
if (pMotorsAgent == nullptr) {
    printf("[DDD] ERROR: MotorsAgent not initialized\n");
    return;
}

// State validation
if (!uRosBridge::getInstance()->isSessionReady()) {
    // Don't attempt to publish if micro-ROS not connected
    return;
}
```

## Testing Strategy

### Unit Testing Approach

The firmware currently has limited unit testing infrastructure. The modernization will:

1. **Preserve Existing Tests**: Keep all tests in `tests/` directory functional
2. **Manual Testing**: Primary validation through hardware testing
3. **Incremental Validation**: Test after each major change

### Test Plan

**Phase 1: Build Verification**
- Compile debug build: `make build`
- Compile release build: `make build_release`
- Verify no new warnings or errors

**Phase 2: Flash and Boot Test**
- Flash firmware to Pico
- Monitor serial output for boot sequence
- Verify all agents start successfully
- Check for heartbeat messages

**Phase 3: Micro-ROS Connection Test**
- Start micro-ROS agent on host
- Verify Pico connects successfully
- Check topic list: `ros2 topic list`
- Verify expected topics appear:
  - `/joint_states`
  - `/imu/data_raw`
  - `/odom`
  - `/cmd_vel` (subscription)

**Phase 4: Motor Control Test**
- Publish test velocity command:
  ```bash
  ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
  ```
- Verify motors respond
- Check `/joint_states` for encoder feedback
- Test all movement directions (forward, backward, strafe, rotate)

**Phase 5: Sensor Data Test**
- Monitor IMU data: `ros2 topic echo /imu/data_raw`
- Verify data rate (~100Hz)
- Check odometry: `ros2 topic echo /odom`
- Verify odometry updates during movement

**Phase 6: Stress Test**
- Run continuous operation for 30+ minutes
- Monitor stack high water marks
- Check for memory leaks or task failures
- Verify no micro-ROS disconnections

### Regression Testing

After each code change:
1. Build both debug and release
2. Flash and verify boot
3. Test micro-ROS connection
4. Test basic motor control
5. Verify sensor data publishing

## Performance Considerations

### Memory Usage

**Stack Allocation:**
- Monitor via `getStakHighWater()` for each agent
- Current allocations are conservative, should not change
- `constexpr` reduces runtime memory usage (compile-time evaluation)

**Code Size:**
- Modern C++ features should not significantly increase binary size
- `inline constexpr` may slightly increase code size but improves performance
- Removing HCSR04 and VL6180X code will reduce binary size

### Timing Constraints

**Critical Paths:**
1. **Motor PID Loop**: Must maintain ~100Hz update rate
2. **IMU Sampling**: 100Hz publication rate
3. **Odometry Calculation**: 10Hz (100ms delay in DDD::run())
4. **Micro-ROS Executor**: Must process messages without blocking

**Impact Analysis:**
- `constexpr` improves performance (compile-time evaluation)
- Scoped enums have zero runtime overhead
- Bounds checking adds minimal overhead (single comparison)
- Null pointer checks add minimal overhead

### Optimization Guidelines

1. **Keep Hot Paths Lean**: Don't add heavy operations in PID or IMU loops
2. **Use constexpr**: Prefer compile-time computation over runtime
3. **Avoid Dynamic Allocation**: Continue using static allocation
4. **Minimize Logging**: Keep printf calls outside critical loops

## Migration Path

### Phase 1: Configuration Constants (Low Risk)
1. Create `config` namespace in new header file
2. Replace `#define` macros with `inline constexpr`
3. Update main.cpp to use new constants
4. Test build and flash

### Phase 2: Comment Out Sensors (Low Risk)
1. Comment out HCSR04 includes and initialization
2. Comment out VL6180X includes and initialization
3. Comment out sensor references in DDD
4. Add TODO markers
5. Test micro-ROS connection

### Phase 3: Type Safety (Medium Risk)
1. Add explicit type suffixes to numeric literals
2. Add bounds checking to array accesses
3. Add null pointer checks
4. Test motor control and sensors

### Phase 4: Documentation (Low Risk)
1. Add Doxygen comments to public APIs
2. Improve inline comments
3. Document design decisions
4. No functional changes

### Phase 5: Complete Sensor Removal (Medium Risk)
1. Verify micro-ROS connection stable
2. Delete commented code
3. Delete sensor source files
4. Update CMakeLists.txt
5. Final integration test

### Phase 6: Data Model Modernization (Low Risk)
1. Replace typedef structs with modern structs
2. Add constructors and methods
3. Update usage sites
4. Test odometry calculations

## Dependencies

### Build System

**CMakeLists.txt Changes:**
- May need to update C++ standard: `set(CMAKE_CXX_STANDARD 14)` or `17`
- Remove HCSR04 and VL6180X source files from build
- No other significant changes expected

### External Libraries

**No Changes Required:**
- FreeRTOS: C API, no impact
- micro-ROS: C API, no impact
- Pico SDK: C API, no impact
- Eigen: C++ library, already compatible

### Compiler Requirements

- GCC ARM toolchain must support C++14 minimum (already does)
- No new library dependencies
- No changes to linker scripts

## Risks and Mitigation

### Risk 1: Breaking Micro-ROS Communication
**Likelihood**: Low  
**Impact**: High  
**Mitigation**:
- Test micro-ROS connection after each phase
- Keep C API interfaces unchanged
- Incremental testing approach

### Risk 2: Performance Degradation
**Likelihood**: Very Low  
**Impact**: Medium  
**Mitigation**:
- Profile critical paths before/after
- Monitor stack usage
- Use constexpr for compile-time optimization

### Risk 3: Build System Issues
**Likelihood**: Low  
**Impact**: Medium  
**Mitigation**:
- Test both debug and release builds
- Minimal CMake changes
- Keep existing build workflow

### Risk 4: Introducing Bugs
**Likelihood**: Medium  
**Impact**: High  
**Mitigation**:
- Incremental changes with testing
- Code review before merging
- Comprehensive test plan
- Keep changes minimal and focused

## Conclusion

This design provides a pragmatic approach to modernizing the firmware while maintaining stability and functionality. The incremental migration path allows for testing and validation at each step, minimizing risk. Modern C++ features are applied judiciously, focusing on improvements that provide clear benefits without adding complexity or overhead.

The removal of unused sensor code simplifies the codebase, and improved type safety and documentation make the code more maintainable for future development.
