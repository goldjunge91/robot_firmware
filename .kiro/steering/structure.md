# Project Structure

## Root Layout

```
├── src/                    # Firmware source code
├── tests/                  # Unit tests (GoogleTest)
├── port/                   # FreeRTOS configuration
├── lib/                    # Third-party libraries (submodules)
├── releases/               # Built firmware binaries (.uf2)
├── build/                  # CMake build output (gitignored)
├── CMakeLists.txt          # Root build configuration
└── Makefile                # Build task wrapper
```

## Source Organization (`src/`)

### Core Architecture

- `main.cpp` - Entry point, agent initialization, FreeRTOS launch
- `Agent.{h,cpp}` - Abstract base class for FreeRTOS task agents
- `config/FirmwareConfig.h` - Centralized configuration (pins, PID, timing)

### Application Layer (`src/application/`)

High-level sensor agents:
- `ImuAgent` - ICM-20948 IMU data acquisition
- `vl6180xAgent` - VL6180X time-of-flight sensor

### Hardware Abstraction (`src/hal/`)

- `hal/hardware/` - Concrete hardware drivers (SPI, I2C devices)
- Testable interfaces separate from Pico SDK dependencies

### Motor Control

- `MotorMgr`, `MotorPID` - Generic motor control interfaces
- `TB6612MotorMgr`, `TB6612MotorPID` - TB6612 driver implementation
- `MotorsAgent`, `TB6612MotorsAgent` - FreeRTOS task wrappers
- `PWMManager` - PWM signal generation

### Robot Control

- `DDD.{h,cpp}` - Main robot controller (odometry, cmd_vel, sensor fusion)
- `uRosBridge` - Singleton managing micro-ROS lifecycle
- `uRosEntities` - Interface for ROS publishers/subscribers
- `PubEntities` - Publishing entity management

### Utilities

- `GPIOInputMgr`, `GPIOObserver` - GPIO interrupt handling
- `BlinkAgent` - LED status indicator

## Test Organization (`tests/`)

```
tests/
├── CMakeLists.txt          # Test build configuration
├── hal/                    # HAL mocks and interface tests
│   ├── interfaces/         # Abstract hardware interfaces
│   ├── mocks/              # Mock implementations for testing
│   └── pico/               # Pico-specific implementations
├── test_motor_pid.cpp      # PID controller tests
├── test_odometry.cpp       # Odometry calculation tests
└── test_basic_math.cpp     # Math utility tests
```

## Configuration (`port/FreeRTOS-Kernel/`)

- `FreeRTOSConfig.h` - RTOS configuration (heap, stack, priorities)
- `logging_levels.h`, `logging_stack.h` - Debug logging setup
- `IdleMemory.c`, `cppMemory.cpp` - Memory management

## Libraries (`lib/`)

Third-party dependencies as git submodules:
- `FreeRTOS-Kernel/` - Real-time operating system
- `micro_ros_raspberrypi_pico_sdk/` - micro-ROS port
- `eigen/` - Linear algebra library
- `pico-distance-sensor/` - Distance sensor utilities

## Architecture Patterns

### Agent Pattern

All active components inherit from `Agent` base class:
- Encapsulates FreeRTOS task creation
- Provides `run()` loop and stack size configuration
- Enables modular, testable design

### Entity Pattern

ROS communication via `uRosEntities` interface:
- `createEntities()` - Initialize publishers/subscribers
- `destroyEntities()` - Cleanup
- `addToExecutor()` - Register with micro-ROS executor
- `handleSubscriptionMsg()` - Process incoming messages

### Hardware Abstraction

Separate interfaces from implementations:
- `hal/interfaces/` - Pure virtual interfaces
- `hal/hardware/` - Pico SDK implementations
- `tests/hal/mocks/` - Test doubles
