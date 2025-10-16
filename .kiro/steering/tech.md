# Technology Stack

## Build System

- **CMake** 3.13+ with custom toolchain for ARM Cortex-M0+
- **Make** wrapper for common tasks
- **Pico SDK** - Hardware abstraction and boot code

## Core Technologies

- **Language**: C++17 (firmware), C11 (drivers)
- **RTOS**: FreeRTOS (dual-core support, configUSE_CORE_AFFINITY=1)
- **ROS**: micro-ROS for Pico SDK (ROS2 communication over USB)
- **Math**: Eigen3 (linear algebra for odometry/kinematics)
- **Testing**: GoogleTest (unit tests run on host)

## Key Libraries

- `pico_stdlib` - Pico SDK core
- `hardware_pwm`, `hardware_i2c`, `hardware_spi` - Hardware drivers
- `FreeRTOS-Kernel-Heap4` - Memory management
- `micro_ros` - ROS2 middleware
- `eigen` - Matrix operations
- `distance_sensor` - Custom sensor library

## Common Commands

### Building

```bash
# Incremental build (preserves build folder)
make build

# Clean build (removes build folder first)
make compile

# Release build (optimized)
make build_release

# Clean all artifacts
make clean
```

### Build Output Management

```bash
# View first 100 lines of build output
make build 2>&1 | head -100

# View last 50 lines of build output (useful for errors)
make build 2>&1 | tail -50
```

### Flashing

```bash
# Flash via USB mass storage or picotool
make flash

# Flash release version
make flash-release
```

### Testing

```bash
# Run all unit tests
make test

# Run with verbose output
make test-verbose

# Run specific test suite
make test-pid
make test-odometry
```

### Release Management

```bash
# Create versioned release
make release VERSION=v1.0.0

# Or use helper script
./create_release.sh v1.0.0
```

## Build Artifacts

- `build/src/my_firmware.uf2` - Flashable firmware binary
- `build/src/my_firmware.elf` - Debug symbols
- `releases/my_firmware_latest.uf2` - Latest build copy
- `tests/build/firmware_tests` - Unit test executable

## Environment Variables

- `PICO_SDK_PATH` - Path to Pico SDK (default: `~/pico-sdk`)
- `VERSION` - Release version tag for `make release`
