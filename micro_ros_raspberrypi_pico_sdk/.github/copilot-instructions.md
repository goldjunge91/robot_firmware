# AI Coding Assistant Instructions for micro-ROS Raspberry Pi Pico SDK

## Project Overview

This is a micro-ROS firmware project for Raspberry Pi Pico that implements a robot control system with omnidirectional movement (Mecanum wheels), IMU sensor fusion, battery monitoring, and shooter control. The firmware communicates with a ROS2 system running on a Raspberry Pi 4B via micro-ROS over USB CDC transport.

**Key Architecture:**
- **Two-tier control system**: High-level ROS2 navigation/planning on SBC, low-level real-time motor control on Pico
- **Communication**: micro-ROS publishers/subscribers for sensor data and motor commands
- **Hardware**: ICM-20948 IMU, Hall encoders, TB6612FNG motor drivers, INA3221 battery monitor, brushless ESCs for shooter

## Critical Developer Workflows

### Build System (CMake + Pico SDK)
```bash
# Set Pico SDK path (required)
export PICO_SDK_PATH=/path/to/pico-sdk

# Build process
mkdir build && cd build
cmake ..
make

# Skip UF2 generation for faster iteration
cmake -DSKIP_PICOTOOL=ON ..
```

### Firmware Flashing
- Build generates `pico_micro_ros_example.uf2`
- Flash by copying UF2 to Pico in bootloader mode (hold BOOTSEL while plugging in)
- Use `robot_utils/scripts/flash_firmware.sh` for automated flashing

### Development Setup
1. Clone with submodules: `git clone --recurse-submodules`
2. Install dependencies: `apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi`
3. Set `PICO_SDK_PATH` environment variable
4. Build and flash firmware
5. Launch ROS2 bringup: `ros2 launch robot_bringup bringup.launch.py drive_type:=mecanum`

## Project-Specific Conventions

### Pin Configuration Authority
- **PINMAP.md is single source of truth** for all hardware pin assignments
- Never hardcode pins - always reference `hardware_cfg.h` or `config.h`
- Update both code and PINMAP.md when changing pinouts

### Motor Control Patterns
```cpp
// Motor initialization with PIO quadrature encoders
MotorClass motor(FL_PWM, FL_IN1, FL_IN2, ENC_FL_A, -1, &timer, pio0, 0);

// PID control with 1000x scaling
#define PID_DEFAULT_KP 49  // KP * 1000
motor.SetPidParameters(49, 8, 0);  // Kp=0.049, Ki=0.008, Kd=0

// Velocity commands in rad/s * 1000
motor.SetPidSetpoint(5000);  // 5 rad/s
```

### ROS2 Integration Patterns
```cpp
// Publisher initialization
rclc_publisher_init_default(&imu_publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw");

// Timer-based publishing (100Hz IMU)
rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(10), imu_timer_callback);

// Executor with multiple handles
rclc_executor_init(&executor, &support.context, 4, &allocator);
rclc_executor_add_timer(&executor, &imu_timer);
rclc_executor_add_subscription(&executor, &motors_cmd_subscriber);
```

### Hardware Initialization Order
1. `stdio_init_all()` - USB CDC for debugging
2. `BoardPheripheralsInit()` - GPIO, I2C, UART setup
3. Peripheral drivers (IMU, motors, battery monitor)
4. `watchdog_manager_init()` - safety watchdog last

### Safety & Watchdog Patterns
```cpp
// Ping agent before initialization
const int timeout_ms = 1000;
const uint8_t attempts = 120;
rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

// Watchdog with safety timeout
watchdog_manager_init();  // 2-second timeout
watchdog_update();        // Feed watchdog in main loop
```

## Key Files & Their Roles

| File | Purpose | Key Patterns |
|------|---------|--------------|
| `src/main.cpp` | Application lifecycle, main loop | Class-based initialization, timer interrupts for LED |
| `src/micro_ros_cfg.cpp` | ROS2 entities, message handling | Publisher/subscriber/service setup, executor management |
| `src/motors.cpp` | Motor control with PID | PIO quadrature encoders, velocity control |
| `src/ImuLib_cfg.cpp` | ICM-20948 IMU driver | I2C communication, sensor fusion data publishing |
| `include/hardware_cfg.h` | Pin definitions, constants | Compile-time configuration, enum definitions |
| `include/config.h` | Motor pin mappings | TB6612FNG pin assignments, encoder channels |
| `CMakeLists.txt` | Build configuration | Pico SDK integration, library linking |

## Common Patterns & Anti-Patterns

### ✅ DO Use These Patterns
- **RAII-style initialization**: Initialize hardware in constructors, cleanup in destructors
- **Timer-based periodic tasks**: Use `rclc_timer` for consistent timing, not `sleep_ms()`
- **Buffer sharing**: Global buffers with `extern` declarations for inter-module communication
- **Feature flags**: Use `#ifdef` for optional peripherals (e.g., `ENABLE_POWERBOARD`)

### ❌ AVOID These Anti-Patterns
- **Arduino-style APIs**: This is bare-metal Pico code, not Arduino - use `hardware/` APIs
- **Blocking delays**: Never use `sleep_ms()` in control loops - use timers or executor
- **Magic numbers**: All pins/constants must be `#define`d in header files
- **Direct hardware access**: Always use BSP abstraction (`BoardPheripheralsInit()`)

## Integration Points

### ROS2 Topics & Services
- **Publishers**: `/imu/data_raw`, `/battery/status`, `/_motors_response`
- **Subscribers**: `/_motors_cmd`, `/shooter_cmd`
- **Services**: `get_cpu_id` (returns Pico unique ID)

### Hardware Communication
- **I2C bus 0**: IMU (0x28/0x29), battery monitor (INA3221)
- **UART**: Power board communication (38400 baud)
- **USB CDC**: micro-ROS transport + debugging output

### Cross-Component Data Flow
```
ROS2 Command → micro-ROS subscriber → Motor PID controller → PWM output
IMU readings → micro-ROS publisher → ROS2 EKF localization
Battery monitor → micro-ROS publisher → ROS2 power management
```

## Debugging & Troubleshooting

### Common Issues
- **"PICO_SDK_PATH not set"**: Export environment variable pointing to pico-sdk installation
- **USB connection fails**: Check `pico_enable_stdio_usb(pico_micro_ros_example 1)`
- **Motor encoder glitches**: Verify quadrature encoder PIO program initialization
- **IMU initialization fails**: Check I2C bus (SDA=0, SCL=1) and ICM-20948 address

### Debug Output
```cpp
// Enable debug mode
firmware_mode = fw_debug;

// Use printf for debugging (goes to USB CDC)
printf("Motor velocity: %d\n", motor.GetVelocity());
```

## Testing & Validation

### Unit Testing Approach
- **Hardware validation**: Blink patterns, encoder counting, PWM output verification
- **Integration testing**: ROS2 topic publishing/subscribing, motor command response
- **Safety testing**: Watchdog timeout behavior, emergency stop functionality

### Build Verification
```bash
# Quick build check
make -j$(nproc)
ls -la *.uf2  # Verify UF2 generation

# Clean rebuild
rm -rf build && mkdir build && cd build && cmake .. && make
```

Remember: This firmware runs on bare-metal RP2040 with real-time constraints. Always prioritize deterministic timing, safety mechanisms, and hardware abstraction over code elegance.</content>
<parameter name="filePath">c:\GIT\my_steel-robot_ws\src\robot_firmware\example_repo\micro_ros_raspberrypi_pico_sdk\.github\copilot-instructions.md