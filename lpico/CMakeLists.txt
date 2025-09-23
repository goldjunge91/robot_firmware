cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(robot_firmware_lpico C CXX ASM)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Wpedantic -Werror -O2)

# Initialize the Pico SDK
pico_sdk_init()

# Add executable and link libraries
add_executable(robot_firmware_lpico
    src/bsp.cpp
    src/command_processor.c
    src/control_loop.c
    src/encoder_reader.c
    src/hal_compat.c
    src/ImuLib_cfg.cpp
    src/interrupts.c
    src/main.c
    src/micro_ros_cfg.cpp
    src/motor_driver.c
    src/motors.cpp
    src/pwm_control.c
    src/safety.c
    src/shooter_control.c
    src/UartLib.cpp
    src/usb_interface.c
    src/watchdog_manager.c
)

# Enable USB CDC for stdio
pico_enable_stdio_usb(robot_firmware_lpico 1)
pico_enable_stdio_uart(robot_firmware_lpico 0)

# Link to Pico SDK libraries
target_link_libraries(robot_firmware_lpico pico_stdlib hardware_pwm hardware_timer hardware_watchdog)

# Create map and uf2 files
pico_add_extra_outputs(robot_firmware_lpico)
