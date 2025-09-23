cmake_minimum_required(VERSION 3.20)

include(pico_sdk_import.cmake)

project(lpico C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

pico_sdk_init()

# Will be populated in later tasks
set(LPICO_SOURCES
    src/bsp.cpp
    src/ImuLib_cfg.cpp
    src/main.cpp
    src/micro_ros_cfg.cpp
    src/motors.cpp
    src/STM32FreeRTOSConfig.h
    src/UartLib.cpp
    src/hal_compat.c
    src/usb_interface.c
    src/motor_driver.c
    src/pwm_control.c
    src/encoder_reader.c
    src/shooter_control.c
    src/watchdog_manager.c
    src/control_loop.c
    src/command_processor.c
    src/interrupts.c
    src/safety.c
)

add_executable(lpico ${LPICO_SOURCES})

target_include_directories(lpico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/include
)

if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(lpico PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif ()

target_link_libraries(lpico
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_timer
    hardware_irq
)

pico_enable_stdio_usb(lpico 1)
pico_enable_stdio_uart(lpico 0)

pico_add_extra_outputs(lpico)

add_custom_target(flash
    COMMAND picotool load -f ${CMAKE_CURRENT_BINARY_DIR}/lpico.uf2
    DEPENDS lpico
    COMMENT "Flashing lpico.uf2 via picotool"
)