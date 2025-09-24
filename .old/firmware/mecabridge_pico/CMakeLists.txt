cmake_minimum_required(VERSION 3.20)

include(pico_sdk_import.cmake)
include(micro_ros_pico_sdk_import.cmake)

project(mecabridge_pico C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Initialize Pico SDK and micro-ROS Pico SDK wrappers
pico_sdk_init()
micro_ros_pico_sdk_init()

option(MICRO_ROS_USE_UART_TRANSPORT "Use UART transport for micro-ROS instead of USB" OFF)

set(MECABRIDGE_PICO_SOURCES
    src/main.cpp
    src/micro_ros_support.cpp
    src/motor_controller.cpp
    src/encoder_reader.cpp
)

add_executable(mecabridge_pico ${MECABRIDGE_PICO_SOURCES})

# Provide include directories for headers that live alongside the sources.
target_include_directories(mecabridge_pico PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${CMAKE_CURRENT_LIST_DIR}/include
)

# Turn on useful warnings by default.
if (CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mecabridge_pico PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif ()

if (MICRO_ROS_USE_UART_TRANSPORT)
    target_compile_definitions(mecabridge_pico PRIVATE MICRO_ROS_USE_UART_TRANSPORT=1)
endif ()

# Link against the required Pico and micro-ROS components.
target_link_libraries(mecabridge_pico
    pico_stdlib
    pico_unique_id
    hardware_gpio
    hardware_pwm
    hardware_irq
    hardware_timer
    hardware_adc
    micro_ros_pico_sdk
)

# Enable USB stdio (CDC) and disable UART stdio by default.
pico_enable_stdio_usb(mecabridge_pico 1)
pico_enable_stdio_uart(mecabridge_pico 0)

# Generate UF2/ELF/BIN outputs.
pico_add_extra_outputs(mecabridge_pico)

# Convenience flash target using picotool (USB bootloader must be in BOOTSEL mode)
add_custom_target(flash
    COMMAND picotool load -f ${CMAKE_CURRENT_BINARY_DIR}/mecabridge_pico.uf2
    DEPENDS mecabridge_pico
    COMMENT "Flashing mecabridge_pico.uf2 via picotool"
)

# Optional tests (off by default). Enable with -DBUILD_TESTS=ON
option(BUILD_TESTS "Build host-run unit tests for mecabridge_pico" OFF)
if (BUILD_TESTS)
    if (EXISTS "${CMAKE_CURRENT_LIST_DIR}/tests/CMakeLists.txt")
        add_subdirectory(tests)
    else()
        message(WARNING "BUILD_TESTS enabled but no tests/CMakeLists.txt found in mecabridge_pico")
    endif()
endif()
