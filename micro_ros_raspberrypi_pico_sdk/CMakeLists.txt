cmake_minimum_required(VERSION 3.12)

if(NOT DEFINED ENV{PICO_SDK_PATH})
  message(FATAL_ERROR "PICO_SDK_PATH is not set. Export it before running cmake, e.g.\n  export PICO_SDK_PATH=/home/you/pico-sdk\n  or set in PowerShell: $env:PICO_SDK_PATH='C:\\path\\to\\pico-sdk'")
endif()

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(pico_micro_ros_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Option to skip building picotool/UF2 outputs for faster iteration
option(SKIP_PICOTOOL "Skip building picotool and generating UF2 outputs" OFF)

pico_sdk_init()

link_directories(libmicroros)

# Build a list of candidate sources and only include those that actually exist.
file(GLOB PROJECT_SOURCES
  "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/transport/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/middleware/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/application/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/lib/imu-driver/ICM-20948/*.cpp"
)

add_executable(pico_micro_ros_example ${PROJECT_SOURCES})

# Generate custom firmware filename with date and version
string(TIMESTAMP BUILD_DATE "%Y%m%d")
set(FIRMWARE_VERSION "1.0.0")
set(CUSTOM_FIRMWARE_NAME "firmware_${BUILD_DATE}_${FIRMWARE_VERSION}")

set_target_properties(pico_micro_ros_example PROPERTIES OUTPUT_NAME ${CUSTOM_FIRMWARE_NAME})

pico_generate_pio_header(pico_micro_ros_example ${CMAKE_CURRENT_SOURCE_DIR}/src/hardware/quadrature_encoder.pio)

target_link_libraries(pico_micro_ros_example
    pico_stdlib
    pico_multicore
    microros
    hardware_uart
    hardware_pwm
    hardware_pio
    hardware_irq
    hardware_clocks
    hardware_i2c
)

target_include_directories(pico_micro_ros_example PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/libmicroros/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware
    ${CMAKE_CURRENT_SOURCE_DIR}/include/middleware
    ${CMAKE_CURRENT_SOURCE_DIR}/include/application
    ${CMAKE_CURRENT_SOURCE_DIR}/include/shared
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/hardware
    ${CMAKE_CURRENT_SOURCE_DIR}/include/middleware
    ${CMAKE_CURRENT_SOURCE_DIR}/include/application
    ${CMAKE_CURRENT_SOURCE_DIR}/include/shared/transport
    ${CMAKE_CURRENT_BINARY_DIR}
)

# Make lib/ available for includes like "ICM-20948/ICM20948Adapter.h"
target_include_directories(pico_micro_ros_example PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/lib
)

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico
pico_enable_stdio_usb(pico_micro_ros_example 1)
pico_enable_stdio_uart(pico_micro_ros_example 0)
add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)
add_compile_definitions(TARGET_PICO=1)

# Generate UF2 (optional)
if(NOT SKIP_PICOTOOL)
  pico_add_extra_outputs(pico_micro_ros_example)
endif()