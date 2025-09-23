cmake_minimum_required(VERSION 3.13)
include(../pico/pico_sdk_import.cmake)

project(lpico_project)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(lpico
  src/main.cpp
  src/motors.cpp
  src/bsp.cpp
)

target_include_directories(lpico PRIVATE include)

# Enable USB CDC stdio
pico_enable_stdio_usb(lpico 1)
pico_enable_stdio_uart(lpico 0)

target_link_libraries(lpico PRIVATE pico_stdlib hardware_pwm hardware_gpio)

pico_add_extra_outputs(lpico)
