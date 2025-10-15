# ==============================================================================
# Robot Firmware Test Suite - CMake Configuration
# ==============================================================================
cmake_minimum_required(VERSION 3.13)

project(robot_firmware_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Compiler warnings
add_compile_options(-Wall -Wextra -Wpedantic -Werror=return-type)

# ==============================================================================
# Google Test Configuration
# ==============================================================================
include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()
include(GoogleTest)

# ==============================================================================
# Mock Pico SDK
# ==============================================================================
add_library(mock_pico_sdk INTERFACE)
target_include_directories(mock_pico_sdk INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/mocks
    ${CMAKE_CURRENT_SOURCE_DIR}/../src
)
target_compile_definitions(mock_pico_sdk INTERFACE PICO_PLATFORM=host UNIT_TEST)

set(FIRMWARE_SRC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../src")

# ==============================================================================
# Test Executables
# ==============================================================================

# Test 1: Motor PID
add_executable(test_motor_pid
    test_motor_pid.cpp
    mocks/mock_motor_mgr.cpp
    ${FIRMWARE_SRC_DIR}/MotorPID.cpp
)
target_link_libraries(test_motor_pid PRIVATE GTest::gtest_main GTest::gmock mock_pico_sdk)
gtest_discover_tests(test_motor_pid PROPERTIES TIMEOUT 30)

# Test 2: TB6612 Motor PID
add_executable(test_tb6612_motor_pid
    test_tb6612_motor_pid.cpp
    mocks/mock_tb6612_motor_mgr.cpp
    ${FIRMWARE_SRC_DIR}/TB6612MotorPID.cpp
)
target_link_libraries(test_tb6612_motor_pid PRIVATE GTest::gtest_main GTest::gmock mock_pico_sdk)
gtest_discover_tests(test_tb6612_motor_pid PROPERTIES TIMEOUT 30)

# ==============================================================================
# Custom Targets
# ==============================================================================
add_custom_target(run_all_tests
    COMMAND ${CMAKE_CTEST_COMMAND} --verbose --output-on-failure
    DEPENDS test_motor_pid test_tb6612_motor_pid
    COMMENT "Running all firmware tests..."
)

message(STATUS "========================================")
message(STATUS "Robot Firmware Test Suite")
message(STATUS "C++ Standard: C++${CMAKE_CXX_STANDARD}")
message(STATUS "========================================")
