# ==============================================================================
# Robot Firmware Test Suite
# ==============================================================================
cmake_minimum_required(VERSION 3.13)
project(robot_firmware_tests CXX)

# GoogleTest
set(GOOGLETEST_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../googletest")

if(EXISTS "${GOOGLETEST_DIR}/CMakeLists.txt")
    message(STATUS "Using local GoogleTest from: ${GOOGLETEST_DIR}")
    add_subdirectory(${GOOGLETEST_DIR} ${CMAKE_CURRENT_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)
else()
    message(STATUS "Local GoogleTest not found, fetching from GitHub...")
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
endif()

# Include directories
include_directories(
    ../src
    .
    mocks
    hal
    ${gtest_SOURCE_DIR}/include
    ${gtest_SOURCE_DIR}
)

# Compiler definitions for tests
add_definitions(-DUNIT_TEST -DPICO_PLATFORM=host)

# ==============================================================================
# Test Source Files
# ==============================================================================
set(TEST_SOURCE_FILES
    tests1.cpp
    test_motor_pid.cpp
    test_tb6612_motor_pid.cpp
    test_basic_math.cpp
    test_hal_mocks.cpp
    test_mecanum_kinematics.cpp
    test_odometry.cpp
    test_topic_names.cpp
)

set(MOCK_FILES
    mocks/mock_motor_mgr.cpp
    mocks/mock_tb6612_motor_mgr.cpp
    hal/mocks/I2CMock.cpp
    hal/mocks/GPIOMock.cpp
)

set(FIRMWARE_FILES
    ../src/MotorPID.cpp
    ../src/TB6612MotorPID.cpp
)

# ==============================================================================
# Test Executable
# ==============================================================================
add_executable(Firmware_gTests_run 
    ${TEST_SOURCE_FILES}
    ${MOCK_FILES}
    ${FIRMWARE_FILES}
)

target_link_libraries(Firmware_gTests_run gtest)