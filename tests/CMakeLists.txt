# ==============================================================================
# Robot Firmware Test Suite
# ==============================================================================
# Can be built standalone or as sub-project
# Standalone: cd tests/build && cmake .. && make
# Sub-project: cmake -B build_tests -DBUILD_TESTS=ON (from root)
#
# Structure:
#   tests/
#   ├── unit/           - Isolated component tests
#   ├── integration/    - Multi-component tests
#   ├── hal/            - Hardware abstraction layer tests
#   └── mocks/          - Shared mock implementations

cmake_minimum_required(VERSION 3.13)
project(robot_firmware_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing
enable_testing()

# Fetch GoogleTest if not already available
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Fetch Eigen if not already available
FetchContent_Declare(
  eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE
)
set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(eigen)

# Create firmware library from source files (for testing)
set(FIRMWARE_SRC_DIR ${CMAKE_SOURCE_DIR}/../src)
add_library(firmware_lib STATIC
    ${FIRMWARE_SRC_DIR}/MotorPID.cpp
    ${FIRMWARE_SRC_DIR}/TB6612MotorPID.cpp
    ${FIRMWARE_SRC_DIR}/MathUtils.cpp
    ${FIRMWARE_SRC_DIR}/MecanumKinematics.cpp
    ${FIRMWARE_SRC_DIR}/Odometry.cpp
)

target_include_directories(firmware_lib PUBLIC
    ${FIRMWARE_SRC_DIR}
    ${FIRMWARE_SRC_DIR}/config
)

target_link_libraries(firmware_lib PUBLIC Eigen3::Eigen)

message(STATUS "")
message(STATUS "Configuring Robot Firmware Test Suite...")
message(STATUS "---------------------------------------------")

# ==============================================================================
# Shared Test Utilities
# ==============================================================================

# Test main library (provides GoogleTest main())
# This is NOT needed anymore since we use gtest_main, but keeping for future custom setup
# add_library(test_main STATIC test_main.cpp)
# target_link_libraries(test_main PUBLIC gtest_main)

# ==============================================================================
# Mock Libraries
# ==============================================================================

message(STATUS "  - Configuring mock libraries...")
add_subdirectory(mocks)
add_subdirectory(hal)

# ==============================================================================
# Test Categories
# ==============================================================================

message(STATUS "  - Configuring unit tests...")
add_subdirectory(unit)

message(STATUS "  - Configuring integration tests...")
add_subdirectory(integration)

# ==============================================================================
# Summary
# ==============================================================================

message(STATUS "---------------------------------------------")
message(STATUS "Test suite configured successfully!")
message(STATUS "")
message(STATUS "Available test targets:")
message(STATUS "  - test_motor_pid")
message(STATUS "  - test_tb6612_motor_pid")
message(STATUS "  - test_basic_math")
message(STATUS "  - test_mecanum_kinematics")
message(STATUS "  - test_odometry")
message(STATUS "  - test_topic_names")
message(STATUS "  - test_hal_mocks")
message(STATUS "")
message(STATUS "Run all tests:")
message(STATUS "  ctest --output-on-failure")
message(STATUS "")
message(STATUS "Run specific test category:")
message(STATUS "  ctest -L unit")
message(STATUS "  ctest -L integration")
message(STATUS "  ctest -L hal")
message(STATUS "")
message(STATUS "Run tests in parallel:")
message(STATUS "  ctest -j8 --output-on-failure")
message(STATUS "---------------------------------------------")
message(STATUS "")
