# Robot Firmware Test Suite

## Quick Start

```bash
cd /Users/marco/Github.tmp/my_steel-robot_ws/robot_firmware/tests
mkdir build && cd build
cmake ..
make
ctest --output-on-failure
```

## Test Structure

- **test_motor_pid.cpp** - PID controller unit tests
- **test_tb6612_motor_pid.cpp** - TB6612 motor driver tests
- **mocks/** - Hardware abstraction mocks

## Running Tests

```bash
# All tests
make test

# Specific test
./test_motor_pid
./test_tb6612_motor_pid

# Verbose
./test_motor_pid --gtest_list_tests
```

## Requirements

- CMake 3.13+
- C++17 compiler
- Google Test (auto-downloaded)

## Test Coverage

Tests validate:
- ✅ PID algorithm correctness
- ✅ Motor control logic
- ✅ TB6612 driver pin states
- ✅ Speed setpoint tracking

For full documentation see artifacts in conversation.
