# Tasks: Rebuild lpico_pio based on micro_ros_raspberrypi_pico_sdk

Input: design documents from `/specs/002-wir-wollen-lpico/` (plan.md, research.md, data-model.md, contracts/)

This file lists small, actionable tasks (T001...) ordered by dependency. Tasks marked [P] can run in parallel when they touch different files.

## Phase 0 — Outline & Research

- **T001**: Research and recommend a suitable unit testing framework for Raspberry Pi Pico.
  - **Output**: Update `research.md` with findings and recommendation.
- **T002**: Research best practices for integrating micro-ROS with Pico SDK and FreeRTOS.
  - **Output**: Update `research.md` with findings.
- **T003**: Research best practices for structuring a complex firmware project for a mobile robot on RP2040.
  - **Output**: Update `research.md` with findings.

## Phase 1 — Setup & Basic Communication (Early Micro-ROS Agent Connection)

- **T004**: Create project skeleton for `lpico_pio` using `example_repo/micro_ros_raspberrypi_pico_sdk/` as template.
  - **Path(s)**: `firmware/mecabridge_pico/` (copy CMake + imports), `firmware/mecabridge_pico/src/`
- **T005**: Configure CMake toolchain and project metadata (CMakeLists + SDK import files).
  - **Files**: `firmware/mecabridge_pico/CMakeLists.txt`, `firmware/mecabridge_pico/micro_ros_pico_sdk_import.cmake`, `firmware/mecabridge_pico/pico_sdk_import.cmake`
- **T006 [P]**: Add repo-level developer tooling: clang-format, cppcheck/clang-tidy stubs, and CI job placeholder.
  - **Files**: `.clang-format`, `.github/workflows/firmware-ci.yml` (placeholder)
- **T007**: Implement basic micro-ROS configuration and transport setup to establish initial connection with micro-ROS agent.
  - **Files**: `src/micro_ros_cfg.cpp`, `include/micro_ros_cfg.h`
- **T008**: Implement a simple ROS2 publisher (e.g., a heartbeat or a counter) to verify the micro-ROS connection.
  - **Files**: `src/ros2_publishers.cpp`

## Phase 2 — Tests first (TDD) — write failing tests
All tests should be added and expected to fail before implementing behavior.

- **T009 [P]**: Contract test for `EncoderTicks.msg` → `tests/contract/test_encoder_ticks.cpp`
- **T010 [P]**: Contract test for `MotorsResponse.msg` → `tests/contract/test_motors_response.cpp`
- **T011 [P]**: Contract test for `MotorsCmd.srv` → `tests/contract/test_motors_cmd.cpp`
- **T012 [P]**: Contract test for `NerfServoCmd.srv` → `tests/contract/test_nerf_servo_cmd.cpp`
- **T013 [P]**: Integration test: verify firmware cmake configure/build produces artifact (`tests/integration/test_build.cpp`).
- **T014 [P]**: Integration test: verify flashing workflow (or mock) (`tests/integration/test_flashing.cpp`).
- **T015 [P]**: Integration test: verify basic micro-ROS communication (e.g., receiving heartbeat from T008).
  - **Files**: `tests/integration/test_ros2_basic_communication.cpp`

## Phase 3 — Models & Core (implementations)
Models first (small headers), then drivers, then control loop and ROS bindings.

- **T016 [P]**: Create model header `include/robot_state.h` (robot_state_t)
- **T017 [P]**: Create model header `include/imu_data.h` (imu_data_t)
- **T018 [P]**: Create model header `include/tof_data.h` (tof_data_t)
- **T019 [P]**: Create model header `include/encoder_data.h` (encoder_data_t)
- **T020 [P]**: Create model header `include/motor_speeds.h` (motor_speeds_t)
- **T021 [P]**: Create model header `include/esc_speeds.h` (esc_speeds_t)
- **T022 [P]**: Create model header `include/servo_angles.h` (servo_angles_t)

- **T023**: Implement HAL + BSP glue (use `PINMAP.md`) → `src/bsp.cpp`, `include/bsp.h`, `include/hardware_cfg.h`
- **T024**: Implement motor driver & PWM layer, migrating and adapting `motor_driver.cpp`, `motors.cpp`, and `pwm_control.cpp` from `lpico_pio` and `rosbot_ros2_firmware` → `src/motor_driver.cpp`, `include/motor_driver.h`
- **T025**: Implement encoder reader (interrupts/timers) → `src/encoder_reader.cpp`, `include/encoder_reader.h`
- **T026**: Implement IMU (ICM-20948) interface → `src/imu_interface.cpp`, `include/imu_interface.h`
- **T027**: Implement ToF (VL53L0X) interface → `src/tof_interface.cpp`, `include/tof_interface.h`
- **T028**: Implement shooter (ESCs + servos) control → `src/shooter_control.cpp`, `include/shooter_control.h`
- **T029**: Implement safety & watchdog manager → `src/safety.cpp`, `include/safety.h`, `src/watchdog_manager.cpp`, `include/watchdog_manager.h`
- **T030**: Implement utility/periphery (UART, USB, LEDs) → `src/UartLib.cpp`, `src/usb_interface.cpp`, `src/pixel_led.cpp`, headers in `include/`
- **T031**: Implement control loop (non-blocking, fixed-rate) → `src/control_loop.cpp`

## Phase 4 — Full micro-ROS / ROS2 bindings

- **T032**: Implement remaining ROS2 publishers: `encoder_ticks`, `imu`, `battery`, `/_motors_response` → `src/ros2_publishers.cpp`
- **T033**: Implement remaining ROS2 services: `/_motors_cmd`, `/nerf/fire_cmd`, `/nerf/servo_cmd` → `src/ros2_services.cpp`

## Phase 5 — Integration & HW wiring

- **T034**: Configure I2C for IMU/ToF per `PINMAP.md` → (code + `CMakeLists` updates)
- **T035**: Configure PWM outputs for motors/servos → (code + hardware_cfg)
- **T036**: Configure UART for micro-ROS transport → (code + wiring notes)
- **T037**: Validate GPIO/interrupt mapping for encoders → (code + wiring notes)

## Phase 6 — Tests, CI & Polish

- **T038 [P]**: Unit tests for control loop, motor driver, encoder reader, IMU, ToF, safety → `tests/unit/*`
- **T039 [P]**: Performance test for control loop (target: up to 1kHz) → `tests/perf/test_control_loop.cpp`
- **T040**: Add CI job to build firmware and run unit tests (`.github/workflows/firmware-ci.yml`)
- **T041**: Update `quickstart.md` with build/flash steps and hardware verification checklist
- **T042**: Update `PINMAP.md` if minor corrections found during integration

## Parallel groups (examples)

- Group A (can run in parallel): T009, T010, T011, T012 (contract tests creation)
- Group B (can run in parallel after models): T016-T022 (model header files)
- Group C: T038 unit tests (different files) can run in parallel in CI

## Numbering & dependency summary

- Tests-first: T009-T015 must exist and fail before implementing T023-T031
- Models (T016-T022) before drivers and ROS bindings (T023-T033)
- Hardware integration (T034-T037) depends on drivers T023-T031
- Polish & CI (T038-T042) after core & integration

## File paths and expected artifacts (short)

- `firmware/mecabridge_pico/` — CMake project for Pico
- `include/` — small model and driver headers
- `src/` — implementation sources
- `tests/contract/`, `tests/integration/`, `tests/unit/`, `tests/perf/`

## Validation checklist

- [ ] Each contract in `specs/002-wir-wollen-lpico/contracts/` has a corresponding contract test (T009-T012)
- [ ] Each entity in `data-model.md` has a model header (T016-T022)
- [ ] Tests added before implementation (T009-T015 present and failing)
- [ ] Drivers & HAL implemented before hardware wiring tasks
- [ ] CI builds firmware and runs unit tests

---
Generated from `specs/002-wir-wollen-lpico/` (plan.md, data-model.md, contracts/, research.md).