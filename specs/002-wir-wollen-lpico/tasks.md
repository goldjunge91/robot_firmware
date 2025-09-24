# Tasks: Rebuild firmware using micro_ros_raspberrypi_pico_sdk

## Overview

This file contains the actionable task list to rebuild the firmware using
`micro_ros_raspberrypi_pico_sdk` as the primary project layout. It is derived
from the design documents in `specs/002-wir-wollen-lpico/` (plan, research,
data-model, quickstart and contracts).

**Prerequisites**: `plan.md`, `research.md`, `data-model.md`, `contracts/`, `quickstart.md`
## Parallel execution guidance

Run these [P] tasks in parallel (they operate on different files):

  - T003 (CI/lint) | T005 (MotorsResponse contract test) | T006 (MotorsCmd service contract test) | T007 (NerfServoCmd service contract test)
  - Example command for parallel execution:
    - Task agent commands (example):
      - `agent run test --file tests/contract/test_motors_response.cpp & agent run test --file tests/contract/test_motors_cmd_srv.cpp & agent run test --file tests/contract/test_nerf_servo_cmd_srv.cpp`

**Input**: Design documents from `specs/002-wir-wollen-lpico/`
**Prerequisites**: `plan.md` (required), `research.md`, `data-model.md`, `contracts/`, `quickstart.md`

## Execution Flow (main)

1. Load `plan.md` from feature directory → Extract tech stack and priorities.
2. Load optional design documents (`data-model.md`, `contracts/`, `research.md`, `quickstart.md`).
3. Generate ordered tasks grouped by Setup → Tests (TDD) → Core → Integration → Polish.
4. Apply task generation rules from template (contracts → contract tests [P], entities → model tasks [P], scenarios → integration tests [P]).
5. Number tasks sequentially (T001...); ensure file paths are absolute or repo‑relative.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies).

---

## Phase 3.1: Setup

- [ ] T001 Create project skeleton and CMake integration for micro-ROS
  - Path: `src/`, `bsp/`, `tests/`, `CMakeLists.txt` at repo root and `specs/002-wir-wollen-lpico/quickstart.md`
  - Action: Create minimal CMake targets that include `micro_ros_raspberrypi_pico_sdk` (add as submodule or reference). Document steps in `specs/002-wir-wollen-lpico/quickstart.md`.
  - Dependencies: none

- [ ] T002 Configure TinyUSB (USB-CDC) as the recommended micro-ROS transport and document in `specs/002-wir-wollen-lpico/research.md`
  # Tasks: Rebuild firmware using micro_ros_raspberrypi_pico_sdk

  ## Overview

  This file contains the actionable task list to rebuild the firmware using
  `micro_ros_raspberrypi_pico_sdk` as the primary project layout. It is derived
  from the design documents in `specs/002-wir-wollen-lpico/` (plan, research,
  data-model, quickstart and contracts).

  **Prerequisites**: `plan.md`, `research.md`, `data-model.md`, `contracts/`, `quickstart.md`
  - Action: Add config stub and documentation for using TinyUSB / USB CDC as primary transport; include fallback notes for UART.
  - Dependencies: T001

- [ ] T003 [P] Add linting, formatting and CI smoke job
  - Path: `.github/workflows/ci.yml` (create or update), repo root config files (`.clang-format`, `clang-tidy` or equivalent)
  - Action: Add a CI job that builds the firmware and runs host-side contract tests (simulated agent) as a smoke check.
  - Dependencies: T001

---

## Phase 3.2: Tests First (TDD) — MUST run before corresponding implementation
**CRITICAL**: Contract and integration tests must be created and initially fail.

Contracts discovered (each → [P] contract test):

- `specs/002-wir-wollen-lpico/contracts/EncoderTicks.msg` → T004 [P]
- `specs/002-wir-wollen-lpico/contracts/MotorsResponse.msg` → T005 [P]
- `specs/002-wir-wollen-lpico/contracts/MotorsCmd.srv` → T006 [P]
- `specs/002-wir-wollen-lpico/contracts/NerfServoCmd.srv` → T007 [P]

- [ ] T004 [P] Write contract test for `EncoderTicks` topic
  - Action: Implement a test that verifies message fields and that messages are received by a ROS2 host when the device publishes (use micro-ROS agent or a simulated publisher). Initially the test should fail.

  - All tasks include exact repo paths. If a task modifies the same file as another [P] task, do not run them in parallel.
- [ ] T016 Integrate motor command subscription/service to motor controller
  - Path: `src/ros/motor_cmd.c`
  - Action: Implement subscriber or service handler for `/_motors_cmd`/`cmd_motors`, call motor controller, publish `MotorsResponse`.
  - Dependencies: T006 (test exists), T013
- [ ] T017 Integrate Nerf services to nerf_controller
  - Path: `src/ros/nerf_srv.c`
  - Action: Implement `/nerf/fire_cmd` and `/nerf/servo_cmd` service handlers using `nerf_controller` and enforce arm/disarm safety.
  - Dependencies: T007, T014
- [ ] T018 Integrate IMU and ToF publishing
  - Path: `src/ros/imu_pub.c`, `src/ros/tof_pub.c`
  - Action: Publish `sensor_msgs/Imu` (map `imu_data_t`) and a custom ToF topic at configured rates.
  - Dependencies: T011, T012

---

## Phase 3.5: Polish

- [ ] T019 [P] Unit tests for state update and control logic
- [ ] T020 [P] Performance tests and benchmarks
  - Action: Add a small benchmark to measure micro-ROS message latency and motor control loop jitter.
  - Dependencies: T008, T013

- [ ] T021 [P] Update docs: `specs/002-wir-wollen-lpico/quickstart.md` with exact build/flash/test steps
  - Path: `specs/002-wir-wollen-lpico/quickstart.md`
  - Action: Expand quickstart with CMake commands, micro-ROS agent setup and contract test commands.

- [ ] T022 Release: produce UF2 artifact and document release steps
  - Path: `build/` (artifact), `RELEASE.md`
  - Action: Add release script that builds and produces `lpico_pio.uf2`, tag version and update `PINMAP.md` if necessary.

---

## Parallel execution guidance

- Run these [P] tasks in parallel (they operate on different files):
  - T003 (CI/lint) | T005 (MotorsResponse contract test) | T006 (MotorsCmd service contract test) | T007 (NerfServoCmd service contract test)
  - Example command for parallel execution:
    - Task agent commands (example):

## Dependencies summary (short)

- Tests (T004–T007) must exist and fail before implementing T015–T017
- Models (T010–T012) before services and integration (T015–T018)
- Core motor/encoder (T009, T013) required before motor ROS integration (T016)


- All tasks include exact repo paths. If a task modifies the same file as another [P] task, do not run them in parallel.
- Commit after each task and open a PR with the task ID in the title (e.g., `T009: Implement encoder driver skeleton`).

# Tasks: Rebuild lpico_pio based on micro_ros_raspberrypi_pico_sdk
Input: design documents from `/specs/002-wir-wollen-lpico/` (plan.md, research.md, data-model.md, contracts/)

This file lists small, actionable tasks (T001...) ordered by dependency. Tasks marked [P] can run in parallel when they touch different files.


- [x] **T001**: Research and recommend a suitable unit testing framework for Raspberry Pi Pico.
  - **Output**: Update `research.md` with findings and recommendation.
- [x] **T002**: Research best practices for integrating micro-ROS with Pico SDK and FreeRTOS.
- [x] **T003**: Research best practices for structuring a complex firmware project for a mobile robot on RP2040.
  - **Output**: Update `research.md` with findings.


- [ ] **T004**: Building a Firmware project skeleton for `robot_firmware` using `micro_ros_raspberrypi_pico_sdk/` as template.
  - **Path(s)**: from `example_repo/rosbot/, .old/` (copy CMake + imports), --> `micro_ros_raspberrypi_pico_sdk/`
- [ ] **T005**: Configure CMake toolchain and project metadata (CMakeLists + SDK import files).
  - **Files**: `example_repo/rosbot/, .old/ firmware/mecabridge_pico/CMakeLists.txt`, `micro_ros_raspberrypi_pico_sdk/micro_ros_pico_sdk_import.cmake`, `firmware/mecabridge_pico/pico_sdk_import.cmake`
- [ ] **T006 [P]**: Add repo-level developer tooling: clang-format, cppcheck/clang-tidy stubs, and CI job placeholder.
  - **Files**: `.clang-format`, `.github/workflows/firmware-ci.yml` (placeholder)

## Phase 2 — Tests first (TDD) — write failing tests

- [ ] **T014 [P]**: Integration test: verify flashing workflow (or mock) (`tests/integration/test_flashing.cpp`). [X]
- [ ] **T015 [P]**: Integration test: verify basic micro-ROS communication (e.g., receiving heartbeat from T008).
  - **Files**: `tests/integration/test_ros2_basic_communication.cpp` [X]
- **T016 [P]**: Create model header `include/robot_state.h` (robot_state_t) [X]
- **T018 [P]**: Create model header `include/tof_data.h` (tof_data_t) [X]
- **T019 [P]**: Create model header `include/encoder_data.h` (encoder_data_t) [X]
- **T020 [P]**: Create model header `include/motor_speeds.h` (motor_speeds_t) [X]
- **T021 [P]**: Create model header `include/esc_speeds.h` (esc_speeds_t) [X]
- **T022 [P]**: Create model header `include/servo_angles.h` (servo_angles_t) [X]

- **T023**: Implement HAL + BSP glue (use `PINMAP.md`) → `src/bsp.cpp`, `include/bsp.h`, `include/hardware_cfg.h`
  - **T023 [X]**: Implement HAL + BSP glue (use `PINMAP.md`) → `src/bsp.cpp`, `include/bsp.h`, `include/hardware_cfg.h`
- **T024**: Implement motor driver & PWM layer, migrating and adapting `motor_driver.cpp`, `motors.cpp`, and `pwm_control.cpp` from `lpico_pio` and `rosbot_ros2_firmware` → `src/motor_driver.cpp`, `include/motor_driver.h`
  - **T024 [X]**: Implement motor driver & PWM layer, migrating and adapting `motor_driver.cpp`, `motors.cpp`, and `pwm_control.cpp` from `lpico_pio` and `rosbot_ros2_firmware` → `src/motor_driver.cpp`, `include/motor_driver.h`
- **T025**: Implement encoder reader (interrupts/timers) → `src/encoder_reader.cpp`, `include/encoder_reader.h`
  - **T025 [X]**: Implement encoder reader (interrupts/timers) → `src/encoder_reader.cpp`, `include/encoder_reader.h`
- **T026 [X]**: Implement IMU (ICM-20948) interface → `src/imu_interface.cpp`, `include/imu_interface.h`
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

`micro_ros_raspberrypi_pico_sdk/` — CMake project for Pico

`include/` — small model and driver headers

`lib/` — third-party libraries (e.g., pico-sdk, micro-ROS)

`src/` — implementation sources

`tests/contract/`, `tests/integration/`, `tests/unit/`, `tests/perf/`

## Validation checklist

- [ ] Each contract in `specs/002-wir-wollen-lpico/contracts/` has a corresponding contract test (T009-T012)
- [ ] Each entity in `data-model.md` has a model header (T016-T022)
- [X] Tests added before implementation (T009-T015 present and failing)
- [ ] Drivers & HAL implemented before hardware wiring tasks
- [ ] CI builds firmware and runs unit tests

---
Generated from `specs/002-wir-wollen-lpico/` (plan.md, data-model.md, contracts/, research.md).
