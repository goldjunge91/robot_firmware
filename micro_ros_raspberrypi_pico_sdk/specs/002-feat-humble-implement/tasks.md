# Tasks: Implement Firmware for Humble

**Input**: Design documents from `/specs/002-feat-humble-implement/`
**Prerequisites**: plan.md (required), research.md, data-model.md, contracts/micro-ros-contracts.md, quickstart.md

## Execution Flow (main)

```txt
1. Load plan.md from feature directory
   → Found: tech stack (C/C++, Pico SDK), embedded firmware project
   → Extract: CMake build system, micro-ROS communication, multi-core utilization
2. Load available design documents:
   → data-model.md: LauncherState, MotorControllerState, EncoderState, ImuState, BatteryStatus
   → contracts/micro-ros-contracts.md: micro-ROS topics and messages
   → research.md: Technical decisions on micro-ROS, motor control, concurrency, sensors, build, safety, persistence, logging
   → quickstart.md: Build/test procedures, specific test commands
3. Generate tasks by category:
   → Setup: Project init, dependencies, linting
   → Tests: Contract tests, integration tests
   → Core: Data structures, micro-ROS publishers/subscribers, motor/launcher/sensor control
   → Integration: Multi-core, thread-safety, safety features, data persistence, watchdog
   → Polish: Optimization, documentation, cleanup
4. Apply task rules:
   → Different files = mark [P] for parallel
   → Same file = sequential (no [P])
   → Tests before implementation (TDD)
5. Number tasks sequentially (T001, T002...)
6. Dependencies: Setup before everything; Test procedures before implementation; Data structures before their usage; Core before Integration; Integration before Polish.
7. Parallel execution: Contract tests, data structure implementations, sensor publishers.
8. Validation: All contracts tested, all data structures implemented, all functional requirements covered.
```

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in firmware directory structure

## Path Conventions

- **Firmware project**: root_path=`micro_ros_raspberrypi_pico_sdk/` (this is the main firmware directory)
- **Tests**: Manual hardware tests and smoke test procedures (documented in quickstart.md)
- **Contracts**: Communication protocols (documented in micro-ros-contracts.md)

## Phase 3.1: Setup

- [x] T001 Use the project structure from our Directory in `root=micro_ros_raspberrypi_pico_sdk/`
- [x] T002 Initialize CMake project with Pico SDK and micro-ROS dependencies in `/CMakeLists.txt`
- [x] T003 [P] Configure build system with proper compiler flags and TinyUSB in CMake
- [x] T004 [P] Create pin definitions header in `/include/config.h`

## Phase 3.2: Tests First (TDD) ⚠️ MUST COMPLETE BEFORE 3.3

### CRITICAL: These tests MUST be written and MUST FAIL before ANY implementation

- [x] T005 [P] Create contract test procedures for `/cmd_vel` (geometry_msgs/Twist) as described in `quickstart.md`
- [x] T006 [P] Create contract test procedures for `/launcher_control` (sensor_msgs/JointState) as described in `quickstart.md`
- [x] T007 [P] Create contract test procedures for `/launcher/fire` (std_msgs/Bool) and `/launcher/fire_srv` (std_srvs/Trigger) as described in `quickstart.md`
- [x] T008 [P] Create contract test procedures for `/imu/data_raw` (sensor_msgs/Imu) as described in `quickstart.md`
- [x] T009 [P] Create contract test procedures for `/odom` (nav_msgs/Odometry) as described in `quickstart.md`
- [x] T010 [P] Create contract test procedures for `/tof_sensor` (sensor_msgs/Range) as described in `quickstart.md`
- [x] T011 [P] Create contract test procedures for `/battery_state` (sensor_msgs/BatteryState) as described in `quickstart.md`
- [x] T012 [P] Create integration test procedures for motor control (e.g., verify motor movement with `geometry_msgs/Twist`)
- [x] T013 [P] Create integration test procedures for launcher control (e.g., verify pan/tilt and fire commands)
- [x] T014 [P] Create integration test procedures for sensor data publishing (e.g., verify data rates and content)
- [x] T015 [P] Create integration test procedures for safety stop and critical battery reporting
- [x] T016 [P] Create integration test procedures for sensor failure behavior
- [x] T017 [P] Create integration test procedures for out-of-scope feature handling
- [x] T018 [P] Create integration test procedures for data persistence
- [x] T019 [P] Create integration test procedures for logging

## Phase 3.3: Core Implementation (ONLY after tests are failing)

- [x] T020 [P] Implement `LauncherState` data structure in `/include/shared/shooter_types.h`
- [x] T021 [P] Implement `MotorControllerState` data structure in `/include/shared/motor_types.h`
- [x] T022 [P] Implement `EncoderState` data structure in `/include/shared/motor_types.h`
- [x] T024 [P] Implement `BatteryStatus` data structure in `/include/shared/system_config.h`
- [x] T025 Implement micro-ROS communication setup and initialization in `/src/middleware/micro_ros_cfg.cpp`
- [x] T026 Implement `/cmd_vel` subscriber in `/src/middleware/command_processor.cpp`
- [x] T027 Implement `/launcher_control` subscriber in `/src/application/shooter_control.cpp`
- [x] T028 Implement `/launcher/fire` subscriber (or service server) in `/src/application/shooter_control.cpp`
- [x] T029 Implement `/imu/data_raw` publisher in `/src/hardware/ImuLib_cfg.cpp`
- [x] T030 Implement `/odom` publisher in `/src/middleware/control_loop.cpp`
- [x] T031 Implement `/tof_sensor` publisher in `/src/middleware/control_loop.cpp`
- [x] T032 Implement `/battery_state` publisher in `/src/middleware/control_loop.cpp`
- [x] T033 Implement motor control (PID, TB6612FNG drivers, PWM) in `/src/hardware/motors.cpp`
- [x] T034 Implement encoder reading (Hall-Encoders) in `/src/middleware/control_loop.cpp`
- [x] T035 Implement launcher control (servos, brushless motors) in `/src/application/shooter_control.cpp`
- [x] T036 Implement logging mechanism in `/src/middleware/usb_interface.cpp`

**Note: All implementations in Phase 3.3 use modern C++ (C++17 or later) only. No mixing of C and C++. Existing .c files (command_processor.c, usb_interface.c) should be renamed to .cpp and updated to C++ syntax without creating new files.**

## Phase 3.4: Integration

- [x] T037 Integrate multi-core utilization (Core0 for micro-ROS, Core1 for motor control) in `/src/application/main.cpp`
- [x] T038 Implement thread-safe access to shared data structures using `critical_section_t` or atomics in `/src/middleware/queue_wrapper.cpp`
- [ ] T039 Implement safety stop (200ms timeout) in `/src/middleware/control_loop.cpp`
- [ ] T040 Implement critical battery status reporting (< 10.5V) in `/src/middleware/control_loop.cpp`
- [ ] T041 Implement safe stop on connection loss in `/src/middleware/control_loop.cpp`
- [ ] T042 Implement sensor failure handling in `/src/middleware/control_loop.cpp`
- [ ] T043 Implement out-of-scope feature handling (safety interlocks) in `/src/middleware/control_loop.cpp`
- [ ] T044 Implement data persistence (flash memory, calibration data) in `/include/shared/system_config.h`
- [x] T045 Integrate watchdog timer in `/src/application/watchdog_manager.cpp`

**Note: All Phase 3.4 tasks are implemented in existing files to avoid creating new ones. Safety features are integrated into `/src/middleware/control_loop.cpp` for centralized control, data persistence into `/include/shared/system_config.h` for configuration management, and thread-safety into `/src/middleware/queue_wrapper.cpp` for queue operations. The project uses a Layer-Based Structure: hardware/ for low-level, middleware/ for communication/control, application/ for high-level logic, and shared/ for common headers.**

## Phase 3.5: Polish

- [ ] T046 [P] Optimize PID parameters for motor control
- [ ] T047 [P] Refine sensor publishing rates
- [ ] T048 [P] Review and optimize logging output
- [ ] T049 [P] Update quickstart guide with final test procedures
- [ ] T050 [P] Final code review and cleanup

## Dependencies

- Setup (T001-T004) before everything
- Test procedures (T005-T019) before Core Implementation (T020-T036)
- Data structures (T020-T024) before their usage in Core Implementation
- Micro-ROS setup (T025) before publishers/subscribers (T026-T032)
- Motor control (T033) and encoder reading (T034) before odometry publisher (T030)
- Launcher control (T035) before launcher subscribers (T027-T028)
- Logging (T036) can be integrated throughout
- Core Implementation (T020-T036) before Integration (T037-T045)
- Multi-core utilization (T037) and thread-safe access (T038) are foundational for Integration
- Safety features (T039-T043, T045) are critical and depend on core components
- Data persistence (T044) depends on core components
- Integration (T037-T045) before Polish (T046-T050)

## Parallel Example

```bash
# Launch contract tests together (Phase 3.2):
Task: "T005 Create contract test procedures for /cmd_vel (geometry_msgs/Twist)"
Task: "T006 Create contract test procedures for /launcher_control (sensor_msgs/JointState)"
Task: "T007 Create contract test procedures for /launcher/fire (std_msgs/Bool) and /launcher/fire_srv (std_srvs/Trigger)"

# Launch data structure implementations together (Phase 3.3):
Task: "T020 Implement LauncherState data structure"
Task: "T021 Implement MotorControllerState data structure"
Task: "T022 Implement EncoderState data structure"
```

## Notes

- [P] tasks target different files and have no dependencies
- Hardware tests require physical Pico and robot hardware
- Preserve exact STM32 function names per constitution requirements (if applicable)
- Safety features (watchdog, output limits) are non-negotiable
- CMake build must work with standard Pico SDK installation
- Micro-ROS communication must not block real-time motor control loop

## Task Generation Rules

**Applied during task generation:**

1. **From Contracts (micro-ros-contracts.md)**:
   - `/cmd_vel` → T005 (test) + T026 (implementation)
   - `/launcher_control` → T006 (test) + T027 (implementation)
   - `/launcher/fire` / `/launcher/fire_srv` → T007 (test) + T028 (implementation)
   - `/imu/data_raw` → T008 (test) + T029 (implementation)
   - `/odom` → T009 (test) + T030 (implementation)
   - `/tof_sensor` → T010 (test) + T031 (implementation)
   - `/battery_state` → T011 (test) + T032 (implementation)

2. **From Data Model (data-model.md)**:
   - `LauncherState` → T020 (implementation)
   - `MotorControllerState` → T021 (implementation)
   - `EncoderState` → T022 (implementation)
   - `ImuState` → T023 (implementation)
   - `BatteryStatus` → T024 (implementation)

3. **From Research Decisions (research.md)**:
   - Micro-ROS Communication → T025 (setup)
   - Motor Control (PID, 1kHz, Core1) → T033 (motor control), T037 (multi-core)
   - Concurrency (`critical_section_t`/atomics) → T038 (thread-safe access)
   - Sensor Integration → T029-T032 (publishers)
   - Build System → T002-T003 (CMake setup)
   - Safety Features → T039-T043, T045 (safety manager, watchdog)
   - Data Persistence → T044 (data persistence)
   - Logging → T036 (logging mechanism)

4. **From Quickstart Scenarios (quickstart.md)**:
   - Build/Flash → T001-T004 (setup)
   - Test procedures → T005-T019 (test tasks)
   - Specific test commands → T049 (update quickstart guide)

## Validation Checklist

- [x] All contracts have corresponding tests
- [x] All data model entities have implementation tasks
- [x] All tests come before implementation
- [x] Parallel tasks truly independent
- [x] Each task specifies exact file path
- [x] No [P] task modifies same file as another [P] task
- [x] Safety requirements covered (watchdog, safety stop, battery reporting, sensor failure, out-of-scope handling)
- [x] Constitution compliance verified (e.g., 1kHz motor loop, logging, data persistence)

---

## Current Folder Structure (Firmware Build Relevant)

```txt
/mnt/wsl/docker-desktop-bind-mounts/Ubuntu-22.04/ca9dcf122dac17ed248e04d826612cb62f802e17963be8f5fc4e8849981882dd/src/robot_firmware/example_repo/micro_ros_raspberrypi_pico_sdk/
├───CMakeLists.txt
├───pico_micro_ros_example.c
├───include/
│   ├───hardware/
│   │   ├───bsp.h
│   │   ├───config.h
│   │   ├───ImuLib_cfg.h
│   │   ├───motors.h
│   │   └───UartLib.h
│   ├───middleware/
│   │   ├───command_processor.h
│   │   ├───control_loop.h
│   │   ├───micro_ros_cfg.h
│   │   ├───queue_wrapper.h
│   │   └───usb_interface.h
│   ├───application/
│   │   ├───shooter_control.h
│   │   └───watchdog_manager.h
│   ├───shared/
│   │   ├───hardware_cfg.h
│   │   ├───imu_types.h
│   │   ├───motor_types.h
│   │   ├───shooter_types.h
│   │   └───system_config.h
│   └───transport/
│       └───pico_uart_transports.h
├───lib/
│   └───imu-driver/
│       └───ICM-20948/
│           ├───ICM20948Adapter.cpp
│           └───ICM20948Adapter.h
├───libmicroros/
│   ├───libmicroros.a
│   └───include/...
├───microros_static_library/
│   └───library_generation/
│       ├───colcon.meta
│       ├───library_generation.sh
│       ├───toolchain.cmake
│       └───extra_packages/
├───src/
│   ├───hardware/
│   │   ├───bsp.cpp
│   │   ├───ImuLib_cfg.cpp
│   │   ├───motors.cpp
│   │   ├───quadrature_encoder.pio
│   │   ├───UartLib.cpp
│   │   └───transport/
│   │       └───pico_uart_transport.cpp
│   ├───middleware/
│   │   ├───command_processor.cpp
│   │   ├───control_loop.cpp
│   │   ├───micro_ros_cfg.cpp
│   │   ├───queue_wrapper.cpp
│   │   └───usb_interface.cpp
│   └───application/
│       ├───main.cpp
│       ├───shooter_control.cpp
│       └───watchdog_manager.cpp
```
