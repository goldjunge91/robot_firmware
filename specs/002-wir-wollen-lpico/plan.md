# Implementation Plan: Rebuild lpico_pio based on micro_ros_raspberrypi_pico_sdk

**Branch**: `002-wir-wollen-lpico` | **Date**: 2025-09-23 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-wir-wollen-lpico/spec.md`

## Key Architectural Documents
- **[PINMAP.md](../../PINMAP.md)**: Single source of truth for all MCU pin assignments.
- **[architecture_and_packages.md](../../architecture_and_packages.md)**: Describes the high-level ROS2 architecture and package responsibilities.

## Summary
The goal is to create a new firmware for the Raspberry Pi Pico based on the `micro_ros_raspberrypi_pico_sdk`. This new firmware will replace the existing `lpico_pio` project. The project structure and parts of the code will be adapted from `rosbot_ros2_firmware`. The firmware will control a robot with 4 motors, encoders, an IMU (ICM-20948), a ToF sensor (VL53L0X), and a Nerf launcher (2 ESCs, 2 Servos), as detailed in the `PINMAP.md`.

## Technical Context
**Language/Version**: C/C++
**Primary Dependencies**: micro-ROS, Raspberry Pi Pico SDK, FreeRTOS
**Storage**: N/A
**Testing**: Pending research task T001 (Investigate and recommend a suitable unit testing framework for the Raspberry Pi Pico and the current project setup.)
**Target Platform**: Raspberry Pi Pico (RP2040)
**Project Type**: single project (firmware for digital board)
**Performance Goals**: Real-time control loop for motors and sensors (target 200-1000 Hz).
**Constraints**: Must be compatible with ROS2 and use micro-ROS. The public API and ROS2 topics/services should align with `architecture_and_packages.md` and be compatible with `rosbot_ros2_firmware` where possible.
**Scale/Scope**: Firmware for a mobile robot with a Nerf launcher, acting as the micro-ROS client in a larger ROS2 ecosystem.

## Constitution Check
*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

- **Real-Time Control Integrity**: The control loop must be non-blocking. This will be documented and verified.
- **Fail-Safe Actuation Envelope**: The firmware will include a watchdog timer, motor output clamping, and a failsafe mechanism to stop motors if control messages are lost.
- **ROS 2 Contract Fidelity**: The ROS 2 API will be designed to be compatible with the definitions in `architecture_and_packages.md`.
- **Test Discipline**: Tests will be created for the new functionality using the chosen testing framework.
- **Reproducible Firmware Delivery**: The existing CMake build system will be used to ensure reproducible builds.

## Project Structure

### Documentation (this feature)
```
specs/002-wir-wollen-lpico/
├── plan.md              # This file (/plan command output)
├── research.md          # Phase 0 output (/plan command)
├── data-model.md        # Phase 1 output (/plan command)
├── quickstart.md        # Phase 1 output (/plan command)
├── contracts/           # Phase 1 output (/plan command)
└── tasks.md             # Phase 2 output (/tasks command - NOT created by /plan)
```

### Source Code (repository root)
```
# Option 1: Single project (DEFAULT)
src/
├── bsp.cpp
├── ImuLib_cfg.cpp
├── main.cpp
├── micro_ros_cfg.cpp
├── motors.cpp
├── STM32FreeRTOSConfig.h
├── UartLib.cpp
└── robot_firmware/

tests/
├── contract/
├── hardware/
└── integration/
```

**Structure Decision**: Option 1: Single project

## Phase 0: Outline & Research
1. **Extract unknowns from Technical Context**:
   - Research task: "Investigate and recommend a suitable unit testing framework for the Raspberry Pi Pico and the current project setup."
2. **Generate and dispatch research agents**:
   - Task: "Research best practices for integrating micro-ROS with the Raspberry Pi Pico SDK and FreeRTOS."
   - Task: "Find best practices for structuring a complex firmware project for a mobile robot on the RP2040, following the architecture in `architecture_and_packages.md`."
3. **Consolidate findings** in `research.md`.

**Output**: `research.md` with all NEEDS CLARIFICATION resolved.

## Phase 1: Design & Contracts
*Prerequisites: research.md complete*

1. **Extract entities from feature spec and PINMAP.md** → `data-model.md`:
   - Define data structures for IMU (ICM-20948), ToF sensor (VL53L0X), motors (TB6612), encoders, servos, and robot state.
2. **Generate API contracts from `architecture_and_packages.md`** → `/contracts/`:
   - **Topics:**
     - `encoder_ticks` (custom message)
     - `imu` (sensor_msgs/Imu)
     - `battery` (sensor_msgs/BatteryState)
     - `/_motors_response` (custom message)
   - **Services:**
     - `/_motors_cmd` (custom service)
     - `/nerf/fire_cmd` (std_srvs/Trigger)
     - `/nerf/servo_cmd` (custom service)
3. **Generate contract tests** from contracts.
4. **Extract test scenarios** from user stories → `quickstart.md`.
5. **Update agent file incrementally**.

**Output**: `data-model.md`, `/contracts/*`, failing tests, `quickstart.md`.

## Phase 2: Task Planning Approach
*This section describes what the /tasks command will do - DO NOT execute during /plan*

**Task Generation Strategy**:
- Generate tasks from the design documents (`data-model.md`, `contracts/`, `quickstart.md`).
- Create tasks for setting up the new project structure based on `micro_ros_raspberrypi_pico_sdk`.
- Create tasks for migrating code from `lpico_pio` and `rosbot_ros2_firmware`, mapping old code to the new hardware defined in `PINMAP.md`.
- Create tasks for implementing the ROS2 contracts.
- Create tasks for writing tests for each module.

**Ordering Strategy**:
1. Project setup.
2. High-priority features (Motor control, safety, encoders) based on `spec.md`.
3. Medium-priority features (micro-ROS communication, IMU).
4. Low-priority features (Shooter, LEDs).
5. Testing.

## Complexity Tracking
| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| (none)    |            |                                     |

## Progress Tracking
*This checklist is updated during execution flow*

**Phase Status**:
- [ ] Phase 0: Research complete (/plan command)
- [ ] Phase 1: Design complete (/plan command)
- [ ] Phase 2: Task planning complete (/plan command - describe approach only)
- [ ] Phase 3: Tasks generated (/tasks command)
- [ ] Phase 4: Implementation complete
- [ ] Phase 5: Validation passed

**Gate Status**:
- [X] Initial Constitution Check: PASS
- [ ] Post-Design Constitution Check: PASS
- [ ] All NEEDS CLARIFICATION resolved
- [ ] Complexity deviations documented