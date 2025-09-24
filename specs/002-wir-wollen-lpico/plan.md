# Implementation Plan: Rebuild lpico_pio based on micro_ros_raspberrypi_pico_sdk

**Branch**: `002-wir-wollen-lpico` | **Date**: 2025-09-23 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-wir-wollen-lpico/spec.md`

## Key Architectural Documents

- **[PINMAP.md](../../PINMAP.md)**: Single source of truth for all MCU pin assignments.
- **[architecture_and_packages.md](../../architecture_and_packages.md)**: Describes the high-level ROS2 architecture and package responsibilities.

## Summary

The goal is to create a production-ready firmware for the Raspberry Pi Pico (RP2040) based on the `micro_ros_raspberrypi_pico_sdk` that is fully compatible with micro-ROS and the project's ROS2 contracts. The firmware will follow the same high-level structure and conventions used by `/example_repo/rosbot_ros2_firmware/` and reuse/adapt components from that repository where it makes sense.

This project must prioritise getting micro-ROS integration correct and robust: the build, the middleware/client configuration, transport (UART/SPI/etc.), and timing behaviour under FreeRTOS must be validated early. Equally important is correct, deterministic control of the robot hardware: a mecanum drive with 4 motors and encoders, an IMU (ICM-20948), a ToF distance sensor (VL53L0X), a Nerf launcher subsystem (2 ESCs + 2 servos), and optional LIDAR/vision components. See `PINMAP.md` for pin-level wiring.

Scope highlights and priorities:
- micro-ROS integration and transport + build/packaging for RP2040 (high priority)
- Real-time-safe motor control for mecanum wheels with encoder feedback (high priority)
- Encoder interfaces and reliable tick publishing (high priority)
- Nerf shooter (ESC safety, servo control, fire sequence) (medium priority)
- IMU and ToF sensor drivers and sensor fusion/filters suitable for control and odometry (medium)
- LIDAR and vision components integration points (optional/low priority initially; provide clear extension hooks)
- Safety: watchdog, motor output clamping, failsafe on lost control messages (must-have)

We will emphasise building small, testable modules with clear ROS2 topics/services and contract tests so micro-ROS behaviour can be validated independently from motor/hardware code.

## Technical Context

**Language/Version**: C/C++
**Primary Dependencies**: micro-ROS, Raspberry Pi Pico SDK
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
## Key Architectural Documents

- **[PINMAP.md](../../PINMAP.md)**: Single source of truth for all MCU pin assignments.
- **[architecture_and_packages.md](../../architecture_and_packages.md)**: Describes the high-level ROS2 architecture and package responsibilities.

```text
specs/002-wir-wollen-lpico/
## Summary

The goal is to create a production-ready firmware for the Raspberry Pi Pico (RP2040) based on the `micro_ros_raspberrypi_pico_sdk` that is fully compatible with micro-ROS and the project's ROS2 contracts. The firmware will follow the same high-level structure and conventions used by `/example_repo/rosbot_ros2_firmware/` and reuse/adapt components from that repository where it makes sense.
├── plan.md              # This file (/plan command output)
├── research.md          # Phase 0 output (/plan command)
├── data-model.md        # Phase 1 output (/plan command)
## Technical Context

**Language/Version**: C/C++
├── quickstart.md        # Phase 1 output (/plan command)
├── contracts/           # Phase 1 output (/plan command)
└── tasks.md             # Phase 2 output (/tasks command - NOT created by /plan)
## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*
```

### Source Code (repository root)

```text
# Option 1: Single project (DEFAULT)
src/
 
├── bsp.cpp
 
├── ImuLib_cfg.cpp
 
## Project Structure

### Documentation (this feature)

```text
specs/002-wir-wollen-lpico/
├── plan.md              # This file (/plan command output)
├── research.md          # Phase 0 output (/plan command)
├── data-model.md        # Phase 1 output (/plan command)
├── quickstart.md        # Phase 1 output (/plan command)
├── contracts/           # Phase 1 output (/plan command)
└── tasks.md             # Phase 2 output (/tasks command - NOT created by /plan)
```
├── main.cpp
├── micro_ros_cfg.cpp
├── motors.cpp
## Phase 0: Outline & Research (revised)

Phase 0 will be heavily focused on micro-ROS correctness and testability. Deliverables should let implementation proceed with low risk.
├── STM32FreeRTOSConfig.h
├── UartLib.cpp
└── robot_firmware/
## Phase 1: Design & Contracts (revised)

*Prerequisite: `research.md` with micro-ROS decisions and a validated minimal prototype.*

tests/
├── contract/
├── hardware/
## Phase 2: Task Planning Approach

### Note

This section describes what the /tasks command will do - DO NOT execute during /plan
└── integration/
```
**Task Generation Strategy**:

- Generate tasks from the design documents (`data-model.md`, `contracts/`, `quickstart.md`).
**Structure Decision**: Option 1: Single project

## Phase 0: Outline & Research (revised)
**Ordering Strategy**:

1. Project setup.

Phase 0 will be heavily focused on micro-ROS correctness and testability. Deliverables should let implementation proceed with low risk.

1. Extract unknowns and risks from the Technical Context:
   - micro-ROS transport choices for RP2040 (UART vs other transports), client config, and reliability under FreeRTOS.
## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
| --------- | ---------- | ------------------------------------ |
| (none)    |            |                                      |
   - CMake/build differences needed to integrate `micro_ros_raspberrypi_pico_sdk` and ensure reproducible binary artifacts.
   - Recommended unit/integration testing approaches on RP2040 (host-side tests, emulation, hardware-in-the-loop) and how to test micro-ROS communications.
   - Hardware driver unknowns: encoder electrical interface and debouncing, ESC arming sequences, IMU register-level quirks, and VL53L0X timing constraints.

## Progress Tracking

### Checklist note

This checklist is updated during execution flow
2. Research tasks (each produces a short actionable note and at least one recommended follow-up):
   - Research best practices for integrating micro-ROS with the Raspberry Pi Pico SDK and FreeRTOS. Produce a micro-ROS checklist (transport, memory, threading, example minimal client).
**Phase Status**:

- [ ] Phase 0: Research complete (/plan command)
- [ ] Phase 1: Design complete (/plan command)
- [ ] Phase 2: Task planning complete (/plan command - describe approach only)
- [ ] Phase 3: Tasks generated (/tasks command)
- [ ] Phase 4: Implementation complete
- [ ] Phase 5: Validation passed

# Implementation Plan: Rebuild lpico_pio based on micro_ros_raspberrypi_pico_sdk

**Branch**: `002-wir-wollen-lpico` | **Date**: 2025-09-24

## Key Architectural Documents

- **`PINMAP.md`**: single source of truth for MCU pin assignments (see repository root).
- **`architecture_and_packages.md`**: ROS2 architecture and package responsibilities.

## Summary (updated)

Create a production-quality firmware for the Raspberry Pi Pico (RP2040) using `micro_ros_raspberrypi_pico_sdk`. The primary emphasis for this rewrite is correctness and robustness of the micro-ROS integration (build, client configuration, transport, memory, and timing under FreeRTOS). Equally critical is deterministic and safe control of the robot hardware:

- Mecanum drive: 4 motors + motor driver (TB6612 or similar) with per-wheel encoders and deterministic motor control loops.
- Encoders: reliable ISR/PIO-based counting or PIO/TPM solution to support high-frequency tick rates and odometry.
- IMU: ICM-20948 driver, calibration and a configurable filter/fusion interface for odometry/pose.
- ToF: VL53L0X driver with timing configuration for the expected update rate.
- Nerf launcher: two ESC channels and two servos with explicit arm/disarm and safety sequencing.
- LIDAR / Vision: provide integration hooks and an extensible driver pattern (optional initial scope).

Top priorities:

1. Micro-ROS correctness and reproducible build/flash flow for RP2040 (USB CDC/TinyUSB transport preferred; validate micro-ROS agent interoperability).
2. Deterministic motor control for mecanum wheels with encoder feedback and safety (watchdog, output clamping, heartbeat/timeout behavior).
3. Encoder reliability and publishable encoder_ticks topic with timestamps for odometry.
4. Nerf subsystem safety and deterministic fire/servo commands.

Design goals:

- Small, testable modules with clear ROS2 topics/services and contract tests.
- Non-blocking control loops suitable for realtime requirements (200–1000 Hz where applicable).
- Clear extension points for LIDAR/vision and for swapping transport mechanisms.

## Technical Context

- Language: C/C++
- Primary dependencies: micro-ROS, Raspberry Pi Pico SDK, TinyUSB (USB-CDC) for host transport
- Target: Raspberry Pi Pico (RP2040)
- Performance goals: deterministic control loop for motors/sensors, low-latency micro-ROS messaging

Constraints: must remain compatible with ROS2 via micro-ROS and align with conventions in `architecture_and_packages.md` and `rosbot_ros2_firmware` where appropriate.

## Constitution Check (gates)

- Real-Time Control Integrity: control loops must be non-blocking and preempt-safe.
- Fail-Safe Actuation Envelope: watchdog, motor output clamping, and safe motor stop when micro-ROS heartbeat is lost.
- ROS2 Contract Fidelity: topics/services must match contracts (see Phase 1).
- Test Discipline: unit/integration tests and host-side contract tests must be specified in Phase 0.
- Reproducible Delivery: CMake + documented build steps produce deterministic firmware artifacts.

## Project Structure (high-level)

Documentation: `specs/002-wir-wollen-lpico/` { plan.md, research.md, data-model.md, quickstart.md, contracts/ }

Source (suggested single-project layout):

- `src/` : drivers (imu, vl53l0x, encoders), motor control, micro_ros glue, main
- `bsp/` : board-specific pinmap and hardware initialisation
- `tests/` : contract, hardware-in-the-loop and unit tests (host-side harness)

Decision: single project to speed iteration and keep build manageable.

## Phase 0: Outline & Research (focus on micro-ROS)

Goal: remove uncertainty around micro-ROS transport, build, and testability. Deliver a short prototype demonstrating a micro-ROS client on RP2040 that publishes a dummy topic and responds to a simple service.

Work items:

1. Micro-ROS checklist: transport (USB CDC vs UART), memory footprint, RTOS threading model, minimal working client example for RP2040.
2. Build/CMake recipe: how to integrate `micro_ros_raspberrypi_pico_sdk` and produce a reproducible binary; document flash steps.
3. Testing approach: recommend host-side contract tests (ROS2 desktop + micro-ROS agent) and hardware-in-the-loop strategy for encoders/motors.
4. Encoder strategy: ISR vs PIO vs PIO+DMA for four channels; recommend preferred approach and trade-offs.
5. Measure micro-ROS latency: plan a micro-benchmark for message round-trip under FreeRTOS at different priorities.

Deliverable: `research.md` with decisions, risk register, and a small prototype plan.

Go/no-go: prototype publishes and receives a micro-ROS message reliably; toolchain build + flash is documented.

## Phase 1: Design & Contracts

Prerequisite: Phase 0 complete and prototype validated.

1. Data model (`data-model.md`): define messages for encoders, IMU, battery, motor commands, and nerf subsystem.

2. Contracts (`/contracts/`) and tests:

- Topics:
  - `encoder_ticks` (custom: per-wheel ticks + timestamp)
  - `imu` (sensor_msgs/Imu)
  - `battery` (sensor_msgs/BatteryState)
  - `cmd_motors` (geometry_msgs/Twist or a defined low-level motor command)
  - `_motors_response` (status/ack)

- Services:
  - `/_motors_cmd` (safe low-level motor service used for testing)
  - `/nerf/fire_cmd` (std_srvs/Trigger with explicit safety precondition)
  - `/nerf/servo_cmd` (custom service for servo control)

3. Contract tests: host-side test suite that runs against a micro-ROS agent and the flashed device; validate message shapes, response times, and safety semantics (e.g., motors stop on heartbeat loss).

4. Quickstart: `quickstart.md` describing build, flash, running micro-ROS agent and running contract tests.

Output: `data-model.md`, `/contracts/*`, and failing (expected) contract tests until implementation completes.

## Phase 2: Task Planning Approach

This phase will break design into concrete implementation tasks and order them by risk and dependency.

Task generation strategy:

- Create tasks for project skeleton and CMake integration with `micro_ros_raspberrypi_pico_sdk`.
- Implement low-level drivers (IMU, VL53L0X, encoders) as independent modules with unit-testable interfaces.
- Implement motor control loop with safety watchdog and encoder feedback; publish `encoder_ticks`.
- Implement micro-ROS glue (publishers/subscribers/services) and run contract tests.
- Implement Nerf subsystem with explicit arm/disarm and safety checks.

Ordering strategy:

1. Project setup & micro-ROS prototype (short loop to validate), 2. Motor control + encoders + safety, 3. micro-ROS integration for topics/services, 4. IMU/ToF drivers + sensor fusion, 5. Nerf subsystem, 6. LIDAR/vision extension points, 7. Testing and CI.

## Tests and Validation

- Unit tests (host-side where possible) for pure logic and message serialization.
- Integration/contract tests using a ROS2 host + micro-ROS agent.
- Hardware-in-the-loop tests for encoder counting and motor driver behavior (bench harness or simulator signals).

CI smoke: build firmware, run host-side contract tests against a simulated micro-ROS agent or recorded messages where hardware not available.

## Next steps (immediate)

1. Finish `research.md` (Prototype micro-ROS client + transport decision).
2. Draft `data-model.md` and `/contracts/` (start with encoder and motors messages/services).
3. Create project skeleton and CMake integration for micro-ROS.

---

## Progress tracking (short)

- Phase 0: Research — TODO (prototype in progress)
- Phase 1: Design — TODO
- Phase 2: Tasks — TODO

Gate: Initial constitution checks passed; Phase 0 must validate micro-ROS prototype before implementing Phase 1.
