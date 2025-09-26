<!--
Sync Impact Report
- Version change: 1.0.0 -> 2.0.0
- Modified principles:
  - "Simplicity First" -> "Safety and Testability"
  - "Naming & Compatibility" -> "Toolchain and Compatibility"
  - "Safety & Fail-safe" -> "Safety and Reliability"
  - "Test-First Iteration" -> "Test Policy"
  - "Reproducible Builds & Documentation" -> removed
- Added sections: None
- Removed sections: "Reproducible Builds & Documentation"
- Templates requiring updates:
  - .specify/templates/plan-template.md ⚠ pending
  - .specify/templates/spec-template.md ⚠ pending
  - .specify/templates/tasks-template.md ⚠ pending
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Original ratification date unknown — please fill or confirm when ratified.
-->

# Constitution for micro-ROS module for Raspberry Pi Pico SDK

## Core Principles

### I. Safety and Testability (REQUIRED)

All requirements MUST be clear, testable, and safety-conscious. This includes the implementation of safety mechanisms such as timeouts.

Rationale: Ensures the firmware is robust, reliable, and behaves predictably, which is critical for robotic applications.

### II. Toolchain and Compatibility (NON-NEGOTIABLE)

The firmware MUST be compatible with the specified GNU Arm Embedded Toolchain (e.g., arm-none-eabi-gcc 9.3.1 / 10.3-2021.10) and the Raspberry Pi Pico SDK. The `PICO_SDK_PATH` and `PICO_TOOLCHAIN_PATH` environment variables must be configured correctly.

Rationale: Strict toolchain and SDK compatibility ensures that the firmware can be built reliably by all developers and in CI/CD environments.

### III. Safety and Reliability (REQUIRED)

The firmware MUST implement the following safety and reliability features:
- A safety stop that halts all motors if no commands are received within 200 ms.
- Reporting of a critical battery status when the voltage drops below 10.5V.
- A safe stop of all motors in case of connection loss to the high-level controller.
- Continuous monitoring of all sensors (IMU, encoders, VL53L0X, INA3221) with status published via micro-ROS.

Rationale: These measures are essential to prevent damage to the robot and its environment, and to provide the high-level controller with the necessary information to make decisions.

### IV. Test Policy (STRONGLY RECOMMENDED)

All requirements MUST be testable and have measurable acceptance criteria. Automated unit and integration tests are preferred. Integration tests and checks MUST be run before merging any changes.

Rationale: A strong test policy ensures that new features work as expected and do not introduce regressions.

## Additional Constraints

- Target hardware: Raspberry Pi Pico (RP2040).
- Language & toolchain: C/C++ with the specified GNU Arm Embedded Toolchain.
- Real-time requirements: Motor control loops SHOULD be non-blocking and run at a rate that is sufficient for stable control.

## Development Workflow & Quality Gates

- Branches: Short-lived feature branches.
- Commits: Small, self-contained, with a clear explanation of the "why".
- Reviews: At least one reviewer for non-trivial changes.
- Tests & Checks before merge:
  - Local build must succeed.
  - Static analysis checks must pass.
  - All automated tests must pass.

## Constitution Check (gates used by `.specify/templates/plan-template.md`)

- Real-Time Control Integrity: Document expected control loop frequency and prove new code is non-blocking.
- Fail-Safe Actuation Envelope: Every actuator change MUST include watchdog coverage, output limits, and a manual procedure to stop the robot.
- ROS 2 Contract Fidelity: List any topic/service name changes (these are discouraged).
- Test Discipline: List unit/HIL/bench tests created for the change.
- Reproducible Firmware Delivery: Provide exact build and upload commands.

## Governance

Amendments to this constitution require a documented rationale and one approving review from a maintainer.
- MAJOR version for backward-incompatible changes.
- MINOR version for new principles or sections.
- PATCH version for clarifications and typo fixes.

**Version**: 2.0.0 | **Ratified**: TODO(RATIFICATION_DATE): please confirm | **Last Amended**: 2025-09-25