# my_steel-robot_ws Firmware Constitution

<!--
Sync Impact Report
- Version change: unspecified -> 1.0.0
- Modified principles: (none previously defined) -> Added: Simplicity First; Naming & Compatibility; Safety & Fail-safe; Test-First Iteration; Reproducible Builds & Docs
- Added sections: Additional Constraints (Target hardware & build), Development Workflow (Beginner-focused gates)
# my_steel-robot_ws Firmware Constitution

<!--
Sync Impact Report
- Version change: unspecified -> 1.0.0
- Modified principles: (none previously defined) -> Added: Simplicity First; Naming & Compatibility; Safety & Fail-safe; Test-First Iteration; Reproducible Builds & Docs
- Added sections: Additional Constraints (Target hardware & build), Development Workflow (Beginner-focused gates)
- Templates requiring updates:
  - .specify/templates/plan-template.md ✅ reviewed and aligned with Constitution Check
  - .specify/templates/spec-template.md ✅ reviewed (no mandatory changes)
  - .specify/templates/tasks-template.md ✅ reviewed (no mandatory changes)
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Original ratification date unknown — please fill or confirm when ratified.
  - Verify automated checks integrate watchdog and hardware-in-loop tests (listed in Constitution Check) — manual review recommended.
-->

## Core Principles

### I. Simplicity First (KISS) - REQUIRED

All firmware code MUST be simple, readable, and obvious to a beginner. Prefer short functions, clear variable names,
and minimal dependencies. Apply DRY (Don't Repeat Yourself) where it reduces complexity, but avoid premature
abstraction. Every new abstraction MUST have a concrete, repeated use-case.

Rationale: the project owner is learning programming; the firmware must be maintainable and approachable.

### II. Naming & Compatibility (NON-NEGOTIABLE)

All public symbols (file names, function names, global variables, constants) required by higher layers MUST keep
the exact names used in the legacy STM32 port. When porting code from `legacy_stm32/` to `pico/`, preserve
the original API names to make porting of controller and ROS-side code trivial.

Rationale: Minimizes integration friction — ROS2-side code and higher-level scripts expect those names.

### III. Safety & Fail-safe (REQUIRED)

Firmware MUST include simple, auditable safety measures: a watchdog timer, motor-output clamping, and a
software-level deadman/failsafe that forces motors to zero if control messages stop. Hardware interfaces MUST
avoid blocking calls in control loops; any blocking operation MUST run in a separate task/thread with timeout.

Rationale: Basic safety prevents runaway motors and protects hardware while learning and iterating quickly.

### IV. Test-First Iteration (STRONGLY RECOMMENDED)

Write small, reproducible tests before changing behavior: unit tests where practical, and simple bench/HIL tests
that validate motor outputs and encoder readings. Tests can be host-side scripts that exercise the microcontroller
via serial or a simulated input. At minimum, every motor-control change MUST include a manual quick-check procedure
documented in the feature directory.

Rationale: Fast feedback reduces debugging time and builds confidence.

### V. Reproducible Builds & Documentation (REQUIRED)

Use deterministic build instructions (PlatformIO or the Pico CMake flow included in `pico/`). Pin tool versions where
possible and document exact upload steps. Every firmware release MUST include: build command, checksum of the
binary, and a short quickstart for flashing and smoke-testing.

Rationale: Reproducible artifacts simplify debugging and sharing with collaborators.

## Additional Constraints

- Target hardware: Raspberry Pi Pico (RP2040) for `src/robot_firmware/pico/`.
- Porting constraint: public API and filenames MUST match `src/robot_firmware/legacy_stm32/` where those symbols
  are referenced by higher layers. Keep header filenames identical and provide compatibility headers if needed.
- Language & toolchain: C/C++ (same flavor used in `pico/` folder). Prefer the existing CMakeLists.txt in `pico/`.
- Real-time requirements: Motor control loop SHOULD be non-blocking and run at a reasonable rate (e.g., 200-1000 Hz
  depending on hardware). Explain choices in the feature quickstart.

## Development Workflow & Quality Gates (Beginner-friendly)

- Branches: short-lived feature branches named `feat/pico-<what>`.
- Commits: small, self-contained; include a sentence explaining WHY the change exists.
- Reviews: request at least one reviewer for non-trivial changes (watchdog, motor mapping, or safety logic).
- Tests & Checks before merge:
  - Build locally using the `pico/` CMake flow or PlatformIO; builds MUST succeed.
  - Run static checks if configured (compiler warnings treated as actionable items).
  - Validate smoke test: motors stop when command absent; watchdog reset observed after simulated hang.

## Constitution Check (gates used by `.specify/templates/plan-template.md`)

- Real-Time Control Integrity: document expected control loop frequency and prove new code is non-blocking.
- Fail-Safe Actuation Envelope: every actuator change MUST include watchdog coverage, output limits, and a manual
  procedure to stop the robot.
- ROS 2 Contract Fidelity: list any topic/service name changes (these are discouraged). If a message shape changes,
  provide a migration plan and keep compatibility headers where possible.
- Test Discipline: list unit/HIL/bench tests created for the change and that they fail before implementation.
- Reproducible Firmware Delivery: provide exact build and upload commands used for validation.

## Governance

Amendments to this constitution require a documented rationale and one approving review from a maintainer. For
material governance changes (adding/removing principles), increment the MAJOR version. For added principles or
material guidance, increment MINOR. For wording fixes or clarifications, increment PATCH.

- Amendment steps:
  1. Draft change in a branch and update `.specify/memory/constitution.md`.
  2. Run the Constitution Check in the feature plan and update templates if required.
  3. Obtain one reviewer approval and merge.

**Version**: 1.0.0 | **Ratified**: TODO(RATIFICATION_DATE): please confirm | **Last Amended**: 2025-09-23
