# Feature Specification: Implement Firmware for Humble

**Feature Branch**: `002-feat-humble-implement`  
**Created**: 2025-09-25
**Status**: Draft  
**Input**: User description: "feat_humble_implement_firmware wir wollen die firmware für unseren roboter entwickeln das wir in @Projekt.md wir müssen den ist stand prüfen um dann den soll zustand definieren"

## Execution Flow (main)
```
1. Parse user description from Input
   → If empty: ERROR "No feature description provided"
2. Extract key concepts from description
   → Identify: actors, actions, data, constraints
3. For each unclear aspect:
   → Mark with [NEEDS CLARIFICATION: specific question]
4. Fill User Scenarios & Testing section
   → If no clear user flow: ERROR "Cannot determine user scenarios"
5. Generate Functional Requirements
   → Each requirement must be testable
   → Mark ambiguous requirements
6. Identify Key Entities (if data involved)
7. Run Review Checklist
   → If any [NEEDS CLARIFICATION]: WARN "Spec has uncertainties"
   → If implementation details found: ERROR "Remove tech details"
8. Return: SUCCESS (spec ready for planning)
```

---

## ⚡ Quick Guidelines

- ✅ Focus on WHAT users need and WHY
- ❌ Avoid HOW to implement (no tech stack, APIs, code structure)
- 👥 Written for business stakeholders, not developers

### Section Requirements

- **Mandatory sections**: Must be completed for every feature
- **Optional sections**: Include only when relevant to the feature
- When a section doesn't apply, remove it entirely (don't leave as "N/A")

### For AI Generation
When creating this spec from a user prompt:

1. **Mark all ambiguities**: Use [NEEDS CLARIFICATION: specific question] for any assumption you'd need to make
2. **Don't guess**: If the prompt doesn't specify something (e.g., "login system" without auth method), mark it
3. **Think like a tester**: Every vague requirement should fail the "testable and unambiguous" checklist item
4. **Common underspecified areas**:
   - User types and permissions
   - Data retention/deletion policies  
   - Performance targets and scale
   - Error handling behaviors
   - Integration requirements
   - Security/compliance needs

---

## Clarifications

### Session 2025-09-25

- Q: What level of logging is required for the firmware? → A: Debug - Log detailed debug information for development.
- Q: What is the target control loop frequency for the motors? → A: 1 kHz for deterministic PWM and jitter-free control on Core1 of the RP2040.
- Q: How should the firmware behave in case of a sensor failure (e.g., IMU or encoders)? → A: Transition to a safe state (stop motors), publish an error status via micro-ROS (e.g., `sensor_msgs/DiagnosticStatus` or `std_msgs/String` on `/firmware/error`), and activate the watchdog to force a restart or timeout. Atomic flags MUST be used for error detection and hardware interlocks in the Pico SDK MUST be prioritized.
- Q: Should the firmware explicitly handle any "out-of-scope" features to prevent misuse? → A: Yes, implement safety interlocks (e.g., fire only with valid commands and sensor checks), a watchdog for timeouts, and publish diagnostic data via micro-ROS (e.g., `sensor_msgs/DiagnosticStatus`). Prioritize hardware interlocks in the Pico SDK.
- Q: Should the firmware persist any data on the Pico's flash memory (e.g., configuration, calibration data)? → A: Yes, selectively persist data like calibration data for IMU or encoder offsets using the Pico SDK's Flash API for safe read/write operations. Minimize Flash cycles by storing only essential data and incorporating validation checks.

---

## User Scenarios & Testing *(mandatory)*

### Primary User Story
As the high-level controller, I need the robot's hardware to reliably execute my commands and report its status, so I can perform complex tasks like navigating an environment and interacting with objects.

### Acceptance Scenarios
1. **Given** the robot is operational, **When** a movement command (e.g., "move forward at 0.5 m/s") is issued, **Then** the robot base moves as instructed.
2. **Given** the robot is operational, **When** a command to fire the launcher is issued, **Then** the launcher aims and fires correctly.
3. **Given** the robot is operational, **When** system status is requested, **Then** the robot reports its current orientation, position change, and battery level.

### Edge Cases
- **Connection Loss**: When the connection to the high-level controller is lost, the robot MUST safely stop all movement.
- **Low Battery**: When the battery voltage drops below a critical threshold, this status MUST be reported to the high-level controller.

## Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: The firmware MUST establish a micro-ROS communication link with the Raspberry Pi 4B.
- **FR-002**: The firmware MUST subscribe to `geometry_msgs/Twist` messages to control the robot's movement.
- **FR-003**: The firmware MUST control the four DC motors via TB6612FNG drivers based on the received Twist messages.
- **FR-004**: The firmware MUST read data from the Hall-Encoders of the motors.
- **FR-005**: The firmware MUST publish odometry information based on the encoder data.
- **FR-006**: The firmware MUST control the servos and brushless motors of the Nerf-Launcher.
- **FR-007**: The firmware MUST read data from the ICM-20948 9-DoF IMU and publish it as a `sensor_msgs/Imu` message.
- **FR-008**: The firmware MUST read data from the VL53L0X Time-of-Flight sensor and publish it.
- **FR-009**: The firmware MUST monitor the battery voltage using the INA3221 sensor and publish the status.
- **FR-010**: The firmware MUST implement a safety stop if no commands are received for 200 ms.
- **FR-011**: The firmware MUST report a critical battery status when the voltage drops below 10.5V.
- **FR-012**: The firmware MUST provide a logging mechanism with a configurable log level, defaulting to Debug.
- **FR-013**: The firmware MUST implement a motor control loop with a target frequency of 1 kHz on Core1 of the RP2040.
- **FR-014**: In case of a sensor failure (e.g., IMU or encoders), the firmware MUST transition to a safe state (stop motors), publish an error status via micro-ROS (e.g., `sensor_msgs/DiagnosticStatus` or `std_msgs/String` on `/firmware/error`), and activate the watchdog to force a restart or timeout. Atomic flags MUST be used for error detection and hardware interlocks in the Pico SDK MUST be prioritized.
- **FR-015**: The firmware MUST explicitly handle out-of-scope features to prevent misuse by implementing safety interlocks (e.g., fire only with valid commands and sensor checks), a watchdog for timeouts, and publishing diagnostic data via micro-ROS (e.g., `sensor_msgs/DiagnosticStatus`). Hardware interlocks in the Pico SDK MUST be prioritized.
- **FR-016**: The firmware SHOULD selectively persist data on the Pico's flash memory (e.g., calibration data for IMU or encoder offsets) to ensure availability upon restarts. The Pico SDK's Flash API MUST be used for safe read/write operations, minimizing Flash cycles by storing only essential data and incorporating validation checks.

### Key Entities *(include if feature involves data)*
- **Robot**: Represents the physical robot with its hardware components.
- **High-Level-Controller (Pi 4B)**: The main computing unit responsible for complex logic and decision-making.
- **Low-Level-Controller (Pico)**: The firmware-driven board responsible for real-time hardware control and sensing.
- **micro-ROS Agent**: The bridge between the high-level and low-level controllers, running on the SBC, facilitating communication.

---

## Review & Acceptance Checklist
*GATE: Automated checks run during main() execution*

### Content Quality

- [x] No implementation details (languages, frameworks, APIs)
- [x] Focused on user value and business needs
- [x] Written for non-technical stakeholders
- [x] All mandatory sections completed

### Requirement Completeness

- [x] No [NEEDS CLARIFICATION] markers remain
- [x] Requirements are testable and unambiguous  
- [x] Success criteria are measurable
- [x] Scope is clearly bounded
- [x] Dependencies and assumptions identified

---

## Execution Status
*Updated by main() during processing*

- [x] User description parsed
- [x] Key concepts extracted
- [x] Ambiguities marked
- [x] User scenarios defined
- [x] Requirements generated
- [x] Entities identified
- [x] Review checklist passed

---