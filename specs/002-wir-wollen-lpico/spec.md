# Feature Specification: Rebuild lpico_pio based on micro_ros_raspberrypi_pico_sdk

**Feature Branch**: `002-wir-wollen-lpico`  
**Created**: 2025-09-23
**Status**: Draft  
**Input**: User description: "wir wollen lpico_pio/ in /example_repo/micro_ros_raspberrypi_pico_sdk/ neu aufbauen, da das dortige build script ordnungsgemäß funktioniert, unser code, ist eine überarbeitung von example_repo\rosbot_ros2_firmware und sollte ähnlich wie dieses aufgebaut sein."

## Key Architectural Documents
- **[PINMAP.md](../../PINMAP.md)**: Single source of truth for all MCU pin assignments.
- **[architecture_and_packages.md](../../architecture_and_packages.md)**: Describes the high-level ROS2 architecture and package responsibilities.

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

## User Scenarios & Testing *(mandatory)*

### Primary User Story
As a developer, I want to have a new `lpico_pio` project structure that uses the build system from `micro_ros_raspberrypi_pico_sdk` so that I can build the firmware reliably. The new project should incorporate the logic from our existing `rosbot_ros2_firmware` revision and be compliant with the project's architecture and pin mappings.

### Acceptance Scenarios
1. **Given** a clean checkout of the new `.old/lpico_pio` project, **When** I run the build script, **Then** the firmware image is created successfully without errors.
2. **Given** the new `lpico_pio` project, **When** I compare its structure to `example_repo/rosbot_ros2_firmware`, **Then** the structure is similar in terms of source code organization.
3. **Given** the flashed firmware, **When** I interact with it using ROS2 tools, **Then** it correctly publishes and subscribes to the topics defined in `architecture_and_packages.md`.

### Edge Cases
- What happens when the dependencies for `micro_ros_raspberrypi_pico_sdk` are not installed? The build should fail with a clear error message about missing dependencies.
- What happens if the `rosbot_ros2_firmware` code is not compatible with the new build system? The build will likely fail, and the developer will need to adapt the code.

## Requirements *(mandatory)*

### Functional Requirements
- **FR-001**: The system MUST provide a new project structure for `lpico_pio`.
- **FR-002**: The new `lpico_pio` project MUST use the build system from `example_repo/micro_ros_raspberrypi_pico_sdk`.
- **FR-003**: The new `lpico_pio` project MUST be structured similarly to `example_repo/rosbot_ros2_firmware`.
- **FR-004**: The system MUST allow building the `lpico_pio` firmware using the new project structure.
- **FR-005**: The build process MUST produce a firmware image.
- **FR-006**: The firmware MUST implement the hardware control logic according to the pin mappings in `PINMAP.md`.
- **FR-007**: The firmware MUST expose the ROS2 interfaces (topics and services) as defined in `architecture_and_packages.md`.
- **FR-008**: The project structure MUST incorporate the existing code from the current `lpico_pio` which is a revision of `rosbot_ros2_firmware`, ensuring a clear migration path for existing functionalities.

### Key Entities *(include if feature involves data)*
- **`lpico_pio` project**: The firmware project to be rebuilt.
- **`micro_ros_raspberrypi_pico_sdk`**: The reference project for the build system.
- **`rosbot_ros2_firmware`**: The reference project for the code structure.
- **`PINMAP.md`**: The authoritative document for pin assignments.
- **`architecture_and_packages.md`**: The authoritative document for the ROS2 architecture.

---

## Clarifications

### Session 2025-09-23
- Q: Wie soll die Projektstruktur und Code-Migration in `spec.md` konsolidiert werden? → A: FR-003 und FR-008 getrennt halten, aber FR-008 umformulieren, um Implementierungsdetails zu entfernen.
- Q: Die Aufgabe T021 in `tasks.md` ("Implementierung der Motor-Treiber- und PWM-Schicht") ist generisch. Wie soll diese Aufgabe verfeinert werden, um die detaillierte Migration widerzuspiegeln, die im ursprünglichen `spec.md` FR-008 (jetzt aus `spec.md` entfernt, aber Grundlage der Klärung) beschrieben wurde? → A: T021 verfeinern, um die Migration und Anpassung von `motor_driver.cpp`, `motors.cpp`, und `pwm_control.cpp` aus `lpico_pio` und `rosbot_ros2_firmware` explizit zu erwähnen.
- Q: Die `plan.md` empfiehlt derzeit "Unity" für Tests, aber die Forschungsaufgabe (T001) zur Untersuchung und Empfehlung eines geeigneten Unit-Test-Frameworks steht noch aus. Sollte der Plan explizit festhalten, dass die Entscheidung über das Test-Framework vom Ergebnis der Forschungsaufgabe T001 abhängt, oder sollen wir Unity vorerst annehmen? → A: In `plan.md` explizit festhalten, dass die Entscheidung über das Test-Framework vom Ergebnis der Forschungsaufgabe T001 abhängt.

## Review & Acceptance Checklist
*GATE: Automated checks run during main() execution*

### Content Quality
- [ ] No implementation details (languages, frameworks, APIs)
- [ ] Focused on user value and business needs
- [ ] Written for non-technical stakeholders
- [ ] All mandatory sections completed

### Requirement Completeness
- [ ] No [NEEDS CLARIFICATION] markers remain
- [ ] Requirements are testable and unambiguous  
- [ ] Success criteria are measurable
- [ ] Scope is clearly bounded
- [ ] Dependencies and assumptions identified

---

## Execution Status
*Updated by main() during processing*

- [ ] User description parsed
- [ ] Key concepts extracted
- [ ] Ambiguities marked
- [ ] User scenarios defined
- [ ] Requirements generated
- [ ] Entities identified
- [ ] Review checklist passed

---
