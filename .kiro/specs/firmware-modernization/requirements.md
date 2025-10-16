# Requirements Document

## Introduction

This specification defines the requirements for modernizing the firmware codebase of the my_steel omnidirectional ROS2 robot. The firmware runs on a Raspberry Pi Pico using FreeRTOS and micro-ROS to interface with motors, sensors (IMU), and the ROS2 ecosystem. The modernization effort aims to improve code quality, maintainability, and performance by adopting modern C++ practices while removing unused functionality.

The refactoring will maintain all existing core functionality (motor control, IMU integration, odometry, micro-ROS communication) while improving code structure and removing deprecated sensors (HCSR04 ultrasonic, VL6180X ToF).

## Requirements

### Requirement 1: Modern C++ Language Features

**User Story:** As a firmware developer, I want the codebase to use modern C++ features, so that the code is more maintainable, type-safe, and follows current best practices.

#### Acceptance Criteria

1. WHEN refactoring type definitions THEN the code SHALL use `ifdef and ifndef`` for compile-time constants instead of `#define` macros where appropriate
2. WHEN defining enumerations THEN the code SHALL use scoped enums (`enum class`) instead of unscoped enums
3. WHEN managing dynamic resources THEN the code SHALL use smart pointers (`std::unique_ptr`, `std::shared_ptr`) instead of raw pointers where ownership semantics benefit from RAII
4. WHEN declaring variables THEN the code SHALL use `auto` keyword for type deduction where it improves readability
5. WHEN defining constants THEN the code SHALL use `inline constexpr` for header-only constants to avoid ODR violations

### Requirement 2: Remove Unused Sensor Code

**User Story:** As a firmware maintainer, I want unused sensor code removed from the codebase, so that the firmware is simpler and easier to understand.

#### Acceptance Criteria

1. WHEN removing HCSR04Agent code THEN all references SHALL be initially commented out with a TODO marker stating "TODO: delete after successful micro-ROS-Agent connection-Test"
2. WHEN removing VL6180X code THEN all references SHALL be initially commented out with the same TODO marker
3. WHEN commenting out sensor code THEN the main.cpp initialization SHALL reflect these changes
4. WHEN commenting out sensor code THEN the DDD agent SHALL have sensor references commented out
5. IF micro-ROS connection tests pass successfully THEN the commented code SHALL be completely removed in a follow-up task

### Requirement 3: Code Documentation Standards

**User Story:** As a developer joining the project, I want comprehensive and clear documentation, so that I can understand the codebase quickly and contribute effectively.

#### Acceptance Criteria

1. WHEN refactoring any class or function THEN existing documentation SHALL be preserved at minimum
2. WHEN adding modern C++ features THEN new documentation SHALL explain the rationale for the approach
3. WHEN documenting classes THEN Doxygen-style comments SHALL be used consistently
4. WHEN documenting complex algorithms THEN inline comments SHALL explain the "why" not just the "what"
5. WHEN removing code THEN commit messages SHALL clearly document what was removed and why

### Requirement 4: Maintain Core Functionality

**User Story:** As a robot operator, I want all existing motor control and navigation features to work exactly as before, so that the refactoring doesn't break my robot's operation.

#### Acceptance Criteria

1. WHEN refactoring is complete THEN the robot SHALL respond to `/cmd_vel` commands identically to the original implementation
2. WHEN refactoring is complete THEN motor control (TB6612MotorsAgent) SHALL function without regression
3. WHEN refactoring is complete THEN IMU data publishing SHALL continue at the same rate and accuracy
4. WHEN refactoring is complete THEN odometry calculations SHALL produce identical results
5. WHEN refactoring is complete THEN micro-ROS bridge communication SHALL maintain the same reliability

### Requirement 5: Preserve Build System Compatibility

**User Story:** As a developer, I want the build system to continue working without changes, so that I can build and flash the firmware using existing workflows.

#### Acceptance Criteria

1. WHEN code is refactored THEN the CMakeLists.txt SHALL require minimal or no changes
2. WHEN building the firmware THEN the `make build` command SHALL succeed without errors
3. WHEN building the firmware THEN the `make build_release` command SHALL succeed without errors
4. WHEN flashing the firmware THEN the `make flash` command SHALL work as before
5. IF C++ standard version needs updating THEN it SHALL be documented in the design phase

### Requirement 6: Type Safety and Error Handling

**User Story:** As a firmware developer, I want improved type safety and error handling, so that bugs are caught at compile-time rather than runtime.

#### Acceptance Criteria

1. WHEN using numeric constants THEN they SHALL have explicit types (e.g., `0.065f` for float, `100u` for unsigned)
2. WHEN performing type conversions THEN explicit casts SHALL be used instead of implicit conversions
3. WHEN handling pointers THEN null checks SHALL be performed before dereferencing
4. WHEN using arrays THEN bounds SHALL be checked or replaced with safer alternatives like `std::array`
5. WHEN defining magic numbers THEN they SHALL be replaced with named constants

### Requirement 7: Agent Architecture Consistency

**User Story:** As a firmware architect, I want the Agent-based architecture to remain consistent, so that the system's modularity and task structure are preserved.

#### Acceptance Criteria

1. WHEN refactoring agents THEN the base `Agent` class interface SHALL remain unchanged
2. WHEN refactoring agents THEN FreeRTOS task creation and management SHALL continue to work
3. WHEN refactoring agents THEN the `uRosEntities` interface SHALL remain compatible
4. WHEN refactoring agents THEN task priorities and stack sizes SHALL be preserved unless explicitly optimized
5. WHEN removing sensor agents THEN the remaining agents SHALL not be affected

### Requirement 8: Incremental Refactoring Approach

**User Story:** As a project manager, I want the refactoring to be done incrementally, so that we can test and validate changes in manageable chunks.

#### Acceptance Criteria

1. WHEN planning the refactoring THEN it SHALL be broken into discrete, testable steps
2. WHEN completing each step THEN the firmware SHALL be buildable and testable
3. WHEN removing commented code THEN it SHALL happen only after connection tests pass
4. WHEN introducing modern C++ features THEN they SHALL be added file-by-file or class-by-class
5. WHEN refactoring is complete THEN a final integration test SHALL verify all functionality

### Requirement 9: Performance Preservation

**User Story:** As a robot operator, I want the refactored firmware to maintain or improve performance, so that real-time control remains responsive.

#### Acceptance Criteria

1. WHEN refactoring is complete THEN motor control loop timing SHALL not degrade
2. WHEN refactoring is complete THEN IMU sampling rate SHALL remain at 100Hz or better
3. WHEN refactoring is complete THEN micro-ROS message publishing SHALL not introduce additional latency
4. WHEN using modern C++ features THEN they SHALL not introduce runtime overhead (e.g., prefer `constexpr` over runtime computation)
5. WHEN measuring performance THEN stack usage SHALL not increase significantly (monitored via `getStakHighWater()`)

### Requirement 10: Code Consistency and Style

**User Story:** As a code reviewer, I want consistent coding style throughout the refactored code, so that the codebase is uniform and professional.

#### Acceptance Criteria

1. WHEN naming constants THEN they SHALL use `kPascalCase` or `UPPER_SNAKE_CASE` consistently
2. WHEN naming variables THEN they SHALL follow existing conventions (e.g., `xVariableName` for FreeRTOS-related)
3. WHEN formatting code THEN it SHALL follow the project's existing clang-format rules
4. WHEN writing comments THEN they SHALL use consistent style (Doxygen for APIs, inline for implementation)
5. WHEN refactoring is complete THEN the code SHALL pass any existing linting or formatting checks
