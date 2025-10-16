# Changelog

All notable changes to the my_steel robot firmware will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- Comprehensive modernization documentation in `MODERNIZATION.md`
- Centralized configuration system in `src/config/FirmwareConfig.h`
- Type-safe namespace-based constants replacing `#define` macros
- Bounds checking for motor array access
- Null pointer validation for agent setters
- Compile-time configuration validation using `static_assert`
- Comprehensive Doxygen documentation for public APIs
- Inline documentation for complex algorithms (kinematics, odometry, PID)
- Integration test documentation and scripts
- Modern C++ data structures with default member initializers

### Changed
- Migrated from `#define` macros to `inline constexpr` constants
- Updated numeric literals with explicit type suffixes (f, u)
- Modernized `DDDOdom_t` to `OdometryState` struct with methods
- Enhanced error handling with clear error messages
- Improved code consistency and naming conventions
- Updated README with modernization guide and migration instructions

### Removed
- HCSR04 ultrasonic sensor support (completely removed)
  - Deleted `src/HCSR04Agent.h` and `src/HCSR04Agent.cpp`
  - Removed `/range_front` and `/range_back` ROS topics
  - Removed all HCSR04 references from DDD agent
- Distance sensor CMake configuration (commented out)

### Fixed
- Potential out-of-bounds array access in motor control
- Missing null pointer checks in agent initialization
- Implicit type conversions that could cause precision loss

## [1.0.0] - 2025-01-11

### Initial Release
- FreeRTOS-based agent architecture
- TB6612 motor driver support for 4 mecanum wheels
- ICM-20948 IMU integration via SPI
- Micro-ROS communication over USB
- Mecanum wheel kinematics and odometry
- PID motor control with encoder feedback
- ROS2 topics:
  - Publishers: `/joint_states`, `/imu/data_raw`, `/odometry/wheels`
  - Subscribers: `/cmd_vel`

---

## Version History

### Modernization (2025-01)
Focus on code quality, type safety, and maintainability while preserving functionality.

### Initial Development (Pre-2025)
Core firmware functionality for mecanum-wheeled robot with ROS2 integration.
