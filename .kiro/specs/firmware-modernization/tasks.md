# Implementation Plan

- [x] 1. Create configuration constants header with modern C++ style
  - Create `src/config/FirmwareConfig.h` with namespace-based constants
  - Define pin configuration constants using `inline ifdef and ifndef`
  - Define robot parameters (PID, wheel dimensions) using `inline constexpr`
  - Define debug configuration constants
  - Add Doxygen documentation for each constant group
  - _Requirements: 1.1, 1.5, 6.1, 6.5_

- [x] 2. Refactor main.cpp to use configuration constants
  - Replace `#define` macros with config namespace constants
  - Update pin assignments to use `config::pins::*` constants
  - Update PID parameters to use `config::pid::*` constants
  - Update task priorities to use `config::robot::*` constants
  - Update debug settings to use `config::debug::*` constants
  - Verify build succeeds without warnings
  - _Requirements: 1.1, 1.5, 5.2, 5.3, 10.1_

- [ ] 3. Comment out HCSR04 sensor code with TODO markers
  - Comment out `#include "HCSR04Agent.h"` in main.cpp with TODO marker
  - Comment out HCSR04Agent instantiation and initialization in mainTask()
  - Comment out HCSR04Agent start() call
  - Add consistent TODO comment: "TODO: delete after successful micro-ROS-Agent connection-Test"
  - Verify firmware builds successfully
  - _Requirements: 2.1, 2.3, 8.2  _

- [ ] 4. Comment out VL6180X sensor code with TODO markers
  - Comment out `#include "application/vl6180xAgent.hpp"` in main.cpp with TODO marker
  - Comment out VL6180X configuration and instantiation in mainTask()
  - Comment out VL6180X start() call
  - Add consistent TODO comment: "TODO: delete after successful micro-ROS-Agent connection-Test"
  - Verify firmware builds successfully
  - _Requirements: 2.2, 2.3, 8.2_

- [ ] 5. Update DDD agent to handle missing sensor references
  - Comment out `setHCSR04Agent()` method declaration in DDD.h
  - Comment out `setVl6180xAgent()` method declaration in DDD.h
  - Comment out sensor pointer members in DDD.h
  - Comment out sensor-related code in DDD.cpp (createEntities, destroyEntities, getCount, getHandles)
  - Add TODO markers to all commented sections
  - Verify firmware builds successfully
  - _Requirements: 2.4, 8.2_

- [ ] 6. Add type safety improvements to numeric constants
  - Update DDD.h constants (WHEEL_RADIUS, WHEELS_SEP, etc.) with explicit float/double types
  - Add suffix notation (f for float, u for unsigned) to all numeric literals
  - Replace magic numbers with named constants
  - Document units in comments (meters, radians, etc.)
  - _Requirements: 6.1, 6.5, 10.1_

- [ ] 7. Add bounds checking to motor array access
  - Add bounds check in TB6612MotorsAgent::setSpeedRadPS()
  - Add bounds check in TB6612MotorsAgent::getMotor()
  - Add bounds check in TB6612MotorsAgent::configPID()
  - Add error logging for out-of-bounds access
  - Use `config::kNumMotors` constant for bounds checking
  - _Requirements: 6.4, 7.4_

- [ ] 8. Add null pointer validation to agent setters
  - Add null check in DDD::setMotorsAgent()
  - Add null check in DDD::setImuAgent()
  - Add error logging for null pointer attempts
  - Document preconditions in method comments
  - _Requirements: 6.3, 7.3_

- [ ] 9. Improve documentation for public APIs
  - Add Doxygen comments to Agent base class methods
  - Add Doxygen comments to BaseMotorsAgent interface
  - Add Doxygen comments to TB6612MotorsAgent public methods
  - Add Doxygen comments to DDD public methods
  - Include parameter descriptions, return values, and preconditions
  - _Requirements: 3.1, 3.2, 3.3, 10.4_

- [ ] 10. Improve inline documentation for complex algorithms
  - Add explanatory comments to mecanum kinematics in DDD::handleSubscriptionMsg()
  - Add comments to odometry calculation in DDD::updateOdom()
  - Add comments to PID algorithm in TB6612MotorPID::doPID()
  - Explain "why" not just "what" in comments
  - _Requirements: 3.4, 10.4_

- [ ] 11. Build and flash firmware for initial testing
  - Build debug firmware: `make build`
  - Build release firmware: `make build_release`
  - Flash debug firmware to Pico: `make flash`
  - Monitor serial output for boot sequence
  - Verify all agents start successfully (BlinkAgent, MotorsAgent, ImuAgent, DDD, uRosBridge)
  - Check for any error messages or warnings
  - _Requirements: 4.2, 4.3, 5.2, 5.3, 5.4, 8.2_

- [ ] 12. Test micro-ROS connection and topic communication
  - Start micro-ROS agent on host PC
  - Verify Pico connects to agent successfully
  - List ROS topics: `ros2 topic list`
  - Verify expected topics exist: `/joint_states`, `/imu/data_raw`, `/odom`, `/cmd_vel`
  - Echo IMU data: `ros2 topic echo /imu/data_raw --once`
  - Echo odometry: `ros2 topic echo /odom --once`
  - Verify data is being published correctly
  - _Requirements: 4.1, 4.4, 4.5, 8.3_

- [ ] 13. Test motor control functionality
  - Publish forward velocity command: `ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once`
  - Verify motors respond and robot moves forward
  - Test backward movement (negative x)
  - Test strafe left (positive y)
  - Test strafe right (negative y)
  - Test rotation (angular z)
  - Monitor `/joint_states` for encoder feedback
  - Verify PID control maintains target speeds
  - _Requirements: 4.1, 4.2, 9.1, 9.2_

- [ ] 14. Verify IMU data publishing
  - Monitor IMU topic: `ros2 topic hz /imu/data_raw`
  - Verify publishing rate is ~100Hz
  - Check data quality (reasonable acceleration and gyro values)
  - Verify frame_id is set correctly ("imu_link")
  - Test with robot stationary and moving
  - _Requirements: 4.3, 9.2_

- [ ] 15. Verify odometry calculations
  - Monitor odometry: `ros2 topic echo /odom`
  - Command robot to move forward 1 meter
  - Verify odometry position updates correctly
  - Test rotation and verify angle updates
  - Test strafe movement (mecanum-specific)
  - Compare odometry with expected values
  - _Requirements: 4.4, 9.1_

- [ ] 16. Completely remove HCSR04 sensor code
  - Delete `src/HCSR04Agent.h`
  - Delete `src/HCSR04Agent.cpp`
  - Remove all commented HCSR04 references from main.cpp
  - Remove all commented HCSR04 references from DDD.h
  - Remove all commented HCSR04 references from DDD.cpp
  - Update CMakeLists.txt to remove HCSR04 source files
  - Verify firmware builds successfully
  - _Requirements: 2.1, 2.5, 8.4_

- [ ] 17. Completely remove VL6180X sensor code
  - Delete `src/application/vl6180xAgent.hpp`
  - Delete `src/application/vl6180xAgent.cpp`
  - Delete `src/hal/hardware/vl6180x/` directory
  - Remove all commented VL6180X references from main.cpp
  - Remove all commented VL6180X references from DDD.h
  - Remove all commented VL6180X references from DDD.cpp
  - Update CMakeLists.txt to remove VL6180X source files
  - Verify firmware builds successfully
  - _Requirements: 2.2, 2.5, 8.4_

- [ ] 18. Modernize odometry data structures
  - Replace `DDDOdom_t` typedef with modern `OdometryState` struct
  - Add default member initializers to struct
  - Add constructor with default parameters
  - Add reset() method to struct
  - Update all usage sites in DDD.cpp
  - Verify odometry calculations remain identical
  - _Requirements: 1.4, 4.4, 10.2_

- [ ] 19. Add compile-time configuration validation
  - Add `static_assert` for NUM_MOTORS == 4
  - Add `static_assert` for positive wheel radius
  - Add `static_assert` for positive wheel separation
  - Add `static_assert` for valid task priorities
  - Document configuration constraints
  - _Requirements: 6.2, 7.4_

- [ ] 20. Perform final integration testing
  - Build both debug and release firmware
  - Flash release firmware to Pico
  - Run 30-minute stress test with continuous operation
  - Monitor stack high water marks for all agents
  - Test all movement patterns (forward, backward, strafe, rotate, combined)
  - Verify no micro-ROS disconnections
  - Verify no memory leaks or task failures
  - Check for any error messages in logs
  - Compare performance metrics with baseline (if available)
  - _Requirements: 4.1, 4.2, 4.3, 4.4, 4.5, 8.5, 9.1, 9.2, 9.3, 9.4, 9.5_

- [ ] 21. Update firmware documentation
  - Update README or firmware documentation with modernization changes
  - Document new configuration constant structure
  - Document removed sensor support
  - Update build instructions if needed
  - Add notes about C++14/17 requirements
  - _Requirements: 3.1, 3.5, 10.5_
