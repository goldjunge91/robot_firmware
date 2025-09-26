# Research for Firmware Implementation

## Research Summary

This document outlines the research done to inform the implementation of the robot firmware on the Raspberry Pi Pico, focusing on micro-ROS integration, real-time motor control, sensor data acquisition, and robust safety features.

## Technical Decisions

### 1. Micro-ROS Communication

*   **Decision**: Utilize `rmw_microxrcedds` middleware over UART for communication between the Raspberry Pi Pico and the Raspberry Pi 4B. Standard ROS 2 messages (`geometry_msgs/Twist`, `sensor_msgs/JointState`, `std_msgs/Bool`, `sensor_msgs/Imu`, `nav_msgs/Odometry`, `sensor_msgs/Range`, `sensor_msgs/BatteryState`, `sensor_msgs/DiagnosticStatus`) will be used to avoid custom implementations and ensure maximum compatibility with existing ROS 2 tools.
*   **Rationale**: This approach leverages the established micro-ROS ecosystem, providing a robust and standardized communication channel. Using standard message types simplifies integration with higher-level ROS 2 nodes and tools like `rqt_monitor`.
*   **Alternatives Considered**: Custom serial protocols were rejected due to increased development effort, lack of ROS 2 ecosystem integration, and reduced maintainability.

### 2. Motor Control

*   **Decision**: Implement a PID controller for each of the four DC motors (Front-Left, Front-Right, Rear-Left, Rear-Right) to ensure accurate velocity control. The motor control loop will run at a target frequency of 1 kHz on Core1 of the RP2040.
*   **Rationale**: A 1 kHz control loop provides deterministic PWM regulation and jitter-free control, crucial for precise robot movement. Offloading this real-time critical task to Core1 prevents interference from micro-ROS communication and other tasks running on Core0.
*   **Implementation Details**: Control will involve TB6612FNG motor drivers and feedback from Hall-Encoders. The PID parameters will be tuned for optimal performance.
*   **Alternatives Considered**: Simpler control methods (e.g., proportional control) were deemed insufficient for the required precision. Running the control loop on a single core was rejected due to potential interference with micro-ROS.

### 3. Concurrency and Multi-Core Utilization

*   **Decision**: Leverage the dual-core architecture of the RP2040. Core0 will handle micro-ROS communication and other less time-critical tasks, while Core1 will be dedicated to the 1 kHz motor control loop and encoder reading.
*   **Rationale**: This separation of concerns ensures that real-time motor control is not impacted by communication overhead or other processing. It maximizes the utilization of the RP2040's capabilities.
*   **Implementation Details**: `critical_section_t` or atomics will be used for thread-safe access to shared data structures (e.g., motor commands, sensor data) between the two cores to ensure data consistency.
*   **Alternatives Considered**: A single-core approach was rejected due to the risk of jitter and non-deterministic behavior in the motor control loop.

### 4. Sensor Integration

*   **Decision**: Integrate the ICM-20948 9-DoF IMU, VL53L0X Time-of-Flight sensor, and INA3221 battery monitor. Data from these sensors will be published via micro-ROS using standard `sensor_msgs/Imu`, `sensor_msgs/Range`, and `sensor_msgs/BatteryState` messages, respectively.
*   **Rationale**: These sensors provide essential data for navigation, obstacle detection, and power management. Using standard ROS 2 messages facilitates easy integration with higher-level ROS 2 systems.
*   **Implementation Details**: Sensor data will be read and processed on the Pico, with appropriate publishing rates (e.g., 100Hz for IMU, 10Hz for ToF and battery status) to balance data freshness and communication bandwidth.

### 5. Build System

*   **Decision**: Continue using the existing CMake build system for the Raspberry Pi Pico.
*   **Rationale**: The project is already configured with a functional CMake setup that supports the Pico SDK. This ensures consistency, reproducibility, and leverages familiar development tools.
*   **Alternatives Considered**: PlatformIO was considered but rejected to avoid introducing an additional build system and potential configuration complexities.

### 6. Safety Features and Error Handling

*   **Decision**: Implement comprehensive safety features including a 200ms safety stop timeout for motor commands, critical battery status reporting (below 10.5V), and a safe stop on connection loss. In case of sensor failure or out-of-scope feature misuse, the firmware will transition to a safe state, publish error diagnostics (`sensor_msgs/DiagnosticStatus` or `std_msgs/String` on `/firmware/error`), and activate the hardware watchdog.
*   **Rationale**: These measures are critical for preventing damage to the robot and its environment, ensuring reliable operation, and providing clear diagnostic feedback to the high-level controller. Prioritizing hardware interlocks and atomic flags for error detection enhances robustness.
*   **Implementation Details**: The hardware watchdog will be configured to force a restart or timeout, minimizing the impact of software hangs. Safety interlocks will prevent misuse of features like the Nerf launcher.

### 7. Data Persistence

*   **Decision**: Selectively persist essential data (e.g., calibration data for IMU or encoder offsets) on the Pico's flash memory.
*   **Rationale**: Persisting calibration data ensures that the robot maintains accuracy and performance across restarts without requiring re-calibration. This leverages the Pico SDK's Flash API for safe and reliable storage.
*   **Implementation Details**: Flash cycles will be minimized by storing only essential data. Validation checks will be incorporated to ensure data integrity upon reading from flash.

### 8. Logging

*   **Decision**: Implement a logging mechanism with a configurable log level, defaulting to Debug.
*   **Rationale**: Detailed debug logging is essential for development, troubleshooting, and understanding the firmware's behavior in various scenarios. A configurable log level allows for optimization in production environments.
*   **Implementation Details**: The logging system will output messages via the micro-ROS agent, making them accessible through ROS 2 logging tools.