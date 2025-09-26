# micro-ROS Contracts

This document describes the micro-ROS topics and messages used for communication between the Raspberry Pi Pico and the Raspberry Pi 4B.

## Subscribers (Pico)

### 1. `/cmd_vel`

*   **Message Type**: `geometry_msgs/Twist`
*   **Description**: Receives velocity commands to control the robot's movement.

### 2. `/launcher_control`

*   **Message Type**: `sensor_msgs/JointState`
*   **Description**: Receives commands to control the pan and tilt of the Nerf launcher. The `name` field should be `["pan", "tilt"]` and the `position` field will contain the target angles in radians.

### 3. `/launcher/fire`

*   **Message Type**: `std_msgs/Bool`
*   **Description**: Receives a boolean value to fire the launcher. `data: true` will trigger a single shot.

*Alternatively, a service can be used:*

*   **Service**: `/launcher/fire_srv`
*   **Service Type**: `std_srvs/Trigger`
*   **Description**: A service to trigger a single shot. Provides feedback on whether the shot was fired successfully.

## Publishers (Pico)

### 1. `/imu/data_raw`

*   **Message Type**: `sensor_msgs/Imu`
*   **Description**: Publishes raw data from the ICM-20948 IMU.

### 2. `/odom`

*   **Message Type**: `nav_msgs/Odometry`
*   **Description**: Publishes odometry data calculated from the motor encoders.

### 3. `/tof_sensor`

*   **Message Type**: `sensor_msgs/Range`
*   **Description**: Publishes distance measurements from the VL53L0X ToF sensor.

### 4. `/battery_state`

*   **Message Type**: `sensor_msgs/BatteryState`
*   **Description**: Publishes the battery voltage and other status information from the INA3221 sensor.
