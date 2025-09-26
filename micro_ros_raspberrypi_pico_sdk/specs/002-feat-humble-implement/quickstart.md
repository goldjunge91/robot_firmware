# Quickstart Guide

This guide describes how to build, flash, and test the firmware.

## Prerequisites

* A configured development environment for Raspberry Pi Pico development.
* The GNU Arm Embedded Toolchain and Raspberry Pi Pico SDK installed and configured.
* A Raspberry Pi Pico board.

## Build

1. Navigate to the `build` directory:

    ```bash
    cd build
    ```

2. Run CMake and Make to build the firmware:

    ```bash
    cmake ..
    make
    ```

## Flash

1. Connect the Raspberry Pi Pico to your computer while holding down the BOOTSEL button.
2. The Pico will mount as a USB mass storage device.
3. Copy the `firmware.uf2` file from the `build` directory to the Pico.

## Test

1. Connect the Pico to the Raspberry Pi 4B via UART.
2. Run the micro-ROS agent on the Raspberry Pi 4B.
3. Verify that the Pico connects to the agent.
4. Use `ros2 topic pub` to send a `geometry_msgs/Twist` message to the `/cmd_vel` topic and verify that the motors move.
5. Use `ros2 topic echo` to verify that the Pico is publishing data on the `/imu/data_raw`, `/odom`, `/tof_sensor`, and `/battery_state` topics.

### Test Pan/Tilt

```bash
ros2 topic pub /launcher_control sensor_msgs/msg/JointState "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'launcher'}, name: ['pan', 'tilt'], position: [0.5, 0.2], velocity: [], effort: []}"
```

* **Expected**: Servo PWM values change, firmware logs a "Target updated" message, and the `/joint_states` topic reflects the new positions.

### Test Fire

```bash
ros2 topic pub /launcher/fire std_msgs/msg/Bool "{data: true}"
```

* **Expected**: Brief ESC impulses, a firmware event on `/launcher/fire_event`, and safety checks are performed before execution.

### Sensor Output Examples

* **/imu/data_raw**: `ros2 topic echo /imu/data_raw`
  * Expected output similar to: `angular_velocity: {x: 0.01, y: -0.02, z: 0.0}`, `linear_acceleration: {x: 0.0, y: 0.0, z: 9.8}`
* **/battery/status**: `ros2 topic echo /battery/status`
  * Expected output similar to: `voltage: 11.1`, `current: 1.2`, `percentage: 0.78`
