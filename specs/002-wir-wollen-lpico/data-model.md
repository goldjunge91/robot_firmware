# Data Model for lpico_pio Firmware

This document defines the data structures used in the `lpico_pio` firmware, based on the hardware defined in `PINMAP.md`.

## Robot State

- **`robot_state_t`**: A struct that holds the current state of the robot.
  - `float x_pos`: The x-coordinate of the robot's position.
  - `float y_pos`: The y-coordinate of the robot's position.
  - `float theta`: The orientation of the robot.
  - `float linear_velocity`: The linear velocity of the robot.
  - `float angular_velocity`: The angular velocity of the robot.

## Sensors

- **`imu_data_t`**: A struct to hold data from the ICM-20948 IMU.
  - `float accel_x`, `accel_y`, `accel_z`: Acceleration data.
  - `float gyro_x`, `gyro_y`, `gyro_z`: Gyroscope data.
  - `float mag_x`, `mag_y`, `mag_z`: Magnetometer data.

- **`tof_data_t`**: A struct to hold data from the VL53L0X ToF sensor.
  - `uint16_t distance`: The measured distance in millimeters.

- **`encoder_data_t`**: A struct to hold data from the four motor encoders.
  - `int32_t motor_fl_encoder`: The tick count for the front-left motor encoder.
  - `int32_t motor_fr_encoder`: The tick count for the front-right motor encoder.
  - `int32_t motor_rl_encoder`: The tick count for the rear-left motor encoder.
  - `int32_t motor_rr_encoder`: The tick count for the rear-right motor encoder.

## Actuators

- **`motor_speeds_t`**: A struct to hold the target speeds for the four motors.
  - `int16_t motor_fl_speed`: The target speed for the front-left motor.
  - `int16_t motor_fr_speed`: The target speed for the front-right motor.
  - `int16_t motor_rl_speed`: The target speed for the rear-left motor.
  - `int16_t motor_rr_speed`: The target speed for the rear-right motor.

- **`esc_speeds_t`**: A struct to hold the target speeds for the two ESCs of the Nerf launcher.
  - `int16_t esc1_speed`: The target speed for ESC 1.
  - `int16_t esc2_speed`: The target speed for ESC 2.

- **`servo_angles_t`**: A struct to hold the target angle for the gear servo.
  - `uint16_t gear_angle`: The target angle for the gear servo.
