# Product Overview

**my_steel Robot Firmware** - Embedded firmware for a Raspberry Pi Pico-based robot platform.

## Purpose

Real-time control system for a mecanum-wheeled robot with:
- Motor control with PID feedback and encoder tracking
- Sensor integration (IMU, ultrasonic, time-of-flight)
- ROS2 communication via micro-ROS over USB
- Odometry calculation and velocity control

## Target Hardware

- **MCU**: Raspberry Pi Pico (RP2040)
- **Motors**: 4x mecanum wheels with TB6612 motor drivers
- **Sensors**: ICM-20948 IMU (SPI), VL6180X ToF (I2C), HC-SR04 ultrasonic
- **Communication**: USB (micro-ROS), UART0 (debug)

## Key Features

- Agent-based modular architecture on FreeRTOS
- Real-time motor PID control at 50Hz
- Odometry publishing and cmd_vel subscription
- Hardware abstraction layer for testability
