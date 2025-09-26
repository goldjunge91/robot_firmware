# Data Model

This document describes the key data structures for the firmware.

## Core Data Structures

### 1. LauncherState

```c
typedef struct {
    float pan_position;         // Current pan position in radians
    float tilt_position;        // Current tilt position in radians
    float pan_target;           // Target pan position in radians
    float tilt_target;          // Target tilt position in radians
    bool is_firing;             // True if the launcher is currently firing
    uint32_t last_update_ms;    // Timestamp of the last update
} LauncherState;
```

### 2. MotorControllerState

```c
typedef struct {
    float pwm_duty[4];          // PWM duty cycle for each motor
    bool dir_pin_state[4];      // Direction pin state for each motor
    bool enabled;               // True if the motor controller is enabled
    uint32_t error_flags;       // Error flags for the motor controller
} MotorControllerState;
```

### 3. EncoderState

```c
typedef struct {
    int32_t ticks;              // Encoder tick count
    float velocity;             // Wheel velocity in rad/s
    uint32_t last_timestamp_ms; // Timestamp of the last encoder update
} EncoderState;
```

### 4. ImuState

```c
typedef struct {
    float accel[3];             // Accelerometer data (x, y, z)
    float gyro[3];              // Gyroscope data (x, y, z)
    float temperature;          // Temperature from the IMU
    uint32_t seq;               // Sequence number
} ImuState;
```

### 5. BatteryStatus

```c
typedef struct {
    int32_t voltage_mv;         // Battery voltage in millivolts
    int32_t current_ma;         // Battery current in milliamps
    uint8_t soc_percent;        // State of charge in percent
    uint32_t last_sample_ms;    // Timestamp of the last sample
} BatteryStatus;
```