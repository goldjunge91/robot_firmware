# IMU Data Publishing Verification Guide

**Task 14: Verify IMU data publishing**  
**Requirements: 4.3, 9.2**

## Overview

This guide provides step-by-step instructions to verify that the IMU (Inertial Measurement Unit) is publishing data correctly via micro-ROS.

## Prerequisites

- ✅ Firmware flashed to Pico (from task 11)
- ✅ micro-ROS agent running on Raspberry Pi
- ✅ Pico connected and communicating (from task 12)
- ✅ ROS2 environment sourced

## Verification Checklist

### 1. Monitor IMU Topic Rate

**Command:**
```bash
ros2 topic hz /imu/data_raw
```

**Expected Output:**
```
average rate: 100.xxx
    min: 0.009s max: 0.011s std dev: 0.00050s window: 100
```

**Success Criteria:**
- ✅ Publishing rate should be approximately **100Hz** (±20Hz tolerance: 80-120Hz)
- ✅ Rate should be stable (low standard deviation)

**Troubleshooting:**
- If rate is too low (<80Hz): Check CPU load, increase task priority
- If rate is unstable: Check for timing issues in IMU task loop
- If no data: Verify IMU sensor initialization in serial logs

---

### 2. Verify Publishing Rate (~100Hz)

The IMU agent is configured with `publish_period_ms_ = 10`, which targets 100Hz.

**Automated Test:**
```bash
./verify_imu_publishing.sh
```

This script will:
- Check topic existence
- Verify topic type (sensor_msgs/msg/Imu)
- Measure publishing rate
- Validate data quality
- Verify frame_id

---

### 3. Check Data Quality (Reasonable Values)

**Command to view live data:**
```bash
ros2 topic echo /imu/data_raw
```

**Expected Values (Robot Stationary):**

#### Linear Acceleration (m/s²)
- **X-axis**: ~0.0 (±2.0 m/s²)
- **Y-axis**: ~0.0 (±2.0 m/s²)
- **Z-axis**: ~9.8 or ~-9.8 (depending on orientation)

The Z-axis should show approximately ±9.8 m/s² (gravity) when the robot is stationary.

#### Angular Velocity (rad/s)
- **X-axis**: ~0.0 (±0.1 rad/s)
- **Y-axis**: ~0.0 (±0.1 rad/s)
- **Z-axis**: ~0.0 (±0.1 rad/s)

All gyro values should be near zero when stationary.

#### Orientation (Quaternion)
```
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 1.0
```

The firmware does not provide absolute orientation, so this remains a unit quaternion.

**Success Criteria:**
- ✅ Acceleration values are not all zeros
- ✅ Z-axis acceleration is approximately ±9.8 m/s²
- ✅ Gyro values are near zero when stationary
- ✅ No NaN or infinite values

**Troubleshooting:**
- **All zeros**: Sensor not initialized or hardware connection issue
- **Unusual Z-axis**: Check sensor mounting orientation
- **High gyro drift**: Sensor may need calibration

---

### 4. Verify frame_id is Set Correctly

**Command:**
```bash
ros2 topic echo /imu/data_raw --once | grep frame_id
```

**Expected Output:**
```
  frame_id: imu_link
```

**Success Criteria:**
- ✅ frame_id must be exactly **"imu_link"**

This is configured in `src/config/FirmwareConfig.h`:
```cpp
namespace config {
namespace imu {
    inline constexpr const char* kFrameId = "imu_link";
}
}
```

---

### 5. Test with Robot Stationary

**Procedure:**
1. Place robot on a flat, stable surface
2. Do not touch or move the robot
3. Monitor IMU data for 30 seconds:
   ```bash
   ros2 topic echo /imu/data_raw
   ```

**Expected Behavior:**
- ✅ Acceleration Z-axis: stable around ±9.8 m/s²
- ✅ Acceleration X/Y-axis: stable around 0.0 m/s²
- ✅ Gyro X/Y/Z-axis: stable around 0.0 rad/s (small drift acceptable)
- ✅ Publishing continues without interruption

**Success Criteria:**
- ✅ Values remain relatively stable (small noise is normal)
- ✅ No sudden jumps or discontinuities
- ✅ No error messages in firmware serial output

---

### 6. Test with Robot Moving

**Procedure:**
1. **Tilt Test**: Slowly tilt the robot forward/backward
   - Monitor acceleration X/Y values change
   - Z-axis should decrease as robot tilts away from vertical

2. **Rotation Test**: Slowly rotate the robot around Z-axis
   - Monitor gyro Z-axis value increases (positive or negative)
   - Acceleration should remain relatively stable

3. **Movement Test**: Move the robot forward/backward
   - Monitor acceleration values change during acceleration/deceleration
   - Gyro values should remain near zero if moving straight

**Expected Behavior:**
- ✅ Acceleration values respond to tilt and linear motion
- ✅ Gyro values respond to rotation
- ✅ Publishing continues without interruption during movement
- ✅ Values return to baseline when robot stops

**Success Criteria:**
- ✅ Sensor responds to all types of motion
- ✅ No data dropouts during movement
- ✅ Values are physically reasonable

---

## Covariance Values

The IMU message includes covariance matrices that indicate sensor noise characteristics:

### Angular Velocity Covariance (rad/s)²
```cpp
imu_msg_.angular_velocity_covariance[0] = 0.033653;  // X variance
imu_msg_.angular_velocity_covariance[4] = 0.002124;  // Y variance
imu_msg_.angular_velocity_covariance[8] = 0.000277;  // Z variance
```

**Note**: X-axis shows higher noise (0.034), possibly due to mechanical stress or vibration.

### Linear Acceleration Covariance (m/s²)²
```cpp
imu_msg_.linear_acceleration_covariance[0] = 0.482400;  // X variance
imu_msg_.linear_acceleration_covariance[4] = 3.712792;  // Y variance (HIGH!)
imu_msg_.linear_acceleration_covariance[8] = 0.246898;  // Z variance
```

**Note**: Y-axis shows abnormally high noise (3.71). This may indicate a mounting or connection issue.

### Orientation Covariance
```cpp
imu_msg_.orientation_covariance[0] = -1.0;  // Not provided
```

The first element is set to -1.0 per ROS2 convention to indicate that orientation is not provided.

---

## Performance Monitoring

The IMU agent includes built-in performance profiling that reports every 1000 cycles:

**Example Serial Output:**
```
[ImuAgent] Performance: avg=8500 us, max=9200 us, target=10000 us (100 Hz)
```

**Success Criteria:**
- ✅ Average cycle time should be less than target (10000 us = 10 ms)
- ✅ Max cycle time should not significantly exceed target
- ✅ If avg > target, publishing rate will be lower than 100Hz

---

## Common Issues and Solutions

### Issue: No IMU data published

**Symptoms:**
- Topic exists but no messages
- Serial log shows "Sensor initialization FAILED"

**Solutions:**
1. Check SPI connections (MISO, MOSI, SCK, CS pins)
2. Verify power supply to IMU sensor
3. Check pin configuration in `FirmwareConfig.h`
4. Review serial logs for initialization errors

---

### Issue: All values are zero

**Symptoms:**
- Acceleration: (0, 0, 0)
- Gyro: (0, 0, 0)

**Solutions:**
1. Sensor not initialized - check serial logs
2. SPI communication failure - verify wiring
3. Sensor in reset state - check power supply

---

### Issue: Publishing rate too low

**Symptoms:**
- `ros2 topic hz` shows <80Hz

**Solutions:**
1. Check CPU load - other tasks may be blocking
2. Increase IMU task priority
3. Reduce `publish_period_ms_` if needed
4. Check for long-running operations in IMU loop

---

### Issue: Noisy or unstable data

**Symptoms:**
- Large fluctuations when stationary
- Unrealistic acceleration values

**Solutions:**
1. Check sensor mounting - ensure rigid connection
2. Verify power supply is stable (no voltage drops)
3. Check for electromagnetic interference
4. Consider adding low-pass filtering in firmware

---

## Quick Test Commands

```bash
# Check if topic exists
ros2 topic list | grep imu

# Check topic type
ros2 topic type /imu/data_raw

# Measure publishing rate (10 seconds)
timeout 10s ros2 topic hz /imu/data_raw

# View one message
ros2 topic echo /imu/data_raw --once

# Monitor live data
ros2 topic echo /imu/data_raw

# Check frame_id
ros2 topic echo /imu/data_raw --once | grep frame_id

# Run automated verification
./verify_imu_publishing.sh
```

---

## Success Criteria Summary

Task 14 is complete when:

- ✅ IMU topic `/imu/data_raw` exists and is publishing
- ✅ Publishing rate is approximately **100Hz** (80-120Hz acceptable)
- ✅ Data quality is reasonable:
  - Acceleration Z-axis shows gravity (~±9.8 m/s²) when stationary
  - Gyro values near zero when stationary
  - Values respond to movement
- ✅ frame_id is set to **"imu_link"**
- ✅ Publishing continues reliably during stationary and moving tests
- ✅ No error messages in firmware serial output

---

## Next Steps

After completing this verification:
- ✅ Mark task 14 as complete
- ➡️ Proceed to task 15: Verify odometry calculations
- 📝 Document any issues or observations

---

## Related Files

- `src/application/ImuAgent.cpp` - IMU agent implementation
- `src/application/ImuAgent.h` - IMU agent header
- `src/config/FirmwareConfig.h` - IMU configuration (frame_id, pins)
- `verify_imu_publishing.sh` - Automated verification script
