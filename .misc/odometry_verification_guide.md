# Odometry Verification Guide

This guide provides detailed instructions for verifying the odometry calculations on the my_steel robot firmware.

**Requirements Tested:** 4.4, 9.1

## Prerequisites

1. **Hardware Setup:**
   - my_steel robot with Raspberry Pi Pico flashed with latest firmware
   - Robot placed on a flat surface with adequate space (at least 2m x 2m)
   - USB connection to Raspberry Pi (running micro-ROS agent)
   - All motors and encoders functioning correctly

2. **Software Setup:**
   - micro-ROS agent running on Raspberry Pi
   - ROS2 environment sourced
   - Robot connected and publishing topics

3. **Verify Prerequisites:**
   ```bash
   # Check that micro-ROS agent is running
   ros2 topic list
   
   # Should see:
   # /cmd_vel
   # /imu/data_raw
   # /joint_states
   # /odom
   ```

## Understanding the Odometry System

### What is Odometry?

Odometry estimates the robot's position and orientation by integrating wheel encoder measurements over time. The my_steel robot uses a **differential drive odometry model** adapted for its mecanum wheel configuration.

### Key Parameters

From `src/RobotController.h`:
- **Wheel Radius:** 0.065 m (65 mm)
- **Wheel Separation:** 0.204 m (204 mm between left and right wheels)
- **Wheel Offset:** 0.010 m (10 mm from wheel axis to robot center)
- **Update Rate:** 10 Hz (100 ms period)

### Coordinate System

- **X-axis:** Forward (positive) / Backward (negative)
- **Y-axis:** Left (positive) / Right (negative)
- **Z-axis (rotation):** Counter-clockwise (positive) / Clockwise (negative)
- **Frame IDs:** `odom` (world frame) → `base_link` (robot frame)

### Expected Accuracy

Odometry accumulates error over time due to:
- Wheel slip
- Uneven surfaces
- Encoder quantization
- Measurement noise

**Typical accuracy:** ±10-20% over short distances (< 5 meters)

## Automated Test Script

The easiest way to verify odometry is using the automated test script:

```bash
./verify_odometry.sh
```

This script performs 5 comprehensive tests:
1. Forward movement (1 meter)
2. Rotation test
3. Strafe movement (mecanum-specific)
4. Combined movement
5. Stationary drift test

Review the output to verify all tests pass.

## Manual Test Procedures

### Test 1: Monitor Odometry Topic

**Purpose:** Verify odometry messages are being published correctly.

```bash
# Monitor odometry at 10 Hz
ros2 topic hz /odom

# Expected output: ~10 Hz
# average rate: 10.000
#   min: 0.099s max: 0.101s std dev: 0.00100s window: 100
```

```bash
# View odometry message structure
ros2 topic echo /odom --once
```

**Verify:**
- ✓ Messages published at ~10 Hz
- ✓ `header.frame_id` is "odom"
- ✓ `child_frame_id` is "base_link"
- ✓ Position (x, y, z) values are reasonable
- ✓ Orientation quaternion (x, y, z, w) is valid
- ✓ Twist (linear and angular velocities) are present
- ✓ Covariance matrices are populated

### Test 2: Forward Movement (1 meter)

**Purpose:** Verify odometry correctly tracks forward motion.

**Procedure:**

1. **Record initial position:**
   ```bash
   ros2 topic echo /odom --once | grep -A 3 "position:"
   ```
   Note the `x` value (e.g., x: 0.0)

2. **Command forward movement:**
   ```bash
   # Move forward at 0.1 m/s for 10 seconds (= 1 meter)
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
     --rate 10
   ```
   
   Let it run for 10 seconds, then press Ctrl+C.

3. **Stop the robot:**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
     --once
   ```

4. **Record final position:**
   ```bash
   ros2 topic echo /odom --once | grep -A 3 "position:"
   ```
   Note the `x` value (e.g., x: 0.95)

5. **Calculate distance:**
   ```
   Distance = final_x - initial_x
   Expected: ~1.0 meter (±0.2 m tolerance)
   ```

**Expected Results:**
- X position increases by approximately 1.0 meter
- Y position remains close to 0 (±0.05 m)
- Robot physically moves forward approximately 1 meter

**Troubleshooting:**
- If X doesn't change: Check encoder connections
- If distance is way off: Verify wheel radius constant (0.065 m)
- If Y changes significantly: Check wheel alignment

### Test 3: Backward Movement

**Purpose:** Verify odometry correctly tracks backward motion.

**Procedure:**

1. Record initial position
2. Command backward movement:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
     --rate 10
   ```
3. Run for 10 seconds, then stop
4. Record final position

**Expected Results:**
- X position decreases by approximately 1.0 meter
- Y position remains close to 0

### Test 4: Rotation Test

**Purpose:** Verify odometry correctly tracks rotation.

**Procedure:**

1. **Record initial orientation:**
   ```bash
   ros2 topic echo /odom --once | grep -A 4 "orientation:"
   ```
   Note the quaternion values (x, y, z, w)

2. **Command rotation:**
   ```bash
   # Rotate counter-clockwise at 0.5 rad/s
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" \
     --rate 10
   ```
   
   Run for ~6.28 seconds (one full rotation: 2π radians)

3. **Stop and record final orientation**

**Expected Results:**
- Orientation quaternion changes
- After one full rotation (2π radians), should return close to initial orientation
- Robot physically rotates approximately 360 degrees

**Note:** Converting quaternion to Euler angles:
```python
import math
from scipy.spatial.transform import Rotation as R

# Example quaternion from /odom
quat = [x, y, z, w]  # Replace with actual values
r = R.from_quat(quat)
euler = r.as_euler('xyz', degrees=True)
print(f"Yaw: {euler[2]} degrees")
```

### Test 5: Strafe Movement (Mecanum-Specific)

**Purpose:** Verify odometry correctly tracks lateral (strafe) motion.

**Procedure:**

1. **Record initial position**

2. **Command strafe left:**
   ```bash
   # Strafe left at 0.1 m/s
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
     --rate 10
   ```
   
   Run for 10 seconds (= 1 meter strafe)

3. **Stop and record final position**

4. **Calculate distance:**
   ```
   Distance = final_y - initial_y
   Expected: ~1.0 meter (±0.2 m tolerance)
   ```

**Expected Results:**
- Y position increases by approximately 1.0 meter
- X position remains close to 0 (±0.05 m)
- Robot physically strafes left approximately 1 meter

**Note:** Mecanum wheels allow omnidirectional movement. If strafe doesn't work:
- Check wheel orientation (rollers at 45° angles)
- Verify all 4 motors are responding
- Check mecanum kinematics in `RobotController::handleSubscriptionMsg()`

### Test 6: Combined Movement

**Purpose:** Verify odometry correctly tracks simultaneous forward, strafe, and rotation.

**Procedure:**

1. **Record initial position and orientation**

2. **Command combined movement:**
   ```bash
   # Forward + strafe + rotation
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.05, y: 0.05, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}" \
     --rate 10
   ```
   
   Run for 10 seconds

3. **Stop and record final position and orientation**

**Expected Results:**
- Both X and Y positions change
- Orientation changes (rotation)
- Robot moves in a curved path

### Test 7: Stationary Drift Test

**Purpose:** Verify odometry doesn't drift when robot is stationary.

**Procedure:**

1. **Ensure robot is stopped:**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
     --once
   ```

2. **Record initial position:**
   ```bash
   ros2 topic echo /odom --once | grep -A 3 "position:"
   ```

3. **Wait 30 seconds** (do not touch the robot)

4. **Record final position**

5. **Calculate drift:**
   ```
   Drift_X = |final_x - initial_x|
   Drift_Y = |final_y - initial_y|
   Expected: < 0.001 m (1 mm)
   ```

**Expected Results:**
- Position should remain essentially constant
- Minimal drift (< 1 mm) is acceptable
- Significant drift indicates encoder noise or vibration

### Test 8: Velocity Verification

**Purpose:** Verify twist (velocity) values in odometry message.

**Procedure:**

1. **Command constant velocity:**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
     "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
     --rate 10
   ```

2. **Monitor odometry twist:**
   ```bash
   ros2 topic echo /odom | grep -A 6 "twist:"
   ```

**Expected Results:**
- `twist.twist.linear.x` should be close to 0.2 m/s
- `twist.twist.linear.y` should be close to 0.0 m/s
- `twist.twist.angular.z` should be close to 0.0 rad/s

**Note:** There will be some variation due to PID control and encoder quantization.

## Comparison with Expected Values

### Forward Movement (1 meter at 0.1 m/s for 10 seconds)

| Metric | Expected | Acceptable Range | Units |
|--------|----------|------------------|-------|
| ΔX | 1.000 | 0.800 - 1.200 | meters |
| ΔY | 0.000 | -0.050 - 0.050 | meters |
| Δθ | 0.000 | -0.087 - 0.087 | radians (±5°) |

### Rotation (0.5 rad/s for 6.28 seconds = 2π radians)

| Metric | Expected | Acceptable Range | Units |
|--------|----------|------------------|-------|
| Δθ | 6.283 (2π) | 5.654 - 6.912 | radians (±10%) |
| ΔX | 0.000 | -0.100 - 0.100 | meters |
| ΔY | 0.000 | -0.100 - 0.100 | meters |

### Strafe Left (1 meter at 0.1 m/s for 10 seconds)

| Metric | Expected | Acceptable Range | Units |
|--------|----------|------------------|-------|
| ΔX | 0.000 | -0.050 - 0.050 | meters |
| ΔY | 1.000 | 0.800 - 1.200 | meters |
| Δθ | 0.000 | -0.087 - 0.087 | radians (±5°) |

## Troubleshooting

### Odometry Not Updating

**Symptoms:** Position values don't change when robot moves.

**Possible Causes:**
1. Encoders not connected or not working
2. Motor agent not initialized
3. Odometry calculation disabled

**Debugging:**
```bash
# Check joint_states (encoder data)
ros2 topic echo /joint_states

# Should see position values changing when motors run
```

### Odometry Values Way Off

**Symptoms:** Position changes don't match physical movement.

**Possible Causes:**
1. Incorrect wheel radius constant
2. Incorrect wheel separation constant
3. Encoder direction inverted
4. Wheel slipping

**Debugging:**
- Verify constants in `src/RobotController.h`:
  - `kWheelRadius = 0.065` (65 mm)
  - `kWheelsSeparation = 0.204` (204 mm)
- Check encoder wiring (A/B channels)
- Test on non-slip surface

### Odometry Drifts Over Time

**Symptoms:** Position error accumulates over long distances.

**This is Normal:** Odometry inherently accumulates error due to:
- Wheel slip
- Uneven surfaces
- Encoder quantization
- Measurement noise

**Mitigation:**
- Fuse with IMU data (already implemented)
- Use visual odometry or SLAM for long-term accuracy
- Reset odometry periodically at known positions

### Y Position Changes During Forward Movement

**Symptoms:** Y position drifts when commanding pure forward motion.

**Possible Causes:**
1. Wheels not aligned (mecanum wheels)
2. Uneven motor speeds
3. Surface not level
4. PID tuning issues

**Debugging:**
```bash
# Check individual motor speeds
ros2 topic echo /joint_states

# All motors should have similar velocities for forward motion
```

### Rotation Doesn't Match Expected Angle

**Symptoms:** Robot rotates more or less than commanded.

**Possible Causes:**
1. Incorrect wheel separation constant
2. Wheel slip during rotation
3. Uneven motor speeds

**Debugging:**
- Measure actual wheel separation (center to center)
- Test on non-slip surface
- Verify PID tuning

## Advanced: Odometry Algorithm Details

The odometry calculation is implemented in `src/RobotController.cpp` in the `updateOdom()` method.

### Algorithm Steps:

1. **Get encoder deltas** (radians rotated since last update)
   ```cpp
   double l = pMotorsAgent->getMotor(0)->getDeltaRadians();
   double r = pMotorsAgent->getMotor(1)->getDeltaRadians() * (-1.0);
   ```

2. **Convert to linear distance**
   ```cpp
   double diam = kWheelRadius * 2.0 * M_PI;
   l = diam * (l / (M_PI * 2.0));
   r = diam * (r / (M_PI * 2.0));
   ```

3. **Calculate average distance and heading change**
   ```cpp
   double avgDist = (r + l) / 2.0;
   double angle = asin((r - l) / kWheelsSeparation);
   ```

4. **Decompose into X/Y components**
   ```cpp
   double deltaX = cos(angle) * avgDist;
   double deltaY = sin(angle) * avgDist;
   ```

5. **Integrate into cumulative odometry**
   ```cpp
   xMotorsOdom.x += deltaX;
   xMotorsOdom.y += deltaY;
   xMotorsOdom.a += angle;
   ```

6. **Transform to robot center**
   ```cpp
   xRobotOdom.x = xMotorsOdom.x + (cos(angle) * kWheelsOffset);
   xRobotOdom.y = xMotorsOdom.y + (sin(angle) * kWheelsOffset);
   xRobotOdom.a = xMotorsOdom.a;
   ```

7. **Calculate velocities**
   ```cpp
   double seconds = (double)(now - xLastVelocityTime) / 1000.0;
   xRobotVelocity.x = deltaX / seconds;
   xRobotVelocity.y = deltaY / seconds;
   xRobotVelocity.a = angle / seconds;
   ```

### Key Assumptions:

- **Differential Drive Model:** Uses left/right wheel velocities
- **Small Angle Approximation:** Works best for small incremental movements
- **No Slip:** Assumes wheels maintain traction
- **Planar Motion:** Assumes robot moves on flat surface (Z = 0)

## Success Criteria

The odometry verification is considered successful if:

✓ **Forward Movement Test:**
- X position changes by 0.8-1.2 meters when commanding 1 meter forward
- Y position remains within ±0.05 meters

✓ **Rotation Test:**
- Orientation quaternion changes appropriately
- After full rotation, returns close to initial orientation (±10%)

✓ **Strafe Test:**
- Y position changes by 0.8-1.2 meters when commanding 1 meter strafe
- X position remains within ±0.05 meters

✓ **Combined Movement Test:**
- Both position and orientation update correctly
- Robot follows expected curved path

✓ **Stationary Drift Test:**
- Position drift < 1 mm over 30 seconds

✓ **Velocity Test:**
- Twist values in odometry message match commanded velocities (±20%)

## Conclusion

After completing these tests, you should have confidence that:
1. Odometry calculations are mathematically correct
2. Encoder data is being read properly
3. Wheel parameters (radius, separation) are accurate
4. The robot can track its position and orientation
5. Mecanum wheel kinematics are working correctly

**Note:** Odometry is a dead-reckoning technique and will accumulate error over time. For production use, consider fusing with other sensors (IMU, visual odometry, GPS) for improved accuracy.

## References

- Source code: `src/RobotController.cpp` (updateOdom method)
- Configuration: `src/RobotController.h` (wheel parameters)
- Requirements: `.kiro/specs/firmware-modernization/requirements.md` (4.4, 9.1)
- Design: `.kiro/specs/firmware-modernization/design.md`
