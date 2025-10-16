# Motor Control Testing Guide

## Prerequisites
- ✅ Firmware flashed to Pico (from task 11)
- ✅ micro-ROS agent running on Raspberry Pi (from task 12)
- ✅ Pico connected via USB
- ✅ All topics verified: `/cmd_vel`, `/imu/data_raw`, `/joint_states`, `/odometry/wheels`

## Test Overview
This guide tests the motor control functionality by sending velocity commands via `/cmd_vel` and verifying:
1. Motors respond correctly to commands
2. Robot moves in expected directions
3. Encoder feedback is published to `/joint_states`
4. PID control maintains target speeds

## Safety Notes
⚠️ **IMPORTANT:**
- Ensure robot has clear space to move (at least 1 meter in all directions)
- Keep emergency stop ready (unplug USB or power)
- Start with low velocities (0.1 m/s) before increasing
- Monitor for unexpected behavior

---

## Test 1: Forward Movement

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Expected Behavior
- All 4 motors should spin forward
- Robot moves forward at ~0.1 m/s
- Movement should be smooth and controlled

### Verification
Monitor joint states for encoder feedback:
```bash
ros2 topic echo /joint_states
```

Expected output:
- `velocity` array should show positive values for all 4 motors
- Values should be approximately equal (mecanum forward kinematics)
- `position` array should increment over time

### Success Criteria
- ✅ Robot moves forward in straight line
- ✅ All motors respond
- ✅ Encoder feedback shows positive velocities
- ✅ No excessive vibration or jerking

---

## Test 2: Backward Movement

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Expected Behavior
- All 4 motors should spin backward
- Robot moves backward at ~0.1 m/s

### Verification
```bash
ros2 topic echo /joint_states
```

Expected output:
- `velocity` array should show negative values for all 4 motors
- Values should be approximately equal

### Success Criteria
- ✅ Robot moves backward in straight line
- ✅ Encoder feedback shows negative velocities
- ✅ Movement mirrors forward test

---

## Test 3: Strafe Left (Mecanum-Specific)

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Expected Behavior
- Robot moves left (perpendicular to forward direction)
- Front-left and rear-right wheels spin forward
- Front-right and rear-left wheels spin backward
- No forward/backward movement

### Verification
```bash
ros2 topic echo /joint_states
```

Expected output:
- Motor 0 (FL): positive velocity
- Motor 1 (FR): negative velocity
- Motor 2 (RL): negative velocity
- Motor 3 (RR): positive velocity

### Success Criteria
- ✅ Robot strafes left without rotating
- ✅ Encoder feedback matches mecanum kinematics
- ✅ Minimal forward/backward drift

---

## Test 4: Strafe Right (Mecanum-Specific)

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Expected Behavior
- Robot moves right (perpendicular to forward direction)
- Front-right and rear-left wheels spin forward
- Front-left and rear-right wheels spin backward
- No forward/backward movement

### Verification
```bash
ros2 topic echo /joint_states
```

Expected output:
- Motor 0 (FL): negative velocity
- Motor 1 (FR): positive velocity
- Motor 2 (RL): positive velocity
- Motor 3 (RR): negative velocity

### Success Criteria
- ✅ Robot strafes right without rotating
- ✅ Encoder feedback matches mecanum kinematics
- ✅ Minimal forward/backward drift

---

## Test 5: Rotation (Counter-Clockwise)

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}" --once
```

### Expected Behavior
- Robot rotates counter-clockwise (left) around its center
- Left wheels spin backward
- Right wheels spin forward
- No translation (stays in same position)

### Verification
```bash
ros2 topic echo /joint_states
```

Expected output:
- Motor 0 (FL): negative velocity
- Motor 1 (FR): positive velocity
- Motor 2 (RL): negative velocity
- Motor 3 (RR): positive velocity

### Success Criteria
- ✅ Robot rotates in place
- ✅ Minimal translation during rotation
- ✅ Encoder feedback shows opposite velocities for left/right

---

## Test 6: Rotation (Clockwise)

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}" --once
```

### Expected Behavior
- Robot rotates clockwise (right) around its center
- Left wheels spin forward
- Right wheels spin backward

### Verification
```bash
ros2 topic echo /joint_states
```

Expected output:
- Motor 0 (FL): positive velocity
- Motor 1 (FR): negative velocity
- Motor 2 (RL): positive velocity
- Motor 3 (RR): negative velocity

### Success Criteria
- ✅ Robot rotates in place (opposite direction from Test 5)
- ✅ Encoder feedback shows opposite velocities for left/right

---

## Test 7: Combined Movement (Forward + Strafe)

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.1, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Expected Behavior
- Robot moves diagonally (forward-left at 45°)
- Demonstrates mecanum omnidirectional capability

### Success Criteria
- ✅ Robot moves at 45° angle
- ✅ All motors respond with appropriate velocities

---

## Test 8: Combined Movement (Forward + Rotation)

### Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}" --once
```

### Expected Behavior
- Robot moves forward while rotating counter-clockwise
- Creates an arc trajectory

### Success Criteria
- ✅ Robot follows curved path
- ✅ Both translation and rotation occur simultaneously

---

## Test 9: PID Control Verification

### Setup
Monitor joint states continuously:
```bash
ros2 topic echo /joint_states --no-arr
```

### Command
Send sustained velocity command:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" -r 10
```

### Expected Behavior
- Initial velocity ramp-up (PID convergence)
- Steady-state velocity maintained at target
- Minimal oscillation around target speed

### Verification
Observe velocity values over 5-10 seconds:
- Initial: velocities increase from 0
- Steady-state: velocities stabilize near target
- PID should prevent overshoot and oscillation

### Stop Command
```bash
# Press Ctrl+C to stop publishing, or send zero velocity:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
```

### Success Criteria
- ✅ Smooth velocity ramp-up (no sudden jumps)
- ✅ Target velocity achieved within 1-2 seconds
- ✅ Minimal steady-state error (<5%)
- ✅ No sustained oscillation

---

## Test 10: Timeout Safety Feature

### Test Procedure
1. Send a velocity command:
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once
   ```

2. Wait 2 seconds without sending another command

3. Observe robot behavior

### Expected Behavior
- Robot should automatically stop after ~1.5 seconds (MAX_TWIST_TIME_MS)
- Safety feature prevents runaway if connection lost

### Success Criteria
- ✅ Robot stops automatically after timeout
- ✅ No manual intervention needed

---

## Troubleshooting

### Motors Don't Respond
**Check:**
- Is micro-ROS agent running? `ros2 node list`
- Is Pico connected? Check USB connection
- Are topics available? `ros2 topic list`
- Check firmware logs via UART0 (115200 baud)

### Uneven Motor Speeds
**Possible Causes:**
- PID tuning needed (adjust kProportional, kIntegral, kDerivative)
- Mechanical issues (friction, wheel alignment)
- Encoder calibration

### Robot Drifts During Movement
**Possible Causes:**
- Wheel slip on surface
- Mecanum wheel alignment
- Weight distribution
- PID tuning

### Encoder Feedback Missing
**Check:**
- Encoder wiring (A/B channels)
- Pull-up resistors on encoder pins
- Encoder power supply
- Check `/joint_states` topic: `ros2 topic hz /joint_states`

---

## Test Results Template

Copy and fill out after testing:

```
## Motor Control Test Results

Date: ___________
Firmware Version: ___________
Tester: ___________

### Test Results
- [ ] Test 1: Forward Movement - PASS/FAIL
- [ ] Test 2: Backward Movement - PASS/FAIL
- [ ] Test 3: Strafe Left - PASS/FAIL
- [ ] Test 4: Strafe Right - PASS/FAIL
- [ ] Test 5: Rotation CCW - PASS/FAIL
- [ ] Test 6: Rotation CW - PASS/FAIL
- [ ] Test 7: Combined (Forward+Strafe) - PASS/FAIL
- [ ] Test 8: Combined (Forward+Rotation) - PASS/FAIL
- [ ] Test 9: PID Control - PASS/FAIL
- [ ] Test 10: Timeout Safety - PASS/FAIL

### Notes
(Any observations, issues, or anomalies)

### Requirements Verification
- [ ] 4.1: Robot responds to /cmd_vel commands identically
- [ ] 4.2: Motor control functions without regression
- [ ] 9.1: Motor control loop timing not degraded
- [ ] 9.2: IMU sampling rate maintained (check with Test 14)
```

---

## Quick Reference: Motor Layout

```
        FRONT
    [0]       [1]
     FL        FR
     
     
    [2]       [3]
     RL        RR
        REAR
```

Motor Indices:
- 0: Front Left (FL)
- 1: Front Right (FR)
- 2: Rear Left (RL)
- 3: Rear Right (RR)

---

## Next Steps

After completing all tests:
1. Document results in test results template
2. If all tests pass, mark task 13 as complete
3. Proceed to task 14 (IMU verification)
4. If issues found, investigate and fix before proceeding
