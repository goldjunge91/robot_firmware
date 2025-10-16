#!/bin/bash

# Odometry Verification Test Script
# This script tests the odometry calculations for the my_steel robot
# Requirements: 4.4, 9.1

set -e

echo "=========================================="
echo "Odometry Verification Test"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Test parameters
FORWARD_VELOCITY=0.1  # m/s
STRAFE_VELOCITY=0.1   # m/s
ROTATION_VELOCITY=0.5 # rad/s
TEST_DURATION=10      # seconds

echo "Test Configuration:"
echo "  Forward velocity: ${FORWARD_VELOCITY} m/s"
echo "  Strafe velocity: ${STRAFE_VELOCITY} m/s"
echo "  Rotation velocity: ${ROTATION_VELOCITY} rad/s"
echo "  Test duration: ${TEST_DURATION} seconds"
echo ""

# Function to publish velocity command
publish_velocity() {
    local vx=$1
    local vy=$2
    local vz=$3
    local duration=$4
    
    echo -e "${YELLOW}Publishing velocity: vx=${vx}, vy=${vy}, vz=${vz} for ${duration}s${NC}"
    
    # Publish the command
    timeout ${duration}s ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: ${vx}, y: ${vy}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${vz}}}" \
        --rate 10 > /dev/null 2>&1 || true
    
    # Stop the robot
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        --once > /dev/null 2>&1
    
    sleep 1
}

# Function to get current odometry
get_odometry() {
    ros2 topic echo /odom --once 2>/dev/null | grep -A 3 "position:" | tail -3
}

# Function to extract position values
extract_position() {
    local odom_output=$1
    local x=$(echo "$odom_output" | grep "x:" | awk '{print $2}')
    local y=$(echo "$odom_output" | grep "y:" | awk '{print $2}')
    local z=$(echo "$odom_output" | grep "z:" | awk '{print $2}')
    echo "$x $y $z"
}

# Function to extract orientation (quaternion)
extract_orientation() {
    ros2 topic echo /odom --once 2>/dev/null | grep -A 4 "orientation:" | tail -4
}

# Function to calculate expected distance
calculate_expected_distance() {
    local velocity=$1
    local duration=$2
    echo "scale=3; $velocity * $duration" | bc
}

echo "=========================================="
echo "Test 1: Forward Movement (1 meter)"
echo "=========================================="
echo ""

echo "Getting initial odometry..."
initial_odom=$(get_odometry)
initial_pos=$(extract_position "$initial_odom")
initial_x=$(echo $initial_pos | awk '{print $1}')
initial_y=$(echo $initial_pos | awk '{print $2}')

echo "Initial position: x=${initial_x}, y=${initial_y}"
echo ""

# Calculate duration to move 1 meter
forward_duration=$(echo "scale=1; 1.0 / $FORWARD_VELOCITY" | bc)
echo "Moving forward for ${forward_duration} seconds to travel 1 meter..."
publish_velocity $FORWARD_VELOCITY 0.0 0.0 $forward_duration

echo "Getting final odometry..."
final_odom=$(get_odometry)
final_pos=$(extract_position "$final_odom")
final_x=$(echo $final_pos | awk '{print $1}')
final_y=$(echo $final_pos | awk '{print $2}')

echo "Final position: x=${final_x}, y=${final_y}"
echo ""

# Calculate actual distance traveled
distance_x=$(echo "scale=3; $final_x - $initial_x" | bc)
distance_y=$(echo "scale=3; $final_y - $initial_y" | bc)

echo "Distance traveled:"
echo "  X: ${distance_x} m (expected: ~1.0 m)"
echo "  Y: ${distance_y} m (expected: ~0.0 m)"
echo ""

# Check if within acceptable range (±20%)
expected=1.0
tolerance=0.2
min_acceptable=$(echo "scale=3; $expected - $tolerance" | bc)
max_acceptable=$(echo "scale=3; $expected + $tolerance" | bc)

if (( $(echo "$distance_x >= $min_acceptable" | bc -l) )) && (( $(echo "$distance_x <= $max_acceptable" | bc -l) )); then
    echo -e "${GREEN}✓ Forward movement test PASSED${NC}"
else
    echo -e "${RED}✗ Forward movement test FAILED (outside ±20% tolerance)${NC}"
fi
echo ""

echo "=========================================="
echo "Test 2: Rotation Test"
echo "=========================================="
echo ""

echo "Stopping and resetting..."
sleep 2

echo "Getting initial orientation..."
initial_orient=$(extract_orientation)
echo "$initial_orient"
echo ""

# Rotate for a specific duration
rotation_duration=6.28  # ~1 full rotation at 0.5 rad/s
echo "Rotating for ${rotation_duration} seconds..."
publish_velocity 0.0 0.0 $ROTATION_VELOCITY $rotation_duration

echo "Getting final orientation..."
final_orient=$(extract_orientation)
echo "$final_orient"
echo ""

echo -e "${GREEN}✓ Rotation test completed (verify quaternion changed)${NC}"
echo ""

echo "=========================================="
echo "Test 3: Strafe Movement (Mecanum-specific)"
echo "=========================================="
echo ""

echo "Getting initial odometry..."
initial_odom=$(get_odometry)
initial_pos=$(extract_position "$initial_odom")
initial_x=$(echo $initial_pos | awk '{print $1}')
initial_y=$(echo $initial_pos | awk '{print $2}')

echo "Initial position: x=${initial_x}, y=${initial_y}"
echo ""

# Strafe left for 10 seconds
echo "Strafing left for ${TEST_DURATION} seconds..."
publish_velocity 0.0 $STRAFE_VELOCITY 0.0 $TEST_DURATION

echo "Getting final odometry..."
final_odom=$(get_odometry)
final_pos=$(extract_position "$final_odom")
final_x=$(echo $final_pos | awk '{print $1}')
final_y=$(echo $final_pos | awk '{print $2}')

echo "Final position: x=${final_x}, y=${final_y}"
echo ""

# Calculate actual distance traveled
distance_x=$(echo "scale=3; $final_x - $initial_x" | bc)
distance_y=$(echo "scale=3; $final_y - $initial_y" | bc)

expected_strafe=$(calculate_expected_distance $STRAFE_VELOCITY $TEST_DURATION)

echo "Distance traveled:"
echo "  X: ${distance_x} m (expected: ~0.0 m)"
echo "  Y: ${distance_y} m (expected: ~${expected_strafe} m)"
echo ""

# Check if Y movement is significant (mecanum wheels allow strafing)
if (( $(echo "$distance_y > 0.5" | bc -l) )); then
    echo -e "${GREEN}✓ Strafe movement test PASSED (mecanum wheels working)${NC}"
else
    echo -e "${YELLOW}⚠ Strafe movement limited (check mecanum wheel alignment)${NC}"
fi
echo ""

echo "=========================================="
echo "Test 4: Combined Movement"
echo "=========================================="
echo ""

echo "Getting initial odometry..."
initial_odom=$(get_odometry)
initial_pos=$(extract_position "$initial_odom")
initial_x=$(echo $initial_pos | awk '{print $1}')
initial_y=$(echo $initial_pos | awk '{print $2}')

echo "Initial position: x=${initial_x}, y=${initial_y}"
echo ""

# Combined forward + strafe + rotation
echo "Executing combined movement (forward + strafe + rotation)..."
publish_velocity 0.05 0.05 0.2 $TEST_DURATION

echo "Getting final odometry..."
final_odom=$(get_odometry)
final_pos=$(extract_position "$final_odom")
final_x=$(echo $final_pos | awk '{print $1}')
final_y=$(echo $final_pos | awk '{print $2}')

echo "Final position: x=${final_x}, y=${final_y}"
echo ""

# Calculate actual distance traveled
distance_x=$(echo "scale=3; $final_x - $initial_x" | bc)
distance_y=$(echo "scale=3; $final_y - $initial_y" | bc)

echo "Distance traveled:"
echo "  X: ${distance_x} m"
echo "  Y: ${distance_y} m"
echo ""

echo -e "${GREEN}✓ Combined movement test completed${NC}"
echo ""

echo "=========================================="
echo "Test 5: Continuous Odometry Monitoring"
echo "=========================================="
echo ""

echo "Monitoring odometry for 5 seconds while stationary..."
echo "This verifies odometry doesn't drift when robot is stopped."
echo ""

initial_odom=$(get_odometry)
initial_pos=$(extract_position "$initial_odom")
initial_x=$(echo $initial_pos | awk '{print $1}')
initial_y=$(echo $initial_pos | awk '{print $2}')

sleep 5

final_odom=$(get_odometry)
final_pos=$(extract_position "$final_odom")
final_x=$(echo $final_pos | awk '{print $1}')
final_y=$(echo $final_pos | awk '{print $2}')

drift_x=$(echo "scale=4; $final_x - $initial_x" | bc | sed 's/-//')
drift_y=$(echo "scale=4; $final_y - $initial_y" | bc | sed 's/-/')

echo "Drift detected:"
echo "  X: ${drift_x} m"
echo "  Y: ${drift_y} m"
echo ""

# Check if drift is minimal (< 1mm)
if (( $(echo "$drift_x < 0.001" | bc -l) )) && (( $(echo "$drift_y < 0.001" | bc -l) )); then
    echo -e "${GREEN}✓ Stationary drift test PASSED (minimal drift)${NC}"
else
    echo -e "${YELLOW}⚠ Some drift detected while stationary (check encoder noise)${NC}"
fi
echo ""

echo "=========================================="
echo "Odometry Verification Complete"
echo "=========================================="
echo ""
echo "Summary:"
echo "  ✓ Forward movement tested"
echo "  ✓ Rotation tested"
echo "  ✓ Strafe movement tested (mecanum-specific)"
echo "  ✓ Combined movement tested"
echo "  ✓ Stationary drift tested"
echo ""
echo "Review the results above to verify odometry calculations are accurate."
echo "Expected behavior:"
echo "  - Forward movement should update X position"
echo "  - Rotation should update orientation quaternion"
echo "  - Strafe should update Y position (mecanum wheels)"
echo "  - Minimal drift when stationary"
echo ""
