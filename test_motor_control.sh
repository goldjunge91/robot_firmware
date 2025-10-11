#!/bin/bash

# Motor Control Test Script
# Automates the motor control testing sequence for my_steel robot

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Test duration for each command (seconds)
TEST_DURATION=3
PAUSE_BETWEEN_TESTS=2

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}  Motor Control Test Suite${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}ERROR: ros2 command not found${NC}"
    echo "Please source your ROS2 workspace first:"
    echo "  source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# Check if micro-ROS agent is running
echo -e "${YELLOW}Checking micro-ROS connection...${NC}"
if ! ros2 topic list | grep -q "/cmd_vel"; then
    echo -e "${RED}ERROR: /cmd_vel topic not found${NC}"
    echo "Is the micro-ROS agent running?"
    echo "Is the Pico connected?"
    exit 1
fi
echo -e "${GREEN}✓ micro-ROS connection OK${NC}"
echo ""

# Function to send velocity command
send_cmd() {
    local test_name=$1
    local linear_x=$2
    local linear_y=$3
    local angular_z=$4
    
    echo -e "${BLUE}Test: ${test_name}${NC}"
    echo "  linear.x=${linear_x}, linear.y=${linear_y}, angular.z=${angular_z}"
    
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: ${linear_x}, y: ${linear_y}, z: 0.0}, angular: {x: 0.0, y: 0.0, z: ${angular_z}}}" \
        --once
    
    echo -e "${YELLOW}  → Robot should move for ${TEST_DURATION} seconds${NC}"
    sleep $TEST_DURATION
    
    # Stop robot
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" \
        --once
    
    echo -e "${GREEN}  ✓ Test complete${NC}"
    echo ""
    
    if [ $PAUSE_BETWEEN_TESTS -gt 0 ]; then
        echo -e "${YELLOW}Pausing ${PAUSE_BETWEEN_TESTS}s before next test...${NC}"
        sleep $PAUSE_BETWEEN_TESTS
    fi
}

# Safety warning
echo -e "${RED}⚠️  SAFETY WARNING ⚠️${NC}"
echo "Ensure the robot has clear space to move!"
echo "Press Ctrl+C to abort at any time."
echo ""
read -p "Press Enter to start tests..."
echo ""

# Test 1: Forward
send_cmd "Forward Movement" 0.1 0.0 0.0

# Test 2: Backward
send_cmd "Backward Movement" -0.1 0.0 0.0

# Test 3: Strafe Left
send_cmd "Strafe Left" 0.0 0.1 0.0

# Test 4: Strafe Right
send_cmd "Strafe Right" 0.0 -0.1 0.0

# Test 5: Rotate Counter-Clockwise
send_cmd "Rotate CCW" 0.0 0.0 0.5

# Test 6: Rotate Clockwise
send_cmd "Rotate CW" 0.0 0.0 -0.5

# Test 7: Diagonal (Forward + Left)
send_cmd "Diagonal Forward-Left" 0.1 0.1 0.0

# Test 8: Arc (Forward + Rotate)
send_cmd "Arc Forward-CCW" 0.1 0.0 0.3

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  All basic tests complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "Additional manual tests:"
echo "  - Test 9: PID Control (see motor_control_test_guide.md)"
echo "  - Test 10: Timeout Safety (see motor_control_test_guide.md)"
echo ""
echo "To monitor encoder feedback:"
echo "  ros2 topic echo /joint_states"
echo ""
echo "To monitor odometry:"
echo "  ros2 topic echo /odom"
