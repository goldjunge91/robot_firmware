#!/bin/bash
# Stress Test Script for Firmware Integration Testing
# Run this script after flashing firmware and connecting to micro-ROS agent

echo "=========================================="
echo "Firmware Stress Test - 30 Minute Duration"
echo "=========================================="
echo ""
echo "Prerequisites:"
echo "1. Firmware flashed to Pico"
echo "2. micro-ROS agent running"
echo "3. Robot in safe testing area"
echo ""
read -p "Press Enter to start stress test..."

# Test duration in seconds (30 minutes = 1800 seconds)
DURATION=1800
START_TIME=$(date +%s)
END_TIME=$((START_TIME + DURATION))
CYCLE=0

echo ""
echo "Starting stress test at $(date)"
echo "Test will run until $(date -r $END_TIME)"
echo ""

# Function to publish velocity command
publish_cmd() {
    local x=$1
    local y=$2
    local z=$3
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
        "{linear: {x: $x, y: $y, z: 0.0}, angular: {x: 0.0, y: 0.0, z: $z}}" --once \
        > /dev/null 2>&1
}

# Main test loop
while [ $(date +%s) -lt $END_TIME ]; do
    CYCLE=$((CYCLE + 1))
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))
    REMAINING=$((END_TIME - CURRENT_TIME))
    
    echo "----------------------------------------"
    echo "Cycle: $CYCLE | Elapsed: ${ELAPSED}s | Remaining: ${REMAINING}s"
    echo "----------------------------------------"
    
    # Test 1: Forward movement
    echo "  [1/6] Forward movement (0.1 m/s)"
    publish_cmd 0.1 0.0 0.0
    sleep 5
    
    # Test 2: Backward movement
    echo "  [2/6] Backward movement (-0.1 m/s)"
    publish_cmd -0.1 0.0 0.0
    sleep 5
    
    # Test 3: Strafe left
    echo "  [3/6] Strafe left (0.1 m/s)"
    publish_cmd 0.0 0.1 0.0
    sleep 5
    
    # Test 4: Strafe right
    echo "  [4/6] Strafe right (-0.1 m/s)"
    publish_cmd 0.0 -0.1 0.0
    sleep 5
    
    # Test 5: Rotate
    echo "  [5/6] Rotate (0.3 rad/s)"
    publish_cmd 0.0 0.0 0.3
    sleep 5
    
    # Test 6: Combined movement
    echo "  [6/6] Combined movement"
    publish_cmd 0.08 0.05 0.2
    sleep 5
    
    # Stop
    echo "  [Stop] Stopping robot"
    publish_cmd 0.0 0.0 0.0
    sleep 2
    
    # Check if we should continue
    if [ $(date +%s) -ge $END_TIME ]; then
        break
    fi
done

# Final stop command
echo ""
echo "=========================================="
echo "Stress test complete!"
echo "=========================================="
echo "Sending final stop command..."
publish_cmd 0.0 0.0 0.0

echo ""
echo "Test Summary:"
echo "  Total cycles completed: $CYCLE"
echo "  Total duration: $(($(date +%s) - START_TIME)) seconds"
echo ""
echo "Please review:"
echo "  1. Serial output for any errors"
echo "  2. Stack high water marks"
echo "  3. micro-ROS connection stability"
echo "  4. Motor performance consistency"
echo ""
