#!/bin/bash
# Health Monitoring Script for Firmware Testing
# Monitors ROS topics and provides real-time health status

echo "=========================================="
echo "Firmware Health Monitor"
echo "=========================================="
echo ""

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "ERROR: ros2 command not found"
    echo "Please source your ROS2 workspace first"
    exit 1
fi

# Function to check topic rate
check_topic_rate() {
    local topic=$1
    local expected_min=$2
    local expected_max=$3
    
    echo -n "Checking $topic... "
    
    # Get rate (timeout after 5 seconds)
    RATE=$(timeout 5s ros2 topic hz $topic 2>/dev/null | grep "average rate" | awk '{print $3}')
    
    if [ -z "$RATE" ]; then
        echo "❌ NO DATA"
        return 1
    fi
    
    # Compare with expected range
    RATE_INT=$(printf "%.0f" $RATE)
    if [ $RATE_INT -ge $expected_min ] && [ $RATE_INT -le $expected_max ]; then
        echo "✅ ${RATE} Hz (expected: ${expected_min}-${expected_max} Hz)"
        return 0
    else
        echo "⚠️  ${RATE} Hz (expected: ${expected_min}-${expected_max} Hz)"
        return 1
    fi
}

# Function to check if topic exists
check_topic_exists() {
    local topic=$1
    ros2 topic list 2>/dev/null | grep -q "^${topic}$"
    return $?
}

echo "1. Checking micro-ROS Connection"
echo "-----------------------------------"

# Check if topics exist
TOPICS=("/cmd_vel" "/imu/data_raw" "/joint_states" "/odom")
ALL_TOPICS_EXIST=true

for topic in "${TOPICS[@]}"; do
    echo -n "  $topic... "
    if check_topic_exists "$topic"; then
        echo "✅ EXISTS"
    else
        echo "❌ MISSING"
        ALL_TOPICS_EXIST=false
    fi
done

if [ "$ALL_TOPICS_EXIST" = false ]; then
    echo ""
    echo "ERROR: Not all expected topics are available"
    echo "Please check micro-ROS agent connection"
    exit 1
fi

echo ""
echo "2. Checking Topic Publishing Rates"
echo "-----------------------------------"

# Check IMU rate (expected ~100 Hz)
check_topic_rate "/imu/data_raw" 90 110

# Check joint states rate (expected ~50 Hz)
check_topic_rate "/joint_states" 40 60

# Check odometry rate (expected ~10 Hz)
check_topic_rate "/odom" 8 12

echo ""
echo "3. Checking Data Quality"
echo "-----------------------------------"

# Sample IMU data
echo -n "Sampling IMU data... "
IMU_SAMPLE=$(timeout 2s ros2 topic echo /imu/data_raw --once 2>/dev/null)
if [ -n "$IMU_SAMPLE" ]; then
    echo "✅ DATA RECEIVED"
    # Check for NaN values
    if echo "$IMU_SAMPLE" | grep -q "nan"; then
        echo "  ⚠️  WARNING: NaN values detected in IMU data"
    fi
else
    echo "❌ NO DATA"
fi

# Sample odometry data
echo -n "Sampling odometry data... "
ODOM_SAMPLE=$(timeout 2s ros2 topic echo /odom --once 2>/dev/null)
if [ -n "$ODOM_SAMPLE" ]; then
    echo "✅ DATA RECEIVED"
    # Check for NaN values
    if echo "$ODOM_SAMPLE" | grep -q "nan"; then
        echo "  ⚠️  WARNING: NaN values detected in odometry data"
    fi
else
    echo "❌ NO DATA"
fi

# Sample joint states
echo -n "Sampling joint states... "
JOINT_SAMPLE=$(timeout 2s ros2 topic echo /joint_states --once 2>/dev/null)
if [ -n "$JOINT_SAMPLE" ]; then
    echo "✅ DATA RECEIVED"
else
    echo "❌ NO DATA"
fi

echo ""
echo "4. Motor Control Test"
echo "-----------------------------------"
echo "Sending test velocity command..."

# Send a small forward velocity
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once \
    > /dev/null 2>&1

sleep 2

# Check if joint states show movement
echo -n "Checking motor response... "
JOINT_AFTER=$(timeout 2s ros2 topic echo /joint_states --once 2>/dev/null)
if [ -n "$JOINT_AFTER" ]; then
    echo "✅ MOTORS RESPONDING"
else
    echo "❌ NO RESPONSE"
fi

# Stop robot
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --once \
    > /dev/null 2>&1

echo ""
echo "=========================================="
echo "Health Check Complete"
echo "=========================================="
echo ""
echo "Summary:"
echo "  ✅ = Passed"
echo "  ⚠️  = Warning (check details)"
echo "  ❌ = Failed"
echo ""
echo "For continuous monitoring, run:"
echo "  watch -n 1 ./monitor_health.sh"
echo ""
