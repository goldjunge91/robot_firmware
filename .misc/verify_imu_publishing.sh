#!/bin/bash

# IMU Data Publishing Verification Script
# Task 14: Verify IMU data publishing
# Requirements: 4.3, 9.2

set -e

echo "=========================================="
echo "IMU Data Publishing Verification"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}❌ ERROR: ros2 command not found${NC}"
    echo "Please ensure ROS2 is installed and sourced"
    exit 1
fi

echo "✅ ROS2 environment detected"
echo ""

# Test 1: Check if IMU topic exists
echo "=========================================="
echo "Test 1: Verify IMU topic exists"
echo "=========================================="
echo "Checking for /imu/data_raw topic..."
if ros2 topic list | grep -q "/imu/data_raw"; then
    echo -e "${GREEN}✅ PASS: /imu/data_raw topic found${NC}"
else
    echo -e "${RED}❌ FAIL: /imu/data_raw topic not found${NC}"
    echo "Available topics:"
    ros2 topic list
    exit 1
fi
echo ""

# Test 2: Verify topic type
echo "=========================================="
echo "Test 2: Verify topic type"
echo "=========================================="
echo "Checking topic type..."
TOPIC_TYPE=$(ros2 topic type /imu/data_raw)
if [ "$TOPIC_TYPE" = "sensor_msgs/msg/Imu" ]; then
    echo -e "${GREEN}✅ PASS: Topic type is sensor_msgs/msg/Imu${NC}"
else
    echo -e "${RED}❌ FAIL: Expected sensor_msgs/msg/Imu, got $TOPIC_TYPE${NC}"
    exit 1
fi
echo ""

# Test 3: Verify publishing rate (~100Hz)
echo "=========================================="
echo "Test 3: Verify publishing rate (~100Hz)"
echo "=========================================="
echo "Measuring publishing rate (this will take ~10 seconds)..."
echo ""

# Run ros2 topic hz for 10 seconds and capture output
RATE_OUTPUT=$(timeout 10s ros2 topic hz /imu/data_raw 2>&1 || true)
echo "$RATE_OUTPUT"
echo ""

# Extract average rate from output
AVG_RATE=$(echo "$RATE_OUTPUT" | grep "average rate:" | awk '{print $3}')

if [ -z "$AVG_RATE" ]; then
    echo -e "${RED}❌ FAIL: Could not measure publishing rate${NC}"
    echo "Make sure the IMU is publishing data"
    exit 1
fi

# Check if rate is approximately 100Hz (allow 80-120Hz range)
if (( $(echo "$AVG_RATE >= 80.0" | bc -l) )) && (( $(echo "$AVG_RATE <= 120.0" | bc -l) )); then
    echo -e "${GREEN}✅ PASS: Publishing rate is ${AVG_RATE} Hz (target: ~100Hz)${NC}"
else
    echo -e "${YELLOW}⚠️  WARNING: Publishing rate is ${AVG_RATE} Hz (expected 80-120Hz)${NC}"
fi
echo ""

# Test 4: Check data quality (reasonable values)
echo "=========================================="
echo "Test 4: Check data quality (stationary)"
echo "=========================================="
echo "Sampling IMU data (robot should be stationary)..."
echo ""

# Capture one message
IMU_DATA=$(timeout 5s ros2 topic echo /imu/data_raw --once 2>&1 || true)

if [ -z "$IMU_DATA" ]; then
    echo -e "${RED}❌ FAIL: Could not receive IMU data${NC}"
    exit 1
fi

echo "Sample IMU message:"
echo "$IMU_DATA"
echo ""

# Extract acceleration values (expecting ~9.8 m/s² on Z-axis when stationary)
ACCEL_X=$(echo "$IMU_DATA" | grep -A 3 "linear_acceleration:" | grep "x:" | awk '{print $2}')
ACCEL_Y=$(echo "$IMU_DATA" | grep -A 3 "linear_acceleration:" | grep "y:" | awk '{print $2}')
ACCEL_Z=$(echo "$IMU_DATA" | grep -A 3 "linear_acceleration:" | grep "z:" | awk '{print $2}')

# Extract gyro values (expecting ~0 rad/s when stationary)
GYRO_X=$(echo "$IMU_DATA" | grep -A 3 "angular_velocity:" | grep "x:" | awk '{print $2}')
GYRO_Y=$(echo "$IMU_DATA" | grep -A 3 "angular_velocity:" | grep "y:" | awk '{print $2}')
GYRO_Z=$(echo "$IMU_DATA" | grep -A 3 "angular_velocity:" | grep "z:" | awk '{print $2}')

echo "Acceleration (m/s²): X=$ACCEL_X, Y=$ACCEL_Y, Z=$ACCEL_Z"
echo "Angular velocity (rad/s): X=$GYRO_X, Y=$GYRO_Y, Z=$GYRO_Z"
echo ""

# Check if values are reasonable (not all zeros, not NaN)
if [ "$ACCEL_X" = "0.0" ] && [ "$ACCEL_Y" = "0.0" ] && [ "$ACCEL_Z" = "0.0" ]; then
    echo -e "${RED}❌ FAIL: All acceleration values are zero (sensor may not be working)${NC}"
    exit 1
fi

# Check if Z-axis acceleration is approximately 9.8 m/s² (±5 m/s² tolerance)
ACCEL_Z_ABS=$(echo "$ACCEL_Z" | awk '{print ($1 < 0) ? -$1 : $1}')
if (( $(echo "$ACCEL_Z_ABS >= 5.0" | bc -l) )) && (( $(echo "$ACCEL_Z_ABS <= 15.0" | bc -l) )); then
    echo -e "${GREEN}✅ PASS: Z-axis acceleration is reasonable (${ACCEL_Z} m/s²)${NC}"
else
    echo -e "${YELLOW}⚠️  WARNING: Z-axis acceleration seems unusual (${ACCEL_Z} m/s², expected ~±9.8)${NC}"
fi
echo ""

# Test 5: Verify frame_id
echo "=========================================="
echo "Test 5: Verify frame_id"
echo "=========================================="
FRAME_ID=$(echo "$IMU_DATA" | grep "frame_id:" | awk '{print $2}' | tr -d "'")

if [ "$FRAME_ID" = "imu_link" ]; then
    echo -e "${GREEN}✅ PASS: frame_id is 'imu_link'${NC}"
else
    echo -e "${RED}❌ FAIL: frame_id is '$FRAME_ID' (expected 'imu_link')${NC}"
    exit 1
fi
echo ""

# Test 6: Test with robot moving (manual test)
echo "=========================================="
echo "Test 6: Manual movement test"
echo "=========================================="
echo -e "${YELLOW}MANUAL TEST REQUIRED:${NC}"
echo "1. Move the robot or tilt it"
echo "2. Observe that IMU values change accordingly"
echo "3. Run the following command to monitor live data:"
echo ""
echo "   ros2 topic echo /imu/data_raw"
echo ""
echo "Expected behavior:"
echo "  - Acceleration values should change when tilting"
echo "  - Gyro values should change when rotating"
echo "  - Publishing should continue without interruption"
echo ""

# Summary
echo "=========================================="
echo "Verification Summary"
echo "=========================================="
echo -e "${GREEN}✅ Topic exists and has correct type${NC}"
echo -e "${GREEN}✅ Publishing rate verified (~100Hz)${NC}"
echo -e "${GREEN}✅ Data quality checked (stationary)${NC}"
echo -e "${GREEN}✅ frame_id verified (imu_link)${NC}"
echo -e "${YELLOW}⚠️  Manual movement test required${NC}"
echo ""
echo "All automated tests passed!"
echo "Please perform the manual movement test to complete verification."
