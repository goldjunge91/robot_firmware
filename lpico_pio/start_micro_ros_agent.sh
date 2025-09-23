#!/bin/bash
# micro-ROS Agent startup script for Steel Robot
# This script starts the micro-ROS agent to communicate with the Pico

echo "=== Steel Robot micro-ROS Agent ==="
echo "Starting micro-ROS agent for Steel Robot firmware..."
echo ""

# Check if micro-ROS agent is installed
if ! command -v micro-ros-agent &> /dev/null; then
    echo "ERROR: micro-ros-agent not found!"
    echo ""
    echo "Install options:"
    echo "1. Using Snap (Ubuntu/Linux):"
    echo "   sudo snap install micro-ros-agent"
    echo ""
    echo "2. Using Docker:"
    echo "   docker run -it --rm -v /dev:/dev --privileged --net=host \\"
    echo "   microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200"
    echo ""
    echo "3. Using ROS 2 (if installed):"
    echo "   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0"
    echo ""
    exit 1
fi

# Default serial device - adjust as needed
SERIAL_DEV="/dev/ttyACM0"
BAUD_RATE="115200"

# Check if device exists
if [ ! -e "$SERIAL_DEV" ]; then
    echo "WARNING: Serial device $SERIAL_DEV not found!"
    echo "Available serial devices:"
    ls -la /dev/tty* | grep -E "(ACM|USB)" || echo "No USB serial devices found"
    echo ""
    echo "Please connect your Pico and adjust SERIAL_DEV in this script if needed."
    echo ""
fi

echo "Configuration:"
echo "  Serial Device: $SERIAL_DEV"
echo "  Baud Rate: $BAUD_RATE"
echo "  ROS 2 Distribution: humble"
echo ""
echo "Expected topics from Steel Robot:"
echo "  /steel_robot/heartbeat (std_msgs/Int32)"
echo "  /steel_robot/status (std_msgs/String)"
echo "  /steel_robot/cmd_vel (geometry_msgs/Twist) [subscriber]"
echo ""
echo "Starting agent..."
echo "Press Ctrl+C to stop"
echo ""

# Start the agent
micro-ros-agent serial --dev "$SERIAL_DEV" -b "$BAUD_RATE"