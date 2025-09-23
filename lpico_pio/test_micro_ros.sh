#!/bin/bash
# micro-ROS Test Script für Steel Robot

echo "=== micro-ROS Steel Robot Test ==="
echo

# 1. Test ob ROS2 verfügbar ist
echo "1. Prüfe ROS2 Installation..."
if command -v ros2 &> /dev/null; then
    echo "✓ ROS2 gefunden: $(ros2 --version)"
else
    echo "✗ ROS2 nicht gefunden - bitte installieren Sie ROS2 Humble"
    exit 1
fi

# 2. Test ob micro-ROS Agent verfügbar ist
echo
echo "2. Prüfe micro-ROS Agent..."
if command -v docker &> /dev/null; then
    echo "✓ Docker gefunden - Agent kann über Docker gestartet werden"
    echo "  Command: docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v6"
elif ros2 pkg list | grep -q micro_ros_agent; then
    echo "✓ micro-ROS Agent als ROS2 Package gefunden"
    echo "  Command: ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0"
else
    echo "⚠ micro-ROS Agent nicht gefunden - Installation erforderlich:"
    echo "  sudo apt install ros-humble-micro-ros-agent"
    echo "  oder verwenden Sie Docker"
fi

# 3. Test verfügbare USB Ports
echo
echo "3. Verfügbare serielle Ports:"
if [ -d "/dev" ]; then
    ls -la /dev/tty* 2>/dev/null | grep -E "(ttyACM|ttyUSB)" || echo "  Keine USB-Seriell-Adapter gefunden"
else
    echo "  /dev Verzeichnis nicht verfügbar (Windows?)"
fi

# 4. Test ROS2 Topics (falls Robot bereits läuft)
echo
echo "4. Teste aktive ROS2 Nodes..."
timeout 3 ros2 node list 2>/dev/null | grep -q steel_robot_node && {
    echo "✓ Steel Robot Node ist bereits aktiv!"
    echo
    echo "Verfügbare Topics:"
    ros2 topic list | grep robot
    echo
    echo "Node-Info:"
    ros2 node info /steel_robot_node
} || {
    echo "○ Steel Robot Node nicht aktiv (normal wenn Robot nicht verbunden)"
}

echo
echo "=== Test-Commands ==="
echo
echo "Robot-Status überwachen:"
echo "  ros2 topic echo /robot/status"
echo
echo "Heartbeat überwachen:"
echo "  ros2 topic echo /robot/heartbeat"
echo
echo "Robot bewegen:"
echo "  ros2 topic pub /robot/cmd_vel geometry_msgs/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'"
echo
echo "Notaus aktivieren:"
echo "  ros2 topic pub /robot/emergency_stop std_msgs/Bool '{data: true}'"
echo
echo "=== Setup-Commands ==="
echo
echo "PlatformIO Build:"
echo "  cd lpico_pio && pio run"
echo
echo "Upload Firmware:"
echo "  cd lpico_pio && pio run --target upload"
echo
echo "Serial Monitor:"
echo "  cd lpico_pio && pio device monitor --baud 115200"
echo
echo "micro-ROS Agent starten (Docker):"
echo "  docker run -it --rm -v /dev:/dev --privileged --net=host microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v6"