# Steel Robot micro-ROS Integration

## Quick Start Guide

### 1. Build and Upload Firmware

```bash
# Clean build (recommended after configuration changes)
pio run --target clean

# Build the firmware
pio run

# Upload to Pico (hold BOOTSEL button, then plug USB)
pio run --target upload

# Monitor serial output
pio device monitor --baud 115200
```

### 2. Start micro-ROS Agent

The robot firmware publishes/subscribes to these ROS topics:
- **Publishers:**
  - `/steel_robot/heartbeat` (std_msgs/Int32) - Heartbeat counter every 1 second
  - `/steel_robot/status` (std_msgs/String) - Status message every 5 seconds
- **Subscribers:**
  - `/steel_robot/cmd_vel` (geometry_msgs/Twist) - Velocity commands

#### Option A: Using Snap (Ubuntu/Linux)
```bash
# Install
sudo snap install micro-ros-agent

# Run
./start_micro_ros_agent.sh
```

#### Option B: Using Docker (Windows/Linux/Mac)
```bash
docker run -it --rm -v /dev:/dev --privileged --net=host \
  microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -b 115200
```

#### Option C: Using ROS 2 (if installed)
```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

### 3. Test ROS Communication

Open a new terminal and test the communication:

```bash
# List topics
ros2 topic list

# Echo heartbeat
ros2 topic echo /steel_robot/heartbeat

# Echo status
ros2 topic echo /steel_robot/status

# Send velocity command
ros2 topic pub /steel_robot/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2}}'
```

## Architecture

### Current Implementation (Phase 1)

- **Framework:** Arduino + micro-ROS Arduino library
- **Platform:** Raspberry Pi Pico
- **Transport:** Serial (USB)
- **ROS Distribution:** Humble
- **Build System:** PlatformIO

### Features Implemented

✅ micro-ROS integration with Arduino framework  
✅ Heartbeat publisher (1 Hz)  
✅ Status publisher (0.2 Hz)  
✅ Velocity command subscriber  
✅ LED heartbeat indicator  
✅ Serial debug output  
✅ Windows-compatible build  

### Next Steps (Phase 2)

🔄 Integration with motor drivers  
🔄 Encoder feedback publishers  
🔄 IMU data publishing  
🔄 Safety system integration  
🔄 Parameter server integration  

## Troubleshooting

### Build Issues

**"micro_ros_arduino library not found"**
- Check internet connection
- Run: `pio lib install mirs240x/micro_ros_arduino@^2.0.7-humble`

**"undefined reference to micro_ros_*"**
- Ensure build flags are correctly set in platformio.ini
- Clean and rebuild: `pio run --target clean && pio run`

### Runtime Issues

**"No micro-ROS agent connection"**
- Verify micro-ROS agent is running
- Check serial device path (Linux: /dev/ttyACM0, Windows: COM ports)
- Ensure baud rate matches (115200)

**"Topics not visible in ROS"**
- Wait 5-10 seconds after connecting
- Check agent output for connection messages
- Verify ROS_DOMAIN_ID matches (default: 0)

### Windows-Specific Notes

- Use Docker for micro-ROS agent if direct installation fails
- Serial device typically appears as COM port (e.g., COM3)
- May need driver installation for Pico recognition

## Configuration Details

### platformio.ini Key Settings

```ini
build_flags = 
    -DUROS_TRANSPORT_SERIAL
    -DMICRO_ROS_TRANSPORT_SERIAL
    -DRCL_LOGGING_ENABLED=0

lib_deps = 
    mirs240x/micro_ros_arduino@^2.0.7-humble
```

### Memory Usage

Current configuration uses approximately:
- Flash: ~180KB (of 2MB available)
- RAM: ~45KB (of 264KB available)

This leaves plenty of room for robot-specific functionality.

## Integration with Existing Robot Modules

The micro-ROS integration is designed to work with your existing robot modules:

- `motor_driver.cpp` - Will receive cmd_vel commands
- `encoder_reader.cpp` - Will publish odometry
- `safety.c` - Will publish safety status
- `shooter_control.c` - Will receive shooting commands

Example integration in main.cpp:
```cpp
// In cmd_vel_callback:
motor_driver_set_velocity(msg->linear.x, msg->angular.z);

// In main loop:
if (encoder_data_available()) {
    publish_odometry();
}
```