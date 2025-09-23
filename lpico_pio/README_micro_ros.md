# micro-ROS Integration für Steel Robot

## Übersicht

Diese Dokumentation beschreibt die vollständige Integration von micro-ROS in die Steel Robot Firmware. Das System bietet eine robuste Kommunikationsschnittstelle zwischen dem Mikrocontroller (Raspberry Pi Pico) und dem ROS2-Host-System.

## Architektur

### micro-ROS Node: `steel_robot_node`

Der Hauptknoten implementiert folgende Schnittstellen:

#### Publisher (Ausgehende Daten vom Robot)

| Topic              | Message Type      | Frequenz | Beschreibung                |
| ------------------ | ----------------- | -------- | --------------------------- |
| `/robot/heartbeat` | `std_msgs/Int32`  | 1 Hz     | Lebenszeichen des Roboters  |
| `/robot/status`    | `std_msgs/String` | 0.5 Hz   | Detaillierter Roboterstatus |
| `/robot/imu/data`  | `sensor_msgs/Imu` | 10 Hz    | IMU-Sensordaten             |

#### Subscriber (Eingehende Befehle an den Robot)

| Topic                   | Message Type          | Beschreibung                                   |
| ----------------------- | --------------------- | ---------------------------------------------- |
| `/robot/cmd_vel`        | `geometry_msgs/Twist` | Geschwindigkeitsbefehle für Differential Drive |
| `/robot/emergency_stop` | `std_msgs/Bool`       | Notaus-Signal                                  |

### Konfiguration

#### PlatformIO Build-Flags

```ini
build_flags = 
    -DUROS_TRANSPORT_SERIAL
    -DMICRO_ROS_TRANSPORT_SERIAL
    -DRMW_UXRCE_MAX_NODES=1
    -DRMW_UXRCE_MAX_PUBLISHERS=10
    -DRMW_UXRCE_MAX_SUBSCRIPTIONS=10
    -DRMW_UXRCE_MAX_SERVICES=5
    -DRMW_UXRCE_MAX_CLIENTS=5
    -DRMW_UXRCE_MAX_HISTORY=4
```

Diese Flags optimieren die micro-ROS Bibliothek für begrenzte Ressourcen.

## Installation und Setup

### Voraussetzungen

1. **PlatformIO** installiert
2. **ROS2 Humble** auf dem Host-System
3. **micro-ROS Agent** verfügbar

### Build-Prozess

#### Linux/WSL (Empfohlen für Source-Build)
```bash
cd lpico_pio
pio run
```

#### Windows (Precompiled Library)
```bash
# Aktivieren Sie in platformio.ini:
# mirs240x/micro_ros_arduino@^2.0.7-humble
pio run
```

### micro-ROS Agent starten

```bash
# Über Docker (empfohlen)
docker run -it --rm -v /dev:/dev --privileged --net=host \
    microros/micro-ros-agent:humble serial --dev /dev/ttyACM0 -v6

# Oder direkt installiert
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

## Verwendung

### Robot starten

1. Firmware auf Pico flashen: `pio run --target upload`
2. micro-ROS Agent starten (siehe oben)
3. Robot erscheint automatisch als ROS2-Node

### ROS2 Commands

```bash
# Node-Liste anzeigen
ros2 node list

# Topics anzeigen
ros2 topic list

# Heartbeat überwachen
ros2 topic echo /robot/heartbeat

# Robot-Status anzeigen
ros2 topic echo /robot/status

# Geschwindigkeitsbefehl senden
ros2 topic pub /robot/cmd_vel geometry_msgs/Twist \
  '{linear: {x: 0.5}, angular: {z: 0.1}}'

# Notaus aktivieren
ros2 topic pub /robot/emergency_stop std_msgs/Bool '{data: true}'
```

## Erweiterte Konfiguration

### Custom Message Types hinzufügen

1. Erstellen Sie das Verzeichnis `micro_ros_extra_packages/`
2. Fügen Sie Ihre .msg/.srv Dateien hinzu
3. Das Build-System erkennt diese automatisch

### Performance-Tuning

#### Memory Pool Größen anpassen

Bearbeiten Sie die Build-Flags in `platformio.ini`:

```ini
# Für mehr Publisher/Subscriber
-DRMW_UXRCE_MAX_PUBLISHERS=20
-DRMW_UXRCE_MAX_SUBSCRIPTIONS=20

# Für größere Message-History
-DRMW_UXRCE_MAX_HISTORY=8
```

#### Transport-Optimierung

```ini
# Für stabilere Verbindungen
-DUCLIENT_MAX_OUTPUT_RELIABLE_STREAMS=16
-DUCLIENT_MAX_INPUT_RELIABLE_STREAMS=16
```

## Debugging

### Serial Monitor

```bash
pio device monitor --baud 115200
```

Ausgabe zeigt:
- Initialisierungsstatus
- micro-ROS Verbindungsstatus
- Eingehende Befehle
- Fehlermeldungen

### ROS2 Debugging

```bash
# Node-Info anzeigen
ros2 node info /steel_robot_node

# Topic-Bandbreite messen
ros2 topic hz /robot/heartbeat

# Message-Inhalt inspizieren
ros2 topic echo /robot/status --once
```

### Häufige Probleme

#### "micro_ros_platformio not available"
- **Lösung**: Verwenden Sie WSL/Linux oder aktivieren Sie precompiled Library

#### Agent Verbindung fehlschlägt
- **Prüfen**: Correct USB port (`/dev/ttyACM0`)
- **Prüfen**: Baudrate (115200)
- **Lösung**: Agent neustarten

#### Memory-Fehler
- **Lösung**: Reduzieren Sie MAX_* Werte in Build-Flags
- **Alternative**: Optimieren Sie Message-Frequenzen

## Integration mit anderen Systemen

### ROS2 Launch Files

Erstellen Sie Launch-Files für automatisches Starten:

```python
# launch/steel_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            arguments=['serial', '--dev', '/dev/ttyACM0'],
            output='screen'
        )
    ])
```

### Navigation Stack Integration

Der Robot ist kompatibel mit ROS2 Navigation:
- `/robot/cmd_vel` für Navigation Commands
- `/robot/imu/data` für Odometry Fusion

## Weiterführende Entwicklung

### Hardware-Integration erweitern

- **IMU**: Echte BNO055 Sensor-Integration
- **Encoder**: Motor-Encoder für Odometry
- **Sensoren**: Zusätzliche Ultraschall/Lidar-Sensoren

### Software-Features

- **Diagnostics**: ROS2 Diagnostic-Messages
- **Parameters**: Dynamic Reconfigure
- **Services**: Remote Procedure Calls
- **Actions**: Long-running Tasks

## Support

Für Fragen und Issues:
- Überprüfen Sie die micro-ROS Dokumentation
- Konsultieren Sie PlatformIO Community
- Testen Sie mit einfachen Beispielen zuerst

---

*Diese Dokumentation wird regelmäßig mit neuen Features und Verbesserungen aktualisiert.*