# robot_firmware

## Overview

This package now targets two firmware generations:

- **`pico/`** – active Raspberry Pi Pico (RP2040) port using the Pico SDK and micro-ROS.
- **`legacy_stm32/`** – frozen STM32F4 firmware kept for reference while we migrate drivers and ROS interfaces.
All new development happens inside `pico/`. The legacy sources remain untouched so we can cross-check behaviours during the port.

## RP2040 firmware (current work)

### Layout

```
pico/
  CMakeLists.txt               # Pico SDK + micro-ROS build script
  pico_sdk_import.cmake        # Toolchain bootstrap (copied from setup_pico_sdk.sh)
  micro_ros_pico_sdk_import.cmake
  include/                     # Hardware abstraction headers
  src/                         # Firmware entry point + subsystem scaffolding
```

## 2. Kernfunktionen

- **Omnidirektionale Mobilität:** Dank des Mecanum-Radantriebs kann sich der Roboter frei in alle Richtungen bewegen (vorwärts/rückwärts, seitwärts, diagonal) und auf der Stelle drehen.

- **Autonome Navigation & Kartierung:** Mithilfe eines LiDAR-Sensors und eines 9-Achsen-IMU (Inertial Measurement Unit) ist der Roboter in der Lage, seine Umgebung zu kartieren (SLAM) und sich darin autonom zu lokalisieren und zu bewegen.

- **Hinderniserkennung:** Ein VL53L0X Time-of-Flight-Sensor liefert zusätzliche Distanzdaten zur Erkennung von nahen Objekten und zur Unterstützung der Navigation.

- **Intelligenter Nerf-Launcher:** Eine integrierte USB-Kamera ermöglicht die Implementierung von Computer-Vision-Algorithmen. Das primäre Anwendungsziel ist die **Gesichtserkennung**, um den Nerf-Launcher automatisch auf erkannte Personen auszurichten und auf Befehl abzufeuern.

- **Fernsteuerung & Telemetrie:** Der Roboter kann manuell über einen Xbox-Controller gesteuert werden. Ein Web-Dashboard visualisiert wichtige Telemetriedaten wie Batteriestatus und Sensordaten in Echtzeit.

## 3. Systemarchitektur

Die Architektur ist in zwei Steuerungsebenen unterteilt, um eine effiziente Aufgabenverteilung zu gewährleisten:

1. **High-Level-Steuerung (Raspberry Pi 4B):**

   - **Gehirn des Roboters:** Führt das ROS2-Framework aus.

   - **Aufgaben:** Verarbeitet Daten von LiDAR, Kamera und Abstandssensoren, führt SLAM-Algorithmen aus, plant Pfade, hostet das Web-Dashboard und führt die Gesichtserkennungs-Software aus.

   - **Kommunikation:** Sendet hochrangige Bewegungsbefehle (z.B. "fahre nach links mit 0,5 m/s") an die Low-Level-Steuerung.

2. **Low-Level-Steuerung (Raspberry Pi Pico):**

   - **Echtzeit-Controller:** Verantwortlich für die direkte, präzise Ansteuerung der Hardware.

   - **Aufgaben:** Empfängt Befehle vom Pi 4B und übersetzt sie in exakte PWM-Signale für die vier DC-Motortreiber (TB6612FNG). Steuert die Servos und Brushless-Motoren des Nerf-Launchers.

   - **Vorteil:** Entlastet den Raspberry Pi 4B von Echtzeitaufgaben und sorgt für eine zuverlässige und jitterfreie Motorsteuerung.

## 4. Technische Komponenten

| **Kategorie** | **Komponente** | **Zweck** |
| **Chassis & Antrieb** | 4x DC-Getriebemotoren (GM3865-520) mit Hall-Encodern | Kraftvoller Antrieb und Feedback zur Raddrehung |
|  | 4x 80mm Mecanum-Räder | Ermöglichen die omnidirektionale Bewegung |
|  | 4x TB6612FNG Motortreiber | Ansteuerung der DC-Motoren |
| **Steuerung & Sensorik** | Raspberry Pi 4B (8GB) | High-Level-Steuerung, ROS2, Computer Vision |
|  | Raspberry Pi Pico | Low-Level-Steuerung (Motoren, Servos) |
|  | Lidar LDS01RR | 360°-Umgebungsscans für SLAM |
|  | ICM-20948 (9-DoF IMU) | Erfassung von Beschleunigung, Rotation und Ausrichtung (Sensorfusion) |
|  | VL53L0X Time-of-Flight-Sensor | Präzise Abstandsmessung für Hinderniserkennung |
|  | USB-Kamera (1080p) | Video-Streaming und Input für die Gesichtserkennung |
| **Nerf-Launcher** | 2x RS2205 Brushless-Motoren & 2x 40A ESCs | Beschleunigung der Nerf-Darts |
|  | 1x Digital-Servo (22kg) & 1x 9g Servo | Zielen des Launchers (Pan/Tilt) |
| **Energieversorgung** | 3S Li-Ion Akku-Pack (18650 Zellen) | Mobile Stromversorgung |
|  | 3S Batterieschutzplatine (BMS) | Schutz vor Überladung, Tiefentladung und Kurzschluss |
|  | INA3221 Sensor | Überwachung von Spannung und Stromverbrauch |
| **Bedienung & Interface** | Xbox Controller | Manuelle Fernsteuerung |
|  | 10" / 5" Display | Lokale Anzeige von Statusinformationen oder Kamerabild |
|  | TM1637 LED-Anzeige | Schnelle Anzeige von Statuscodes oder Werten |

Key modules:

- `hardware_cfg.h` – pin mapping derived from `PINMAP.md`.
- `motors.*` – PWM + H-bridge control (PID TODO).
- `encoder_reader.*` – quadrature capture stub (PIO implementation pending).
- `ImuLib_cfg.*` – BNO055 I2C driver stub.
- `micro_ros_cfg.*` – micro-ROS publishers, subscribers, watchdog, services.
- `PixelLedLib_cfg.*` – WS2812 status LED skeleton.
- `UartLib.*` – optional UART telemetry channel (stub).

### Build & flash

1. Initialise the Pico SDK (once per checkout):

   ```bash
   ./setup_pico_sdk.sh
   ```

2. Configure and build:

   ```bash
   cmake -S src/robot_firmware/pico -B build/robot_firmware_pico
   cmake --build build/robot_firmware_pico
   ```

3. Flash (with Pico in BOOTSEL mode):

   ```bash
   picotool load -f build/robot_firmware_pico/robot_firmware_pico.uf2
   ```

   or use the convenience target:

   ```bash
   cmake --build build/robot_firmware_pico --target flash_robot_firmware
   ```

### Status

- Working: micro-ROS scaffolding compiles (publishers/subscribers/services wired up).
- Working: Motor PWM + direction GPIO initialisation.
- Pending: Encoder, IMU, power board, pixel strip logic still stubbed with TODOs.
- Pending: PID loop and watchdog enforcement pending hardware bring-up.
Track open porting items in `docs/robot_firmware_pico_port_plan.md`.

## Legacy STM32 firmware

The original STM32F4 codebase now lives in `legacy_stm32/` untouched. All PlatformIO, FreeRTOS tasks, and Husarion-specific integrations remain there for reference.
To rebuild or debug the historical firmware, cd into `legacy_stm32/` and use the existing PlatformIO workflow. No further updates are planned.

## Contribution guide

- Keep `PINMAP.md` aligned with `hardware_cfg.h` before wiring changes.
- Extend or replace the stub implementations before enabling features in ROS bringup.
- Document new hardware requirements inside `docs/` alongside firmware changes.
- Use conventional commits (`feat:`, `fix:`, etc.) and run `just clean` before pushing artifacts.

wsl -d ubuntu-22.04 -- bash -lc "export PICO_SDK_PATH=/home/marco/pico-sdk; set -euo pipefail; cd /mnt/wsl/docker-desktop-bind-mounts/Ubuntu-22.04/ca9dcf122dac17ed248e04d826612cb62f802e17963be8f5fc4e8849981882dd/src/robot_firmware/example_repo/micro_ros_raspberrypi_pico_sdk; rm -rf build; mkdir build; cd build; cmake .. -DCMAKE_BUILD_TYPE=Release; cmake --build . -- -j 1"

From WSL or a shell with PICO_SDK_PATH set (e.g. export PICO_SDK_PATH=$HOME/pico-sdk):
cd example_repo/micro_ros_raspberrypi_pico_sdk
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j 1
rm -rf build
mkdir build;
cd build;
cmake .. -DCMAKE_BUILD_TYPE=Release -D PICO_BOARD=pico || (cat CMakeFiles/CMakeError.log 2>/dev/null || true; exit 1); cmake --build . -- -j 1
export PICO_SDK_PATH=/home/marco/pico-sdk
cd /mnt/c/GIT/my_steel-robot_ws/src/robot_firmware/example_repo/micro_ros_raspberrypi_pico_sdk
rm -rf build && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DSKIP_PICOTOOL=ON
cmake --build . -- -j 2