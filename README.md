# robot_firmware

Firmware for the robot digital board (Raspberry Pi Pico).

This directory holds the Pico firmware used as the digital board (encoders, motor drivers, IMU bridge, micro-ROS client).

Quick links and references
- micro-ROS agent (Docker): https://github.com/husarion/micro-ros-agent-docker
- micro-ROS Agent repo: https://github.com/husarion/micro-ROS-Agent
- micro-ROS Pico SDK: https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk

## Firmware layout (micro-ROS client)
- `firmware/mecabridge_pico/` — CMake-based Pico project with micro-ROS integration
  - `src/config.h` — authoritative pin map (mirrors `PINMAP.md`)
  - `src/motor_controller.*` — TB6612FNG motor driver routines using `PWM_MAX`, `OFFSET_*`, and trims
  - `src/encoder_reader.*` — quadrature decoder (GPIO interrupt based) feeding the ROS publisher
  - `src/micro_ros_support.*` — wraps the rclc entities (USB transport, publisher/subscriber, watchdog)

## Build (CMake toolchain)
1. Install the Pico SDK toolchain (`cmake`, `ninja`, `gcc-arm-none-eabi`, `picotool`).
2. Clone the SDKs and export the paths in your shell profile:
   ```bash
   export PICO_SDK_PATH=/opt/pico-sdk
   export MICRO_ROS_PICO_SDK_PATH=/opt/micro_ros_raspberrypi_pico_sdk
   ```
3. Configure and build:
   ```bash
   cd src/robot_firmware/firmware/mecabridge_pico
   cmake -B build -G Ninja
   cmake --build build
   ```
4. Flash via USB bootloader (`BOOTSEL` mode):
   ```bash
   picotool load -f build/mecabridge_pico.uf2
   ```

See `install.md` for the full installation walkthrough (SDK clone, environment exports, agent launch) and `next_steps.md` for the firmware task backlog.

micro-ROS interface
- Subscribes: `wheel_pwm` (`std_msgs/msg/Int32MultiArray`, 4 entries for FL/FR/RL/RR PWM commands)
- Publishes: `encoder_ticks` (`std_msgs/msg/Int32MultiArray`, encoder counts in tick units)
- Transport: USB-CDC by default (uses `set_microros_usb_transports()` from the micro-ROS Pico SDK). Define `MICRO_ROS_USE_UART_TRANSPORT` at configure time to switch to UART.
- Watchdog: motors are stopped if no command arrives within `WATCHDOG_MS` (300 ms).

Build
- See the `mecabridge_pico` subfolder for a CMake build wrapper. You may need to clone the Raspberry Pi Pico SDK and micro-ROS pico SDK into the paths expected by the CMakeLists.

```markdown name=robot_firmware/README.md
```markdown
# robot_firmware (Raspberry Pi Pico)

Kurz: Firmware für das Digital‑Board (Raspberry Pi Pico). Implementiert Encoder‑Lesung, Motorsteuerung, Sicherheits‑Watchdog und das Host‑Protokoll (USB‑CDC oder UART). Optional: micro-ROS Client.

Wichtige Verzeichnisse
- src/        — Haupt‑Quellcode (C / C++ / pico-sdk oder PlatformIO)
- include/    — Header & pinmap (pinmap.h)
- platformio.ini oder CMakeLists.txt — Build-Konfiguration
- docs/       — Protokoll & Board‑Spezifikationen
- README.md   — dieses Dokument

Schnellstart (Lokaler Build + Flash)
1) Voraussetzungen
   - PlatformIO oder Pico SDK Toolchain (cmake + arm gcc)
   - pyserial, esptool (je nach Board/usb)

2) Build & Flash (PlatformIO Beispiel)
   platformio run -e pico
   platformio run -t upload -e pico --upload-port /dev/ttyACM0

3) Flash Script (empfohlen)
   ./scripts/flash_firmware.sh --device /dev/ttyACM0 --board robot_pico

Kommunikationsprotokoll
- Host -> Pico: ASCII framed, Beispiel: `S v0 v1 v2 v3\r\n` (wheel velocities in rad/s)
- Pico -> Host: Encoder report: `R e0 e1 e2 e3 ts\r\n`
- Optional: CRC8 optional vor `\r\n`

Testing
- Unit tests (Host) mit simulierten encoder packets
- Integration: Host (ros2_control) gegen Pico im loopback / Hardware-in-the-loop

Wichtig: PINMAP.md in repo root ist autoritativ — halte board_config.h synchron.

## Analysis and recommended next steps

I created `firmware/mecabridge_pico/src/config.h` with the pinmap you provided. This pin mapping should be used by the Pico firmware sources (motor control, encoder ISR, ESC control).

Recommended immediate actions:
- Scaffold a minimal `CMakeLists.txt` and `main.c` that include `config.h` and implement a micro-ROS node which:
  - publishes encoder counts (e.g., as sensor_msgs/Int32MultiArray or a custom message)
  - subscribes to a velocity or PWM command topic and applies PWM to the TB6612 pins
- Decide how the Pico will expose micro-ROS transport to the host: USB CDC (recommended) or UART. USB CDC is convenient because the Pico enumerates as a serial device over USB.
- On the host (Raspberry Pi): run the micro-ROS agent. Docker image from Husarion is the easiest path. Make sure the Docker container has access to the Pico's serial device (use `--device /dev/ttyACM0`).
- Add a udev rule on the host to create a stable symlink for the Pico, e.g. `/dev/robot_pico`.

If you want, I can now scaffold the minimal `CMakeLists.txt` and `main.c` and add a tiny example publisher/subscriber using micro-ROS for Pico. Do you want me to scaffold the firmware app now?
````
