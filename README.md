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
