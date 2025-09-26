# Firmware Pico-Native Refactoring Plan

This document tracks the refactoring of the firmware to remove Arduino/STM32 compatibility layers and create a clean, Pico-native implementation.

## Phase 1: Remove Compatibility Layers & Refactor Dependencies

- [x] Delete Arduino compatibility headers:
    - [x] `include/ArduinoCompat.h`
    - [x] `include/PicoSerial.h`
    - [x] `include/Wire.h`
    - [x] `include/pico_compat.h`
- [x] Delete unused sensor stubs for wrong hardware (BNO055):
    - [x] `include/Adafruit_BNO055.h`
    - [x] `include/Adafruit_Sensor.h`
- [x] Refactor `UartLib` to use native Pico SDK `hardware/uart`.
- [x] Refactor `bsp` (Board Support Package) to use native Pico SDK `hardware/i2c` and `hardware/gpio`.
- [x] Refactor `micro_ros_cfg` to use `printf` for logging instead of Arduino `Serial`.

## Phase 2: Integrate Core Peripherals

- [ ] **Integrate ICM-20948 IMU:**
    - [ ] In `src/main.cpp`, instantiate and initialize the `ImuDriver`.
    - [ ] In `src/main.cpp`, call the `ImuDriver`'s loop handler and pass data to the micro-ROS buffer.
    - [ ] Verify the `ICM20948Adapter` correctly uses the refactored I2C from the `bsp`.
- [ ] **Verify Power Board Communication:**
    - [ ] In `src/bsp.cpp`, ensure the `PowerBoardSerial` object is correctly initialized.
    - [ ] In `src/main.cpp`, add a call to the `PowerBoardSerial.UartProtocolLoopHandler()` to process battery data.

## Phase 3: Final Cleanup & Verification

- [ ] **Review `hardware_cfg.h`:**
    - [ ] Remove any remaining obsolete definitions not relevant to the Pico hardware.
    - [ ] Add comments to clarify pin assignments.
- [ ] **Define Missing Globals:**
    - [ ] Ensure `FirmwareModeTypeDef firmware_mode` is defined and initialized correctly in `main.cpp`.
- [ ] **Update Build System:**
    - [ ] Review `CMakeLists.txt` to remove references to any deleted files and ensure all sources are included.
- [ ] **Final Build Verification:**
    - [ ] Compile the entire project to ensure all changes integrate and build successfully.
    - [ ] Review the final code to confirm it adheres to the principles in `.specify/memory/constitution.md`.