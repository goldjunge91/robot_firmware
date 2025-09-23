# Quickstart Guide for lpico_pio Firmware

This guide provides instructions on how to build, flash, and test the `lpico_pio` firmware.

## Prerequisites

- A Raspberry Pi Pico with the robot hardware connected.
- A computer with the Raspberry Pi Pico SDK and toolchain installed.
- A terminal or command prompt.

## Building the Firmware

1.  Navigate to the `lpico_pio` directory.
2.  Create a `build` directory: `mkdir build`
3.  Navigate to the `build` directory: `cd build`
4.  Run CMake: `cmake ..`
5.  Run Make: `make`

This will create the firmware image `lpico_pio.uf2` in the `build` directory.

## Flashing the Firmware

1.  Connect the Raspberry Pi Pico to your computer while holding down the `BOOTSEL` button.
2.  The Pico will appear as a USB mass storage device.
3.  Copy the `lpico_pio.uf2` file to the Pico.
4.  The Pico will automatically reboot and run the new firmware.

## Testing the Firmware

1.  Connect to the Pico's serial port using a serial monitor (e.g., `minicom`, `putty`).
2.  You should see log messages from the firmware.
3.  Use a ROS2 client to send commands to the robot and verify that it behaves as expected.
