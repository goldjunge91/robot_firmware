# lpico Firmware

This firmware is a port of the legacy STM32 firmware to the Raspberry Pi Pico.

## Building the Firmware

1.  Create a `build` directory: `mkdir build`
2.  Navigate to the `build` directory: `cd build`
3.  Run CMake: `cmake ..`
4.  Run Make: `make`

## Flashing the Firmware

1.  Connect the Pico in BOOTSEL mode.
2.  Copy the `lpico.uf2` file to the Pico's mass storage device.
