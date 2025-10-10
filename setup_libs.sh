#!/bin/bash
# Clone all required libraries

cd lib

echo "Cloning micro_ros_raspberrypi_pico_sdk..."
if [ ! -d "micro_ros_raspberrypi_pico_sdk" ]; then
    git clone -b humble https://github.com/goldjunge91/micro_ros_raspberrypi_pico_sdk.git
fi

echo "Cloning FreeRTOS-Kernel with RP2040 port..."
if [ ! -d "FreeRTOS-Kernel" ]; then
    # Use the Raspberry Pi fork which includes the RP2040 port
    git clone -b smp https://github.com/raspberrypi/FreeRTOS-Kernel
fi

echo "Cloning FreeRTOS..."
if [ ! -d "FreeRTOS" ]; then
    git clone -b 202411.00 https://github.com/FreeRTOS/FreeRTOS.git
fi

echo "Cloning eigen..."
if [ ! -d "eigen" ]; then
    git clone https://gitlab.com/libeigen/eigen
fi

echo "Cloning pico-distance-sensor..."
if [ ! -d "pico-distance-sensor" ]; then
    git clone https://github.com/dangarbri/pico-distance-sensor
fi

echo "All libraries cloned successfully!"
