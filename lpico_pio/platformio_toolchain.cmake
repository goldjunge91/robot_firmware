include(CMakeForceCompiler)
set(CMAKE_SYSTEM_NAME Generic)

set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

SET (CMAKE_C_COMPILER_WORKS 1)
SET (CMAKE_CXX_COMPILER_WORKS 1)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_AR arm-none-eabi-ar)

set(CMAKE_C_FLAGS_INIT "-std=gnu11 -DMBED_TRAP_ERRORS_ENABLED=1 -Os -Wall -Wextra -Wno-missing-field-initializers -Wno-unused-parameter -c -fdata-sections -ffunction-sections -fmessage-length=0 -fno-exceptions -fomit-frame-pointer -funsigned-char -mcpu=cortex-m0plus -mthumb -iprefix/home/ros/.platformio/packages/framework-arduino-mbed/cores/arduino @/home/ros/.platformio/packages/framework-arduino-mbed/variants/RASPBERRY_PI_PICO/includes.txt -nostdlib -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "-Wvla -fno-rtti -std=gnu++14 -DMBED_TRAP_ERRORS_ENABLED=1 -Os -Wall -Wextra -Wno-missing-field-initializers -Wno-unused-parameter -c -fdata-sections -ffunction-sections -fmessage-length=0 -fno-exceptions -fomit-frame-pointer -funsigned-char -mcpu=cortex-m0plus -mthumb -iprefix/home/ros/.platformio/packages/framework-arduino-mbed/cores/arduino @/home/ros/.platformio/packages/framework-arduino-mbed/variants/RASPBERRY_PI_PICO/includes.txt -nostdlib -fno-rtti -DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

set(__BIG_ENDIAN__ 0)