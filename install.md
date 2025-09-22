# Raspberry Pi Pico firmware setup (single direct path)

Follow these exact steps on your development machine or Raspberry Pi host. This file provides one deterministic path — follow it line-by-line.

## 1. Install prerequisites

Run the following once to install required packages:

```bash
sudo apt update
sudo apt install -y cmake ninja-build gcc-arm-none-eabi libnewlib-arm-none-eabi \
    python3-venv python3-pip git g++ doxygen
```

## 2. Prepare workspace and SDKs (single canonical location)

All SDKs and tools will be placed under `$HOME/git_clone`.

```bash
# create location and ensure ownership

mkdir -p "$HOME/git_clone"
sudo chown -R "${SUDO_USER:-$(id -un)}":"${SUDO_USER:-$(id -un)}" "$HOME/git_clone"

cd "$HOME/git_clone"

# Clone Pico SDK (with submodules) into the expected path
git clone --recurse-submodules https://github.com/raspberrypi/pico-sdk.git "$HOME/git_clone/pico-sdk"

# Clone micro-ROS Raspberry Pi Pico SDK (use kilted branch which is used by examples)
git clone --depth 1 https://github.com/micro-ROS/micro_ros_raspberrypi_pico_sdk.git "$HOME/git_clone/micro_ros_raspberrypi_pico_sdk"

# Create a deterministic CMake import wrapper so downstream projects can include
# micro_ros_pico_sdk_import.cmake reliably. This file is created only when
# missing and points to the prebuilt static library shipped in this repo.
if [ ! -f "$HOME/git_clone/micro_ros_raspberrypi_pico_sdk/micro_ros_pico_sdk_import.cmake" ]; then
cat > "$HOME/git_clone/micro_ros_raspberrypi_pico_sdk/micro_ros_pico_sdk_import.cmake" <<'EOF'
set(MICRO_ROS_PICO_SDK_PATH "${MICRO_ROS_PICO_SDK_PATH}" CACHE PATH "Path to the micro-ROS Raspberry Pi Pico SDK")
if (NOT MICRO_ROS_PICO_SDK_PATH)
  set(MICRO_ROS_PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}" CACHE PATH "Path to the micro-ROS Raspberry Pi Pico SDK")
endif()

if (EXISTS "${MICRO_ROS_PICO_SDK_PATH}/libmicroros/libmicroros.a")
  set(_MICRO_ROS_LIB "${MICRO_ROS_PICO_SDK_PATH}/libmicroros/libmicroros.a")
elseif (EXISTS "${MICRO_ROS_PICO_SDK_PATH}/libmicroros.a")
  set(_MICRO_ROS_LIB "${MICRO_ROS_PICO_SDK_PATH}/libmicroros.a")
else()
  message(WARNING "micro-ROS static library not found under ${MICRO_ROS_PICO_SDK_PATH}")
endif()

add_library(micro_ros_pico_sdk STATIC IMPORTED GLOBAL)
set_target_properties(micro_ros_pico_sdk PROPERTIES IMPORTED_LOCATION "${_MICRO_ROS_LIB}")
target_include_directories(micro_ros_pico_sdk INTERFACE "${MICRO_ROS_PICO_SDK_PATH}/libmicroros/include" "${MICRO_ROS_PICO_SDK_PATH}/include")

function(micro_ros_pico_sdk_init)
  message(STATUS "micro_ros_pico_sdk_init() - import wrapper initialized (no-op)")
endfunction()
EOF
echo "Created micro_ros_pico_sdk_import.cmake in $HOME/git_clone/micro_ros_raspberrypi_pico_sdk"
fi
```

## 3. Export environment variables (persistent)

Add these lines to your shell RC and source them now so the current shell has them:

```bash
echo 'export PICO_SDK_PATH=$HOME/git_clone/pico-sdk' >> ~/.bashrc
echo 'export MICRO_ROS_PICO_SDK_PATH=$HOME/git_clone/micro_ros_raspberrypi_pico_sdk' >> ~/.bashrc
source ~/.bashrc
```

Also set them in the current shell immediately:

```bash
export PICO_SDK_PATH=$HOME/git_clone/pico-sdk
export MICRO_ROS_PICO_SDK_PATH=$HOME/git_clone/micro_ros_raspberrypi_pico_sdk
```

## 4. Verify the environment (required check)

Run this verification before attempting to build. If any line reports "missing", stop and re-run step 2.

```bash
echo "PICO_SDK_PATH=$PICO_SDK_PATH"
echo "MICRO_ROS_PICO_SDK_PATH=$MICRO_ROS_PICO_SDK_PATH"
ls -la "$PICO_SDK_PATH" || (echo "pico-sdk missing"; exit 1)
ls -la "$MICRO_ROS_PICO_SDK_PATH" || (echo "micro-ros pico sdk missing"; exit 1)
```

## 5. Configure and build the firmware

From the firmware directory run the canonical build commands:

```bash
cd /workspaces/my_steel-robot_ws/src/robot_firmware/firmware/mecabridge_pico
cmake -B build -G Ninja
cmake --build build
```

To enable UART instead of USB CDC transport for micro-ROS, run an explicit configured build:

```bash
cmake -B build -G Ninja -DMICRO_ROS_USE_UART_TRANSPORT=ON
cmake --build build
```

The firmware artifacts (`mecabridge_pico.uf2`, `.elf`, `.bin`) will be generated in `build/`.

## 6. Flash the Pico (deterministic)

Use the CMake flash target (no optional picotool step):

1. Hold the BOOTSEL button on the Pico and connect it via USB so it appears as a mass-storage device.
2. Run:

```bash
cd /workspaces/my_steel-robot_ws/src/robot_firmware/firmware/mecabridge_pico
cmake --build build --target flash
```

After flashing the Pico will reboot and enumerate as a USB CDC device (e.g. `/dev/ttyACM0`).

## 7. Start the micro-ROS agent on the Raspberry Pi

Ensure the Pico is connected to the Pi via USB. Create a stable symlink with a udev rule if desired (e.g. `/dev/robot_pico`).

Start the agent (native or Docker). Example using Docker:

```bash
sudo docker run --rm --net host \
  --device /dev/ttyACM0:/dev/ttyACM0 \
  husarion/micro-ros-agent:humble \
  serial --dev /dev/ttyACM0 -b 576000
```

Or the native agent if installed:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
micro_ros_agent serial --dev /dev/ttyACM0 -b 576000
```

## 8. Verify communication

On the Pi, in another terminal:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic list
ros2 topic echo /encoder_ticks
```

If the Pico publishes encoder ticks and the agent logs `micro-ROS client connected`, the connection is healthy.
