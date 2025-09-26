# --- Configuration ---
# Path to the pico-sdk. Should be available inside the container.
# You can override this by setting the PICO_SDK_PATH environment variable.
PICO_SDK_PATH ?= /opt/pico-sdk
# The name of your project's executable and generated UF2 file
TARGET = pico_micro_ros_example

# The mount point for the Pico inside the container when in BOOTSEL mode.
PICO_DRIVE_MOUNT ?= /pico

# Build directory
BUILD_DIR = build

# --- Build Targets ---
.PHONY: all build clean flash get_pico_mount

all: build

# Configure and build the project
build: clean compile

# Clean the build directory
clean:
	@echo "--- Cleaning build directory ---"
	rm -rf $(BUILD_DIR)
	@echo "--- Cleaning directory finished ---"
	@if [ -d "$(BUILD_DIR)" ]; then \
		# If the pico sdk lives inside the build dir, preserve it; otherwise remove build entirely. \
		BUILD_REAL=$$(realpath "$(BUILD_DIR)"); \
		PICO_REAL=$$(realpath "$(PICO_SDK_PATH)" 2>/dev/null || true); \
		case "$$PICO_REAL" in "$$BUILD_REAL"/*) \
			echo "Preserving pico-sdk at $$PICO_REAL inside $(BUILD_DIR)"; \
			# compute pico name relative to build and remove other entries in build \
			PICO_REL=$${PICO_REAL#$$BUILD_REAL/}; \
			for e in "$$BUILD_REAL"/*; do \
				name=$$(basename "$$e"); \
				if [ "$$name" != "$$PICO_REL" ]; then rm -rf "$$e"; fi; \
			done; \
		;; \
		*) rm -rf "$(BUILD_DIR)";; \
		esac; \
	else \
		echo "No build directory to clean."; \
	fi

# List connected USB devices
list_usb:
	@echo "--- Listing connected USB devices ---"
	@lsusb
	
compile:
	@echo "--- Building firmware using CMakeLists.docker.txt ---"
	@if [ -f CMakeLists.txt ]; then cp CMakeLists.txt CMakeLists.txt.bak.$(shell date +%Y%m%d_%H%M%S); fi
	cp CMakeLists.docker.txt CMakeLists.txt
	@echo "Using PICO_SDK_PATH=$(PICO_SDK_PATH)"
	@if [ ! -d "$(PICO_SDK_PATH)" ]; then \
		echo "Error: PICO_SDK_PATH not found at $(PICO_SDK_PATH)"; \
		echo "Usage hint: Ensure the Pico SDK is available at this path inside the container,"; \
		echo "or set the PICO_SDK_PATH environment variable when running 'make'."; \
		exit 1; \
	fi
	PICO_SDK_PATH=$(PICO_SDK_PATH) cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=Debug
	make -C $(BUILD_DIR)
	
compile2:
	@echo "--- Building firmware using CMakeLists.docker.txt ---"
# 	@if [ -f CMakeLists.txt ]; then cp CMakeLists.txt CMakeLists.txt.bak.$(shell date +%Y%m%d_%H%M%S); fi
# 	cp CMakeLists.docker.txt CMakeLists.txt
	@echo "Using PICO_SDK_PATH=$(PICO_SDK_PATH)"
	@if [ ! -d "$(PICO_SDK_PATH)" ]; then \
		echo "Error: PICO_SDK_PATH not found at $(PICO_SDK_PATH)"; \
		echo "Usage hint: Ensure the Pico SDK is available at this path inside the container,"; \
		echo "or set the PICO_SDK_PATH environment variable when running 'make'."; \
		exit 1; \
	fi
	PICO_SDK_PATH=$(PICO_SDK_PATH) cmake -B $(BUILD_DIR) -S . -DCMAKE_BUILD_TYPE=Debug
	make -C $(BUILD_DIR)

# Check for Pico in BOOTSEL mode and find its mount point. Fails if not found.
get_pico_mount:
	@echo "--- Checking for Pico in BOOTSEL mode (RPI-RP2) ---"
	@if [ -z "$$(lsblk -o LABEL,MOUNTPOINT | grep -w 'RPI-RP2' | awk '{print $$2}')" ]; then \
		echo "Error: Raspberry Pi Pico in BOOTSEL mode not found."; \
		echo "Please connect the Pico while holding the BOOTSEL button and try again."; \
		exit 1; \
	fi
	@echo "Pico found. Mount point will be detected automatically."


# Flash the firmware to the Pico
flash: build get_pico_mount
	@echo "--- Flashing Firmware ---"
	cp $(BUILD_DIR)/$(TARGET).uf2 $(PICO_DRIVE_MOUNT)
	@echo "Firmware successfully flashed to $(PICO_DRIVE_MOUNT)."


# Flash the firmware without rebuilding
piflash: get_pico_mount
	@echo "--- Flashing Firmware (without building) ---"
	@if [ ! -f "$(BUILD_DIR)/$(TARGET).uf2" ]; then \
		echo "Error: Firmware file not found at $(BUILD_DIR)/$(TARGET).uf2"; \
		echo "Run 'make build' first."; \
		exit 1; \
	fi
	cp $(BUILD_DIR)/$(TARGET).uf2 $(PICO_DRIVE_MOUNT)
	@echo "Firmware successfully flashed to $(PICO_DRIVE_MOUNT)."