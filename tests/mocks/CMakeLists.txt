# ==============================================================================
# Shared Mock Library
# ==============================================================================
# Provides mock implementations of firmware components for testing.

add_library(firmware_mocks STATIC
    mock_motor_mgr.cpp
    mock_tb6612_motor_mgr.cpp
)

target_include_directories(firmware_mocks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/..      # tests/
    ${CMAKE_CURRENT_SOURCE_DIR}         # tests/mocks/
    ${CMAKE_SOURCE_DIR}/src             # Firmware headers
    ${CMAKE_SOURCE_DIR}/src/config      # FirmwareConfig.h
)

# Pico-Header-Mocks (stdlib.h, etc.)
# These provide stub implementations of Pico SDK functions
target_include_directories(firmware_mocks PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/pico
)

target_compile_definitions(firmware_mocks PUBLIC
    UNIT_TEST
    PICO_PLATFORM=host
)

target_compile_features(firmware_mocks PUBLIC cxx_std_17)

# Enable warnings
target_compile_options(firmware_mocks PRIVATE
    -Wall
    -Wextra
)
