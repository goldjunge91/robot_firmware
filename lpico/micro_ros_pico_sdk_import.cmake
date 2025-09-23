if (DEFINED ENV{MICRO_ROS_PICO_SDK_PATH} AND (NOT MICRO_ROS_PICO_SDK_PATH))
    set(MICRO_ROS_PICO_SDK_PATH $ENV{MICRO_ROS_PICO_SDK_PATH})
    message("Using MICRO_ROS_PICO_SDK_PATH from environment ('${MICRO_ROS_PICO_SDK_PATH}')")
endif ()

set(MICRO_ROS_PICO_SDK_PATH "${MICRO_ROS_PICO_SDK_PATH}" CACHE PATH "Path to the micro-ROS Raspberry Pi Pico SDK")

if (NOT MICRO_ROS_PICO_SDK_PATH)
    message(FATAL_ERROR "MICRO_ROS_PICO_SDK_PATH is not set. Export it to the root of micro_ros_raspberrypi_pico_sdk.")
endif ()

set(_MICRO_ROS_IMPORT_CANDIDATES
    ${MICRO_ROS_PICO_SDK_PATH}/micro_ros_pico_sdk_import.cmake
    ${MICRO_ROS_PICO_SDK_PATH}/cmake/micro_ros_pico_sdk_import.cmake
)

set(_MICRO_ROS_IMPORT_FILE "")
foreach(candidate ${_MICRO_ROS_IMPORT_CANDIDATES})
    if (EXISTS ${candidate})
        set(_MICRO_ROS_IMPORT_FILE ${candidate})
        break()
    endif ()
endforeach()

if (NOT _MICRO_ROS_IMPORT_FILE)
    message(FATAL_ERROR "Could not locate micro_ros_pico_sdk_import.cmake inside ${MICRO_ROS_PICO_SDK_PATH}. Check the path and the repository layout.")
endif ()

include(${_MICRO_ROS_IMPORT_FILE})
