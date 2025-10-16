add_executable(${NAME}
        # Core
        main.cpp
        Agent.cpp
        uRosBridge.cpp
        uRosEntities.cpp
        pico_usb_transports.cpp
        freertos_allocators.c
        
        # Application Layer
        application/BlinkAgent.cpp
        application/ImuAgent.cpp
        application/MotorsAgent.cpp
        application/TB6612MotorsAgent.cpp
        application/RobotController.cpp
        # TODO: delete after successful micro-ROS-Agent connection-Test
        # application/vl6180xAgent.cpp

        # Hardware Abstraction Layer
        hal/GPIOInputMgr.cpp
        hal/GPIOObserver.cpp
        hal/hardware/motor_driver/MotorMgr.cpp
        hal/hardware/motor_driver/MotorPID.cpp
        hal/hardware/motor_driver/TB6612MotorMgr.cpp
        hal/hardware/motor_driver/TB6612MotorPID.cpp
        hal/hardware/imu-driver/ICM-20948/icm20948_spi.cpp
        # TODO: delete after successful micro-ROS-Agent connection-Test
        # hal/hardware/vl6180x/vl6180x_driver.cpp
        )

target_include_directories(${NAME} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(${NAME} 
	pico_stdlib 
	hardware_pwm
    hardware_i2c
    hardware_spi
	FreeRTOS-Kernel-Heap4
	micro_ros
	pico_rand
	eigen
	)

# create map/bin/hex file etc.
pico_add_extra_outputs(${NAME})

# enable usb output, enable uart output
pico_enable_stdio_usb(${NAME} 1)
pico_enable_stdio_uart(${NAME} 1)
target_compile_definitions(${NAME} PRIVATE
    PICO_DEFAULT_UART=0
    PICO_DEFAULT_UART_TX_PIN=0
    PICO_DEFAULT_UART_RX_PIN=1
)
