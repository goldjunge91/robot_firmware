#include <Arduino.h>
#include "micro_ros_cfg.h"
#include "UartLib.h"
#include "ImuLib_cfg.h"
#include "hardware_cfg.h"

extern "C"
{
#include "usb_interface.h"
#include "control_loop.h"
#include "command_processor.h"
#include "watchdog_manager.h"
}

// Global buffer variables to replace FreeRTOS queues
double SetpointBuffer[4] = {0};
bool SetpointAvailable = false;
motor_state_queue_t MotorStateBuffer;
bool MotorStateAvailable = false;
imu_queue_t ImuBuffer;
bool ImuAvailable = false;
battery_state_queue_t BatteryStateBuffer;
bool BatteryStateAvailable = false;

void setup()
{
    usb_interface_init();
    control_loop_init(); // Re-enabled now that functions are available
    watchdog_manager_init();
}

void loop()
{
    command_processor_task();
    control_loop_update(); // Call the control loop update function
}