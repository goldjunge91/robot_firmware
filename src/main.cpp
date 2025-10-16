#include "application/BlinkAgent.h"
#include "application/RobotController.h"
#include "FreeRTOS.h"
// #include "HCSR04Agent.h"
#include "application/TB6612MotorsAgent.h"
#include "application/ImuAgent.h"
// #include "application/vl6180xAgent.hpp"
#include "config/FirmwareConfig.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "task.h"
#include "uRosBridge.h"

#include <stdio.h>

extern "C" {
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_uart.h"
#include "pico/stdio_usb.h"  // Nötig für die manuelle Initialisierung
}

// Note: All configuration constants have been moved to config/FirmwareConfig.h
// This includes pin assignments, PID parameters, task priorities, and debug settings.

static void debugHeartbeatTask(void* params)
{
  (void)params;
  const TickType_t delay_ticks = pdMS_TO_TICKS(config::debug::kHeartbeatIntervalMs);
  for (;;)
  {
    uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
    printf("[HEARTBEAT] Uptime: %lu ms\n", (unsigned long)uptime_ms);
    vTaskDelay(delay_ticks);
  }
}

/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void* params)
{
  printf("mainTask started. Initializing agents...\n");

  static BlinkAgent blink(config::pins::kBlinkLed);

  static TB6612MotorsAgent motors;
  motors.addMotor(0, config::pins::kMotor0In1, config::pins::kMotor0In2, config::pins::kMotor0Pwm, 
                  config::pins::kMotor0EncA, config::pins::kMotor0EncB);
  motors.addMotor(1, config::pins::kMotor1In1, config::pins::kMotor1In2, config::pins::kMotor1Pwm, 
                  config::pins::kMotor1EncA, config::pins::kMotor1EncB);
  motors.addMotor(2, config::pins::kMotor2In1, config::pins::kMotor2In2, config::pins::kMotor2Pwm, 
                  config::pins::kMotor2EncA, config::pins::kMotor2EncB);
  motors.addMotor(3, config::pins::kMotor3In1, config::pins::kMotor3In2, config::pins::kMotor3Pwm, 
                  config::pins::kMotor3EncA, config::pins::kMotor3EncB);
  motors.configAllPID(config::pid::kProportional, config::pid::kIntegral, config::pid::kDerivative);

  // static HCSR04Agent range;
  // range.addSensor(0, "range_front");
  // range.addSensor(18, "range_back");

  // Konfiguration für den VL6180X Time-of-Flight Sensor (I2C)
  // hal::hardware::Vl6180x::Config tof_cfg{};
  // tof_cfg.bus = VL6180X_I2C_PORT;
  // tof_cfg.sda_pin = VL6180X_SDA_PIN;
  // tof_cfg.scl_pin = VL6180X_SCL_PIN;
  // static application::Vl6180xAgent tof(tof_cfg);

  // Konfiguration für die IMU (SPI)
  hal::hardware::Icm20948Simple::Config imu_cfg{};
  imu_cfg.bus = config::imu::kSpiBus;
  imu_cfg.baudrate_hz = config::imu::kSpiBaudrate;
  imu_cfg.cs_pin = config::pins::kImuCs;
  imu_cfg.sck_pin = config::pins::kImuSck;
  imu_cfg.mosi_pin = config::pins::kImuMosi;
  imu_cfg.miso_pin = config::pins::kImuMiso;
  static application::ImuAgent imu(imu_cfg);
  imu.setFrameId(config::imu::kFrameId);

  static RobotController robot;
  robot.setMotorsAgent(&motors);
  // robot.setHCSR04Agent(&range);
  robot.setImuAgent(&imu);
  // robot.setVl6180xAgent(&tof);

  // Starten der Agenten-Tasks (mit kleinen Delays für saubere UART-Ausgabe)
  printf("Starting BlinkAgent...\n");
  blink.start("Blink", config::robot::kTaskPriority);
  vTaskDelay(pdMS_TO_TICKS(100));

  printf("Starting MotorsAgent...\n");
  motors.start("Motors", config::robot::kTaskPriority);
  vTaskDelay(pdMS_TO_TICKS(100));

  // HCSR04Agent and VL6180X disabled for now
  // printf("Starting HCSR04Agent...\n");
  // range.start("Range", config::robot::kTaskPriority);
  // vTaskDelay(pdMS_TO_TICKS(100));

  // printf("Starting Vl6180xAgent...\n");
  // tof.start("VL6180X", config::robot::kTaskPriority);
  // vTaskDelay(pdMS_TO_TICKS(100));

  printf("Starting ImuAgent...\n");
  imu.start("IMU", config::robot::kTaskPriority);
  vTaskDelay(pdMS_TO_TICKS(100));

  printf("Starting RobotController Agent...\n");
  robot.start("Robot", config::robot::kTaskPriority);
  vTaskDelay(pdMS_TO_TICKS(100));

  // Starten der uROS Bridge
  printf("Starting uRosBridge...\n");
  static uRosBridge* bridge = uRosBridge::getInstance();
  bridge->setuRosEntities(&robot);
  bridge->setLed(config::pins::kConnectionLed);
  bridge->start("Bridge", config::robot::kTaskPriority + 2u);
  vTaskDelay(pdMS_TO_TICKS(100));
  printf("micro-ROS bridge task started.\n");

  printf("All agents started. MainTask will now suspend.\n");

  // Dieser Task hat seine Aufgabe erledigt und kann sich nun schlafen legen.
  for (;;)
  {
    vTaskDelay(portMAX_DELAY);
  }
}

/***
 * Launch the tasks and scheduler
 */
void vLaunch(void)
{
  // Erhöhe den Stack für den mainTask, um die Erstellung aller Objekte sicherzustellen
  TaskHandle_t task;
  xTaskCreate(mainTask, "MainThread", config::robot::kMainTaskStackSize, NULL, 
              config::robot::kTaskPriority, &task);

  if (config::debug::kEnableHeartbeat)
  {
    xTaskCreate(debugHeartbeatTask, "DbgBeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
  }

  /* Start the tasks and timer running. */
  vTaskStartScheduler();
}

/***
 * Main
 * @return
 */
int main(void)
{
  // Stellt sicher, dass die grundlegende Hardware initialisiert ist
  stdio_init_all();

  // USB manuell für micro-ROS initialisieren
  stdio_usb_init();  // Macht USB für den Transport verfügbar

  // UART0 for debugging (115200 baud, 8N1)
  stdio_uart_init_full(uart0, 115200, 0, 1);  // Leitet printf auf UART um

  sleep_ms(2000);
  printf("\n\n-- Booting %s firmware --\n", config::robot::kName);

  // Start tasks and scheduler
  const char* rtos_name = "FreeRTOS";
  printf("Starting %s on core 0...\n", rtos_name);
  vLaunch();

  return 0;  // Should not be reached
}