#include "BlinkAgent.h"
#include "DDD.h"
#include "FreeRTOS.h"
// #include "HCSR04Agent.h"
#include "TB6612MotorsAgent.h"
#include "PubEntities.h"
#include "application/ImuAgent.h"
// #include "application/vl6180xAgent.hpp"
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


// {
//   "pico_pinmap": {
//     "UART0_TX": 0,
//       "UART0_RX" : 1,
//       "VL6180X_SDA" : 0,
//       "VL6180X_SCL" : 0,
//       "IMU_SPI0_MISO" : 16,
//       "IMU_SPI0_CS" : 17,
//       "IMU_SPI0_SCK" : 18,
//       "IMU_SPI0_MOSI" : 19,
//       "LED_PIN" : 25,
//       "M0_IN1" : 2,
//       "M0_IN2" : 3,
//       "M0_PWM" : 4,
//       "M0_ENC_A" : 5,
//       "M0_ENC_B" : 6,
//       "M1_IN1" : 7,
//       "M1_IN2" : 8,
//       "M1_PWM" : 9,
//       "M1_ENC_A" : 10,
//       "M1_ENC_B" : 11,
//       "M2_IN1" : 12,
//       "M2_IN2" : 13,
//       "M2_PWM" : 14,
//       "M2_ENC_A" : 15,
//       "M2_ENC_B" : 20,
//       "M3_IN1" : 21,
//       "M3_IN2" : 22,
//       "M3_PWM" : 23,
//       "M3_ENC_A" : 24,
//       "M3_ENC_B" : 26
//   },
//     "pwm_target_hz": 20000
// }

#ifndef ENABLE_DEBUG_HEARTBEAT
#define ENABLE_DEBUG_HEARTBEAT 1
#endif

#ifndef DEBUG_HEARTBEAT_INTERVAL_MS
#define DEBUG_HEARTBEAT_INTERVAL_MS 5000
#endif

// Standard Task priority
#define TASK_PRIORITY (tskIDLE_PRIORITY + 1UL)

// LED PAD to use
#define BLINK_LED_PAD 25
#define CONN_LED_PAD 26

// TB6612 Motor Driver Pins
// Note: Pin 0, 1 reserved for UART0
// Front Left Motor (Motor 0)
#define FRONT_LEFT_IN1 2        // Direction pin 1
#define FRONT_LEFT_IN2 3        // Direction pin 2
#define FRONT_LEFT_PWM 4        // Speed control (PWM)
#define FRONT_LEFT_ROTENC_A 5
#define FRONT_LEFT_ROTENV_B 6

// Front Right Motor (Motor 1)
#define FRONT_RIGHT_IN1 7        // Direction pin 1
#define FRONT_RIGHT_IN2 8        // Direction pin 2
#define FRONT_RIGHT_PWM 8        // Speed control (PWM)
#define FRONT_RIGHT_ROTENC_A 10
#define FRONT_RIGHT_ROTENV_B 11

// Rear Left Motor (Motor 2)
#define REAR_LEFT_IN1 12         // Direction pin 1
#define REAR_LEFT_IN2 13         // Direction pin 2
#define REAR_LEFT_PWM 14          // Speed control (PWM)
#define REAR_LEFT_ROTENC_A 15
#define REAR_LEFT_ROTENV_B 20

// Rear Right Motor (Motor 3)
#define REAR_RIGHT_IN1 21        // Direction pin 1
#define REAR_RIGHT_IN2 22        // Direction pin 2
#define REAR_RIGHT_PWM 23        // Speed control (PWM)
#define REAR_RIGHT_ROTENC_A 24
#define REAR_RIGHT_ROTENV_B 26

// PID
#define KP 1.0  // war 0.55
#define KI 0.1  // war 0.019
#define KD 0.5  // war 0.24

// IMU (SPI) Pins DONT CHANGE IT
#define IMU_SPI_PORT spi0
#define IMU_MISO_PIN 16  // AD0
#define IMU_CS_PIN 17    // NCS
#define IMU_SCK_PIN 18   // SCLK
#define IMU_MOSI_PIN 19  //  SDI

// // VL6180X (I2C) Pins DONT CHANGE IT
// #define VL6180X_I2C_PORT i2c1
// #define VL6180X_SDA_PIN 2
// #define VL6180X_SCL_PIN 3

char ROBOT_NAME[] = "robot_xl";

#if ENABLE_DEBUG_HEARTBEAT
static void debugHeartbeatTask(void* params)
{
  (void)params;
  const TickType_t delay_ticks = pdMS_TO_TICKS(DEBUG_HEARTBEAT_INTERVAL_MS);
  for (;;)
  {
    uint32_t uptime_ms = to_ms_since_boot(get_absolute_time());
    printf("[HEARTBEAT] Uptime: %lu ms\n", (unsigned long)uptime_ms);
    vTaskDelay(delay_ticks);
  }
}
#endif

/***
 * Main task to boot the other Agents
 * @param params - unused
 */
void mainTask(void* params)
{
  printf("mainTask started. Initializing agents...\n");

  static BlinkAgent blink(BLINK_LED_PAD);

  static TB6612MotorsAgent motors;
  motors.addMotor(0, FRONT_LEFT_IN1, FRONT_LEFT_IN2, FRONT_LEFT_PWM, FRONT_LEFT_ROTENC_A, FRONT_LEFT_ROTENV_B);
  motors.addMotor(1, FRONT_RIGHT_IN1, FRONT_RIGHT_IN2, FRONT_RIGHT_PWM, FRONT_RIGHT_ROTENC_A, FRONT_RIGHT_ROTENV_B);
  motors.addMotor(2, REAR_LEFT_IN1, REAR_LEFT_IN2, REAR_LEFT_PWM, REAR_LEFT_ROTENC_A, REAR_LEFT_ROTENV_B);
  motors.addMotor(3, REAR_RIGHT_IN1, REAR_RIGHT_IN2, REAR_RIGHT_PWM, REAR_RIGHT_ROTENC_A, REAR_RIGHT_ROTENV_B);
  motors.configAllPID(KP, KI, KD);

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
  imu_cfg.bus = IMU_SPI_PORT;
  imu_cfg.baudrate_hz = 1000 * 1000;  // 1MHz
  imu_cfg.cs_pin = IMU_CS_PIN;
  imu_cfg.sck_pin = IMU_SCK_PIN;
  imu_cfg.mosi_pin = IMU_MOSI_PIN;
  imu_cfg.miso_pin = IMU_MISO_PIN;
  static application::ImuAgent imu(imu_cfg);
  imu.setFrameId("imu_link");

  static DDD ddd;
  ddd.setMotorsAgent(&motors);
  // ddd.setHCSR04Agent(&range);
  ddd.setImuAgent(&imu);
  // ddd.setVl6180xAgent(&tof);

  // Starten der Agenten-Tasks
  printf("Starting BlinkAgent...\n");
  blink.start("Blink", TASK_PRIORITY);

  printf("Starting MotorsAgent...\n");
  motors.start("Motors", TASK_PRIORITY);

  // HCSR04Agent and VL6180X disabled for now
  // printf("Starting HCSR04Agent...\n");
  // range.start("Range", TASK_PRIORITY);

  // printf("Starting Vl6180xAgent...\n");
  // tof.start("VL6180X", TASK_PRIORITY);

  printf("Starting ImuAgent...\n");
  imu.start("IMU", TASK_PRIORITY);

  printf("Starting DDD Agent...\n");
  ddd.start("DDD", TASK_PRIORITY);

  // Starten der uROS Bridge
  printf("Starting uRosBridge...\n");
  static uRosBridge* bridge = uRosBridge::getInstance();
  bridge->setuRosEntities(&ddd);
  bridge->setLed(CONN_LED_PAD);
  bridge->start("Bridge", TASK_PRIORITY + 2);
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
  xTaskCreate(mainTask, "MainThread", 2048, NULL, TASK_PRIORITY, &task);

#if ENABLE_DEBUG_HEARTBEAT
  xTaskCreate(debugHeartbeatTask, "DbgBeat", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
#endif

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

  // UART0 debugging disabled for performance optimization (uncomment to enable)
  // stdio_uart_init_full(uart0, 115200, 0, 1);  // Leitet printf auf UART um

  sleep_ms(2000);
  printf("\n\n-- Booting %s firmware (UART0 debug disabled for performance) --\n", ROBOT_NAME);

  // Start tasks and scheduler
  const char* rtos_name = "FreeRTOS";
  printf("Starting %s on core 0...\n", rtos_name);
  vLaunch();

  return 0;  // Should not be reached
}
