/**
 * @file hardware_cfg.h
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef HARDWARE_CFG
#define HARDWARE_CFG

#include <stdint.h>

// Define all inputs/outputs etc.

/* OTHERS */
#define DEFAULT_FIRMWARE_MODE 2  // 0 - normal; 1 - error; 2 - debug
#define RTOS_FREQUENCY 1000      // hz
#define FREQ_TO_DELAY_TIME(freq) (TickType_t)(RTOS_FREQUENCY / freq * portTICK_PERIOD_MS)

// POWER OFF
#define SBC_ETH_CONNECT_TIMEOUT 10  // ms
#define POWEROFF_DELAY 5000         // ms

/* REAR PANEL */
#define GRN_LED 3
#define RD_LED 4
#define PUSH_BUTTON1 11
#define PUSH_BUTTON2 12

/* FAN	*/
#define FAN_PP_PIN 13
#define FAN_PWM_PIN 0
// #define FAN_PWM_TIMER TIM3
#define FAN_PWM_CHANNEL 3
#define FAN_PWM_FREQUENCY 1000
#define FAN_TEMP_THRSH_UP 35
#define FAN_TEMP_THRSH_DOWN 30

/* THERMISTOR NTC*/
#define NTC_SENS_PIN 26  // ADC0
#define NTC_SENS_C1 0.001112613927
#define NTC_SENS_C2 0.000237277392
#define NTC_SENS_C3 0.000000071670
#define NTC_PULLUP_RES 5230          // NTC pull up resisior
#define NTC_OFFSET_VAL (273.15 + 3)  // Kelvin to Celsius offset + calibration offset

/* PERIPHERALS */
#define EN_LOC_5V 13
#define DIP_SW 2

/* AUDIO */
#define AUDIO_SHDN 2
#define AUDIO_DAC_OUT 4
#define AUDIO_DAC_CH OUT1

/* POWER BOARD */
#define PWR_BRD_GPIO_INPUT 4
#define PWR_BRD_GPIO_OUTPUT 7
#define PWR_BRD_SERIAL Serial1
#define PWR_BRD_SERIAL_BAUDRATE 38400
#define PWR_BRD_SERIAL_RX 6
#define PWR_BRD_SERIAL_TX 5
#define PWR_BRD_SERIAL_CONFIG 0x06
#define PWR_BRD_SERIAL_TIMEOUT 1  // ms

/* SBC */
#define SBC_SERIAL Serial
#define SBC_SERIAL_BAUDRATE 460800
#define SBC_SERIAL_RX 10
#define SBC_SERIAL_TX 9

/* IMU */
#define IMU_I2C i2c0
#define IMU_SDA 0
#define IMU_SCL 1
#define IMU_GPIO_IT 2
#define IMU_SAMPLE_FREQ 25  // Hz
#define IMU_ID 0x37
#define IMU_ADDR_A 0x28
#define IMU_ADDR_B 0x29

/* PIXEL LED */
#define PIXEL_SPI spi0
#define PIXEL_MOSI 15
#define PIXEL_SCK 10
#define PIXEL_LENGTH 18
#define VIRTUAL_LED_LENGTH 10
#define PIXEL_SPI_SPEED 4000000
#define PIXEL_ANIMATION_FREQ 0.5

/* ETHERNET */
#define CLIENT_IP "192.168.77.3"
#define SBC_AGENT_IP "192.168.77.2"  // SBC
// #define SBC_AGENT_IP 	"192.168.77.5"	//External device
#define AGENT_PORT 8888
#define SHUTDOWN_PORT 3000

/* ETH LINK STATUS DEFINES */
#define ETH_LINK_STATUS_CONNECTED_BIT (1 << 0)  // if set - connected
#define ETH_LINK_STATUS_ERROR_BIT (1 << 1)

/* EEPROM */
#define EEPROM_BLOCK_ADDR_0 0x00
#define EEPROM_BLOCK_ADDR_1 0x01
#define EEPROM_BLOCK_ADDR_2 0x02
#define EEPROM_BLOCK_ADDR_3 0x03
#define EEPROM_BLOCK_ADDR_4 0x04
#define EEPROM_BLOCK_ADDR_5 0x05
#define EEPROM_BLOCK_ADDR_6 0x06
#define EEPROM_BLOCK_ADDR_7 0x07
#define EEPROM_DEV_ID 0x50
#define EEPROM_CONTROL_BYTE(DevId, BlockAddr) (DevId | BlockAddr)

// Board version eeprom defines
#define BOARD_VER_MEM_BLOCK 0x00
#define BOARD_VER_MEM_ADDR 0x00
#define BOARD_VER_MEM_SIZE 0x04
#define BOARD_VER_READ_ATTEMPTS 5

/* EXTERNAL PERIPHERALS */

// EXT SPI
#define EXT_SPI spi1
#define EXT_SPI_SCK 5
#define EXT_SPI_MISO 6
#define EXT_SPI_MOSI 5
// EXT I2C1
#define EXT_I2C1 i2c1
#define EXT_I2C1_SDA 7
#define EXT_I2C1_SCL 6
// EXT I2C2
#define EXT_I2C2 i2c0
#define EXT_I2C2_SDA 9
#define EXT_I2C2_SCL 8
// EXT Serial
#define EXT_SERIAL_EN_FLAG 1
#define EXT_SERIAL Serial2
#define EXT_SERIAL_BAUDRATE 115200
#define EXT_SERIAL_RX 9
#define EXT_SERIAL_TX 14
// EXT PWM1
// #define EXT_PWM1_TIM TIM9
#define EXT_PWM1_CH CH1
#define EXT_PWM1_PIN 5
// EXT PWM2
// #define EXT_PWM2_TIM TIM9
#define EXT_PWM2_CH CH2
#define EXT_PWM2_PIN 6
// EXT PWM3
// #define EXT_PWM3_TIM TIM12
#define EXT_PWM3_CH CH1
#define EXT_PWM3_PIN 14
// EXT ANALOG
#define EXT_ANALOG_IN1 27 // ADC1
#define EXT_ANALOG_IN2 28 // ADC2
// EXT GPIO
#define EXT_GPIO1 2
#define EXT_GPIO2 3
#define EXT_GPIO3 4

/* WATCHDOG */
#define WATCHDOG_TIMEOUT 20000000  // microseconds

/* BATTERY */

#define BATTERY_CELLS_SERIES 3
#define BATTERY_CELLS_PARALLEL 3
#define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE 1  // in unmeasured
#define BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE 1      // in unmeasured
// #define BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE
// (BATTERY_CELLS_PARALLEL * BATTERY_CELLS_SERIES) #define
// BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE (BATTERY_CELLS_PARALLEL *
// BATTERY_CELLS_SERIES)

typedef enum {
  unknown_status = 0,
  charging = 1,
  discharging = 2,
  not_charging = 3,
  full = 4
} BatteryStatusTypeDef;

typedef enum {
  unknown_health = 0,
  good = 1,
  overhaet = 2,
  dead = 3,
  overvoltage = 4,
  unspec_failure = 5,
  cold = 6,
  watchdog_timer_expire = 7,
  safety_timer_expire = 8
} BatteryHealthTypeDef;

typedef enum {
  unknown_type = 0,
  NIMH = 1,
  LION = 2,
  LIPO = 3,
  LIFE = 4,
  NICD = 5,
  LIMN = 6
} BatteryTechnologyTypeDef;

typedef struct
{
  // ROS battery msgs variables
  float voltage;
  float temperature;
  float current;
  float charge_current;
  float capacity;
  float design_capacity;
  float percentage;
  float cell_temperature[BATTERY_STATE_MSG_CELL_TEMPERATURE_ARRAY_SIZE];
  float cell_voltage[BATTERY_STATE_MSG_CELL_VOLTAGE_ARRAY_SIZE];
  BatteryStatusTypeDef status;
  BatteryHealthTypeDef health;
  BatteryTechnologyTypeDef technology;
  bool present;
} battery_state_queue_t;

/* FIRMWARE MODE */

typedef enum { fw_normal = 0, fw_error = 1, fw_debug = 2 } FirmwareModeTypeDef;

#endif /* HARDWARE_CFG */
