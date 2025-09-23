// Pico pin configuration (keeps same macro names as legacy STM32/UNO config)
#pragma once

// ===== Laufzeit-Defaults =====
#define PWM_MAX            255
#define WATCHDOG_MS        300
#define LOOP_MS            10
#define DEADZONE_DEFAULT   30
#define SLEW_DEFAULT       8

// ===== Offset Motoren(pi pico) =====
#define OFFSET_FL (+1)
#define OFFSET_FR (+1)
#define OFFSET_RL (+1)
#define OFFSET_RR (+1)

// TB6612 wiring
#define STBY 28

#define FL_IN1 18
#define FL_IN2 19
#define FL_PWM 2
#define ENC_FL_A   8
#define ENC_FL_B   9

#define FR_IN1 17
#define FR_IN2 20
#define FR_PWM 3
#define ENC_FR_A   10
#define ENC_FR_B   11

#define RL_IN1 21
#define RL_IN2 22
#define RL_PWM 4
#define ENC_RL_A   12
#define ENC_RL_B   13

#define RR_IN1 26
#define RR_IN2 27
#define RR_PWM 5
#define ENC_RR_A   6
#define ENC_RR_B   7

#define TRIM_FL     0
#define TRIM_FR     0
#define TRIM_RL     0
#define TRIM_RR     0

#define ESC1_PIN   14
#define ESC2_PIN   15
#define GEAR_PIN   16
