#ifndef CONFIG_H
#define CONFIG_H

// TB6612 Motor Driver Pins
#define STBY 28 // Common standby for both TB6612s

// Front-Left Motor (TB1 A)
#define FL_IN1 18
#define FL_IN2 19
#define FL_PWM 2
#define ENC_FL_A 8
#define ENC_FL_B 9

// Front-Right Motor (TB1 B)
#define FR_IN1 17
#define FR_IN2 20
#define FR_PWM 3
#define ENC_FR_A 10
#define ENC_FR_B 11

// Rear-Left Motor (TB2 A)
#define RL_IN1 21
#define RL_IN2 22
#define RL_PWM 4
#define ENC_RL_A 12
#define ENC_RL_B 13

// Rear-Right Motor (TB2 B)
#define RR_IN1 26
#define RR_IN2 27
#define RR_PWM 5
#define ENC_RR_A 6
#define ENC_RR_B 7

// Shooter Control
#define ESC1_PIN 14
#define ESC2_PIN 15
#define GEAR_PIN 16

#endif // CONFIG_H