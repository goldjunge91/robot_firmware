#include "shooter_control.h"
#include <Arduino.h>
#include "config.h"

void shooter_control_init(void)
{
    pinMode(ESC1_PIN, OUTPUT);
    pinMode(ESC2_PIN, OUTPUT);
    pinMode(GEAR_PIN, OUTPUT);
}

void shooter_control_update(shooter_state_t *shooter)
{
    digitalWrite(ESC1_PIN, shooter->esc1_enabled ? HIGH : LOW);
    digitalWrite(ESC2_PIN, shooter->esc2_enabled ? HIGH : LOW);
    digitalWrite(GEAR_PIN, shooter->gear_state ? HIGH : LOW);
}