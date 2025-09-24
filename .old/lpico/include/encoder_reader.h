#ifndef ENCODER_READER_H
#define ENCODER_READER_H

#include "motor_types.h"

typedef struct {
    volatile int32_t count;
    volatile uint32_t last_update_ms;
    uint8_t direction;
} encoder_state_t;

typedef struct {
    encoder_state_t front_left;
    encoder_state_t front_right;
    encoder_state_t rear_left;
    encoder_state_t rear_right;
} robot_encoders_t;

void encoder_reader_init(void);
void encoder_reader_update(robot_encoders_t* encoders);

#endif // ENCODER_READER_H
