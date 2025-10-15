/**
 * @file mock_tb6612_motor_mgr.cpp
 * @brief Mock implementation of TB6612MotorMgr
 */

#include "mock_tb6612_motor_mgr.h"
#include <algorithm>
#include <cmath>

TB6612MotorMgr::TB6612MotorMgr(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                               uint8_t encoderAPin, uint8_t encoderBPin)
    : m_pin_in1(in1Pin), m_pin_in2(in2Pin), m_pin_pwm(pwmPin),
    m_pin_encoder_a(encoderAPin), m_pin_encoder_b(encoderBPin) {
}

TB6612MotorMgr::~TB6612MotorMgr() {}

void TB6612MotorMgr::setSpeed(float speed) {
    speed = std::max(-1.0f, std::min(1.0f, speed));
    updateControlSignals(speed);
    m_throttle = std::abs(speed);
    m_forward = (speed >= 0.0f);
}

void TB6612MotorMgr::setThrottle(float throttle, bool forward) {
    m_throttle = std::max(0.0f, std::min(1.0f, throttle));
    m_forward = forward;
    float speed = forward ? m_throttle : -m_throttle;
    updateControlSignals(speed);
}

float TB6612MotorMgr::getThrottle() const { return m_throttle; }

void TB6612MotorMgr::brake() {
    m_in1_state = true;
    m_in2_state = true;
    m_pwm_duty = 1.0f;
    m_throttle = 0.0f;
}

void TB6612MotorMgr::coast() {
    m_in1_state = false;
    m_in2_state = false;
    m_pwm_duty = 0.0f;
    m_throttle = 0.0f;
}

float TB6612MotorMgr::getRPM() const { return m_current_rpm; }
float TB6612MotorMgr::getMovingAvgRPM() const { return m_moving_avg_rpm; }

void TB6612MotorMgr::setMovingAvgRPM(float rpm) {
    m_moving_avg_rpm = rpm;
    m_current_rpm = rpm;
}

int32_t TB6612MotorMgr::getPosition() const { return m_position; }

void TB6612MotorMgr::updateControlSignals(float speed) {
    m_pwm_duty = std::abs(speed);
    if (speed > 0.0f) {
        m_in1_state = true;
        m_in2_state = false;
    }
    else if (speed < 0.0f) {
        m_in1_state = false;
        m_in2_state = true;
    }
    else {
        m_in1_state = false;
        m_in2_state = false;
    }
}
