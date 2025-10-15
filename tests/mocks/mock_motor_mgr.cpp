/**
 * @file mock_motor_mgr.cpp
 * @brief Mock implementation of MotorMgr
 */

#include "mock_motor_mgr.h"
#include <algorithm>

MotorMgr::MotorMgr(uint8_t gpCW, uint8_t gpCCW, uint8_t gpA, uint8_t gpB)
    : m_gpio_cw(gpCW), m_gpio_ccw(gpCCW), m_gpio_encoder_a(gpA), m_gpio_encoder_b(gpB) {
}

MotorMgr::~MotorMgr() {}

void MotorMgr::setThrottle(float throttle, bool cw) {
    m_throttle = std::max(0.0f, std::min(1.0f, throttle));
    m_direction_cw = cw;
    xCW = cw;
}

float MotorMgr::getThrottle() const { return m_throttle; }

void MotorMgr::handleRotate(bool cw) {
    m_position += cw ? 1 : -1;
}

float MotorMgr::getRPM() const { return m_current_rpm; }
float MotorMgr::getMovingAvgRPM() const { return m_moving_avg_rpm; }

void MotorMgr::setMovingAvgRPM(float rpm) {
    m_moving_avg_rpm = rpm;
    m_current_rpm = rpm;
}

int32_t MotorMgr::getPosition() const { return m_position; }
