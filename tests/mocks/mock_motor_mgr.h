/**
 * @file mock_motor_mgr.h
 * @brief Mock implementation of MotorMgr for unit testing
 */

#ifndef MOCK_MOTOR_MGR_H_
#define MOCK_MOTOR_MGR_H_

#include <cstdint>

class MotorMgr {
public:
    MotorMgr(uint8_t gpCW, uint8_t gpCCW, uint8_t gpA, uint8_t gpB);
    virtual ~MotorMgr();

    virtual void setThrottle(float throttle, bool cw);
    virtual float getThrottle() const;
    virtual void handleRotate(bool cw);
    virtual float getRPM() const;
    virtual float getMovingAvgRPM() const;
    void setMovingAvgRPM(float rpm);
    virtual int32_t getPosition() const;

protected:
    float m_throttle = 0.0f;
    bool m_direction_cw = true;
    float m_current_rpm = 0.0f;
    float m_moving_avg_rpm = 0.0f;
    int32_t m_position = 0;
    uint8_t m_gpio_cw, m_gpio_ccw, m_gpio_encoder_a, m_gpio_encoder_b;
};

#endif
