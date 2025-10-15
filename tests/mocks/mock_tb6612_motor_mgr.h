/**
 * @file mock_tb6612_motor_mgr.h
 * @brief Mock TB6612 motor driver manager
 */

#ifndef MOCK_TB6612_MOTOR_MGR_H_
#define MOCK_TB6612_MOTOR_MGR_H_

#include <cstdint>

class TB6612MotorMgr {
public:
    TB6612MotorMgr(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                   uint8_t encoderAPin, uint8_t encoderBPin);
    virtual ~TB6612MotorMgr();
    
    virtual void setSpeed(float speed);
    virtual void setThrottle(float throttle, bool forward);
    virtual float getThrottle() const;
    virtual void brake();
    virtual void coast();
    virtual float getRPM() const;
    virtual float getMovingAvgRPM() const;
    void setMovingAvgRPM(float rpm);
    virtual int32_t getPosition() const;
    
    bool getIN1State() const { return m_in1_state; }
    bool getIN2State() const { return m_in2_state; }
    float getPWMDuty() const { return m_pwm_duty; }

protected:
    bool m_in1_state = false;
    bool m_in2_state = false;
    float m_pwm_duty = 0.0f;
    float m_throttle = 0.0f;
    bool m_forward = true;
    float m_current_rpm = 0.0f;
    float m_moving_avg_rpm = 0.0f;
    int32_t m_position = 0;
    uint8_t m_pin_in1, m_pin_in2, m_pin_pwm, m_pin_encoder_a, m_pin_encoder_b;
    
    void updateControlSignals(float speed);
};

#endif
