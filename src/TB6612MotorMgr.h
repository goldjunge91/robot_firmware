/*
 * TB6612MotorMgr.h
 *
 * TB6612FNG-specific motor manager
 * Uses IN1/IN2/PWM control scheme instead of dual PWM
 *
 * Created on: October 9, 2025
 */

#ifndef FIRMWARE_SRC_TB6612MOTORMGR_H_
#define FIRMWARE_SRC_TB6612MOTORMGR_H_

#include "MotorMgr.h"

class TB6612MotorMgr : public MotorMgr {
public:
    /***
     * Constructor for TB6612 motor control
     * @param in1Pin - TB6612 IN1 pin (direction control)
     * @param in2Pin - TB6612 IN2 pin (direction control)
     * @param pwmPin - TB6612 PWM pin (speed control)
     * @param encoderAPin - Encoder A pin
     * @param encoderBPin - Encoder B pin
     */
    TB6612MotorMgr(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                   uint8_t encoderAPin, uint8_t encoderBPin);
    
    virtual ~TB6612MotorMgr();
    
    /***
     * Set throttle for TB6612 (replaces MotorMgr implementation)
     * @param percent - 0.0 to 1.0
     * @param cw - true for clockwise
     */
    void setThrottle(float percent, bool cw);

private:
    uint8_t xIN1Pin;
    uint8_t xIN2Pin;
    uint8_t xPWMPin;
    float xThrottle;
    bool xCW;
};

#endif /* FIRMWARE_SRC_TB6612MOTORMGR_H_ */
