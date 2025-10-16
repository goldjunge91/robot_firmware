/*
 * TB6612MotorMgr.cpp
 *
 * TB6612FNG-specific motor manager implementation
 *
 * Created on: October 9, 2025
 */

#include "TB6612MotorMgr.h"

TB6612MotorMgr::TB6612MotorMgr(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                               uint8_t encoderAPin, uint8_t encoderBPin)
    : MotorMgr(in1Pin, in2Pin, encoderAPin, encoderBPin)
    , xIN1Pin(in1Pin)
    , xIN2Pin(in2Pin)
    , xPWMPin(pwmPin)
    , xThrottle(0.0f)
    , xCW(true)
{
    // Configure IN1 and IN2 as GPIO outputs for direction control
    gpio_init(xIN1Pin);
    gpio_set_dir(xIN1Pin, GPIO_OUT);
    gpio_put(xIN1Pin, 0);
    
    gpio_init(xIN2Pin);
    gpio_set_dir(xIN2Pin, GPIO_OUT);
    gpio_put(xIN2Pin, 0);
    
    // Configure PWM pin for speed control
    gpio_set_function(xPWMPin, GPIO_FUNC_PWM);
    uint slice_num = pwm_gpio_to_slice_num(xPWMPin);
    pwm_set_enabled(slice_num, true);
    pwm_set_gpio_level(xPWMPin, 0);
}

TB6612MotorMgr::~TB6612MotorMgr() {
}

void TB6612MotorMgr::setThrottle(float percent, bool cw) {
    xThrottle = percent;
    xCW = cw;
    
    // Clamp throttle
    if (xThrottle < 0.0f) {
        xThrottle = 0.0f;
    }
    if (xThrottle > 1.0f) {
        xThrottle = 1.0f;
    }
    
    // Stop motor if throttle is zero
    if (xThrottle == 0.0f) {
        gpio_put(xIN1Pin, 0);
        gpio_put(xIN2Pin, 0);
        pwm_set_gpio_level(xPWMPin, 0);
        return;
    }
    
    // Calculate PWM value
    uint16_t pwm = (uint16_t)((float)(0xFFFF) * xThrottle);
    
    // Set direction via IN1/IN2 and speed via PWM
    if (cw) {
        // Clockwise: IN1=1, IN2=0
        gpio_put(xIN1Pin, 1);
        gpio_put(xIN2Pin, 0);
    } else {
        // Counter-clockwise: IN1=0, IN2=1
        gpio_put(xIN1Pin, 0);
        gpio_put(xIN2Pin, 1);
    }
    
    // Set speed via PWM
    pwm_set_gpio_level(xPWMPin, pwm);
}
