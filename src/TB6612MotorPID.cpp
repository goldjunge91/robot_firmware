/*
 * TB6612MotorPID.cpp
 *
 * TB6612FNG motor with PID control implementation
 *
 * Created on: October 9, 2025
 */

#include "TB6612MotorPID.h"
#include <math.h>

TB6612MotorPID::TB6612MotorPID(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                               uint8_t encoderAPin, uint8_t encoderBPin)
    : TB6612MotorMgr(in1Pin, in2Pin, pwmPin, encoderAPin, encoderBPin)
{
}

TB6612MotorPID::~TB6612MotorPID() {
}

void TB6612MotorPID::setSpeedRPM(float rpm, bool cw) {
    xSP = fabs(rpm);
    xCWTarget = cw;
}

void TB6612MotorPID::setSpeedRadPS(float rps, bool cw) {
    // Convert rad/s to RPM: rps * 60 / (2*PI)
    float rpm = (rps * 60.0f) / (2.0f * M_PI);
    setSpeedRPM(rpm, cw);
}

void TB6612MotorPID::configPID(float kP, float kI, float kD) {
    xKp = kP;
    xKi = kI;
    xKd = kD;
}

float TB6612MotorPID::pid(float &sp, float &pv, float &err,
                          float &p, float &i, float &d) {
    float e = sp - pv;
    
    p = xKp * e;
    i += xKi * e;
    d = xKd * (e - err);
    err = e;
    
    // Clamp integral term (anti-windup)
    if (i > 1.0f) i = 1.0f;
    if (i < -1.0f) i = -1.0f;
    
    float output = p + i + d;
    
    // Clamp output to [0, 1]
    if (output < 0.0f) output = 0.0f;
    if (output > 1.0f) output = 1.0f;
    
    return output;
}

float TB6612MotorPID::doPID() {
    // Get current speed
    xPV = getMovingAvgRPM();
    
    // If setpoint is zero, stop motor
    if (xSP < 0.1f) {
        setThrottle(0.0f, xCWTarget);
        xErr = 0.0f;
        xI = 0.0f;
        return 0.0f;
    }
    
    // Calculate PID output
    float throttle = pid(xSP, xPV, xErr, xP, xI, xD);
    
    // Apply to motor
    setThrottle(throttle, xCWTarget);
    
    return throttle;
}
