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
    // ============================================================================
    // PID CONTROL ALGORITHM
    // ============================================================================
    //
    // Purpose: Calculate control output to minimize error between setpoint and
    //          process variable using proportional, integral, and derivative terms.
    //
    // Why PID control?
    // - Proportional (P): Responds to current error - larger error = larger correction
    // - Integral (I): Eliminates steady-state error by accumulating past errors
    // - Derivative (D): Dampens oscillations by predicting future error trend
    //
    // The three terms work together:
    // - P provides immediate response but can overshoot
    // - I ensures we reach the target exactly but can cause instability
    // - D reduces overshoot and oscillation but is sensitive to noise
    //
    // Control equation:
    //   output = Kp*e + Ki*Σe + Kd*Δe
    //
    // Where:
    //   e  = error (setpoint - actual)
    //   Σe = accumulated error over time (integral)
    //   Δe = rate of change of error (derivative)
    //   Kp, Ki, Kd = tuning gains
    //
    // ============================================================================
    
    // Calculate current error
    // Positive error means we're going too slow, negative means too fast
    float e = sp - pv;
    
    // Proportional term: Immediate response proportional to current error
    // Why? Provides the main driving force to correct errors
    // Larger Kp = faster response but more overshoot
    p = xKp * e;
    
    // Integral term: Accumulate error over time
    // Why? Eliminates steady-state error (when P and D settle but don't reach target)
    // Without I, friction or load might prevent reaching exact setpoint
    // Larger Ki = faster elimination of steady-state error but can cause oscillation
    i += xKi * e;
    
    // Derivative term: Rate of change of error
    // Why? Predicts future error trend and dampens oscillations
    // If error is decreasing rapidly, D reduces output to prevent overshoot
    // Larger Kd = more damping but more sensitive to noise
    d = xKd * (e - err);
    
    // Store current error for next derivative calculation
    err = e;
    
    // Anti-windup: Clamp integral term to prevent excessive accumulation
    // Why? If motor is saturated (at max throttle), error keeps accumulating
    // When setpoint changes, this accumulated error causes huge overshoot
    // Clamping prevents this "integral windup" problem
    if (i > 1.0f) i = 1.0f;
    if (i < -1.0f) i = -1.0f;
    
    // Combine all three terms
    float output = p + i + d;
    
    // Clamp output to valid throttle range [0, 1]
    // Why? Motor driver only accepts 0-100% throttle
    // Negative values would reverse direction (handled separately)
    if (output < 0.0f) output = 0.0f;
    if (output > 1.0f) output = 1.0f;
    
    return output;
}

float TB6612MotorPID::doPID() {
    // ============================================================================
    // PID CONTROL LOOP EXECUTION
    // ============================================================================
    //
    // Purpose: Execute one iteration of the PID control loop to maintain target
    //          motor speed despite varying loads and conditions.
    //
    // Why call this repeatedly?
    // - PID is a feedback control loop - it must run continuously
    // - Each iteration measures current speed, calculates error, and adjusts throttle
    // - Typical call rate: 50-100Hz for motor control
    //
    // Control flow:
    // 1. Measure current motor speed (process variable)
    // 2. Check if we should stop (setpoint near zero)
    // 3. Calculate PID output based on error
    // 4. Apply throttle to motor
    //
    // ============================================================================
    
    // Step 1: Get current motor speed from encoder
    // Why moving average? Raw encoder readings are noisy
    // Averaging smooths out noise while maintaining responsiveness
    xPV = getMovingAvgRPM();
    
    // Step 2: Handle stop condition
    // Why check for near-zero instead of exactly zero?
    // - Floating point comparison with zero is unreliable
    // - Small setpoints (< 0.1 RPM) are effectively stopped
    // - Prevents PID from trying to maintain tiny speeds (wastes energy, causes jitter)
    //
    // When stopping, we also reset integral and error terms
    // Why? Prevents accumulated error from affecting next movement
    if (xSP < 0.1f) {
        setThrottle(0.0f, xCWTarget);
        xErr = 0.0f;  // Reset error for next movement
        xI = 0.0f;    // Reset integral to prevent windup
        return 0.0f;
    }
    
    // Step 3: Calculate PID control output
    // This computes how much throttle to apply based on:
    // - How far we are from target (proportional)
    // - How long we've been away from target (integral)
    // - How fast we're approaching target (derivative)
    float throttle = pid(xSP, xPV, xErr, xP, xI, xD);
    
    // Step 4: Apply calculated throttle to motor
    // Direction (xCWTarget) is set separately via setSpeedRPM/setSpeedRadPS
    // Why separate direction from speed? Simplifies PID logic - it only controls magnitude
    setThrottle(throttle, xCWTarget);
    
    return throttle;
}
