/*
 * TB6612MotorPID.h
 *
 * TB6612FNG motor with PID control
 * Combines TB6612 hardware control with PID speed regulation
 *
 * Created on: October 9, 2025
 */

#ifndef FIRMWARE_SRC_TB6612MOTORPID_H_
#define FIRMWARE_SRC_TB6612MOTORPID_H_

#ifdef UNIT_TEST
    #include "mock_tb6612_motor_mgr.h"
#else
    #include "TB6612MotorMgr.h"
#endif

class TB6612MotorPID : public TB6612MotorMgr {
public:
    /***
     * Constructor
     * @param in1Pin - TB6612 IN1 pin
     * @param in2Pin - TB6612 IN2 pin  
     * @param pwmPin - TB6612 PWM pin
     * @param encoderAPin - Encoder A
     * @param encoderBPin - Encoder B
     */
    TB6612MotorPID(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                   uint8_t encoderAPin, uint8_t encoderBPin);
    
    virtual ~TB6612MotorPID();
    
    /***
     * Set target speed in RPM
     * @param rpm - revolutions per minute
     * @param cw - direction
     */
    void setSpeedRPM(float rpm, bool cw);
    
    /***
     * Set target speed in radians per second
     * @param rps - radians per second
     * @param cw - direction
     */
    void setSpeedRadPS(float rps, bool cw);
    
    /***
     * Configure PID parameters
     * @param kP - proportional gain
     * @param kI - integral gain
     * @param kD - derivative gain
     */
    void configPID(float kP, float kI, float kD);
    
    /***
     * Execute PID control loop (call periodically)
     * @return current throttle value
     */
    float doPID();

protected:
    /***
     * PID calculation
     * @param sp - setpoint
     * @param pv - process variable
     * @param err - error accumulator
     * @param p - proportional term
     * @param i - integral term
     * @param d - derivative term
     * @return control output
     */
    float pid(float &sp, float &pv, float &err,
              float &p, float &i, float &d);

private:
    float xKp = 0.01;
    float xKi = 0.001;
    float xKd = 0.0;
    
    float xSP = 0.0;   // Setpoint (target RPM)
    float xPV = 0.0;   // Process variable (actual RPM)
    float xErr = 0.0;  // Error accumulator
    float xP = 0.0;    // Proportional term
    float xI = 0.0;    // Integral term
    float xD = 0.0;    // Derivative term
    
    bool xCWTarget = true;
};

#endif /* FIRMWARE_SRC_TB6612MOTORPID_H_ */
