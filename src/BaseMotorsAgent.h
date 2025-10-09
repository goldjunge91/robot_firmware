/*
 * BaseMotorsAgent.h
 *
 * Abstract base class for motor agents
 */

#ifndef FIRMWARE_SRC_BASEMOTORSAGENT_H_
#define FIRMWARE_SRC_BASEMOTORSAGENT_H_

#include "pico/stdlib.h"
#include "Agent.h"
#include "MotorMgr.h"
#include "uRosEntities.h"

class BaseMotorsAgent : public Agent, public uRosEntities {
public:
	virtual ~BaseMotorsAgent() = default;

	/***
	 * Return specific motor or NULL if none
	 * @param index
	 * @return
	 */
	virtual MotorMgr * getMotor(uint index) = 0;

	/***
	 * Set the speed of the motor to be controlled
	 * @param index of the motor
	 * @param rps radians per second
	 * @param cw direction - true if clockwise
	 */
	virtual void setSpeedRadPS(uint index, float rps, bool cw) = 0;

	/***
	 * Configure PID for motor
	 * @param index - of the motor
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	virtual void configPID(uint index, float kP, float kI, float kD) = 0;

	/***
	 * Configure PID for all the motors
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	virtual void configAllPID(float kP, float kI, float kD) = 0;
};

#endif /* FIRMWARE_SRC_BASEMOTORSAGENT_H_ */
