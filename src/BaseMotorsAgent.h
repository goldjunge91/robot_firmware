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

/**
 * @brief Abstract interface for motor control agents
 * 
 * BaseMotorsAgent defines the common interface for controlling multiple motors
 * in a robot system. It combines the Agent pattern (for FreeRTOS task management)
 * with uRosEntities (for ROS2 communication) to provide a complete motor control
 * subsystem.
 * 
 * Implementations must provide:
 * - Motor speed control with PID feedback
 * - Individual motor access
 * - PID parameter configuration
 * - ROS2 joint state publishing
 * 
 * @note This is a pure virtual interface - use concrete implementations like TB6612MotorsAgent
 */
class BaseMotorsAgent : public Agent, public uRosEntities {
public:
	/**
	 * @brief Destroy the BaseMotorsAgent object
	 */
	virtual ~BaseMotorsAgent() = default;

	/**
	 * @brief Get a specific motor by index
	 * 
	 * Returns a pointer to the motor manager for the specified motor index.
	 * This allows direct access to motor state and control.
	 * 
	 * @param index Motor index (0-based, typically 0-3 for 4-wheel robots)
	 * 
	 * @return MotorMgr* Pointer to the motor manager, or NULL if index is invalid
	 * 
	 * @note The returned pointer is owned by the agent - do not delete it
	 * @note Check for NULL before using the returned pointer
	 */
	virtual MotorMgr * getMotor(uint index) = 0;

	/**
	 * @brief Set the target speed for a motor in radians per second
	 * 
	 * Commands the specified motor to rotate at the given angular velocity.
	 * The PID controller will adjust motor power to maintain this speed.
	 * 
	 * @param index Motor index (0-based)
	 * @param rps Target angular velocity in radians per second
	 * @param cw Direction: true for clockwise, false for counter-clockwise
	 * 
	 * @pre The motor at the specified index must be initialized
	 * @post The motor's PID controller will target the specified speed
	 * 
	 * @note The actual speed will converge to the target over several PID iterations
	 * @note Invalid index values may be silently ignored or cause errors (implementation-dependent)
	 * 
	 * @warning Excessive speeds may damage motors or cause mechanical failure
	 */
	virtual void setSpeedRadPS(uint index, float rps, bool cw) = 0;

	/**
	 * @brief Configure PID parameters for a specific motor
	 * 
	 * Sets the proportional, integral, and derivative gains for the motor's
	 * PID controller. These parameters determine how aggressively the controller
	 * responds to speed errors.
	 * 
	 * @param index Motor index (0-based)
	 * @param kP Proportional gain (response to current error)
	 * @param kI Integral gain (response to accumulated error)
	 * @param kD Derivative gain (response to rate of error change)
	 * 
	 * @pre The motor at the specified index must be initialized
	 * @post The motor's PID controller uses the new parameters
	 * 
	 * @note Typical values: kP=1.0, kI=0.1, kD=0.5 (tune for your system)
	 * @note Higher kP increases responsiveness but may cause oscillation
	 * @note kI eliminates steady-state error but may cause overshoot
	 * @note kD dampens oscillations but amplifies noise
	 * 
	 * @warning Poorly tuned PID parameters can cause instability or poor performance
	 */
	virtual void configPID(uint index, float kP, float kI, float kD) = 0;

	/**
	 * @brief Configure PID parameters for all motors
	 * 
	 * Sets the same PID parameters for all motors in the system. This is
	 * convenient when all motors have similar characteristics.
	 * 
	 * @param kP Proportional gain for all motors
	 * @param kI Integral gain for all motors
	 * @param kD Derivative gain for all motors
	 * 
	 * @post All motors use the same PID parameters
	 * 
	 * @note Use this for initial configuration, then fine-tune individual motors if needed
	 * @note See configPID() for parameter descriptions
	 */
	virtual void configAllPID(float kP, float kI, float kD) = 0;
};

#endif /* FIRMWARE_SRC_BASEMOTORSAGENT_H_ */
