/*
 * TB6612MotorsAgent.h
 *
 * Motor Agent for TB6612FNG motor driver
 * Separate implementation from standard MotorsAgent
 */

#ifndef FIRMWARE_SRC_TB6612MOTORSAGENT_H_
#define FIRMWARE_SRC_TB6612MOTORSAGENT_H_

#include "pico/stdlib.h"
#include "Agent.h"
#include "TB6612MotorPID.h"
#include "BaseMotorsAgent.h"
#include "uRosEntities.h"

extern"C"{
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
}

#ifndef NUM_MOTORS
#define NUM_MOTORS 4
#endif

class TB6612MotorsAgent : public BaseMotorsAgent {
public:
	TB6612MotorsAgent();
	virtual ~TB6612MotorsAgent();

	/***
	 * Add TB6612 Motor
	 * @param index - Index of the Motor
	 * @param in1Pin - TB6612 IN1 pin (direction)
	 * @param in2Pin - TB6612 IN2 pin (direction)
	 * @param pwmPin - TB6612 PWM pin (speed)
	 * @param gpA - RotEnc input A
	 * @param gpB - RotEnc input B
	 */
	void addMotor(uint index,
			uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
			uint8_t gpA, uint8_t gpB);

	/***
	 * Return specific motor or NULL if none
	 * @param index
	 * @return
	 */
	virtual TB6612MotorPID * getMotor(uint index) override;

	/***
	 * Configure PID for motor
	 * @param index - of the motor
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	virtual void configPID(uint index,
			float kP, float kI, float kD) override;

	/***
	 * Configure PID for all the motors
	 * @param kP
	 * @param kI
	 * @param kD
	 */
	virtual void configAllPID(float kP, float kI, float kD) override;

	/***
	 * Set the speed of the motor to be controlled
	 * @param index of the motor
	 * @param rpm rev per minute
	 * @param cw direction - true if clockwise
	 */
	void setSpeedRPM(uint index,
			float rpm, bool cw);

	/***
	 * Set the speed of the motor to be controlled
	 * @param index of the motor
	 * @param rps radians per second
	 * @param cw direction - true if clockwise
	 */
	virtual void setSpeedRadPS(uint index, float rps, bool cw) override;

	/***
	 * Create the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Destroy the publishing entities
	 * @param node
	 * @param support
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/***
	 * Provide a count of the number of entities
	 * @return number of entities >=0
	 */
	virtual uint getCount();

	/***
	 * Return the number of handles needed by the executor
	 * @return
	 */
	virtual uint getHandles();


	/***
	 * Add subscribers, guards and timers to the executor
	 * @param executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);

protected:

	/***
	 * Run loop for the agent.
	 */
	virtual void run();


	/***
	 * Get the static depth required in words
	 * @return - words
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

	/***
	 * Handle subscription msg
	 * @param msg
	 * @param localContext
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);


private:

	void initJointState();
	void pubJointState();


	TB6612MotorPID *pMotors[NUM_MOTORS];

	rcl_publisher_t xPubJoint;
	sensor_msgs__msg__JointState xJointStateMsg;
};

#endif /* FIRMWARE_SRC_TB6612MOTORSAGENT_H_ */
