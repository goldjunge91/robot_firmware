/*
 * TB6612MotorsAgent.cpp
 *
 * Motor Agent for TB6612FNG motor driver
 */

#include "TB6612MotorsAgent.h"

#include "uRosBridge.h"
#include "config/FirmwareConfig.h"

#include <inttypes.h>
#include <cmath>

const char* tb6612_joint_names[NUM_MOTORS] = {
    "front_left_wheel_joint",    // Motor 0
    "front_right_wheel_joint",   // Motor 1  
    "rear_left_wheel_joint",     // Motor 2
    "rear_right_wheel_joint"     // Motor 3
};

TB6612MotorsAgent::TB6612MotorsAgent() {
	for (uint i=0; i < NUM_MOTORS; i++){
		pMotors[i] = NULL;
	}
}

TB6612MotorsAgent::~TB6612MotorsAgent() {
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			delete pMotors[i];
		}
	}
}

/***
 * Add TB6612 Motor
 * @param index - Index of the Motor
 * @param in1Pin - TB6612 IN1 pin (direction)
 * @param in2Pin - TB6612 IN2 pin (direction)
 * @param pwmPin - TB6612 PWM pin (speed)
 * @param gpA - RotEnc input A
 * @param gpB - RotEnc input B
 */
void TB6612MotorsAgent::addMotor(uint index,
                uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                uint8_t gpA, uint8_t gpB){
        if (index < NUM_MOTORS){
                pMotors[index] = new TB6612MotorPID(in1Pin, in2Pin, pwmPin, gpA, gpB);
        }
}

/***
 * Configure PID for motor
 * @param index - of the motor
 * @param kP
 * @param kI
 * @param kD
 */
void TB6612MotorsAgent::configPID(uint index,
		float kP, float kI, float kD){
	if (index >= config::robot::kNumMotors) {
		printf("[TB6612MotorsAgent] ERROR: Motor index %u out of bounds (max: %u)\n",
		       index, config::robot::kNumMotors - 1);
		return;
	}
	if (pMotors[index] != NULL){
		pMotors[index]->configPID(kP, kI, kD);
	}
}

/***
 * Configure PID for all the motors
 * @param kP
 * @param kI
 * @param kD
 */
void TB6612MotorsAgent::configAllPID(float kP, float kI, float kD){
	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL){
			pMotors[i]->configPID(kP, kI, kD);
		}
	}
}


/***
 * Set the speed of the motor to be controlled
 * @param index of the motor
 * @param rpm rev per minute
 * @param cw direction - true if clockwise
 */
void TB6612MotorsAgent::setSpeedRPM(uint index,
		float rpm, bool cw){
	if (index >= config::robot::kNumMotors) {
		printf("[TB6612MotorsAgent] ERROR: Motor index %u out of bounds (max: %u)\n",
		       index, config::robot::kNumMotors - 1);
		return;
	}
	if (pMotors[index] != NULL){
		pMotors[index]->setSpeedRPM(rpm, cw);
	}
}

/***
 * Set the speed of the motor to be controlled
 * @param index of the motor
 * @param rps radians per second
 * @param cw direction - true if clockwise
 */
void TB6612MotorsAgent::setSpeedRadPS(uint index,
		float rps, bool cw){
	if (index >= config::robot::kNumMotors) {
		printf("[TB6612MotorsAgent] ERROR: Motor index %u out of bounds (max: %u)\n",
		       index, config::robot::kNumMotors - 1);
		return;
	}
	if (pMotors[index] != NULL){
		if (rps >= 0.0){
			pMotors[index]->setSpeedRadPS(rps, cw);
		} else {
			pMotors[index]->setSpeedRadPS(fabs(rps), !cw);
		}
	}
}

/***
 * Run loop for the agent.
 */
void TB6612MotorsAgent::run(){

	initJointState();

	for (;;){
		for (uint i=0; i < NUM_MOTORS; i++){
			if (pMotors[i] != NULL){
				float err = pMotors[i]->doPID();
			}
		}

		pubJointState();

		vTaskDelay(200);
	}
}


/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE TB6612MotorsAgent::getMaxStackSize(){
	return 1024;
}


/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void TB6612MotorsAgent::createEntities(
		rcl_node_t *node,
		rclc_support_t *support){
	rclc_publisher_init_default(
		&xPubJoint,
		node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
		"joint_states");
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void TB6612MotorsAgent::destroyEntities(
		rcl_node_t *node,
		rclc_support_t *support){

	rcl_publisher_fini(&xPubJoint, node);
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint TB6612MotorsAgent::getCount(){
	return 0;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint TB6612MotorsAgent::getHandles(){
	return 1;
}


/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void TB6612MotorsAgent::addToExecutor(rclc_executor_t *executor){
	//NOP
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void TB6612MotorsAgent::handleSubscriptionMsg(
		const void* msg,
		uRosSubContext_t* context){
	//NOP
}


void TB6612MotorsAgent::initJointState(){
	sensor_msgs__msg__JointState__init(&xJointStateMsg);

	//Position
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.position, NUM_MOTORS);
	for (uint i = 0; i < NUM_MOTORS; i++) {
		xJointStateMsg.position.data[i] = 0.0;
	}
	xJointStateMsg.position.size = NUM_MOTORS;
	xJointStateMsg.position.capacity = NUM_MOTORS;

	//Velocity
	rosidl_runtime_c__double__Sequence__init(
			&xJointStateMsg.velocity, NUM_MOTORS);
	for (uint i = 0; i < NUM_MOTORS; i++) {
		xJointStateMsg.velocity.data[i] = 0.0;
	}
	xJointStateMsg.velocity.size = NUM_MOTORS;
	xJointStateMsg.velocity.capacity = NUM_MOTORS;

	//Name
	rosidl_runtime_c__String__Sequence__init(
			&xJointStateMsg.name, NUM_MOTORS);
	
	for (uint i=0; i < NUM_MOTORS; i++){
		if (!rosidl_runtime_c__String__assign(
				&xJointStateMsg.name.data[i], tb6612_joint_names[i])){
			printf("ERROR: Joint name assignment failed\n");
		}
	}
	xJointStateMsg.name.size=NUM_MOTORS;
	xJointStateMsg.name.capacity=NUM_MOTORS;
}


void TB6612MotorsAgent::pubJointState(){
	//Populate the Joint position message
	int64_t time = rmw_uros_epoch_nanos();

	xJointStateMsg.header.stamp.sec = time / 1000000000;
	xJointStateMsg.header.stamp.nanosec = time % 1000000000;

	for (uint i=0; i < NUM_MOTORS; i++){
		if (pMotors[i] != NULL) {
			xJointStateMsg.position.data[i] =
					pMotors[i]->getRadians() - M_PI;

			xJointStateMsg.velocity.data[i] =
					pMotors[i]->getAvgRadPerSec();
		}
	}
	if (!uRosBridge::getInstance()->publish(&xPubJoint,
			&xJointStateMsg,
			this,
			NULL)){
		printf("Joint Pub failed\n");
	}
}

/***
 * Return specific motor or NULL if none
 * @param index
 * @return
 */
TB6612MotorPID * TB6612MotorsAgent::getMotor(uint index){
        if (index >= config::robot::kNumMotors){
                printf("[TB6612MotorsAgent] ERROR: Motor index %u out of bounds (max: %u)\n",
                       index, config::robot::kNumMotors - 1);
                return NULL;
        }
        return pMotors[index];
}
