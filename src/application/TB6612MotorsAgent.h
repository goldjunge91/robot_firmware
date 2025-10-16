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
#include "hal/hardware/motor_driver/TB6612MotorPID.h"
#include "BaseMotorsAgentInterface.h"
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

/**
 * @brief Motor control agent for TB6612FNG dual motor driver
 * 
 * TB6612MotorsAgent provides complete motor control functionality for robots
 * using the TB6612FNG motor driver IC. It manages multiple motors with:
 * - PID-based speed control
 * - Rotary encoder feedback
 * - ROS2 joint state publishing
 * - Individual motor configuration
 * 
 * The agent runs as a FreeRTOS task and publishes motor states to ROS2 topics
 * for integration with navigation and control systems.
 * 
 * @note Supports up to NUM_MOTORS (typically 4) motors
 * @note Each motor requires 5 GPIO pins: IN1, IN2, PWM, ENC_A, ENC_B
 */
class TB6612MotorsAgent : public BaseMotorsAgent {
public:
	/**
	 * @brief Construct a new TB6612MotorsAgent object
	 * 
	 * Initializes the motor agent with no motors configured. Use addMotor()
	 * to configure each motor before starting the agent.
	 */
	TB6612MotorsAgent();
	
	/**
	 * @brief Destroy the TB6612MotorsAgent object
	 * 
	 * Cleans up motor resources and stops all motors.
	 */
	virtual ~TB6612MotorsAgent();

	/**
	 * @brief Add and configure a TB6612-controlled motor
	 * 
	 * Configures a motor at the specified index with its GPIO pin assignments.
	 * The motor will be controlled via the TB6612FNG driver using:
	 * - IN1/IN2 pins for direction control
	 * - PWM pin for speed control
	 * - Encoder pins for position/velocity feedback
	 * 
	 * @param index Motor index (0 to NUM_MOTORS-1, typically 0-3)
	 * @param in1Pin GPIO pin for TB6612 IN1 (direction control)
	 * @param in2Pin GPIO pin for TB6612 IN2 (direction control)
	 * @param pwmPin GPIO pin for TB6612 PWM (speed control)
	 * @param gpA GPIO pin for rotary encoder channel A
	 * @param gpB GPIO pin for rotary encoder channel B
	 * 
	 * @pre index must be less than NUM_MOTORS
	 * @pre All GPIO pins must be valid and not already in use
	 * @post The motor is initialized and ready for speed commands
	 * 
	 * @note Call this method before start() for each motor
	 * @note Motors are indexed 0-3 for typical 4-wheel configurations
	 * 
	 * @warning Duplicate indices will overwrite previous motor configuration
	 */
	void addMotor(uint index,
			uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
			uint8_t gpA, uint8_t gpB);

	/**
	 * @brief Get a specific motor by index
	 * 
	 * Returns a pointer to the TB6612MotorPID object for the specified motor.
	 * This provides access to motor state, encoder counts, and PID parameters.
	 * 
	 * @param index Motor index (0 to NUM_MOTORS-1)
	 * 
	 * @return TB6612MotorPID* Pointer to motor object, or NULL if not configured
	 * 
	 * @note The returned pointer is owned by the agent - do not delete it
	 * @note Always check for NULL before using the returned pointer
	 */
	virtual TB6612MotorPID * getMotor(uint index) override;

	/**
	 * @brief Configure PID parameters for a specific motor
	 * 
	 * Sets the PID controller gains for the specified motor. These parameters
	 * control how the motor responds to speed errors.
	 * 
	 * @param index Motor index (0 to NUM_MOTORS-1)
	 * @param kP Proportional gain (typical: 1.0)
	 * @param kI Integral gain (typical: 0.1)
	 * @param kD Derivative gain (typical: 0.5)
	 * 
	 * @pre Motor at index must be configured via addMotor()
	 * @post Motor uses new PID parameters for speed control
	 * 
	 * @note See BaseMotorsAgent::configPID() for parameter tuning guidance
	 */
	virtual void configPID(uint index,
			float kP, float kI, float kD) override;

	/**
	 * @brief Configure PID parameters for all motors
	 * 
	 * Sets the same PID parameters for all configured motors. Convenient
	 * for initial setup when all motors have similar characteristics.
	 * 
	 * @param kP Proportional gain for all motors
	 * @param kI Integral gain for all motors
	 * @param kD Derivative gain for all motors
	 * 
	 * @post All configured motors use the new PID parameters
	 * 
	 * @note Only affects motors that have been added via addMotor()
	 */
	virtual void configAllPID(float kP, float kI, float kD) override;

	/**
	 * @brief Set motor speed in revolutions per minute
	 * 
	 * Commands the motor to rotate at the specified RPM. This is a convenience
	 * method that converts RPM to radians per second internally.
	 * 
	 * @param index Motor index (0 to NUM_MOTORS-1)
	 * @param rpm Target speed in revolutions per minute
	 * @param cw Direction: true for clockwise, false for counter-clockwise
	 * 
	 * @pre Motor at index must be configured via addMotor()
	 * @post Motor PID controller targets the specified speed
	 * 
	 * @note Internally converts to radians/second: rps = rpm * 2π / 60
	 */
	void setSpeedRPM(uint index,
			float rpm, bool cw);

	/**
	 * @brief Set motor speed in radians per second
	 * 
	 * Commands the motor to rotate at the specified angular velocity.
	 * The PID controller will adjust motor power to maintain this speed.
	 * 
	 * @param index Motor index (0 to NUM_MOTORS-1)
	 * @param rps Target angular velocity in radians per second
	 * @param cw Direction: true for clockwise, false for counter-clockwise
	 * 
	 * @pre Motor at index must be configured via addMotor()
	 * @post Motor PID controller targets the specified speed
	 * 
	 * @note The actual speed converges to target over several PID iterations
	 * @note Out-of-bounds index will log an error and be ignored
	 */
	virtual void setSpeedRadPS(uint index, float rps, bool cw) override;

	/**
	 * @brief Create ROS2 publishing entities
	 * 
	 * Initializes the ROS2 publisher for joint states. This publisher sends
	 * motor position and velocity data to the /joint_states topic.
	 * 
	 * @param node Pointer to the ROS2 node
	 * @param support Pointer to the rclc support structure
	 * 
	 * @pre micro-ROS must be initialized
	 * @pre node and support must be valid
	 * @post Joint state publisher is ready to publish
	 * 
	 * @note Called automatically by uRosBridge during initialization
	 */
	virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

	/**
	 * @brief Destroy ROS2 publishing entities
	 * 
	 * Cleans up the joint state publisher and frees associated resources.
	 * 
	 * @param node Pointer to the ROS2 node
	 * @param support Pointer to the rclc support structure
	 * 
	 * @post Publisher is destroyed and resources are freed
	 * 
	 * @note Called automatically by uRosBridge during shutdown
	 */
	virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

	/**
	 * @brief Get the number of ROS2 entities
	 * 
	 * Returns the count of ROS2 entities (publishers/subscribers) managed
	 * by this agent. Used by uRosBridge for resource allocation.
	 * 
	 * @return uint Number of entities (typically 1 for joint state publisher)
	 */
	virtual uint getCount();

	/**
	 * @brief Get the number of executor handles needed
	 * 
	 * Returns the number of handles required by the ROS2 executor for this
	 * agent's entities. Used for executor sizing.
	 * 
	 * @return uint Number of handles needed (0 for publishers only)
	 * 
	 * @note Returns 0 because publishers don't need executor handles
	 */
	virtual uint getHandles();


	/**
	 * @brief Add entities to the ROS2 executor
	 * 
	 * Registers subscribers, timers, and guards with the ROS2 executor.
	 * This agent has no subscribers, so this is a no-op.
	 * 
	 * @param executor Pointer to the rclc executor
	 * 
	 * @note This agent only publishes, so nothing is added to executor
	 */
	virtual void addToExecutor(rclc_executor_t *executor);

protected:

	/**
	 * @brief Main task execution loop
	 * 
	 * Continuously updates motor PID controllers and publishes joint states
	 * to ROS2. Runs at approximately 50Hz to maintain responsive motor control.
	 * 
	 * @note This method runs in its own FreeRTOS task
	 * @note Uses vTaskDelay() to yield to other tasks
	 */
	virtual void run();


	/**
	 * @brief Get the required stack size for this agent
	 * 
	 * Returns the stack size needed for the motor control task.
	 * 
	 * @return configSTACK_DEPTH_TYPE Stack size in words
	 * 
	 * @note Stack size accounts for PID calculations and ROS2 publishing
	 */
	virtual configSTACK_DEPTH_TYPE getMaxStackSize();

	/**
	 * @brief Handle incoming subscription messages
	 * 
	 * Processes messages from subscribed ROS2 topics. This agent has no
	 * subscriptions, so this is a no-op.
	 * 
	 * @param msg Pointer to the received message
	 * @param context Subscription context information
	 * 
	 * @note Not used by this agent (no subscriptions)
	 */
	virtual void handleSubscriptionMsg(const void* msg, uRosSubContext_t* context);


private:

	/**
	 * @brief Initialize the joint state message structure
	 * 
	 * Allocates and configures the joint state message with motor names
	 * and array sizes.
	 */
	void initJointState();
	
	/**
	 * @brief Publish current joint states to ROS2
	 * 
	 * Updates the joint state message with current motor positions and
	 * velocities, then publishes to /joint_states topic.
	 */
	void pubJointState();


	/// Array of motor controller pointers (NULL if not configured)
	TB6612MotorPID *pMotors[NUM_MOTORS];

	/// ROS2 publisher for joint states
	rcl_publisher_t xPubJoint;
	
	/// Joint state message structure
	sensor_msgs__msg__JointState xJointStateMsg;
};

#endif /* FIRMWARE_SRC_TB6612MOTORSAGENT_H_ */
