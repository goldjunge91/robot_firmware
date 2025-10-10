/*
 * RobotController.h
 *
 * Created on: 7 Aug 2023
 * Author: jondurrant
 * 
 * Renamed from DDD.h to RobotController.h for clarity
 */

#ifndef FIRMWARE_SRC_ROBOTCONTROLLER_H_
#define FIRMWARE_SRC_ROBOTCONTROLLER_H_

#include "Agent.h"
// TODO: delete after successful micro-ROS-Agent connection-Test
// #include "HCSR04Agent.h"
#include "BaseMotorsAgent.h"
#include "uRosEntities.h"

extern "C" {
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
}

#include <Eigen/Core>
#include <Eigen/Geometry>

#define TWIST_DEBUG

// Robot physical constants with explicit types and units
inline constexpr double kWheelRadius = 0.065;      // meters - radius of mecanum wheels
inline constexpr double kWheelDepth = 0.055;       // meters - depth/width of wheels
inline constexpr double kWheelsSeparation = 0.204; // meters - distance between left and right wheels
inline constexpr double kWheelsOffset = 0.010;     // meters - offset to Y center of robot from wheel axis

// Timeout constants
inline constexpr uint32_t kMaxTwistTimeMs = 1500u; // milliseconds - max time without twist command before stopping

// Deprecated macros for backward compatibility - use constexpr versions above
#define WHEEL_RADIUS kWheelRadius
#define WHEEL_DEPTH kWheelDepth
#define WHEELS_SEP kWheelsSeparation
#define WHEELS_OFFSET kWheelsOffset
#define MAX_TWIST_TIME_MS kMaxTwistTimeMs

namespace application {
class ImuAgent;
// TODO: delete after successful micro-ROS-Agent connection-Test
// class Vl6180xAgent;
}  // namespace application

struct RobotOdom {
    double x;
    double y;
    double a;
};

typedef struct RobotOdom RobotOdom_t;

class RobotController : public Agent, public uRosEntities {
public:
    /**
     * @brief Construct a new RobotController agent
     * 
     * Initializes the robot controller with default values.
     * Agent references must be set via setter methods before starting.
     */
    RobotController();
    
    /**
     * @brief Destroy the RobotController agent
     * 
     * Cleans up ROS2 entities and stops the agent task.
     */
    virtual ~RobotController();

    /**
     * @brief Set the motors agent for controlling robot movement
     * 
     * Configures the motor control interface used for executing velocity commands.
     * The RobotController uses this to translate /cmd_vel messages into individual
     * motor speeds using mecanum kinematics.
     * 
     * @param p Pointer to the motors agent instance
     * 
     * @pre p must not be NULL
     * @pre Must be called before start()
     * @post Motor commands will be sent to the configured agent
     * 
     * @note The RobotController does not take ownership of the pointer
     * @warning Passing NULL will log an error and be rejected
     */
    void setMotorsAgent(BaseMotorsAgent *p);

    // TODO: delete after successful micro-ROS-Agent connection-Test
    // void setHCSR04Agent(HCSR04Agent *p);

    /**
     * @brief Set the IMU agent for inertial measurement data
     * 
     * Configures the IMU interface for accessing orientation and angular velocity
     * data. This data is used for odometry calculations and published in the
     * odometry message.
     * 
     * @param p Pointer to the IMU agent instance
     * 
     * @pre p must not be NULL
     * @pre Must be called before start()
     * @post IMU data will be included in odometry messages
     * 
     * @note The RobotController does not take ownership of the pointer
     * @warning Passing NULL will log an error and be rejected
     */
    void setImuAgent(application::ImuAgent *p);

    // TODO: delete after successful micro-ROS-Agent connection-Test
    // void setVl6180xAgent(application::Vl6180xAgent *p);

    /**
     * @brief Create ROS2 publishing and subscription entities
     * 
     * Initializes the ROS2 entities for this agent:
     * - Publisher: /odom (nav_msgs/Odometry) - Robot odometry
     * - Subscriber: /cmd_vel (geometry_msgs/Twist) - Velocity commands
     * 
     * @param node Pointer to the ROS2 node
     * @param support Pointer to the rclc support structure
     * 
     * @pre micro-ROS must be initialized
     * @pre node and support must be valid
     * @post Odometry publisher and cmd_vel subscriber are ready
     * 
     * @note Called automatically by uRosBridge during initialization
     */
    virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

    /**
     * @brief Destroy ROS2 entities
     * 
     * Cleans up the odometry publisher and cmd_vel subscriber, freeing
     * associated resources.
     * 
     * @param node Pointer to the ROS2 node
     * @param support Pointer to the rclc support structure
     * 
     * @post All ROS2 entities are destroyed and resources freed
     * 
     * @note Called automatically by uRosBridge during shutdown
     */
    virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

    /**
     * @brief Get the number of ROS2 entities
     * 
     * Returns the count of ROS2 entities (publishers and subscribers) managed
     * by this agent. Used by uRosBridge for resource allocation.
     * 
     * @return uint Number of entities (2: odometry publisher + cmd_vel subscriber)
     */
    virtual uint getCount();

    /**
     * @brief Get the number of executor handles needed
     * 
     * Returns the number of handles required by the ROS2 executor for this
     * agent's subscriptions and timers.
     * 
     * @return uint Number of handles (1 for cmd_vel subscriber)
     * 
     * @note Publishers don't need executor handles, only subscribers
     */
    virtual uint getHandles();

    /**
     * @brief Add entities to the ROS2 executor
     * 
     * Registers the cmd_vel subscriber with the ROS2 executor so it can
     * receive and process velocity commands.
     * 
     * @param executor Pointer to the rclc executor
     * 
     * @pre createEntities() must have been called
     * @post cmd_vel subscriber is registered with executor
     * 
     * @note The executor will call handleSubscriptionMsg() when messages arrive
     */
    virtual void addToExecutor(rclc_executor_t *executor);

    /**
     * @brief Handle incoming cmd_vel messages
     * 
     * Processes velocity commands from the /cmd_vel topic and translates them
     * into individual motor speeds using mecanum wheel inverse kinematics.
     * 
     * The mecanum kinematics formula used:
     * - Front Left:  v_x - v_y - ω*L
     * - Front Right: v_x + v_y + ω*L
     * - Rear Left:   v_x + v_y - ω*L
     * - Rear Right:  v_x - v_y + ω*L
     * 
     * where L is the distance from robot center to wheel.
     * 
     * @param msg Pointer to the geometry_msgs/Twist message
     * @param context Subscription context (contains callback info)
     * 
     * @pre Motors agent must be configured via setMotorsAgent()
     * @post Motor speeds are updated to achieve commanded velocity
     * @post Timestamp is recorded for timeout detection
     * 
     * @note If no cmd_vel is received for MAX_TWIST_TIME_MS, motors stop
     * @note Linear velocities are in m/s, angular velocity in rad/s
     */
    virtual void handleSubscriptionMsg(const void *msg, uRosSubContext_t *context);

protected:
    /**
     * @brief Main task execution loop
     * 
     * Continuously updates odometry calculations and publishes to ROS2.
     * Runs at approximately 10Hz (100ms delay) to balance accuracy with
     * CPU usage.
     * 
     * The loop performs:
     * 1. Update odometry from motor encoders
     * 2. Publish odometry message to /odom
     * 3. Check for cmd_vel timeout and stop if needed
     * 4. Delay 100ms before next iteration
     * 
     * @note This method runs in its own FreeRTOS task
     * @note Uses vTaskDelay() to yield to other tasks
     */
    virtual void run();

    /**
     * @brief Get the required stack size for this agent
     * 
     * Returns the stack size needed for the RobotController task, accounting for
     * odometry calculations, ROS2 message handling, and Eigen matrix operations.
     * 
     * @return configSTACK_DEPTH_TYPE Stack size in words
     * 
     * @note Stack size must accommodate Eigen library usage
     */
    virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
    /**
     * @brief Update odometry from motor encoder data
     * 
     * Calculates robot position and orientation by integrating motor encoder
     * velocities using mecanum wheel forward kinematics. Updates both motor
     * odometry and fused robot odometry estimates.
     * 
     * @note Called at 10Hz from run() loop
     * @note Uses Eigen for rotation matrix calculations
     */
    void updateOdom();

    /**
     * @brief Publish odometry message to ROS2
     * 
     * Publishes the current robot pose (position and orientation) and twist
     * (linear and angular velocities) to the /odom topic.
     * 
     * @note Called at 10Hz from run() loop
     * @note Includes IMU orientation if IMU agent is configured
     */
    void publishOdom();

    /**
     * @brief Initialize the odometry message structure
     * 
     * Allocates and configures the nav_msgs/Odometry message with frame IDs
     * and covariance matrices.
     */
    void setupOdomMsg();
    
    /**
     * @brief Initialize the twist message structure
     * 
     * Allocates and configures the geometry_msgs/Twist message for receiving
     * velocity commands.
     */
    void setupTwistMsg();

    /**
     * @brief Stop all robot motors
     * 
     * Commands all motors to zero velocity. Called when cmd_vel timeout
     * is detected or on emergency stop.
     * 
     * @post All motors are commanded to 0 rad/s
     */
    void robotStop();

    BaseMotorsAgent *pMotorsAgent = NULL;
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // HCSR04Agent *pHCSR04Agent = NULL;
    application::ImuAgent *pImuAgent = NULL;
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // application::Vl6180xAgent *pVl6180xAgent = NULL;

    RobotOdom_t xMotorsOdom;
    RobotOdom_t xRobotOdom;
    RobotOdom_t xRobotVelocity;

    uint32_t xLastVelocityTime = 0u;  // milliseconds - last velocity calculation time

    rcl_publisher_t xPubOdom;
    nav_msgs__msg__Odometry xOdomMsg;

    rcl_subscription_t xSubTwist;
    uRosSubContext_t xSubTwistContext;
    geometry_msgs__msg__Twist xTwistMsg;

    uint32_t xLastTwistTimestamp = 0u;  // milliseconds - last received twist command time
};

#endif /* FIRMWARE_SRC_ROBOTCONTROLLER_H_ */
/*
 * DDD.h
 *
 * Created on: 7 Aug 2023
 * Author: jondurrant
 */

#ifndef FIRMWARE_SRC_DDD_H_
#define FIRMWARE_SRC_DDD_H_

#include "Agent.h"
// TODO: delete after successful micro-ROS-Agent connection-Test
// #include "HCSR04Agent.h"
#include "BaseMotorsAgent.h"
#include "uRosEntities.h"

extern "C" {
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "rosidl_runtime_c/string_functions.h"

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/time_sync.h>
}

#include <Eigen/Core>
#include <Eigen/Geometry>

#define TWIST_DEBUG

// Robot physical constants with explicit types and units
inline constexpr double kWheelRadius = 0.065;      // meters - radius of mecanum wheels
inline constexpr double kWheelDepth = 0.055;       // meters - depth/width of wheels
inline constexpr double kWheelsSeparation = 0.204; // meters - distance between left and right wheels
inline constexpr double kWheelsOffset = 0.010;     // meters - offset to Y center of robot from wheel axis

// Timeout constants
inline constexpr uint32_t kMaxTwistTimeMs = 1500u; // milliseconds - max time without twist command before stopping

// Deprecated macros for backward compatibility - use constexpr versions above
#define WHEEL_RADIUS kWheelRadius
#define WHEEL_DEPTH kWheelDepth
#define WHEELS_SEP kWheelsSeparation
#define WHEELS_OFFSET kWheelsOffset
#define MAX_TWIST_TIME_MS kMaxTwistTimeMs

namespace application {
class ImuAgent;
// TODO: delete after successful micro-ROS-Agent connection-Test
// class Vl6180xAgent;
}  // namespace application

struct DDDOdom {
    double x;
    double y;
    double a;
};

typedef struct DDDOdom DDDOdom_t;

class DDD : public Agent, public uRosEntities {
public:
    /**
     * @brief Construct a new DDD agent
     * 
     * Initializes the DDD (Drive, Detect, Decide) agent with default values.
     * Agent references must be set via setter methods before starting.
     */
    DDD();
    
    /**
     * @brief Destroy the DDD agent
     * 
     * Cleans up ROS2 entities and stops the agent task.
     */
    virtual ~DDD();

    /**
     * @brief Set the motors agent for controlling robot movement
     * 
     * Configures the motor control interface used for executing velocity commands.
     * The DDD agent uses this to translate /cmd_vel messages into individual
     * motor speeds using mecanum kinematics.
     * 
     * @param p Pointer to the motors agent instance
     * 
     * @pre p must not be NULL
     * @pre Must be called before start()
     * @post Motor commands will be sent to the configured agent
     * 
     * @note The DDD agent does not take ownership of the pointer
     * @warning Passing NULL will log an error and be rejected
     */
    void setMotorsAgent(BaseMotorsAgent *p);

    // TODO: delete after successful micro-ROS-Agent connection-Test
    // void setHCSR04Agent(HCSR04Agent *p);

    /**
     * @brief Set the IMU agent for inertial measurement data
     * 
     * Configures the IMU interface for accessing orientation and angular velocity
     * data. This data is used for odometry calculations and published in the
     * odometry message.
     * 
     * @param p Pointer to the IMU agent instance
     * 
     * @pre p must not be NULL
     * @pre Must be called before start()
     * @post IMU data will be included in odometry messages
     * 
     * @note The DDD agent does not take ownership of the pointer
     * @warning Passing NULL will log an error and be rejected
     */
    void setImuAgent(application::ImuAgent *p);

    // TODO: delete after successful micro-ROS-Agent connection-Test
    // void setVl6180xAgent(application::Vl6180xAgent *p);

    /**
     * @brief Create ROS2 publishing and subscription entities
     * 
     * Initializes the ROS2 entities for this agent:
     * - Publisher: /odom (nav_msgs/Odometry) - Robot odometry
     * - Subscriber: /cmd_vel (geometry_msgs/Twist) - Velocity commands
     * 
     * @param node Pointer to the ROS2 node
     * @param support Pointer to the rclc support structure
     * 
     * @pre micro-ROS must be initialized
     * @pre node and support must be valid
     * @post Odometry publisher and cmd_vel subscriber are ready
     * 
     * @note Called automatically by uRosBridge during initialization
     */
    virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

    /**
     * @brief Destroy ROS2 entities
     * 
     * Cleans up the odometry publisher and cmd_vel subscriber, freeing
     * associated resources.
     * 
     * @param node Pointer to the ROS2 node
     * @param support Pointer to the rclc support structure
     * 
     * @post All ROS2 entities are destroyed and resources freed
     * 
     * @note Called automatically by uRosBridge during shutdown
     */
    virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

    /**
     * @brief Get the number of ROS2 entities
     * 
     * Returns the count of ROS2 entities (publishers and subscribers) managed
     * by this agent. Used by uRosBridge for resource allocation.
     * 
     * @return uint Number of entities (2: odometry publisher + cmd_vel subscriber)
     */
    virtual uint getCount();

    /**
     * @brief Get the number of executor handles needed
     * 
     * Returns the number of handles required by the ROS2 executor for this
     * agent's subscriptions and timers.
     * 
     * @return uint Number of handles (1 for cmd_vel subscriber)
     * 
     * @note Publishers don't need executor handles, only subscribers
     */
    virtual uint getHandles();

    /**
     * @brief Add entities to the ROS2 executor
     * 
     * Registers the cmd_vel subscriber with the ROS2 executor so it can
     * receive and process velocity commands.
     * 
     * @param executor Pointer to the rclc executor
     * 
     * @pre createEntities() must have been called
     * @post cmd_vel subscriber is registered with executor
     * 
     * @note The executor will call handleSubscriptionMsg() when messages arrive
     */
    virtual void addToExecutor(rclc_executor_t *executor);

    /**
     * @brief Handle incoming cmd_vel messages
     * 
     * Processes velocity commands from the /cmd_vel topic and translates them
     * into individual motor speeds using mecanum wheel inverse kinematics.
     * 
     * The mecanum kinematics formula used:
     * - Front Left:  v_x - v_y - ω*L
     * - Front Right: v_x + v_y + ω*L
     * - Rear Left:   v_x + v_y - ω*L
     * - Rear Right:  v_x - v_y + ω*L
     * 
     * where L is the distance from robot center to wheel.
     * 
     * @param msg Pointer to the geometry_msgs/Twist message
     * @param context Subscription context (contains callback info)
     * 
     * @pre Motors agent must be configured via setMotorsAgent()
     * @post Motor speeds are updated to achieve commanded velocity
     * @post Timestamp is recorded for timeout detection
     * 
     * @note If no cmd_vel is received for MAX_TWIST_TIME_MS, motors stop
     * @note Linear velocities are in m/s, angular velocity in rad/s
     */
    virtual void handleSubscriptionMsg(const void *msg, uRosSubContext_t *context);

protected:
    /**
     * @brief Main task execution loop
     * 
     * Continuously updates odometry calculations and publishes to ROS2.
     * Runs at approximately 10Hz (100ms delay) to balance accuracy with
     * CPU usage.
     * 
     * The loop performs:
     * 1. Update odometry from motor encoders
     * 2. Publish odometry message to /odom
     * 3. Check for cmd_vel timeout and stop if needed
     * 4. Delay 100ms before next iteration
     * 
     * @note This method runs in its own FreeRTOS task
     * @note Uses vTaskDelay() to yield to other tasks
     */
    virtual void run();

    /**
     * @brief Get the required stack size for this agent
     * 
     * Returns the stack size needed for the DDD task, accounting for
     * odometry calculations, ROS2 message handling, and Eigen matrix operations.
     * 
     * @return configSTACK_DEPTH_TYPE Stack size in words
     * 
     * @note Stack size must accommodate Eigen library usage
     */
    virtual configSTACK_DEPTH_TYPE getMaxStackSize();

private:
    /**
     * @brief Update odometry from motor encoder data
     * 
     * Calculates robot position and orientation by integrating motor encoder
     * velocities using mecanum wheel forward kinematics. Updates both motor
     * odometry and fused DDD odometry estimates.
     * 
     * @note Called at 10Hz from run() loop
     * @note Uses Eigen for rotation matrix calculations
     */
    void updateOdom();

    /**
     * @brief Publish odometry message to ROS2
     * 
     * Publishes the current robot pose (position and orientation) and twist
     * (linear and angular velocities) to the /odom topic.
     * 
     * @note Called at 10Hz from run() loop
     * @note Includes IMU orientation if IMU agent is configured
     */
    void publishOdom();

    /**
     * @brief Initialize the odometry message structure
     * 
     * Allocates and configures the nav_msgs/Odometry message with frame IDs
     * and covariance matrices.
     */
    void setupOdomMsg();
    
    /**
     * @brief Initialize the twist message structure
     * 
     * Allocates and configures the geometry_msgs/Twist message for receiving
     * velocity commands.
     */
    void setupTwistMsg();

    /**
     * @brief Stop all robot motors
     * 
     * Commands all motors to zero velocity. Called when cmd_vel timeout
     * is detected or on emergency stop.
     * 
     * @post All motors are commanded to 0 rad/s
     */
    void robotStop();

    BaseMotorsAgent *pMotorsAgent = NULL;
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // HCSR04Agent *pHCSR04Agent = NULL;
    application::ImuAgent *pImuAgent = NULL;
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // application::Vl6180xAgent *pVl6180xAgent = NULL;

    DDDOdom_t xMotorsOdom;
    DDDOdom_t xDDDOdom;
    DDDOdom_t xDDDVelocity;

    uint32_t xLastVelocityTime = 0u;  // milliseconds - last velocity calculation time

    rcl_publisher_t xPubOdom;
    nav_msgs__msg__Odometry xOdomMsg;

    rcl_subscription_t xSubTwist;
    uRosSubContext_t xSubTwistContext;
    geometry_msgs__msg__Twist xTwistMsg;

    uint32_t xLastTwistTimestamp = 0u;  // milliseconds - last received twist command time
};

#endif /* FIRMWARE_SRC_DDD_H_ */

// /*
//  * DDD.h
//  *
//  *  Created on: 7 Aug 2023
//  *      Author: jondurrant
//  */

// #ifndef FIRMWARE_SRC_DDD_H_
// #define FIRMWARE_SRC_DDD_H_

// #include "Agent.h"
// #include "HCSR04Agent.h"
// #include "MotorsAgent.h"
// #include "uRosEntities.h"

// extern "C" {
// #include "rosidl_runtime_c/primitives_sequence_functions.h"
// #include "rosidl_runtime_c/string_functions.h"

// #include <geometry_msgs/msg/twist.h>
// #include <nav_msgs/msg/odometry.h>
// #include <rcl/error_handling.h>
// #include <rcl/rcl.h>
// #include <rclc/rclc.h>
// #include <rmw_microros/time_sync.h>
// }

// #include <Eigen/Core>
// #include <Eigen/Geometry>

// //#define TWIST_DEBUG

// #define WHEEL_RADIUS 0.065
// #define WHEEL_DEPTH 0.055
// #define WHEELS_SEP 0.204
// #define WHEELS_OFFSET 0.010  // To Y center of Robot

// #define MAX_TWIST_TIME_MS 1500

// namespace application {
// class ImuAgent;
// }

// struct DDDOdom {
//     double x;
//     double y;
//     double a;
// };

// typedef struct DDDOdom DDDOdom_t;

// class DDD : public Agent, public uRosEntities {
// public:
//     DDD();
//     virtual ~DDD();

//     void setMotorsAgent(MotorsAgent *p);

//     void setHCSR04Agent(HCSR04Agent *p);

//     void setImuAgent(application::ImuAgent *p);

//     /***
//      * Create the publishing entities
//      * @param node
//      * @param support
//      */
//     virtual void createEntities(rcl_node_t *node, rclc_support_t *support);

//     /***
//      * Destroy the publishing entities
//      * @param node
//      * @param support
//      */
//     virtual void destroyEntities(rcl_node_t *node, rclc_support_t *support);

//     /***
//      * Provide a count of the number of entities
//      * @return number of entities >=0
//      */
//     virtual uint getCount();

//     /***
//      * Return the number of handles needed by the executor
//      * @return
//      */
//     virtual uint getHandles();

//     /***
//      * Add subscribers, guards and timers to the executor
//      * @param executor
//      */
//     virtual void addToExecutor(rclc_executor_t *executor);

//     /***
//      * Handle subscription msg
//      * @param msg
//      * @param localContext
//      */
//     virtual void handleSubscriptionMsg(const void *msg, uRosSubContext_t *context);

// protected:
//     /***
//      * Run loop for the agent.
//      */
//     virtual void run();

//     /***
//      * Get the static depth required in words
//      * @return - words
//      */
//     virtual configSTACK_DEPTH_TYPE getMaxStackSize();

// private:
//     void updateOdom();

//     void publishOdom();

//     void setupOdomMsg();
//     void setupTwistMsg();

//     void robotStop();

//     MotorsAgent *pMotorsAgent = NULL;
//     HCSR04Agent *pHCSR04Agent = NULL;
//     application::ImuAgent *pImuAgent = NULL;

//     DDDOdom_t xMotorsOdom;
//     DDDOdom_t xDDDOdom;
//     DDDOdom_t xDDDVelocity;

//     uint32_t xLastVelocityTime = 0;

//     rcl_publisher_t xPubOdom;
//     nav_msgs__msg__Odometry xOdomMsg;

//     rcl_subscription_t xSubTwist;
//     uRosSubContext_t xSubTwistContext;
//     geometry_msgs__msg__Twist xTwistMsg;

//     uint32_t xLastTwistTimestamp = 0;
// };

// #endif /* FIRMWARE_SRC_DDD_H_ */
