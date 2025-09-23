/**
 * @file micro_ros_robot.h
 * @brief Erweiterte micro-ROS Integration für Robot Firmware
 *
 * Diese Header-Datei definiert die Schnittstellen für eine vollständige
 * micro-ROS Robot-Integration mit Publisher, Subscriber und Services.
 */

#ifndef MICRO_ROS_ROBOT_H
#define MICRO_ROS_ROBOT_H

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// Standard ROS2 Message Types
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/bool.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>

// Robot-spezifische Definitionen
#define ROBOT_NODE_NAME "steel_robot_node"
#define ROBOT_NAMESPACE ""

// micro-ROS Konfiguration
#define MAX_PUBLISHERS 5
#define MAX_SUBSCRIBERS 5
#define MAX_SERVICES 2
#define MAX_TIMERS 3
#define EXECUTOR_HANDLES (MAX_PUBLISHERS + MAX_SUBSCRIBERS + MAX_SERVICES + MAX_TIMERS)

// Error Handling Macros
#define RCCHECK(fn)                  \
    {                                \
        rcl_ret_t temp_rc = fn;      \
        if ((temp_rc != RCL_RET_OK)) \
        {                            \
            error_loop();            \
        }                            \
    }

#define RCSOFTCHECK(fn)                         \
    {                                           \
        rcl_ret_t temp_rc = fn;                 \
        if ((temp_rc != RCL_RET_OK))            \
        {                                       \
            /* Soft error - log but continue */ \
        }                                       \
    }

// Robot State Structure
typedef struct
{
    bool is_connected;
    bool emergency_stop;
    uint32_t heartbeat_counter;
    float battery_voltage;
    int32_t motor_left_speed;
    int32_t motor_right_speed;
} robot_state_t;

// Function Declarations
void micro_ros_robot_init(void);
void micro_ros_robot_spin(void);
void micro_ros_robot_cleanup(void);
void error_loop(void);

// Publisher Functions
void publish_heartbeat(void);
void publish_robot_status(void);
void publish_imu_data(void);

// Subscriber Callbacks
void cmd_vel_callback(const void *msgin);
void emergency_stop_callback(const void *msgin);

// Timer Callbacks
void heartbeat_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void status_timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void sensor_timer_callback(rcl_timer_t *timer, int64_t last_call_time);

// Utility Functions
bool micro_ros_check_connection(void);
void update_robot_state(void);

#endif // MICRO_ROS_ROBOT_H