#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/twist.h>

// Steel Robot with micro-ROS integration
// Phase 1: Basic publisher setup with heartbeat and status

// micro-ROS entities
rcl_publisher_t heartbeat_publisher;
rcl_publisher_t status_publisher;
rcl_subscription_t cmd_vel_subscriber;
std_msgs__msg__Int32 heartbeat_msg;
std_msgs__msg__String status_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;

// Robot state
static int heartbeat_counter = 0;
static bool micro_ros_connected = false;
char status_buffer[100];

// Callback for velocity commands
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

  Serial.print("Received cmd_vel - linear.x: ");
  Serial.print(msg->linear.x);
  Serial.print(", angular.z: ");
  Serial.println(msg->angular.z);

  // TODO: Forward to motor controller
  // motor_driver_set_velocity(msg->linear.x, msg->angular.z);
}

void setup()
{
  Serial.begin(115200);

  // Initialize LED
  pinMode(LED_BUILTIN, OUTPUT);

  // Variable for return codes
  rcl_ret_t ret;

  // Wait for Serial connection
  int wait_count = 0;
  while (!Serial && wait_count < 50)
  {
    delay(100);
    wait_count++;
  }

  Serial.println("=== Steel Robot micro-ROS Firmware ===");
  Serial.println("Platform: Raspberry Pi Pico");
  Serial.println("Framework: Arduino + micro-ROS");
  Serial.println("Initializing micro-ROS...");

  // Initialize micro-ROS
  set_microros_transports();

  allocator = rcl_get_default_allocator();

  // Create init_options
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  ret = rcl_init_options_init(&init_options, allocator);
  if (ret != RCL_RET_OK)
  {
    Serial.println("ERROR: Failed to initialize init_options");
    return;
  }

  // Initialize rclc support
  ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
  if (ret != RCL_RET_OK)
  {
    Serial.println("ERROR: Failed to initialize rclc support");
    return;
  } // Create node
  rclc_node_init_default(&node, "steel_robot_node", "", &support);

  // Create publishers
  rclc_publisher_init_default(
      &heartbeat_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
      "steel_robot/heartbeat");

  rclc_publisher_init_default(
      &status_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "steel_robot/status");

  // Create subscriber
  rclc_subscription_init_default(
      &cmd_vel_subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "steel_robot/cmd_vel");

  // Initialize executor
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg,
                                 &cmd_vel_callback, ON_NEW_DATA);

  // Initialize messages
  status_msg.data.data = status_buffer;
  status_msg.data.capacity = sizeof(status_buffer);

  micro_ros_connected = true;
  Serial.println("micro-ROS initialization complete!");
  Serial.println("Topics:");
  Serial.println("  Publisher: /steel_robot/heartbeat (std_msgs/Int32)");
  Serial.println("  Publisher: /steel_robot/status (std_msgs/String)");
  Serial.println("  Subscriber: /steel_robot/cmd_vel (geometry_msgs/Twist)");
  Serial.println("Ready for ROS communication!");
}

void loop()
{
  static unsigned long last_heartbeat = 0;
  static unsigned long last_status = 0;
  static bool led_state = false;

  unsigned long current_time = millis();

  // Handle micro-ROS callbacks
  if (micro_ros_connected)
  {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  }

  // Publish heartbeat every 1 second
  if (current_time - last_heartbeat > 1000)
  {
    last_heartbeat = current_time;

    if (micro_ros_connected)
    {
      heartbeat_msg.data = heartbeat_counter++;
      rcl_ret_t ret = rcl_publish(&heartbeat_publisher, &heartbeat_msg, NULL);
      if (ret != RCL_RET_OK)
      {
        Serial.println("WARNING: Failed to publish heartbeat");
      }
    }

    // LED blink
    led_state = !led_state;
    digitalWrite(LED_BUILTIN, led_state);

    Serial.print("Heartbeat #");
    Serial.print(heartbeat_counter - 1);
    Serial.print(" - LED: ");
    Serial.print(led_state ? "ON" : "OFF");
    Serial.print(" - micro-ROS: ");
    Serial.println(micro_ros_connected ? "CONNECTED" : "DISCONNECTED");
  }

  // Publish status every 5 seconds
  if (current_time - last_status > 5000)
  {
    last_status = current_time;

    if (micro_ros_connected)
    {
      snprintf(status_buffer, sizeof(status_buffer),
               "Steel Robot OK - Uptime: %lu ms, Heartbeat: %d",
               current_time, heartbeat_counter);
      status_msg.data.size = strlen(status_buffer);
      rcl_ret_t ret = rcl_publish(&status_publisher, &status_msg, NULL);
      if (ret != RCL_RET_OK)
      {
        Serial.println("WARNING: Failed to publish status");
      }
    }
  }

  delay(10);
}