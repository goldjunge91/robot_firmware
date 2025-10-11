/*
 * RobotController.cpp
 *
 * Created on: 7 Aug 2023
 * Author: jondurrant
 * 
 * Renamed from RobotController.cpp to RobotController.cpp for clarity
 */

#include "RobotController.h"

#include "application/ImuAgent.h"
// TODO: delete after successful micro-ROS-Agent connection-Test
// #include "application/vl6180xAgent.hpp"
#include "uRosBridge.h"

#include <cmath>
#include <cstdio>

using namespace Eigen;

RobotController::RobotController() 
    : xMotorsOdom(0.0 - kWheelsOffset, 0.0, 0.0),  // Initialize motors odometry with X offset
      xRobotOdom(0.0, 0.0, 0.0),                    // Initialize robot odometry at origin
      xRobotVelocity(0.0, 0.0, 0.0)                 // Initialize velocity to zero
{
    // Odometry state initialized via member initializer list
}

RobotController::~RobotController() {
    // TODO Auto-generated destructor stub
}

/**
 * @brief Set the motors agent for controlling robot movement
 * 
 * @param p Pointer to the motors agent instance
 * 
 * @pre p must not be NULL
 * @note This method must be called before starting the RobotController agent
 */
void RobotController::setMotorsAgent(BaseMotorsAgent *p) {
    if (p == NULL) {
        printf("[RobotController] ERROR: Attempted to set null MotorsAgent pointer\n");
        return;
    }
    pMotorsAgent = p;
}

/***
 * Run loop for the agent.
 */
void RobotController::run() {
    setupOdomMsg();
    for (;;) {
        if (pMotorsAgent != NULL) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            uint32_t timeSinceTwise = now - xLastTwistTimestamp;
            if (xLastTwistTimestamp > 0u) {
                if (timeSinceTwise > kMaxTwistTimeMs) {
                    robotStop();
                    xLastTwistTimestamp = 0u;
                    printf("STOPPED due to twist lost\n");
                }
            }
            updateOdom();

            /*
            printf("X: %.3f Y: %.3f A: %.3f Deg: %.3f\n",
                    xRobotOdom.x,
                    xRobotOdom.y,
                    xRobotOdom.a,
                    xRobotOdom.a/M_PI * 180.0);
             */

            publishOdom();
        }

        vTaskDelay(100u);  // milliseconds - odometry update period
    }
}
/* Old test of Odom
void RobotController::run(){
    float leftCum = 0.0;
    float rightCum = 0.0;
    float mOdomX = 0.0;
    float mOdomY = 0.0;
    float mOdomA = 0.0;
    float odomX = 0.0;
    float odomY = 0.0;
    float odomA = 0.0;
    for (;;){

        if (pMotorsAgent != NULL){


            float l = pMotorsAgent->getMotor(0)->getDeltaRadians(false);
            float r = pMotorsAgent->getMotor(1)->getDeltaRadians(false);

            l=l*WHEEL_RADIUS;
            r=r*WHEEL_RADIUS* -1.0;
            leftCum  += l;
            rightCum += r;

            float avgDist = (r+l)/2.0;
            float angle = asin((r-l)/WHEELS_SEP);
            float deltaX = cos(angle)* avgDist;
            float deltaY = sin(angle)* avgDist;

            mOdomX += deltaX;
            mOdomY += deltaY;
            mOdomA += angle;

            odomX = mOdomX + (cos(angle) * WHEELS_OFFSET);
            odomY = mOdomY + (sin(angle) * WHEELS_OFFSET);
            odomA = mOdomA;


            printf("l: %.3f r: %0.3f dist: %.3f ang: %.3f "
                   "x: %.3f y: %.3f a: %.3f "
                    "MX: %.3f MY: %.3f MA: %.3f "
                    "X: %.3f, Y: %.3f A: %.3f\n",
                    l, r, avgDist, angle,
                    deltaX, deltaY, angle/M_PI * 180.0,
                    mOdomX, mOdomY, mOdomA/M_PI * 180.0,
                    odomX, odomY, odomA/M_PI * 180.0
                    );

        }
        vTaskDelay(500);
    }
}
*/

/***
 * Get the static depth required in words
 * @return - words
 */
configSTACK_DEPTH_TYPE RobotController::getMaxStackSize() {
    return 1024u;  // words - stack size for RobotController agent task
}

void RobotController::updateOdom() {
    // ============================================================================
    // DIFFERENTIAL DRIVE ODOMETRY CALCULATION
    // ============================================================================
    //
    // Purpose: Calculate robot position and orientation by integrating wheel
    //          encoder measurements over time.
    //
    // Why odometry?
    // - Provides real-time estimate of robot pose (x, y, theta) without external sensors
    // - Essential for navigation, path planning, and closed-loop control
    // - Accumulated from incremental wheel movements measured by encoders
    //
    // Limitations:
    // - Accumulates error over time (wheel slip, uneven surfaces, measurement noise)
    // - Should be fused with other sensors (IMU, visual odometry) for accuracy
    // - Assumes wheels maintain contact with ground and don't slip
    //
    // Algorithm overview:
    // 1. Get encoder deltas (radians rotated since last update)
    // 2. Convert wheel rotations to linear distances traveled
    // 3. Calculate average distance and heading change (differential drive model)
    // 4. Decompose into X/Y displacement in robot frame
    // 5. Integrate into global odometry estimate
    // 6. Transform from wheel axis to robot center
    // 7. Calculate instantaneous velocities
    //
    // Coordinate frames:
    // - Global frame: Fixed world coordinates (where robot started)
    // - Robot frame: Moves with robot, X forward, Y left
    // - Wheel axis frame: At the midpoint between left/right wheels
    // - Robot center frame: Offset from wheel axis by WHEELS_OFFSET
    //
    // ============================================================================

    // Step 1: Get raw encoder deltas (radians rotated since last call)
    // Motor 0 is left wheel, Motor 1 is right wheel
    // Right wheel is inverted because it's mounted opposite to left wheel
    double l = pMotorsAgent->getMotor(0)->getDeltaRadians();
    double r = pMotorsAgent->getMotor(1)->getDeltaRadians() * (-1.0);

    // Step 2: Convert wheel rotations to linear distance traveled
    // Why? Encoders measure angular displacement, but we need linear distance
    // Formula: arc_length = radius * angle
    // Simplified: distance = circumference * (radians / 2π)
    //
    // This accounts for wheel radius - larger wheels travel farther per rotation
    double diam = kWheelRadius * 2.0 * M_PI;  // meters - wheel circumference
    l = diam * (l / (M_PI * 2.0));  // meters - left wheel linear distance
    r = diam * (r / (M_PI * 2.0));  // meters - right wheel linear distance

    // Step 3: Calculate robot motion using differential drive kinematics
    //
    // Average distance: How far the robot center moved forward
    // Why average? The robot center is equidistant from both wheels
    double avgDist = (r + l) / 2.0;  // meters - forward distance traveled
    
    // Heading change: How much the robot rotated
    // Why this formula? Differential drive: when wheels move at different speeds,
    // the robot rotates around a point. The arc length difference (r-l) divided
    // by wheel separation gives the angle subtended.
    //
    // Derivation: For a circular arc with radius R:
    //   r = (R + sep/2) * angle
    //   l = (R - sep/2) * angle
    //   r - l = sep * angle
    //   angle = (r - l) / sep
    //
    // We use asin() to handle small angles accurately and account for non-circular paths
    double angle = asin((r - l) / kWheelsSeparation);  // radians - heading change
    
    // Step 4: Decompose motion into X/Y components in global frame
    // Why? The robot moved forward by avgDist, but it was also rotating by angle
    // We need to project this motion onto global X and Y axes
    //
    // The motion is along an arc, but for small angles we approximate as:
    //   deltaX = cos(angle) * distance  (forward component)
    //   deltaY = sin(angle) * distance  (lateral component due to rotation)
    //
    // This assumes the rotation happened uniformly during the motion
    double deltaX = cos(angle) * avgDist;  // meters - X displacement in global frame
    double deltaY = sin(angle) * avgDist;  // meters - Y displacement in global frame

    // Step 5: Integrate into cumulative odometry (at wheel axis)
    // Why accumulate? Odometry is the sum of all incremental movements
    // This gives us the wheel axis position in the global frame
    xMotorsOdom.x += deltaX;  // meters - cumulative X position
    xMotorsOdom.y += deltaY;  // meters - cumulative Y position
    xMotorsOdom.a += angle;   // radians - cumulative heading angle

    // Step 6: Transform from wheel axis to robot center
    // Why? The wheel axis is not at the robot's center of mass
    // WHEELS_OFFSET is the distance from wheel axis to robot center (typically forward)
    //
    // We rotate the offset vector by the current heading to get it in global frame:
    //   offset_x = cos(heading) * WHEELS_OFFSET
    //   offset_y = sin(heading) * WHEELS_OFFSET
    //
    // This ensures the offset rotates with the robot
    xRobotOdom.x = xMotorsOdom.x + (cos(angle) * kWheelsOffset);  // meters - robot center X
    xRobotOdom.y = xMotorsOdom.y + (sin(angle) * kWheelsOffset);  // meters - robot center Y
    xRobotOdom.a = xMotorsOdom.a;  // radians - heading (same for both frames)

    // Step 7: Calculate instantaneous velocities
    // Why? ROS odometry messages include both pose and twist (velocity)
    // Velocity = displacement / time
    //
    // We measure time since last update to get accurate velocity estimates
    // This is important because update rate may vary slightly
    uint32_t now = to_ms_since_boot(get_absolute_time());
    double seconds = (double)(now - xLastVelocityTime) / 1000.0;  // seconds - time since last update
    xLastVelocityTime = now;
    
    xRobotVelocity.x = deltaX / seconds;  // meters/second - forward velocity
    xRobotVelocity.y = deltaY / seconds;  // meters/second - lateral velocity
    xRobotVelocity.a = angle / seconds;   // radians/second - angular velocity
}

void RobotController::publishOdom() {
    // Update header timestamp
    int64_t time = rmw_uros_epoch_nanos();
    xOdomMsg.header.stamp.sec = time / 1000000000LL;      // nanoseconds to seconds
    xOdomMsg.header.stamp.nanosec = time % 1000000000LL;  // remaining nanoseconds

    // POSE
    xOdomMsg.pose.pose.position.x = xRobotOdom.x;
    xOdomMsg.pose.pose.position.y = xRobotOdom.y;
    Quaterniond q;
    Matrix3d m;
    m = AngleAxisd(0.0, Vector3d::UnitX()) * AngleAxisd(0.0, Vector3d::UnitY()) *
        AngleAxisd(xRobotOdom.a, Vector3d::UnitZ());
    q = m;
    xOdomMsg.pose.pose.orientation.x = q.x();
    xOdomMsg.pose.pose.orientation.y = q.y();
    xOdomMsg.pose.pose.orientation.z = q.z();
    xOdomMsg.pose.pose.orientation.w = q.w();

    // TWIST
    xOdomMsg.twist.twist.linear.x = xRobotVelocity.x;
    xOdomMsg.twist.twist.linear.y = xRobotVelocity.y;
    xOdomMsg.twist.twist.angular.z = xRobotVelocity.a;

    if (!uRosBridge::getInstance()->publish(&xPubOdom, &xOdomMsg, this, NULL)) {
        printf("Odom Pub failed\n");
    }
}

void RobotController::setupOdomMsg() {
    nav_msgs__msg__Odometry__init(&xOdomMsg);
    if (!rosidl_runtime_c__String__assign(&xOdomMsg.header.frame_id, "odom")) {
        printf("ERROR: Odom frameID assignment failed\n");
    }
    if (!rosidl_runtime_c__String__assign(&xOdomMsg.child_frame_id, "base_link")) {
        printf("ERROR: Odom frameID assignment failed\n");
    }

    // POSE
    xOdomMsg.pose.pose.position.x = 0.0;
    xOdomMsg.pose.pose.position.y = 0.0;
    xOdomMsg.pose.pose.position.z = 0.0;
    xOdomMsg.pose.pose.orientation.x = 0.0;
    xOdomMsg.pose.pose.orientation.y = 0.0;
    xOdomMsg.pose.pose.orientation.z = 0.0;
    xOdomMsg.pose.pose.orientation.w = 1.0;  // Fixed: should be 1.0 for identity quaternion

    // TWIST
    xOdomMsg.twist.twist.linear.x = 0.0;
    xOdomMsg.twist.twist.linear.y = 0.0;
    xOdomMsg.twist.twist.linear.z = 0.0;
    xOdomMsg.twist.twist.angular.x = 0.0;
    xOdomMsg.twist.twist.angular.y = 0.0;
    xOdomMsg.twist.twist.angular.z = 0.0;

    // Initialize covariance matrices to zero
    for (int i = 0; i < 36; i++) {
        xOdomMsg.pose.covariance[i] = 0.0;
        xOdomMsg.twist.covariance[i] = 0.0;
    }

    // Set diagonal elements to indicate uncertainty
    // Covariance matrix is 6x6, stored row-major: [x, y, z, roll, pitch, yaw]
    xOdomMsg.pose.covariance[0] = 0.1;    // x position variance (m²)
    xOdomMsg.pose.covariance[7] = 0.1;    // y position variance (m²)
    xOdomMsg.pose.covariance[35] = 0.1;   // yaw variance (rad²)
    xOdomMsg.twist.covariance[0] = 0.1;   // x velocity variance (m²/s²)
    xOdomMsg.twist.covariance[7] = 0.1;   // y velocity variance (m²/s²)
    xOdomMsg.twist.covariance[35] = 0.1;  // yaw velocity variance (rad²/s²)
}

void RobotController::setupTwistMsg() {
    geometry_msgs__msg__Twist__init(&xTwistMsg);
}

/***
 * Create the publishing entities
 * @param node
 * @param support
 */
void RobotController::createEntities(rcl_node_t *node, rclc_support_t *support) {
    if (pMotorsAgent != NULL) {
        pMotorsAgent->createEntities(node, support);
    }
    if (pImuAgent != NULL) {
        pImuAgent->createEntities(node, support);
    }
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // if (pVl6180xAgent != NULL) {
    //     pVl6180xAgent->createEntities(node, support);
    // }
    rclc_publisher_init_default(
        &xPubOdom, node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");

    rclc_subscription_init_default(
        &xSubTwist, node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
}

/***
 * Destroy the publishing entities
 * @param node
 * @param support
 */
void RobotController::destroyEntities(rcl_node_t *node, rclc_support_t *support) {
    if (pMotorsAgent != NULL) {
        pMotorsAgent->destroyEntities(node, support);
    }
    if (pImuAgent != NULL) {
        pImuAgent->destroyEntities(node, support);
    }
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // if (pVl6180xAgent != NULL) {
    //     pVl6180xAgent->destroyEntities(node, support);
    // }
    rcl_ret_t fini_pub = rcl_publisher_fini(&xPubOdom, node);
    if (fini_pub != RCL_RET_OK) {
        printf("[RobotController] Failed to fini odom publisher: %d\n", fini_pub);
    }
    rcl_ret_t fini_sub = rcl_subscription_fini(&xSubTwist, node);
    if (fini_sub != RCL_RET_OK) {
        printf("[RobotController] Failed to fini twist subscription: %d\n", fini_sub);
    }
}

/***
 * Provide a count of the number of entities
 * @return number of entities >=0
 */
uint RobotController::getCount() {
    uint res = 2u;  // 1 publisher (odom) + 1 subscriber (cmd_vel)
    if (pMotorsAgent != NULL) {
        res += pMotorsAgent->getCount();
    }
    if (pImuAgent != NULL) {
        res += pImuAgent->getCount();
    }
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // if (pVl6180xAgent != NULL) {
    //     res += pVl6180xAgent->getCount();
    // }
    return res;
}

/***
 * Return the number of handles needed by the executor
 * @return
 */
uint RobotController::getHandles() {
    uint res = 1u;  // 1 subscription handle (cmd_vel)
    if (pMotorsAgent != NULL) {
        res += pMotorsAgent->getHandles();
    }
    if (pImuAgent != NULL) {
        res += pImuAgent->getHandles();
    }
    // TODO: delete after successful micro-ROS-Agent connection-Test
    // if (pVl6180xAgent != NULL) {
    //     res += pVl6180xAgent->getHandles();
    // }
    return res;
}

/***
 * Add subscribers, guards and timers to the executor
 * @param executor
 */
void RobotController::addToExecutor(rclc_executor_t *executor) {
    if (pMotorsAgent != NULL) {
        pMotorsAgent->addToExecutor(executor);
    }

    if (pImuAgent != NULL) {
        pImuAgent->addToExecutor(executor);
    }

    // TODO: delete after successful micro-ROS-Agent connection-Test
    // if (pVl6180xAgent != NULL) {
    //     pVl6180xAgent->addToExecutor(executor);
    // }

    buildContext(&xSubTwistContext, NULL);
    rclc_executor_add_subscription_with_context(executor,
                                                &xSubTwist,
                                                &xTwistMsg,
                                                uRosEntities::subscriptionCallback,
                                                &xSubTwistContext,
                                                ON_NEW_DATA);
}

/***
 * Handle subscription msg
 * @param msg
 * @param localContext
 */
void RobotController::handleSubscriptionMsg(const void *msg, uRosSubContext_t *context) {
    if (pMotorsAgent == NULL) {
        return;
    }

    if (context == &xSubTwistContext) {
        geometry_msgs__msg__Twist *pTwistMsg = (geometry_msgs__msg__Twist *)msg;

#ifdef TWIST_DEBUG
        printf("TWIST x: %.3f y: %.3f rz: %.3f\n", pTwistMsg->linear.x, pTwistMsg->linear.y, pTwistMsg->angular.z);
#endif  // TWIST_DEBUG

        xLastTwistTimestamp = to_ms_since_boot(get_absolute_time());

        // ============================================================================
        // MECANUM WHEEL INVERSE KINEMATICS
        // ============================================================================
        // 
        // Purpose: Convert desired robot velocity (vx, vy, omega) into individual
        //          wheel velocities for a mecanum drive system.
        //
        // Why mecanum wheels?
        // - Mecanum wheels have rollers at 45° angles that allow omnidirectional movement
        // - The robot can move forward/backward, strafe left/right, and rotate simultaneously
        // - This requires calculating how fast each wheel must spin to achieve the desired motion
        //
        // Coordinate system:
        // - X-axis: forward (positive) / backward (negative)
        // - Y-axis: left (positive) / right (negative)  
        // - Z-axis (rotation): counter-clockwise (positive) / clockwise (negative)
        //
        // Wheel configuration (top view):
        //     FRONT
        //   FL     FR
        //     \   /
        //      \ /
        //      / \
        //     /   \
        //   RL     RR
        //     BACK
        //
        // The inverse kinematics formula for mecanum drive:
        //   v_wheel = (v_x ± v_y ± ω*L) / r
        //
        // Where:
        //   v_x     = desired forward/backward velocity (m/s)
        //   v_y     = desired strafe left/right velocity (m/s)
        //   ω       = desired rotation rate (rad/s)
        //   L       = distance from robot center to wheel (m)
        //   r       = wheel radius (m)
        //   v_wheel = resulting wheel angular velocity (rad/s)
        //
        // Sign convention for each wheel (determined by roller orientation):
        //   Front-Left:  v_x - v_y - ω*L  (rollers at +45°)
        //   Front-Right: v_x + v_y + ω*L  (rollers at -45°)
        //   Rear-Left:   v_x + v_y - ω*L  (rollers at -45°)
        //   Rear-Right:  v_x - v_y + ω*L  (rollers at +45°)
        //
        // Why these signs?
        // - Forward motion (v_x > 0): All wheels spin forward (positive contribution)
        // - Strafe left (v_y > 0): FL/RR spin backward, FR/RL spin forward
        // - Rotate CCW (ω > 0): Left wheels backward, right wheels forward
        //
        // The wheelbase L is calculated as the distance from robot center to each wheel:
        //   L = sqrt((wheel_sep_x/2)² + (wheel_sep_y/2)²)
        // For this robot: L = sqrt(0.085² + 0.135²) ≈ 0.159m
        //
        // ============================================================================
        
        double vx = pTwistMsg->linear.x;      // meters/second - forward velocity
        double vy = pTwistMsg->linear.y;      // meters/second - strafe velocity (left positive)
        double omega = pTwistMsg->angular.z;  // radians/second - rotation rate (CCW positive)
        
        // Distance from robot center to each wheel (diagonal distance)
        // Calculated from wheel_separation_x=0.170m and wheel_separation_y=0.270m
        constexpr double kMecanumWheelBase = 0.159;  // meters
        
        // Apply inverse kinematics to calculate individual wheel velocities
        // Each wheel velocity is a combination of forward, strafe, and rotational components
        double vfl = (vx - vy - omega * kMecanumWheelBase) / kWheelRadius;  // radians/second - Front Left
        double vfr = (vx + vy + omega * kMecanumWheelBase) / kWheelRadius;  // radians/second - Front Right
        double vrl = (vx + vy - omega * kMecanumWheelBase) / kWheelRadius;  // radians/second - Rear Left
        double vrr = (vx - vy + omega * kMecanumWheelBase) / kWheelRadius;  // radians/second - Rear Right

#ifdef TWIST_DEBUG
        printf("MECANUM TWIST vx: %.3f vy: %.3f omega: %.3f rad/s\n", vx, vy, omega);
        printf("MECANUM Wheel Base L: %.3f m, Wheel Radius: %.3f m\n", kMecanumWheelBase, kWheelRadius);
        printf("MECANUM Wheel Velocities: FL=%.3f FR=%.3f RL=%.3f RR=%.3f rad/s\n", 
               vfl, vfr, vrl, vrr);
        printf("MECANUM Detail: vx: %.3f vy: %.3f omega: %.3f becomes FL: %.3f FR: %.3f RL: %.3f RR: %.3f rad/s\n",
               vx, vy, omega, vfl, vfr, vrl, vrr);
#endif  // TWIST_DEBUG

        // Set all 4 motor speeds (index, speed magnitude, direction)
        pMotorsAgent->setSpeedRadPS(0u, fabs(vfl), vfl >= 0.0);  // Motor 0: Front Left
        pMotorsAgent->setSpeedRadPS(1u, fabs(vfr), vfr >= 0.0);  // Motor 1: Front Right
        pMotorsAgent->setSpeedRadPS(2u, fabs(vrl), vrl >= 0.0);  // Motor 2: Rear Left
        pMotorsAgent->setSpeedRadPS(3u, fabs(vrr), vrr >= 0.0);  // Motor 3: Rear Right
    }
}

void RobotController::robotStop() {
    xLastTwistTimestamp = to_ms_since_boot(get_absolute_time());

    if (pMotorsAgent != NULL) {
        // Stop all 4 motors for mecanum drive
        pMotorsAgent->setSpeedRadPS(0u, 0.0, true);   // Motor 0: Front Left
        pMotorsAgent->setSpeedRadPS(1u, 0.0, false);  // Motor 1: Front Right
        pMotorsAgent->setSpeedRadPS(2u, 0.0, true);   // Motor 2: Rear Left
        pMotorsAgent->setSpeedRadPS(3u, 0.0, false);  // Motor 3: Rear Right
    }
}

/**
 * @brief Set the IMU agent for inertial measurement data
 * 
 * @param p Pointer to the IMU agent instance
 * 
 * @pre p must not be NULL
 * @note This method must be called before starting the RobotController agent
 */
void RobotController::setImuAgent(application::ImuAgent *p) {
    if (p == NULL) {
        printf("[RobotController] ERROR: Attempted to set null ImuAgent pointer\n");
        return;
    }
    pImuAgent = p;
}
