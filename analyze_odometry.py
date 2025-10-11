#!/usr/bin/env python3
"""
Odometry Analysis Tool

This script subscribes to /odom and /cmd_vel topics to analyze odometry accuracy
in real-time. It calculates expected vs actual position and provides statistics.

Requirements: 4.4, 9.1
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
from collections import deque


class OdometryAnalyzer(Node):
    def __init__(self):
        super().__init__('odometry_analyzer')
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # State variables
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_omega = 0.0
        
        self.last_cmd_time = None
        self.start_time = time.time()
        
        # Statistics
        self.position_errors = deque(maxlen=100)
        self.velocity_errors = deque(maxlen=100)
        
        # Timer for periodic reporting
        self.timer = self.create_timer(1.0, self.report_status)
        
        self.get_logger().info('Odometry Analyzer started')
        self.get_logger().info('Waiting for initial odometry message...')
    
    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def odom_callback(self, msg):
        """Process odometry messages"""
        # Extract position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract orientation (convert quaternion to yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        _, _, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        # Set initial position on first message
        if self.initial_x is None:
            self.initial_x = x
            self.initial_y = y
            self.initial_theta = yaw
            self.get_logger().info(f'Initial position set: x={x:.3f}, y={y:.3f}, theta={math.degrees(yaw):.1f}°')
        
        # Update current position
        self.current_x = x
        self.current_y = y
        self.current_theta = yaw
        
        # Extract velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z
        
        # Calculate velocity error if we have a command
        if self.last_cmd_time is not None:
            time_since_cmd = time.time() - self.last_cmd_time
            if time_since_cmd < 2.0:  # Only compare if command is recent
                vx_error = abs(vx - self.cmd_vx)
                vy_error = abs(vy - self.cmd_vy)
                omega_error = abs(omega - self.cmd_omega)
                
                total_vel_error = math.sqrt(vx_error**2 + vy_error**2 + omega_error**2)
                self.velocity_errors.append(total_vel_error)
    
    def cmd_vel_callback(self, msg):
        """Process velocity command messages"""
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_omega = msg.angular.z
        self.last_cmd_time = time.time()
    
    def report_status(self):
        """Periodic status report"""
        if self.initial_x is None:
            return
        
        # Calculate displacement from initial position
        dx = self.current_x - self.initial_x
        dy = self.current_y - self.initial_y
        dtheta = self.current_theta - self.initial_theta
        
        # Normalize angle to [-pi, pi]
        while dtheta > math.pi:
            dtheta -= 2 * math.pi
        while dtheta < -math.pi:
            dtheta += 2 * math.pi
        
        # Calculate total distance traveled
        distance = math.sqrt(dx**2 + dy**2)
        
        # Calculate statistics
        elapsed_time = time.time() - self.start_time
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Odometry Status (t={elapsed_time:.1f}s)')
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Position:')
        self.get_logger().info(f'  X: {self.current_x:.3f} m (Δ={dx:+.3f} m)')
        self.get_logger().info(f'  Y: {self.current_y:.3f} m (Δ={dy:+.3f} m)')
        self.get_logger().info(f'  θ: {math.degrees(self.current_theta):.1f}° (Δ={math.degrees(dtheta):+.1f}°)')
        self.get_logger().info(f'  Distance: {distance:.3f} m')
        
        if self.last_cmd_time is not None:
            time_since_cmd = time.time() - self.last_cmd_time
            if time_since_cmd < 2.0:
                self.get_logger().info(f'Current Command:')
                self.get_logger().info(f'  vx: {self.cmd_vx:.3f} m/s')
                self.get_logger().info(f'  vy: {self.cmd_vy:.3f} m/s')
                self.get_logger().info(f'  ω:  {self.cmd_omega:.3f} rad/s')
        
        if len(self.velocity_errors) > 0:
            avg_vel_error = sum(self.velocity_errors) / len(self.velocity_errors)
            max_vel_error = max(self.velocity_errors)
            self.get_logger().info(f'Velocity Tracking:')
            self.get_logger().info(f'  Avg error: {avg_vel_error:.4f}')
            self.get_logger().info(f'  Max error: {max_vel_error:.4f}')
        
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    analyzer = OdometryAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final summary
        if analyzer.initial_x is not None:
            dx = analyzer.current_x - analyzer.initial_x
            dy = analyzer.current_y - analyzer.initial_y
            dtheta = analyzer.current_theta - analyzer.initial_theta
            distance = math.sqrt(dx**2 + dy**2)
            
            print('\n' + '=' * 60)
            print('FINAL ODOMETRY SUMMARY')
            print('=' * 60)
            print(f'Total displacement:')
            print(f'  X: {dx:+.3f} m')
            print(f'  Y: {dy:+.3f} m')
            print(f'  θ: {math.degrees(dtheta):+.1f}°')
            print(f'  Distance: {distance:.3f} m')
            print('=' * 60)
        
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
