#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
import math
import time
import signal
import sys
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg  

class RoverController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # Initialize PID gains (will be updated dynamically)
        self.Kp = 0.1
        self.Ki = 0.0
        self.Kd = 0.1

        # Initialize error variables
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        self.setpoint_y = 0.0  # Target orientation (straight motion)

        # Command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to IMU data (for orientation feedback)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscriber to optimized PID gains from SFOA node
        self.create_subscription(Float32MultiArray, '/optimized_pid_gains', self.pid_update_callback, 10)

        # Publisher for real-time PID feedback (error, control effort, timestamp)
        self.pid_feedback_pub = self.create_publisher(Float32MultiArray, '/robot_pid_feedback', 10)

        # Timer to run the PID control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.pid_control_loop)

        # Initialize current orientation and twist command
        self.current_position_y = 0.0
        self.cmd_vel = Twist()

        # Set up signal handling for keyboard interrupt
        signal.signal(signal.SIGINT, self.signal_handler)

    def odom_callback(self, msg):
        """Extract yaw (orientation about Z-axis) from IMU message"""
        
        self.current_position_y = msg.pose.pose.position.y


    def pid_update_callback(self, msg):
        """Receive optimized Kp, Ki, Kd from the SFOA node"""
        self.Kp, self.Ki, self.Kd = msg.data
        self.get_logger().info(f"Updated PID Gains: Kp={self.Kp:.3f}, Ki={self.Ki:.3f}, Kd={self.Kd:.3f}")

    def pid_control_loop(self):
        """Compute PID control output and publish command"""
        # Calculate error (difference between setpoint and current orientation)
        self.error = self.setpoint_y - self.current_position_y
        derivative = self.error - self.prev_error
        self.integral += self.error

        # Compute PID output
        pid_output = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update previous error
        self.prev_error = self.error

        # Adjust wheel velocities
        left_wheel_speed = 0.5 + pid_output  # Base speed + correction
        right_wheel_speed = 0.5 - pid_output  # Base speed - correction

        # Limit wheel speeds
        left_wheel_speed = max(min(left_wheel_speed, 1.0), 0.0)
        right_wheel_speed = max(min(right_wheel_speed, 1.0), 0.0)

        # Publish velocity command
        self.cmd_vel.linear.x = 0.5  # Constant forward speed
        self.cmd_vel.angular.z = max(min(-(right_wheel_speed - left_wheel_speed) / 0.45, 0.25), -0.25)

        self.cmd_vel_pub.publish(self.cmd_vel)

        # Publish PID feedback for SFOA tuning
        self.publish_pid_feedback(self.cmd_vel.angular.z)

    def publish_pid_feedback(self, control_effort):
        """Publish error, control effort, and timestamp for SFOA tuning"""
        feedback_msg = Float32MultiArray()
        ros_time = self.get_clock().now().to_msg()  # Get ROS2 time

        # Use seconds + nanoseconds as a float for easy processing
        timestamp = ros_time.sec + ros_time.nanosec * 1e-9

        feedback_msg.data = [self.error, control_effort, timestamp]
        self.pid_feedback_pub.publish(feedback_msg)

    def signal_handler(self, sig, frame):
        """Handle keyboard interrupt (Ctrl+C)"""
        self.stop_rover()
        sys.exit(0)

    def stop_rover(self):
        """Stop the rover by publishing zero velocity"""
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.get_logger().info('Rover stopped.')

def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    rclpy.spin(rover_controller)
    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

