#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import signal
import sys

class RoverController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # PID constants (tune these as necessary)
        self.Kp = 0.1  # Proportional gain
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.1  # Derivative gain

        # Initialize error variables
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        # Setpoint for angular stabilization (keep orientation x close to 0)
        self.setpoint_y = 0.0

        # Command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to IMU data to get current orientation
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)


        # Timer to run the PID control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.pid_control_loop)  # 50 Hz update rate

        # Initialize current orientation and twist command
        self.current_position_y = 0.0
        self.cmd_vel = Twist()
        
        

        # Set up signal handling for keyboard interrupt
        signal.signal(signal.SIGINT, self.signal_handler)

    def odom_callback(self, msg):
        # Extract orientation x value from IMU message
        self.current_position_y = msg.pose.pose.position.y


    def pid_control_loop(self):
        # Calculate the current error (difference between setpoint and current orientation)
        self.error = self.setpoint_y - self.current_position_y
        if abs(self.error) > 0.05:
            # Integral term (accumulation of error over time)
            self.integral += self.error

            # Derivative term (change in error)
            derivative = self.error - self.prev_error

            # PID output (adjust this formula based on your needs)
            pid_output = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * derivative)

            # Update previous error
            self.prev_error = self.error

            # Adjust wheel velocities based on PID output
            left_wheel_speed = 0.5 + pid_output  # Base speed minus correction
            right_wheel_speed = 0.5 - pid_output  # Base speed plus correction

            # Limit the speeds to a reasonable range
            left_wheel_speed = max(min(left_wheel_speed, 1.0), 0.0)
            right_wheel_speed = max(min(right_wheel_speed, 1.0), 0.0)

            # Publish velocity command for differential drive
            self.cmd_vel.linear.x = 0.5  # Constant forward speed
            self.cmd_vel.angular.z = -(right_wheel_speed - left_wheel_speed) / 0.45#Adjust turning based on wheel difference
            self.cmd_vel_pub.publish(self.cmd_vel)
        else:
            self.cmd_vel.linear.x = 0.5
            self.cmd_vel.angular.z=0.0
            self.cmd_vel_pub.publish(self.cmd_vel)  

    def signal_handler(self, sig, frame):
        self.stop_rover()
        sys.exit(0)  # Exit the program

    def stop_rover(self):
        # Stop the rover by publishing zero velocity
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(self.cmd_vel)
        self.get_logger().info('Rover stopped.')

        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    rclpy.spin(rover_controller)
    rover_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

