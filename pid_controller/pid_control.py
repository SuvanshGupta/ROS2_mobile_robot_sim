#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import math
import signal
import sys

class RoverController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # PID constants (tune these as necessary)
        self.Kp = 0.04  # Proportional gain
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.1  # Derivative gain

        # Initialize error variables
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        # Setpoint for angular stabilization (keep orientation x close to 0)
        self.setpoint = 0.0

        # Command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to IMU data to get current orientation
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)


        # Timer to run the PID control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.pid_control_loop)  # 50 Hz update rate

        # Initialize current orientation and twist command
        self.current_orientation_x = 0.0
        self.cmd_vel = Twist()
        
        self.create_subscription(Float32, '/kp', self.kp_callback, 10)
        self.create_subscription(Float32, '/ki', self.ki_callback, 10)
        self.create_subscription(Float32, '/kd', self.kd_callback, 10)

        # Signal handler for clean exit
        signal.signal(signal.SIGINT, self.signal_handler)

    def kp_callback(self, msg):
        self.Kp = msg.data
        self.get_logger().info(f"Updated Kp: {self.Kp}")

    def ki_callback(self, msg):
        self.Ki = msg.data
        self.get_logger().info(f"Updated Ki: {self.Ki}")

    def kd_callback(self, msg):
        self.Kd = msg.data
        self.get_logger().info(f"Updated Kd: {self.Kd}")

        # Set up signal handling for keyboard interrupt
        signal.signal(signal.SIGINT, self.signal_handler)

    def imu_callback(self, imu):
        # Extract orientation x value from IMU message
        orientation_x = imu.orientation.x

        # Convert orientation.x to range -180 to 180 degrees
        self.current_orientation_x = self.normalize_orientation(orientation_x)

    def normalize_orientation(self, x):
        # If x is greater than 180, adjust it to be in the range -180 to 180
        if x > 180:
            x -= 360
        return x

    def pid_control_loop(self):
        # Calculate the current error (difference between setpoint and current orientation)
        self.error = self.setpoint - self.current_orientation_x
        if abs(self.error) > 1:
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
            self.cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed) / 0.8  # Adjust turning based on wheel difference
            self.cmd_vel.angular.z= (self.cmd_vel.angular.z)
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

    def quaternion_to_euler(self, x, y, z, w):
        """
        Convert a quaternion into Euler angles (roll, pitch, yaw)
        roll is rotation around x-axis
        pitch is rotation around y-axis
        yaw is rotation around z-axis
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
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

