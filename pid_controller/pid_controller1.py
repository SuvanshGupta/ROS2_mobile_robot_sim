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
        self.Kp = 1.0 # proportional gain
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.5  # Derivative gain

        # Initialize error variables
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0
        # Setpoint for angular stabilization (keep orientation x close to 0)
        self.setpoint = 0.0

        # Command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to IMU data to get current orientation
        self.create_subscription(Imu, '/imu/out', self.imu_callback, 10)

        # Timer to run the PID control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.pid_control_loop)  # 50 Hz update rate

        # Initialize current orientation and twist command
        self.current_orientation_z = 0.0
        self.cmd_vel = Twist()

        # Set up signal handling for keyboard interrupt
        signal.signal(signal.SIGINT, self.signal_handler)

    def imu_callback(self, imu):
        # Extract orientation quaternion from IMU message
        orientation = imu.orientation

        # Convert quaternion to Euler angles
        _, yaw, _ = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)

        # Update current orientation about the z-axis (yaw)
        self.current_orientation_z = orientation.z

    def pid_control_loop(self):
        # Calculate the current error (difference between setpoint and current orientation)
        self.error = self.setpoint - self.current_orientation_z

        # Integral term (accumulation of error over time)
        self.integral += self.error

        # Derivative term (change in error)
        derivative = self.error - self.prev_error

        # PID output (adjust this formula based on your needs)
        pid_output = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update previous error
        self.prev_error = self.error

        # Adjust wheel velocities based on PID output
        left_wheel_speed = 1.0 + pid_output  # Base speed minus correction
        right_wheel_speed = 1.0 - pid_output  # Base speed plus correction

        # Limit the speeds to a reasonable range
        left_wheel_speed = max(min(left_wheel_speed, 1.0), 0.0)
        right_wheel_speed = max(min(right_wheel_speed, 1.0), 0.0)

        # Publish velocity command for differential drive
        self.cmd_vel.linear.x = 1.0  # Constant forward speed
        self.cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed) /0.8  # Adjust turning based on wheel difference

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

