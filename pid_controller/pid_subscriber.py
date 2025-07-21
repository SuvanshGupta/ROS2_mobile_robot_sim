#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

class RoverController(Node):
    def __init__(self):
        super().__init__('pid_controller')

        # PID constants (initial values set to 0)
        self.Kp = 0.0  # Proportional gain
        self.Ki = 0.0  # Integral gain
        self.Kd = 0.0  # Derivative gain

        # Setpoint for y position
        self.setpoint_y = 0.0  # Target y position

        # Initialize error variables
        self.error = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

        # Command publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to odometry and PID parameters
        self.create_subscription(Odometry, '/odom_data_quat', self.odom_callback, 10)
        self.create_subscription(Float32, 'kp', self.update_kp, 10)
        self.create_subscription(Float32, 'ki', self.update_ki, 10)
        self.create_subscription(Float32, 'kd', self.update_kd, 10)

        # Timer to run the PID control loop at 50 Hz
        self.timer = self.create_timer(0.02, self.pid_control_loop)  # 50 Hz update rate

        # Initialize current y position and twist command
        self.current_position_y = 0.0
        self.cmd_vel = Twist()

    def odom_callback(self, msg):
        # Update current y position
        self.current_position_y = msg.pose.pose.position.y

    def update_kp(self, msg):
        self.Kp = msg.data
        self.get_logger().info(f'Updated Kp: {self.Kp}')

    def update_ki(self, msg):
        self.Ki = msg.data
        self.get_logger().info(f'Updated Ki: {self.Ki}')

    def update_kd(self, msg):
        self.Kd = msg.data
        self.get_logger().info(f'Updated Kd: {self.Kd}')

    def pid_control_loop(self):
        # Calculate the current error (difference between setpoint and current y position)
        self.error = self.setpoint_y - self.current_position_y

        # Integral term (accumulation of error over time)
        self.integral += self.error

        # Derivative term (change in error)
        derivative = self.error - self.prev_error

        # PID output (adjust this formula based on your needs)
        pid_output = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * derivative)

        # Update previous error
        self.prev_error = self.error

        # Base speed (forward movement)
        base_speed = 0.5

        # Adjust left and right wheel speeds based on PID output
        left_wheel_speed = base_speed - pid_output  # Base speed minus correction
        right_wheel_speed = base_speed + pid_output  # Base speed plus correction

        # Limit the wheel speeds to a reasonable range
        left_wheel_speed = max(min(left_wheel_speed, 1.0), 0.0)
        right_wheel_speed = max(min(right_wheel_speed, 1.0), 0.0)

        # Publish velocity command for differential drive
        self.cmd_vel.linear.x = (left_wheel_speed + right_wheel_speed) / 2  # Average forward speed
        self.cmd_vel.angular.z = (right_wheel_speed - left_wheel_speed) / 1.02  # Turning based on wheel speed difference

        self.cmd_vel_pub.publish(self.cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    rover_controller = RoverController()
    rclpy.spin(rover_controller)
    rover_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
