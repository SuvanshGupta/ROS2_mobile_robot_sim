#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import random
import numpy as np

class SFOAPIDOptimizer(Node):
    def __init__(self):
        super().__init__('sfoa_pid_optimizer')

        # Initialize PID gains
        self.Kp = 0.05
        self.Ki = 0.0
        self.Kd = 0.1

        # Step size for adaptive tuning
        self.step_size = 0.01
        self.prev_fitness = float('inf')

        # Lists to store error, control effort, and timestamps for optimization
        self.error_list = []
        self.control_effort_list = []
        self.time_list = []

        # Define ROS2 subscribers and publishers
        self.pid_feedback_sub = self.create_subscription(
            Float32MultiArray, '/robot_pid_feedback', self.pid_feedback_callback, 10)
        
        self.optimized_pid_pub = self.create_publisher(
            Float32MultiArray, '/optimized_pid_gains', 10)

        # Timer to run optimization at 10Hz (adjustable)
        self.timer = self.create_timer(0.1, self.run_optimization)

    def pid_feedback_callback(self, msg):
        """Receive PID feedback (error, control effort, timestamp) from the PID controller node"""
        if len(msg.data) < 3:
            return  # Ensure we have enough data

        error, control_effort, timestamp = msg.data

        # Append new values to lists
        self.error_list.append(error)
        self.control_effort_list.append(control_effort)
        self.time_list.append(timestamp)

        # Keep the lists limited to avoid excessive memory usage
        if len(self.error_list) > 50:
            self.error_list.pop(0)
            self.control_effort_list.pop(0)
            self.time_list.pop(0)

    def fitness_function(self, settling_time, overshoot, steady_state_error, control_effort):
        """Compute the fitness value for a given PID performance."""
        w1, w2, w3, w4 = 1.0, 1.0, 1.0, 0.5  # Weight factors for tuning
        return (w1 * settling_time) + (w2 * overshoot) + (w3 * steady_state_error) + (w4 * control_effort)

    def calculate_pid_performance(self):
        """Analyze system response to calculate performance metrics."""
        if len(self.error_list) < 10:
            return 1.0, 1.0, 1.0, 1.0  # Default values until enough data is collected

        settling_threshold = 0.05 * max(abs(np.array(self.error_list)))  

        # Calculate overshoot
        max_error = max(abs(np.array(self.error_list)))
        overshoot = (max_error - abs(self.error_list[-1])) / max_error if max_error > 0 else 0

        # Calculate settling time
        settling_time = self.time_list[-1]  
        for i in range(len(self.error_list) - 1, -1, -1):
            if abs(self.error_list[i]) > settling_threshold:
                settling_time = self.time_list[i]
                break

        # Calculate steady-state error
        steady_state_error = abs(self.error_list[-1])

        # Compute control effort
        control_effort = sum(abs(np.array(self.control_effort_list))) / len(self.control_effort_list)

        return settling_time, overshoot, steady_state_error, control_effort

    def predict_error(self, e_t, e_t_minus1, e_t_minus2):
        """Predict future error using weighted sum of past errors."""
        a1, a2, a3 = 0.6, 0.3, 0.1
        return a1 * e_t + a2 * e_t_minus1 + a3 * e_t_minus2

    def update_step_size(self, prev_fitness, new_fitness):
        """Adjust step size based on improvement in fitness."""
        if new_fitness < prev_fitness:
            self.step_size *= 0.9  # Reduce step size for fine-tuning
        else:
            self.step_size *= 1.1  # Increase step size for broader search
        return max(0.001, min(self.step_size, 0.1))

    def long_jump(self, jump_prob=0.000000001):
        """Occasionally perform a long jump to avoid local minima."""
        if random.random() < jump_prob:  
            self.Kp += random.uniform(-0.005, 0.005)
            self.Ki += random.uniform(-0.001, 0.001)
            self.Kd += random.uniform(-0.005, 0.005)
        self.Kp, self.Ki, self.Kd = max(0.01, self.Kp), max(0.0, self.Ki), max(0.01, self.Kd)

    def adjust_gains_for_overshoot(self, error, threshold=0.05):
        """Apply soft braking when approaching setpoint too fast."""
        if abs(error) < threshold:
            self.Kp *= 0.8  # Reduce proportional gain
            self.Kd *= 1.2  # Increase derivative gain for damping
        self.Kp, self.Kd = max(0.01, self.Kp), max(0.01, self.Kd)

    def energy_saving_mode(self, error, steady_state_threshold=0.01):
        """Reduce control effort when error is low."""
        if abs(error) < steady_state_threshold:
            self.Kp *= 0.7
            self.Ki *= 0.7
            self.Kd *= 0.7
        self.Kp, self.Ki, self.Kd = max(0.01, self.Kp), max(0.0, self.Ki), max(0.01, self.Kd)

    def run_optimization(self):
        """Main optimization function using Snow Falcon Optimization Algorithm."""
        if len(self.error_list) < 10:
            return  # Wait until enough data is collected

        # Calculate PID performance metrics
        settling_time, overshoot, steady_state_error, control_effort = self.calculate_pid_performance()

        # Compute fitness function
        new_fitness = self.fitness_function(settling_time, overshoot, steady_state_error, control_effort)

        # Predict next error
        if len(self.error_list) >= 3:
            predicted_error = self.predict_error(self.error_list[-1], self.error_list[-2], self.error_list[-3])
        else:
            predicted_error = self.error_list[-1]

        # Update step size based on fitness improvement
        self.step_size = self.update_step_size(self.prev_fitness, new_fitness)

        # Adjust gains dynamically
        self.Kp -= self.step_size * predicted_error
        self.Ki -= self.step_size * (predicted_error / 10)
        self.Kd += self.step_size * predicted_error

        # Apply soft braking if needed
        self.adjust_gains_for_overshoot(predicted_error)

        # Perform occasional long jump to escape local minima
        self.long_jump()

        # Enter energy-saving mode if error is very low
        self.energy_saving_mode(steady_state_error)

        # Ensure gains remain positive
        self.Kp, self.Ki, self.Kd = max(0.01, self.Kp), max(0.0, self.Ki), max(0.01, self.Kd)

        # Publish optimized PID gains
        self.publish_optimized_pid()

        # Update previous fitness
        self.prev_fitness = new_fitness

    def publish_optimized_pid(self):
        """Publish optimized PID gains to the PID controller node."""
        pid_msg = Float32MultiArray()
        pid_msg.data = [self.Kp, self.Ki, self.Kd]
        self.optimized_pid_pub.publish(pid_msg)

def main(args=None):
    rclpy.init(args=args)
    optimizer = SFOAPIDOptimizer()
    rclpy.spin(optimizer)
    optimizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

