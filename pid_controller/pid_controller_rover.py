import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class IMUBaseNode(Node):
    def _init_(self):
        super()._init_('imu_base_node')

        # Subscriber to the IMU topic
        self.subscription = self.create_subscription(
            Imu,
            '/imu',  # Replace with your IMU topic
            self.imu_callback,
            10
        )

        # Publisher for the base length
        self.publisher = self.create_publisher(Float64, '/base_length', 10)

        # Initial variables
        self.previous_angle = 0.0  # Store the previous angle
        self.total_base_length = 0.0  # Accumulated base length
        self.step_distance = 1.0  # Hypotenuse step length (can be adjusted)

    def imu_callback(self, msg):
        # Extract the current orientation in x (angle in radians assumed)
        current_angle = msg.orientation.x

        # Calculate the incremental base contribution
        delta_angle = current_angle - self.previous_angle
        base_increment = self.step_distance * math.cos(current_angle)

        # Update the total base length
        self.total_base_length += base_increment

        # Publish the base length
        base_msg = Float64()
        base_msg.data = self.total_base_length
        self.publisher.publish(base_msg)

        # Log the result
        self.get_logger().info(f'Orientation.x: {current_angle}, Base Length: {self.total_base_length}')

        # Update the previous angle
        self.previous_angle = current_angle

def main(args=None):
    # Initialize ROS 2
    rclpy.init(args=args)

    # Create the IMU Base Node
    imu_base_node = IMUBaseNode()

    # Spin the node
    rclpy.spin(imu_base_node)

    # Shutdown
    imu_base_node.destroy_node()
    rclpy.shutdown()

if _name_ == '_main_':
    main()
    
    
    
    
    
