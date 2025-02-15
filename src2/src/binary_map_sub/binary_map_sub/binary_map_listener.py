import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import numpy as np

class BinaryMapSubscriber(Node):
    def __init__(self):
        super().__init__('binary_map_subscriber')
        self.subscription = self.create_subscription(
            Int8MultiArray,
            '/binary_map',
            self.binary_map_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

    def binary_map_callback(self, msg):
        # Extract dimensions from the layout
        if len(msg.layout.dim) < 2:
            self.get_logger().warn("Received message without dimensions!")
            return

        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size

        # Convert 1D data to 2D NumPy array
        binary_map = np.array(msg.data, dtype=np.int8).reshape((rows, cols))

        # Print for debugging
        self.get_logger().info(f"Received binary map: \n{binary_map}")

        # Perform further processing here (e.g., visualization, path planning, etc.)

def main(args=None):
    rclpy.init(args=args)
    node = BinaryMapSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
