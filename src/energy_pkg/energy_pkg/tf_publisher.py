import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

class TFMonitorNode(Node):
    def __init__(self):
        super().__init__('tf_monitor_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.print_all_frames)  # Print frames every 1 second

    def print_all_frames(self):
        try:
            frames = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f"All Frames:\n{frames}")
        except Exception as e:
            self.get_logger().error(f"Error retrieving frames: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TFMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
