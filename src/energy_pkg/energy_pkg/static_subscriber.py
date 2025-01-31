import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage

class StaticSubscriber(Node):

    def __init__(self):
        super().__init__('static_subscriber')

        self.s = self.create_subscription(TFMessage, '/tf', self.static_transform_callback, 1)
        print("INITILIZED NODE")

    def static_transform_callback(self, msg_in):
        print("ENTERED THE CALLBACK FUNCTION")
        self.get_logger().info(msg_in)


def main(args=None):
    rclpy.init(args=args)
    subscriber = StaticSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()