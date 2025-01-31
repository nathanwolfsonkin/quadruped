import yaml

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from tf2_msgs.msg import TFMessage

from src.energy_model.quadruped_energy import Quadruped

class EnergyNode(Node):

    def __init__(self):
        super().__init__('energy_node')

        # Gather quadruped parameters from single source of truth yaml file
        config_file = "/workspace/src/quadruped_description/config/params.yaml"

        with open(config_file, 'r') as file:
            params = yaml.safe_load(file)
        
        
        self.quadruped = Quadruped(leg_params={'l':params['LF']['L1'], 'I':1, 'm':1},
                                   body_params={'l':1, 'I':1, 'm':1, 'origin':[0,0], 'orientation':0})
        
        # Initialize transfrom values
        tf = {
            "FL_hip":   0,
            "FL_thigh": 0,
            "FL_calf":  0,
        }

        self.s = self.create_subscription(TFMessage, '/tf', self.transform_update_callback, 1)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.print_all_frames)  # Print frames every 1 second

    def print_all_frames(self):
        try:
            frames = self.tf_buffer.all_frames_as_yaml()
            self.get_logger().info(f"All Frames:\n{frames}")
        except Exception as e:
            self.get_logger().error(f"Error retrieving frames: {e}")

    def transform_update_callback(self, msg_in):

        
        pass


def main(args=None):
    rclpy.init(args=args)
    energy = EnergyNode()
    rclpy.spin(energy)
    energy.destroy_node()
    rclpy.shutdown()


def test():

    pass

    
if __name__ == '__main__':
    test()