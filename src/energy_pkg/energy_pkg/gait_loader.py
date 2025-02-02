import rclpy
from rclpy.node import Node

from src.energy_model.quadruped_energy import Quadruped

class GaitLoader(Node):

    def __init__(self):
        super().__init__('energy_node')
        

def main(args=None):
    rclpy.init(args=args)
    gait_loader = GaitLoader()
    rclpy.spin(gait_loader)
    gait_loader.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()