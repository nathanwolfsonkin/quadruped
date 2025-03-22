import yaml
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

import simulation.parameters as sim_params

class GaitLoader(Node):

    def __init__(self):
        super().__init__('gait_loader')
        
        # Set up publishers for every quadruped command topic
        self.pub_dict = {}
        for leg in ['FR', 'FL', 'RL', 'RR']:
            for joint in ['hip', 'thigh', 'calf']:
                topic_name = f'/quadruped/cmd_{leg}_{joint}_joint'
                self.pub_dict[f'{leg}_{joint}'] = self.create_publisher(Float64, topic_name, 10)        
        
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Subscribe to the /clock topic to update time
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.update_clock, 10)

        # Default time
        self.sim_time = 0.0
        
        # Define analytical gait functions        
        self.gait_functions()
        
        # Create timer using the ROS clock (not a Clock message!)
        self.timer = self.create_timer(sim_params.gait_command_publishing_rate, self.send_command, clock=self.get_clock())

    def compute_trajectory(self, t, data: dict):
        """Computes the sum of sine waves based on frequency, amplitude, and phase."""
        total = 0
        for entry in data.values():
            freq = entry[0]
            amp = entry[1]
            phase = entry[2]
            total += amp * np.cos(2 * np.pi * freq * t + phase)
        return total

    def gait_functions(self):
        filepath = '/workspace/install/gait_generation/share/gait_generation/gait_trajectory/gait_sin_waves.yaml'
        traj_data = self.load_yaml(filepath)

        self.traj_funcs = {}

        for leg in traj_data:
            self.traj_funcs[leg] = {}

            for angle in traj_data[leg]:
                freq_amp_phase_list = traj_data[leg][angle]

                # Define a dedicated function for each leg-angle pair
                def trajectory_function(t, data=freq_amp_phase_list.copy()):
                    return self.compute_trajectory(t, data)

                self.traj_funcs[leg][angle] = trajectory_function

    def send_command(self):
        start_time = sim_params.start_walking_time
        # Publish to quadruped commands
        for leg in ['FR','FL','RL','RR']:
            for joint in ['hip','thigh','calf']:
                msg_out = Float64()
                if self.sim_time < start_time:
                    msg_out.data = self.traj_funcs[leg][joint](start_time)
                else:
                    msg_out.data = self.traj_funcs[leg][joint](self.sim_time)
                
                self.pub_dict[leg + '_' + joint].publish(msg_out)
        

    def update_clock(self, msg_in: Clock):
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9

    def load_yaml(self, file_path):
        data = {}
        try:
            with open(file_path, mode='r') as file:
                data = yaml.safe_load(file)

        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file: {e}")
        
        return data

def main(args=None):
    rclpy.init(args=args)
    gait_loader = GaitLoader()
    rclpy.spin(gait_loader)
    gait_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()