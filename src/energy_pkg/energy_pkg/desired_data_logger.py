import yaml
import csv
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64

import simulation.parameters as sim_params

class DesiredDataLogger(Node):

    def __init__(self):
        super().__init__('desired_data_logger')

        ###################################  SETUP PARAMS DICTIONARY  ###########################################
        # Gather quadruped parameters from single source of truth yaml file
        config_file = "/workspace/install/quadruped_description/share/quadruped_description/config/params.yaml"

        with open(config_file, 'r') as file:
            params = yaml.safe_load(file)
        
        legs = {}
        for leg in ['FL','FR','RL','RR']:
            legs[leg] = {
                'l':[params[leg]['L2'], params[leg]['L3']],
                'I':[params[leg]['I2']['Izz'], params[leg]['I3']['Izz']],
                'm':[params[leg]['m2'], params[leg]['m3']],
            }
        
        ############################  SET UP CLOCK  ##########################################
        
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        ##########################  SET UP CSV  #################################################
        
        # Setup logging directory
        log_dir = os.path.expanduser(sim_params.desired_logging_directory)
        os.makedirs(log_dir, exist_ok=True)

        # Create a timestamped log file
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(log_dir, f"joint_states_{timestamp}.csv")

        # Open CSV file and write headers
        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "time",
                "FL_thigh_pos", "FL_calf_pos",
                "FR_thigh_pos", "FR_calf_pos",
                "RL_thigh_pos", "RL_calf_pos",
                "RR_thigh_pos", "RR_calf_pos",
                ])
        
        ##########################  INITILIZATIONS  #################################################

        # Default time
        self.sim_time = 0.0
        self.state_dict = {}
        for leg in ['FR', 'FL', 'RL', 'RR']:
            self.state_dict[leg] = {}
            
            for joint in ['hip', 'thigh', 'calf']:
                self.state_dict[leg][joint] = {}
                
                for state in ['pos']:
                    self.state_dict[leg][joint][state] = 0.0
                
        
        
        ######################## SUBSCRIPTIONS AND TIMERS  #########################################
        
        self.create_subscription(Float64, '/quadruped/cmd_FR_hip_joint', self.FR_hip_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_FR_thigh_joint', self.FR_thigh_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_FR_calf_joint', self.FR_calf_position, 10)
        
        self.create_subscription(Float64, '/quadruped/cmd_FL_hip_joint', self.FL_hip_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_FL_thigh_joint', self.FL_thigh_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_FL_calf_joint', self.FL_calf_position, 10)
        
        self.create_subscription(Float64, '/quadruped/cmd_RL_hip_joint', self.RL_hip_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_RL_thigh_joint', self.RL_thigh_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_RL_calf_joint', self.RL_calf_position, 10)
        
        self.create_subscription(Float64, '/quadruped/cmd_RR_hip_joint', self.RR_hip_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_RR_thigh_joint', self.RR_thigh_position, 10)
        self.create_subscription(Float64, '/quadruped/cmd_RR_calf_joint', self.RR_calf_position, 10)
        
        self.logging_timer = self.create_timer(sim_params.synthetic_logging_interval, self.log_data, clock=self.get_clock())
        
    # Called at timer intervals based on simulation clock
    def log_data(self):
        """Logs quadruped joint states at regular intervals."""
        
        # Avoid logging before start recording time
        if self.sim_time < sim_params.start_recording_time:
            return

        data = [
            self.sim_time,
            self.state_dict['FL']['thigh']['pos'], self.state_dict['FL']['calf']['pos'],  # FL
            self.state_dict['FR']['thigh']['pos'], self.state_dict['FR']['calf']['pos'],  # FR
            self.state_dict['RL']['thigh']['pos'], self.state_dict['RL']['calf']['pos'],  # RL
            self.state_dict['RR']['thigh']['pos'], self.state_dict['RR']['calf']['pos'],  # RR
        ]

        # Append data to CSV file
        with open(self.csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)
                        
    def clock_callback(self, msg_in: Clock):        
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9
        
        if self.sim_time >= sim_params.stop_recording_time:
            self.get_logger().info("Simulation Complete. Stopping Logging")
            rclpy.shutdown()
    
    def FR_hip_position(self, msg:Float64):
        self.state_dict['FR']['hip']['pos'] = msg.data
    
    def FR_thigh_position(self, msg:Float64):
        self.state_dict['FR']['thigh']['pos'] = msg.data
    
    def FR_calf_position(self, msg:Float64):
        self.state_dict['FR']['calf']['pos'] = msg.data
    
    def FL_hip_position(self, msg:Float64):
        self.state_dict['FL']['hip']['pos'] = msg.data
    
    def FL_thigh_position(self, msg:Float64):
        self.state_dict['FL']['thigh']['pos'] = msg.data
    
    def FL_calf_position(self, msg:Float64):
        self.state_dict['FL']['calf']['pos'] = msg.data
    
    def RL_hip_position(self, msg:Float64):
        self.state_dict['RL']['hip']['pos'] = msg.data
    
    def RL_thigh_position(self, msg:Float64):
        self.state_dict['RL']['thigh']['pos'] = msg.data
    
    def RL_calf_position(self, msg:Float64):
        self.state_dict['RL']['calf']['pos'] = msg.data
    
    def RR_hip_position(self, msg:Float64):
        self.state_dict['RR']['hip']['pos'] = msg.data
    
    def RR_thigh_position(self, msg:Float64):
        self.state_dict['RR']['thigh']['pos'] = msg.data
    
    def RR_calf_position(self, msg:Float64):
        self.state_dict['RR']['calf']['pos'] = msg.data
        

def main(args=None):
    rclpy.init(args=args)
    desired_data_logger = DesiredDataLogger()
    rclpy.spin(desired_data_logger)
    desired_data_logger.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()