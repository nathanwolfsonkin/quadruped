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

class RawDataLogger(Node):

    def __init__(self):
        super().__init__('raw_data_logger')

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
        log_dir = os.path.expanduser(sim_params.raw_logging_directory)
        os.makedirs(log_dir, exist_ok=True)

        # Create a timestamped log file
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(log_dir, f"joint_states_{timestamp}.csv")

        # Open CSV file and write headers
        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "time",
                "FL_hip_pos", "FL_thigh_pos", "FL_calf_pos",
                "FR_hip_pos", "FR_thigh_pos", "FR_calf_pos",
                "RL_hip_pos", "RL_thigh_pos", "RL_calf_pos",
                "RR_hip_pos", "RR_thigh_pos", "RR_calf_pos",
                "FL_hip_vel", "FL_thigh_vel", "FL_calf_vel",
                "FR_hip_vel", "FR_thigh_vel", "FR_calf_vel",
                "RL_hip_vel", "RL_thigh_vel", "RL_calf_vel",
                "RR_hip_vel", "RR_thigh_vel", "RR_calf_vel",
                "FL_hip_tor", "FL_thigh_tor", "FL_calf_tor",
                "FR_hip_tor", "FR_thigh_tor", "FR_calf_tor",
                "RL_hip_tor", "RL_thigh_tor", "RL_calf_tor",
                "RR_hip_tor", "RR_thigh_tor", "RR_calf_tor",
                "distance"
                ])
        
        ##########################  INITILIZATIONS  #################################################

        # Default time
        self.sim_time = 0.0
        self.state_dict = {}
        for leg in ['FR', 'FL', 'RL', 'RR']:
            self.state_dict[leg] = {}
            
            for joint in ['hip', 'thigh', 'calf']:
                self.state_dict[leg][joint] = {}
                
                for state in ['pos', 'vel', 'tor']:
                    self.state_dict[leg][joint][state] = 0.0
                
        
        
        ######################## SUBSCRIPTIONS AND TIMERS  #########################################
        
        self.create_subscription(Float64, '/quadruped/FR_hip_joint_torque', self.FR_hip_torque, 10)
        self.create_subscription(Float64, '/quadruped/FR_thigh_joint_torque', self.FR_thigh_torque, 10)
        self.create_subscription(Float64, '/quadruped/FR_calf_joint_torque', self.FR_calf_torque, 10)
        
        self.create_subscription(Float64, '/quadruped/FL_hip_joint_torque', self.FL_hip_torque, 10)
        self.create_subscription(Float64, '/quadruped/FL_thigh_joint_torque', self.FL_thigh_torque, 10)
        self.create_subscription(Float64, '/quadruped/FL_calf_joint_torque', self.FL_calf_torque, 10)
        
        self.create_subscription(Float64, '/quadruped/RL_hip_joint_torque', self.RL_hip_torque, 10)
        self.create_subscription(Float64, '/quadruped/RL_thigh_joint_torque', self.RL_thigh_torque, 10)
        self.create_subscription(Float64, '/quadruped/RL_calf_joint_torque', self.RL_calf_torque, 10)
        
        self.create_subscription(Float64, '/quadruped/RR_hip_joint_torque', self.RR_hip_torque, 10)
        self.create_subscription(Float64, '/quadruped/RR_thigh_joint_torque', self.RR_thigh_torque, 10)
        self.create_subscription(Float64, '/quadruped/RR_calf_joint_torque', self.RR_calf_torque, 10)
        
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.logging_timer = self.create_timer(sim_params.raw_logging_interval, self.log_data, clock=self.get_clock())
        
    # Called at timer intervals based on simulation clock
    def log_data(self):
        """Logs quadruped joint states at regular intervals."""
        
        # Avoid logging before start recording time
        if self.sim_time < sim_params.start_recording_time:
            return

        data = [
            self.sim_time - sim_params.start_recording_time,
            self.state_dict['FL']['hip']['pos'], self.state_dict['FL']['thigh']['pos'], self.state_dict['FL']['calf']['pos'],  # FL position
            self.state_dict['FR']['hip']['pos'], self.state_dict['FR']['thigh']['pos'], self.state_dict['FR']['calf']['pos'],  # FR position
            self.state_dict['RL']['hip']['pos'], self.state_dict['RL']['thigh']['pos'], self.state_dict['RL']['calf']['pos'],  # RL position
            self.state_dict['RR']['hip']['pos'], self.state_dict['RR']['thigh']['pos'], self.state_dict['RR']['calf']['pos'],  # RR position
            self.state_dict['FL']['hip']['vel'], self.state_dict['FL']['thigh']['vel'], self.state_dict['FL']['calf']['vel'],  # FL velocity
            self.state_dict['FR']['hip']['vel'], self.state_dict['FR']['thigh']['vel'], self.state_dict['FR']['calf']['vel'],  # FR velocity
            self.state_dict['RL']['hip']['vel'], self.state_dict['RL']['thigh']['vel'], self.state_dict['RL']['calf']['vel'],  # RL velocity
            self.state_dict['RR']['hip']['vel'], self.state_dict['RR']['thigh']['vel'], self.state_dict['RR']['calf']['vel'],  # RR velocity
            self.state_dict['FL']['hip']['tor'], self.state_dict['FL']['thigh']['tor'], self.state_dict['FL']['calf']['tor'],  # FL torque
            self.state_dict['FR']['hip']['tor'], self.state_dict['FR']['thigh']['tor'], self.state_dict['FR']['calf']['tor'],  # FR torque
            self.state_dict['RL']['hip']['tor'], self.state_dict['RL']['thigh']['tor'], self.state_dict['RL']['calf']['tor'],  # RL torque
            self.state_dict['RR']['hip']['tor'], self.state_dict['RR']['thigh']['tor'], self.state_dict['RR']['calf']['tor'],  # RR torque
            self.state_dict['distance'] - self.dist_offset,
        ]

        # Append data to CSV file
        with open(self.csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)
    
    # Updates the quadruped state based on the gazebo model
    def joint_state_callback(self, msg_in: JointState):
        
        # Account for the NaN velocity case
        try: 
            for index, _ in enumerate(msg_in.velocity):
                msg_in.velocity[index]
        except:
            for index in range(11):
                msg_in.velocity[index] = 0.0
        
        for leg in ['FR','FL','RL','RR']:
            
            for joint in ['hip','thigh','calf']:

                for state in ['pos','vel']:
                    
                    id = leg + '_' + joint + '_' + state
                    match id:
                        # Position
                        case 'FL_hip_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[0]
                        case 'FL_thigh_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[1]
                        case 'FL_calf_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[2]
                        case 'FR_hip_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[3]
                        case 'FR_thigh_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[4]
                        case 'FR_calf_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[5]
                        case 'RL_hip_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[6]
                        case 'RL_thigh_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[7]
                        case 'RL_calf_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[8]
                        case 'RR_hip_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[9]
                        case 'RR_thigh_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[10]
                        case 'RR_calf_pos':
                            self.state_dict[leg][joint][state] = msg_in.position[11]
                        
                        # Velocity
                        case 'FL_hip_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[0]
                        case 'FL_thigh_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[1]
                        case 'FL_calf_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[2]
                        case 'FR_hip_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[3]
                        case 'FR_thigh_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[4]
                        case 'FR_calf_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[5]
                        case 'RL_hip_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[6]
                        case 'RL_thigh_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[7]
                        case 'RL_calf_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[8]
                        case 'RR_hip_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[9]
                        case 'RR_thigh_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[10]
                        case 'RR_calf_vel':
                            self.state_dict[leg][joint][state] = msg_in.velocity[11]
                        
                        # Failed to match
                        case _:
                            raise ValueError('Case was not matched for id: ', leg + '_' + joint + '_' + state)
        
        self.state_dict['distance'] = msg_in.position[12]
        if self.sim_time < sim_params.start_recording_time:
            self.dist_offset = msg_in.position[12]
                        
    def clock_callback(self, msg_in: Clock):        
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9
        
        if self.sim_time >= sim_params.stop_recording_time:
            self.get_logger().info("Simulation Complete. Stopping Logging")
            rclpy.shutdown()
    
    def FR_hip_torque(self, msg:Float64):
        self.state_dict['FR']['hip']['tor'] = msg.data
    
    def FR_thigh_torque(self, msg:Float64):
        self.state_dict['FR']['thigh']['tor'] = msg.data
    
    def FR_calf_torque(self, msg:Float64):
        self.state_dict['FR']['calf']['tor'] = msg.data
    
    def FL_hip_torque(self, msg:Float64):
        self.state_dict['FL']['hip']['tor'] = msg.data
    
    def FL_thigh_torque(self, msg:Float64):
        self.state_dict['FL']['thigh']['tor'] = msg.data
    
    def FL_calf_torque(self, msg:Float64):
        self.state_dict['FL']['calf']['tor'] = msg.data
    
    def RL_hip_torque(self, msg:Float64):
        self.state_dict['RL']['hip']['tor'] = msg.data
    
    def RL_thigh_torque(self, msg:Float64):
        self.state_dict['RL']['thigh']['tor'] = msg.data
    
    def RL_calf_torque(self, msg:Float64):
        self.state_dict['RL']['calf']['tor'] = msg.data
    
    def RR_hip_torque(self, msg:Float64):
        self.state_dict['RR']['hip']['tor'] = msg.data
    
    def RR_thigh_torque(self, msg:Float64):
        self.state_dict['RR']['thigh']['tor'] = msg.data
    
    def RR_calf_torque(self, msg:Float64):
        self.state_dict['RR']['calf']['tor'] = msg.data
        

def main(args=None):
    rclpy.init(args=args)
    raw_data_logger = RawDataLogger()
    rclpy.spin(raw_data_logger)
    raw_data_logger.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()