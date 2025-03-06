import yaml
import csv
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

import simulation.parameters as sim_params

class GazeboGaitLogger(Node):

    def __init__(self):
        super().__init__('gazebo_gait_logger')

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
        log_dir = os.path.expanduser(sim_params.logging_directory)
        os.makedirs(log_dir, exist_ok=True)

        # Create a timestamped log file
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(log_dir, f"joint_states_{timestamp}.csv")

        # Open CSV file and write headers
        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time",
                             "FL_thigh_pos", "FL_calf_pos", "FR_thigh_pos", "FR_calf_pos",
                             "RL_thigh_pos", "RL_calf_pos", "RR_thigh_pos", "RR_calf_pos",
                             "FL_thigh_vel", "FL_calf_vel", "FR_thigh_vel", "FR_calf_vel",
                             "RL_thigh_vel", "RL_calf_vel", "RR_thigh_vel", "RR_calf_vel"])
        
        ##########################  INITILIZATIONS  #################################################

        # Default time
        self.sim_time = 0.0
        self.state_dict = {}
        self.input_dict = {}
        
        ######################## SUBSCRIPTIONS AND TIMERS  #########################################
        
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.logging_timer = self.create_timer(sim_params.logging_interval, self.log_data, clock=self.get_clock())

        
    def clock_callback(self, msg_in: Clock):        
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9
    
    def torque_callback(self):
        pass
    
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
            
            # Create sub-dict if not already created
            if leg not in self.state_dict:
                self.state_dict[leg] = {}
            
            for joint in ['hip','thigh','calf']:
                
                # Create sub-dict if not already created
                if joint not in self.state_dict[leg]:
                    self.state_dict[leg][joint] = {}

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
            
    # Called at timer intervals based on simulation clock
    def log_data(self):
        """Logs quadruped joint states at regular intervals."""
        
        if self.sim_time == 0.0:
            return  # Avoid logging before time is set

        data = [
            self.sim_time,
            self.state_dict['FL']['thigh']['pos'], self.state_dict['FL']['calf']['pos'],  # FL
            self.state_dict['FR']['thigh']['pos'], self.state_dict['FR']['calf']['pos'],  # FR
            self.state_dict['RL']['thigh']['pos'], self.state_dict['RL']['calf']['pos'],  # RL
            self.state_dict['RR']['thigh']['pos'], self.state_dict['RR']['calf']['pos'],  # RR
            self.state_dict['FL']['thigh']['vel'], self.state_dict['FL']['calf']['vel'],  # FL velocity
            self.state_dict['FR']['thigh']['vel'], self.state_dict['FR']['calf']['vel'],  # FR velocity
            self.state_dict['RL']['thigh']['vel'], self.state_dict['RL']['calf']['vel'],  # RL velocity
            self.state_dict['RR']['thigh']['vel'], self.state_dict['RR']['calf']['vel'],  # RR velocity
        ]

        # Append data to CSV file
        with open(self.csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)
            
def main(args=None):
    rclpy.init(args=args)
    energy = GazeboGaitLogger()
    rclpy.spin(energy)
    energy.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()