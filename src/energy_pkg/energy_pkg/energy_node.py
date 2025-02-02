import yaml
import csv
from datetime import datetime
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

from src.energy_model.quadruped_energy import Quadruped

class EnergyNode(Node):

    def __init__(self):
        super().__init__('energy_node')

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
        
        # Initialize python energy model quadruped
        self.quadruped = Quadruped(FR_params=legs['FR'],
                                   FL_params=legs['FL'],
                                   RL_params=legs['RL'],
                                   RR_params=legs['RR'],
                                   body_params={'l':params['base']['Lx'],
                                                'I':params['base']['I']['Iyy'],
                                                'm':params['base']['m'], 
                                                'origin':[0,0], 'orientation':0})
        
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # ROS Clock to use simulation time
        self.clock = self.get_clock()

        # Subscribe to the /clock topic to update time
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # Default time
        self.sim_time = 0.0  

        # Create timer using the ROS clock (not a Clock message!)
        self.timer = self.create_timer(0.1, self.log_data, clock=self.clock)
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        
        # Setup logging directory
        log_dir = os.path.expanduser("/workspace/energy_logs")
        os.makedirs(log_dir, exist_ok=True)

        # Create a timestamped log file
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.csv_filename = os.path.join(log_dir, f"joint_states_{timestamp}.csv")

        # Open CSV file and write headers
        with open(self.csv_filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["time", "FL_thigh_pos", "FL_calf_pos", "FR_thigh_pos", "FR_calf_pos",
                             "RL_thigh_pos", "RL_calf_pos", "RR_thigh_pos", "RR_calf_pos",
                             "FL_thigh_vel", "FL_calf_vel", "FR_thigh_vel", "FR_calf_vel",
                             "RL_thigh_vel", "RL_calf_vel", "RR_thigh_vel", "RR_calf_vel"])

        
    def log_data(self):
        """Logs quadruped joint states at regular intervals."""
        
        if self.sim_time == 0.0:
            return  # Avoid logging before time is set

        data = [
            self.sim_time,
            self.quadruped.leg_list[1].t1, self.quadruped.leg_list[1].t2,  # FL
            self.quadruped.leg_list[0].t1, self.quadruped.leg_list[0].t2,  # FR
            self.quadruped.leg_list[2].t1, self.quadruped.leg_list[2].t2,  # RL
            self.quadruped.leg_list[3].t1, self.quadruped.leg_list[3].t2,  # RR
            self.quadruped.leg_list[1].dt1, self.quadruped.leg_list[1].dt2,  # FL velocities
            self.quadruped.leg_list[0].dt1, self.quadruped.leg_list[0].dt2,  # FR velocities
            self.quadruped.leg_list[2].dt1, self.quadruped.leg_list[2].dt2,  # RL velocities
            self.quadruped.leg_list[3].dt1, self.quadruped.leg_list[3].dt2,  # RR velocities
        ]

        # Append data to CSV file
        with open(self.csv_filename, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)

        self.get_logger().info(f"Logged data at {self.sim_time:.2f} sec")
        
        
    def clock_callback(self, msg_in: Clock):        
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9
        
    # Updates the quadruped state based on the gazebo model
    def joint_state_callback(self, msg_in: JointState):
        """
        0  - FL_hip_joint
        1  - FL_thigh_joint
        2  - FL_calf_joint
        3  - FR_hip_joint
        4  - FR_thigh_joint
        5  - FR_calf_joint
        6  - RL_hip_joint
        7  - RL_thigh_joint
        8  - RL_calf_joint
        9  - RR_hip_joint
        10 - RR_thigh_joint
        11 - RR_calf_joint
        """
        
        # FL_hip_pos = msg_in.position[0]
        FL_thigh_pos = msg_in.position[1]
        FL_calf_pos = msg_in.position[2]
        # FR_hip_pos = msg_in.position[3]
        FR_thigh_pos = msg_in.position[4]
        FR_calf_pos = msg_in.position[5]
        # RL_hip_pos = msg_in.position[6]
        RL_thigh_pos = msg_in.position[7]
        RL_calf_pos = msg_in.position[8]
        # RR_hip_pos = msg_in.position[9]
        RR_thigh_pos = msg_in.position[10]
        RR_calf_pos = msg_in.position[11]
        
        try: 
            # FL_hip_vel = msg_in.velocity[0]
            FL_thigh_vel = msg_in.velocity[1]
            FL_calf_vel = msg_in.velocity[2]
            # FR_hip_vel = msg_in.velocity[3]
            FR_thigh_vel = msg_in.velocity[4]
            FR_calf_vel = msg_in.velocity[5]
            # RL_hip_vel = msg_in.velocity[6]
            RL_thigh_vel = msg_in.velocity[7]
            RL_calf_vel = msg_in.velocity[8]
            # RR_hip_vel = msg_in.velocity[9]
            RR_thigh_vel = msg_in.velocity[10]
            RR_calf_vel = msg_in.velocity[11]
        except:
            # FL_hip_vel = 0.0
            FL_thigh_vel = 0.0
            FL_calf_vel = 0.0
            # FR_hip_vel = 0.0
            FR_thigh_vel = 0.0
            FR_calf_vel = 0.0
            # RL_hip_vel = 0.0
            RL_thigh_vel = 0.0
            RL_calf_vel = 0.0
            # RR_hip_vel = 0.0
            RR_thigh_vel = 0.0
            RR_calf_vel = 0.0
        
        # Update model positions
        self.quadruped.leg_list[0].t1 = FR_thigh_pos # FR_thigh_joint
        self.quadruped.leg_list[0].t2 = FR_calf_pos  # FR_calf_joint
        self.quadruped.leg_list[1].t1 = FL_thigh_pos # FL_thigh_joint
        self.quadruped.leg_list[1].t2 = FL_calf_pos  # FL_calf_joint
        self.quadruped.leg_list[2].t1 = RL_thigh_pos # RL_thigh_joint
        self.quadruped.leg_list[2].t2 = RL_calf_pos  # RL_calf_joint
        self.quadruped.leg_list[3].t1 = RR_thigh_pos # RR_thigh_joint
        self.quadruped.leg_list[3].t2 = RR_calf_pos  # RR_calf_joint
        
        # Update model velocities
        self.quadruped.leg_list[0].dt1 = FR_thigh_vel # FR_thigh_joint
        self.quadruped.leg_list[0].dt2 = FR_calf_vel  # FR_calf_joint
        self.quadruped.leg_list[1].dt1 = FL_thigh_vel # FL_thigh_joint
        self.quadruped.leg_list[1].dt2 = FL_calf_vel  # FL_calf_joint
        self.quadruped.leg_list[2].dt1 = RL_thigh_vel # RL_thigh_joint
        self.quadruped.leg_list[2].dt2 = RL_calf_vel  # RL_calf_joint
        self.quadruped.leg_list[3].dt1 = RR_thigh_vel # RR_thigh_joint
        self.quadruped.leg_list[3].dt2 = RR_calf_vel  # RR_calf_joint
        

def main(args=None):
    rclpy.init(args=args)
    energy = EnergyNode()
    rclpy.spin(energy)
    energy.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()