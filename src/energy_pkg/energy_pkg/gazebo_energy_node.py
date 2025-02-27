import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

class GazeboEnergyNode(Node):

    def __init__(self):
        super().__init__('gazebo_energy_node')

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
        
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.state_dict = {}
        
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # ROS Clock to use simulation time
        self.clock = self.get_clock()

        # Subscribe to the /clock topic to update time
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)

        # Default time
        self.sim_time = 0.0

    # def calc_inst_power(self):
    #     J = 
    #     omega = self.state_dict[leg][joint]['vel']
    #     omegadot = # Get effort from jointstate?
    #     tau_g = leg_length/2 * 
    #     joint_power = J * omega * omegadot - tau_g * omega
        
    def clock_callback(self, msg_in: Clock):        
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9
        
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

def main(args=None):
    rclpy.init(args=args)
    energy = GazeboEnergyNode()
    rclpy.spin(energy)
    energy.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()