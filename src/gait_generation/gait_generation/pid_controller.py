import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

import simulation.parameters as params

class GaitLoader(Node):

    def __init__(self):
        super().__init__('pid_controller')
        
        self.desired_state_dict = {
            'FR_hip' : 0.0,
            'FR_thigh' : 0.0,
            'FR_calf' : 0.0, 
            'FL_hip' : 0.0,
            'FL_thigh' : 0.0,
            'FL_calf' : 0.0, 
            'RL_hip' : 0.0,
            'RL_thigh' : 0.0,
            'RL_calf' : 0.0,
            'RR_hip' : 0.0,
            'RR_thigh' : 0.0,
            'RR_calf' : 0.0
        }      
        
        self.true_state_dict = {
            'FR_hip' : 0.0,
            'FR_thigh' : 0.0,
            'FR_calf' : 0.0, 
            'FL_hip' : 0.0,
            'FL_thigh' : 0.0,
            'FL_calf' : 0.0, 
            'RL_hip' : 0.0,
            'RL_thigh' : 0.0,
            'RL_calf' : 0.0,
            'RR_hip' : 0.0,
            'RR_thigh' : 0.0,
            'RR_calf' : 0.0
        }      
        
        self.error_dict = {
            'FR_hip' : 0.0,
            'FR_thigh' : 0.0,
            'FR_calf' : 0.0, 
            'FL_hip' : 0.0,
            'FL_thigh' : 0.0,
            'FL_calf' : 0.0, 
            'RL_hip' : 0.0,
            'RL_thigh' : 0.0,
            'RL_calf' : 0.0,
            'RR_hip' : 0.0,
            'RR_thigh' : 0.0,
            'RR_calf' : 0.0
        }
        
        self.prev_error_dict = {
            'FR_hip' : 0.0,
            'FR_thigh' : 0.0,
            'FR_calf' : 0.0, 
            'FL_hip' : 0.0,
            'FL_thigh' : 0.0,
            'FL_calf' : 0.0, 
            'RL_hip' : 0.0,
            'RL_thigh' : 0.0,
            'RL_calf' : 0.0,
            'RR_hip' : 0.0,
            'RR_thigh' : 0.0,
            'RR_calf' : 0.0
        }
        
        self.input_dict = {
            'FR_hip' : 0.0,
            'FR_thigh' : 0.0,
            'FR_calf' : 0.0, 
            'FL_hip' : 0.0,
            'FL_thigh' : 0.0,
            'FL_calf' : 0.0, 
            'RL_hip' : 0.0,
            'RL_thigh' : 0.0,
            'RL_calf' : 0.0,
            'RR_hip' : 0.0,
            'RR_thigh' : 0.0,
            'RR_calf' : 0.0
        }   
        
        self.desired_subscription_dict = {
            'FR_hip' : self.create_subscription(Float64, '/quadruped/des/FR_hip_joint', self.FR_hip_callback, 10),
            'FR_thigh' : self.create_subscription(Float64, '/quadruped/des/FR_thigh_joint', self.FR_thigh_callback, 10),    
            'FR_calf' : self.create_subscription(Float64, '/quadruped/des/FR_calf_joint', self.FR_calf_callback, 10), 
            'FL_hip' : self.create_subscription(Float64, '/quadruped/des/FL_hip_joint', self.FL_hip_callback, 10),
            'FL_thigh' : self.create_subscription(Float64, '/quadruped/des/FL_thigh_joint', self.FL_thigh_callback, 10),
            'FL_calf' : self.create_subscription(Float64, '/quadruped/des/FL_calf_joint', self.FL_calf_callback, 10), 
            'RL_hip' : self.create_subscription(Float64, '/quadruped/des/RL_hip_joint', self.RL_hip_callback, 10),
            'RL_thigh' : self.create_subscription(Float64, '/quadruped/des/RL_thigh_joint', self.RL_thigh_callback, 10),
            'RL_calf' : self.create_subscription(Float64, '/quadruped/des/RL_calf_joint', self.RL_calf_callback, 10),
            'RR_hip' : self.create_subscription(Float64, '/quadruped/des/RR_hip_joint', self.RR_hip_callback, 10),
            'RR_thigh' : self.create_subscription(Float64, '/quadruped/des/RR_thigh_joint', self.RR_thigh_callback, 10),    
            'RR_calf' : self.create_subscription(Float64, '/quadruped/des/RR_calf_joint', self.RR_calf_callback, 10)
        }  
        
        self.true_state_subscriber = self.create_subscription(JointState, 'joint_states', self.true_state_callback, 10)
        
        self.pub_dict = {}
        for leg in ['FR', 'FL', 'RL', 'RR']:
            for joint in ['hip', 'thigh', 'calf']:
                topic_name = f'/quadruped/cmd/{leg}_{joint}_joint'
                self.pub_dict[f'{leg}_{joint}'] = self.create_publisher(Float64, topic_name, 10)
                
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Subscribe to the /clock topic to update time
        self.sim_time = 0.0
        self.prev_time = -0.1
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.update_clock, 10)
        
        # Create timer using the ROS clock (not a Clock message!)
        self.timer = self.create_timer(0.0001, self.send_command, clock=self.get_clock())


    def send_command(self):
        self.compute_error()
        
        for key in self.error_dict:
            self.input_dict[key] = (
                params.K_p * self.error_dict[key] +
                params.K_d * (self.error_dict[key] - self.prev_error_dict[key]) / (self.sim_time - self.prev_time)
                )
            input = Float64()
            input.data = self.input_dict[key]
            self.pub_dict[key].publish(input)

    def compute_error(self):
        for key in self.error_dict:
            self.prev_error_dict[key] = self.error_dict[key]
            self.error_dict[key] = self.desired_state_dict[key] - self.true_state_dict[key]

    def true_state_callback(self, msg_in: JointState):
        for key in self.true_state_dict:
            match key:
                case 'FL_hip_pos':
                    self.true_state_dict[key] = msg_in.position[0]
                case 'FL_thigh_pos':
                    self.true_state_dict[key] = msg_in.position[1]
                case 'FL_calf_pos':
                    self.true_state_dict[key] = msg_in.position[2]
                case 'FR_hip_pos':
                    self.true_state_dict[key] = msg_in.position[3]
                case 'FR_thigh_pos':
                    self.true_state_dict[key] = msg_in.position[4]
                case 'FR_calf_pos':
                    self.true_state_dict[key] = msg_in.position[5]
                case 'RL_hip_pos':
                    self.true_state_dict[key] = msg_in.position[6]
                case 'RL_thigh_pos':
                    self.true_state_dict[key] = msg_in.position[7]
                case 'RL_calf_pos':
                    self.true_state_dict[key] = msg_in.position[8]
                case 'RR_hip_pos':
                    self.true_state_dict[key] = msg_in.position[9]
                case 'RR_thigh_pos':
                    self.true_state_dict[key] = msg_in.position[10]
                case 'RR_calf_pos':
                    self.true_state_dict[key] = msg_in.position[11]
    
    def update_clock(self, msg_in: Clock):
        self.prev_time = self.sim_time
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9
        

    def FR_hip_callback(self, msg : Float64):
        self.desired_state_dict['FR_hip'] = msg.data

    def FR_thigh_callback(self, msg : Float64):
        self.desired_state_dict['FR_thigh'] = msg.data

    def FR_calf_callback(self, msg : Float64):
        self.desired_state_dict['FR_calf'] = msg.data
        
    def FL_hip_callback(self, msg : Float64):
        self.desired_state_dict['FL_hip'] = msg.data

    def FL_thigh_callback(self, msg : Float64):
        self.desired_state_dict['FL_thigh'] = msg.data

    def FL_calf_callback(self, msg : Float64):
        self.desired_state_dict['FL_calf'] = msg.data
        
    def RL_hip_callback(self, msg : Float64):
        self.desired_state_dict['RL_hip'] = msg.data

    def RL_thigh_callback(self, msg : Float64):
        self.desired_state_dict['RL_thigh'] = msg.data

    def RL_calf_callback(self, msg : Float64):
        self.desired_state_dict['RL_calf'] = msg.data
        
    def RR_hip_callback(self, msg : Float64):
        self.desired_state_dict['RR_hip'] = msg.data

    def RR_thigh_callback(self, msg : Float64):
        self.desired_state_dict['RR_thigh'] = msg.data

    def RR_calf_callback(self, msg : Float64):
        self.desired_state_dict['RR_calf'] = msg.data
        
    

def main(args=None):
    rclpy.init(args=args)
    gait_loader = GaitLoader()
    rclpy.spin(gait_loader)
    gait_loader.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()