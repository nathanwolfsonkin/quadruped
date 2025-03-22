import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock

import simulation.parameters as sim_params

class LowPassFilterNode(Node):
    def __init__(self):
        super().__init__('low_pass_filter')
        
        # Get parameters
        self.cutoff_frequency = sim_params.tor_filt_cutoff_freq
        self.sampling_rate = sim_params.filt_sampling_rate
        self.dt = 1.0 / self.sampling_rate
        self.tau = 1.0 / (2 * np.pi * self.cutoff_frequency)
        self.alpha = self.dt / (self.tau + self.dt)
        
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        self.sim_time = 0.0
        
        # Initialize filter state
        self.filtered_value = None
        
        # Initilize input dict
        self.raw_input_dict = {
            'FR_hip': 0.0, 'FR_thigh': 0.0, 'FR_calf': 0.0,
            'FL_hip': 0.0, 'FL_thigh': 0.0, 'FL_calf': 0.0,
            'RL_hip': 0.0, 'RL_thigh': 0.0, 'RL_calf': 0.0,
            'RR_hip': 0.0, 'RR_thigh': 0.0, 'RR_calf': 0.0
        }
        
        self.filt_input_dict = {
            'FR_hip': 0.0, 'FR_thigh': 0.0, 'FR_calf': 0.0,
            'FL_hip': 0.0, 'FL_thigh': 0.0, 'FL_calf': 0.0,
            'RL_hip': 0.0, 'RL_thigh': 0.0, 'RL_calf': 0.0,
            'RR_hip': 0.0, 'RR_thigh': 0.0, 'RR_calf': 0.0
        }
        
        # Subscribers and publishers
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
        
        
        self.filtered_signal_publisher = self.create_publisher(JointState, 'filtered_torque', 10)
        
        self.pub_timer = self.create_timer(.001, self.signal_callback, clock=self.get_clock())

    def signal_callback(self):
        
        for key in self.filt_input_dict:
            self.filt_input_dict[key] = self.alpha * self.raw_input_dict[key] + (1 - self.alpha) * self.filt_input_dict[key]
        
        # create output message
        filtered_signals = JointState()
        
        filtered_signals.name = [
            'FR_hip', 'FR_thigh', 'FR_calf',
            'FL_hip', 'FL_thigh', 'FL_calf',
            'RL_hip', 'RL_thigh', 'RL_calf',
            'RR_hip', 'RR_thigh', 'RR_calf'
        ]
                
        filtered_signals.effort = [
            self.filt_input_dict['FR_hip'], self.filt_input_dict['FR_thigh'], self.filt_input_dict['FR_calf'],
            self.filt_input_dict['FL_hip'], self.filt_input_dict['FL_thigh'], self.filt_input_dict['FL_calf'],
            self.filt_input_dict['RL_hip'], self.filt_input_dict['RL_thigh'], self.filt_input_dict['RL_calf'],
            self.filt_input_dict['RR_hip'], self.filt_input_dict['RR_thigh'], self.filt_input_dict['RR_calf']
        ]

        self.filtered_signal_publisher.publish(filtered_signals)
        
    def clock_callback(self, msg_in: Clock):        
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9

    def FR_hip_torque(self, msg:Float64):
        self.raw_input_dict['FR_hip'] = msg.data
    
    def FR_thigh_torque(self, msg:Float64):
        self.raw_input_dict['FR_thigh'] = msg.data
    
    def FR_calf_torque(self, msg:Float64):
        self.raw_input_dict['FR_calf'] = msg.data
    
    def FL_hip_torque(self, msg:Float64):
        self.raw_input_dict['FL_hip'] = msg.data
    
    def FL_thigh_torque(self, msg:Float64):
        self.raw_input_dict['FL_thigh'] = msg.data
    
    def FL_calf_torque(self, msg:Float64):
        self.raw_input_dict['FL_calf'] = msg.data
    
    def RL_hip_torque(self, msg:Float64):
        self.raw_input_dict['RL_hip'] = msg.data
    
    def RL_thigh_torque(self, msg:Float64):
        self.raw_input_dict['RL_thigh'] = msg.data
    
    def RL_calf_torque(self, msg:Float64):
        self.raw_input_dict['RL_calf'] = msg.data
    
    def RR_hip_torque(self, msg:Float64):
        self.raw_input_dict['RR_hip'] = msg.data
    
    def RR_thigh_torque(self, msg:Float64):
        self.raw_input_dict['RR_thigh'] = msg.data
    
    def RR_calf_torque(self, msg:Float64):
        self.raw_input_dict['RR_calf'] = msg.data
        
        
def main(args=None):
    rclpy.init(args=args)
    node = LowPassFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
