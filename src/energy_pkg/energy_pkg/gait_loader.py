import csv

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

class GaitLoader(Node):

    def __init__(self):
        super().__init__('gait_loader')
        
        # Set up publishers for every quadruped command topic
        self.FR_hip_publisher = self.create_publisher(Float64, '/quadruped/cmd_FR_hip_joint', 10)
        self.FR_thigh_publisher = self.create_publisher(Float64, '/quadruped/cmd_FR_thigh_joint', 10)
        self.FR_calf_publisher = self.create_publisher(Float64, '/quadruped/cmd_FR_calf_joint', 10)
        
        self.FL_hip_publisher = self.create_publisher(Float64, '/quadruped/cmd_FL_hip_joint', 10)
        self.FL_thigh_publisher = self.create_publisher(Float64, '/quadruped/cmd_FL_thigh_joint', 10)
        self.FL_calf_publisher = self.create_publisher(Float64, '/quadruped/cmd_FL_calf_joint', 10)
        
        self.RL_hip_publisher = self.create_publisher(Float64, '/quadruped/cmd_RL_hip_joint', 10)
        self.RL_thigh_publisher = self.create_publisher(Float64, '/quadruped/cmd_RL_thigh_joint', 10)
        self.RL_calf_publisher = self.create_publisher(Float64, '/quadruped/cmd_RL_calf_joint', 10)
        
        self.RR_hip_publisher = self.create_publisher(Float64, '/quadruped/cmd_RR_hip_joint', 10)
        self.RR_thigh_publisher = self.create_publisher(Float64, '/quadruped/cmd_RR_thigh_joint', 10)
        self.RR_calf_publisher = self.create_publisher(Float64, '/quadruped/cmd_RR_calf_joint', 10)
        
        self.pub_dict = {
            'FR_hip':self.FR_hip_publisher,
            'FR_thigh':self.FR_thigh_publisher,
            'FR_calf':self.FR_calf_publisher,
            'FL_hip':self.FL_hip_publisher,
            'FL_thigh':self.FL_thigh_publisher,
            'FL_calf':self.FL_calf_publisher,
            'RL_hip':self.RL_hip_publisher,
            'RL_thigh':self.RL_thigh_publisher,
            'RL_calf':self.RL_calf_publisher,
            'RR_hip':self.RR_hip_publisher,
            'RR_thigh':self.RR_thigh_publisher,
            'RR_calf':self.RR_calf_publisher,
        }
        
        
        # Set up clock
        # Use simulated time
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])

        # Subscribe to the /clock topic to update time
        self.clock_subscriber = self.create_subscription(Clock, '/clock', self.update_clock, 10)

        # Default time
        self.sim_time = 0.0

        # Create timer using the ROS clock (not a Clock message!)
        self.timer = self.create_timer(0.001, self.send_command, clock=self.get_clock())
        
        # Load CSV data as a dictionary of lists
        file_path = '/workspace/install/energy_pkg/share/energy_pkg/gait_trajectory/approx_gait_traj.csv'
        self.trajectory_data = self.load_csv(file_path)
        
        self.current_index = 0

    def send_command(self):
        
        if self.current_index != len(self.trajectory_data['timelist']) - 1:
            next_time = self.trajectory_data['timelist'][self.current_index + 1]
            
            if self.sim_time >= next_time:
                self.current_index += 1
        
        # Publish to quadruped commands
        for leg in ['FR','FL','RL','RR']:
            for joint in ['hip','thigh','calf']:
                msg_out = Float64()
                msg_out.data = self.trajectory_data[leg + '_' + joint][self.current_index]
                self.pub_dict[leg + '_' + joint].publish(msg_out)
                


    def update_clock(self, msg_in: Clock):
        self.sim_time = msg_in.clock.sec + msg_in.clock.nanosec * 1e-9

    def load_csv(self, file_path):
        """Load CSV data as a dictionary of lists where each column is a key."""
        data = {}
        try:
            with open(file_path, mode='r') as file:
                reader = csv.DictReader(file)
                for key in reader.fieldnames:
                    data[key] = []  # Initialize lists for each column
                
                for row in reader:
                    for key in row:
                        data[key].append(float(row[key]))  # Convert values to float

            # self.get_logger().info(f"Loaded CSV file with {len(data['timelist'])} rows")
        except Exception as e:
            self.get_logger().error(f"Failed to load CSV file: {e}")
        return data

def main(args=None):
    rclpy.init(args=args)
    gait_loader = GaitLoader()
    rclpy.spin(gait_loader)
    gait_loader.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()