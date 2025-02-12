import csv

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock

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

        # Create timer using the ROS clock (not a Clock message!)
        self.timer = self.create_timer(0.001, self.send_command, clock=self.get_clock())
        
        self.trajectory_reset()
        
    def loop_trajectory(self):
        # Shift the trajectory data back to it's original position
        if self.first_reset == True:
            # self.get_logger().info(f"Entered First Reset")
            for index, _ in enumerate(self.trajectory_data['timelist']):
                if index == 0:
                    self.trajectory_data['timelist'][index] += self.delay_time
            
            self.first_reset = False        
            
        time_offset = self.sim_time - (self.trajectory_data['timelist'][0])
                
        for index, _ in enumerate(self.trajectory_data['timelist']):
            self.trajectory_data['timelist'][index] += time_offset
                
        self.current_index = 0

    def send_command(self):
        # If at end of list, loop the list
        if self.current_index == len(self.trajectory_data['timelist']) - 1:
            self.loop_trajectory()               
        else: # else check if next trajectory should be loaded
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
        if self.sim_time == 0.0:
            self.trajectory_reset()
            
    def trajectory_reset(self):
        # Load CSV data as a dictionary of lists
        file_path = '/workspace/install/energy_pkg/share/energy_pkg/gait_trajectory/approx_gait_traj.csv'
        self.trajectory_data = self.load_csv(file_path)
                
        # Delay the inputs by some amount of time
        self.delay_time = 5 #seconds
        for index, _ in enumerate(self.trajectory_data['timelist']):
            if index != 0:
                self.trajectory_data['timelist'][index] += self.delay_time
                        
        self.first_reset = True
        self.current_index = 0

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