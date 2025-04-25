import yaml
import sympy as sp

from energy_model.quadruped_energy import Quadruped, QuadrupedData

class SyntheticEnergy:
    def __init__(self, quadruped_params_file: str, gait_params_file: str, timelist: list):
        
        # Define the quadruped using the simulation
        with open(quadruped_params_file, 'r') as file:
            params = yaml.safe_load(file)
        
        # Define quadruped parameters
        leg_params = {
            'l': [params['FR']['L2'], 
                params['FR']['L3']], 
            'I': [params['FR']['I2']['Izz'], 
                params['FR']['I3']['Izz']], 
            'm': [params['FR']['m2'], 
                params['FR']['m3']]
            }
        
        body_params = {
            'l': params['base']['Lx'], 
            'I': params['base']['I']['Iyy'], 
            'm': params['base']['m'], 
            'origin': [0, 0], 
            'orientation': 0
            }
        
        quadruped = Quadruped(leg_params=leg_params, body_params=body_params)
        
        # Read in the gait data achieved by the simulation
        self.calulate_synthetic_kinematics(gait_params_file)
        gait_pos_data, gait_vel_data = self.generate_gait_data(timelist)
        
        self.quadruped_data = QuadrupedData(quadruped=quadruped, timelist=timelist, gait_data=gait_pos_data)
        self.quadruped_data.manual_set_velocity(gait_vel_data)
        self.generate_distance_list()

    def generate_distance_list(self):
        self.dist_list = []
        velocity = self.quadruped_data.calculate_vel()
        for index, value in enumerate(self.quadruped_data.timelist):
            if index == 0:
                offset = value
            
            current_dist = self.quadruped_data.calc_distance(time_override=True, time_index=index, vel_override=True, vel=velocity) - offset
            self.dist_list.append(current_dist)


        
    def generate_gait_data(self, timelist: list):
        gait_pos_data = [[[],[]],[[],[]],[[],[]],[[],[]]]
        gait_vel_data = [[[],[]],[[],[]],[[],[]],[[],[]]]
        for index, time in enumerate(timelist):
            for leg in self.joint_functions:
                
                # Map leg name to leg number
                match leg:
                    case 'FR':
                        leg_index = 0
                    case 'FL':
                        leg_index = 1
                    case 'RL':
                        leg_index = 2
                    case 'RR':
                        leg_index = 3
                
                for joint in self.joint_functions[leg]:
                    
                    # Map joint name to joint number
                    match joint:
                        case 'thigh':
                            joint_index = 0
                            valid_joint = True
                        case 'calf':
                            joint_index = 1
                            valid_joint = True
                        case 'hip':
                            joint_index = None
                            valid_joint = False
                    
                    if valid_joint:
                        gait_pos_data[leg_index][joint_index].append(self.joint_functions[leg][joint]['pos'](time))
                        gait_vel_data[leg_index][joint_index].append(self.joint_functions[leg][joint]['vel'](time))
        
        return gait_pos_data, gait_vel_data
    
    def calulate_synthetic_kinematics(self, gait_params_file: str) -> None:
        # Load the YAML file
        with open(gait_params_file, "r") as file:
            data = yaml.safe_load(file)

        # Define symbolic time variable
        t = sp.Symbol("t")

        # Function to construct position and velocity equations
        def generate_joint_functions(joint_data):
            """Generate symbolic position and velocity functions for a joint."""
            position_expr = 0  # Initialize position expression
            for key, (freq, amp, phase) in joint_data.items():
                position_expr += amp * sp.cos(2 * sp.pi * freq * t + phase)

            # Compute the symbolic derivative
            velocity_expr = sp.diff(position_expr, t)

            return position_expr, velocity_expr

        # Dictionary to store symbolic functions for each joint
        joint_functions = {}

        for leg, joints in data.items():
            joint_functions[leg] = {}
            for joint, joint_data in joints.items():
                # Convert list format to dictionary format
                joint_terms = {k: v for k, v in enumerate(joint_data.values())}
                
                pos_func, vel_func = generate_joint_functions(joint_terms)
                joint_functions[leg][joint] = {
                    "pos": sp.lambdify(t, pos_func, 'numpy'),
                    "vel": sp.lambdify(t, vel_func, 'numpy')
                }
                
        self.joint_functions = joint_functions

def main():
    import matplotlib.pyplot as plt
    import simulation.parameters as sim_params
    from simulation.post_processing.gazebo_energy import GazeboEnergy
    
    
    # Get timelist
    raw_data_log = sim_params.get_latest_log(sim_params.raw_logging_directory)
    raw_gz = GazeboEnergy(raw_data_log)
    timelist = raw_gz.data_dict['time']
    
    # Get gait_params_file
    gait_params_file = sim_params.analytical_gait_params_file
    
    # Get quadruped_params_file
    quadruped_params_file = sim_params.quadruped_params_file
    
    synthetic_energy = SyntheticEnergy(quadruped_params_file, gait_params_file, timelist)
    
    plt.figure()
    plt.title('Synthetic Kinematics')
    plt.plot(synthetic_energy.quadruped_data.timelist, synthetic_energy.quadruped_data.leg_list[0].t1, label='Synthetic Leg Position')
    plt.plot(synthetic_energy.quadruped_data.timelist, synthetic_energy.quadruped_data.leg_list[0].dt1, label='Synthetic Leg Velocity')
    plt.legend()
        
    
if __name__ == "__main__":
    main()