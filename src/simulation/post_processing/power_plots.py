import matplotlib.pyplot as plt

import simulation.parameters as sim_params
from simulation.post_processing.gazebo_energy import GazeboEnergy
from simulation.post_processing.python_energy import PythonEnergy
from simulation.post_processing.low_pass_filter import Filter
            
class PowerPlots:
    def __init__(self):
        
        # Desired Data Analysis
        desired_data_log = sim_params.get_latest_log(sim_params.desired_logging_directory)
        
        # Raw Data Analysis
        raw_data_log = sim_params.get_latest_log(sim_params.raw_logging_directory)
        
        # Filtering of position, velocity, and torque
        # Generate filtered CSV log
        Filter.generate_filtered_csv(raw_data_log)
        filtered_data_log = sim_params.get_latest_log(sim_params.postprocess_filtered_logging_directory)

        self.model_dict = {
            'synth': {
                'gz': None,
                'py': PythonEnergy(sim_params.quadruped_params_file, desired_data_log, override_velocity=False)
            },
            'raw': {
                'gz': GazeboEnergy(raw_data_log),
                'py': PythonEnergy(sim_params.quadruped_params_file, raw_data_log, override_velocity=True)
            },
            'filt': {
                'gz': GazeboEnergy(filtered_data_log),
                'py': PythonEnergy(sim_params.quadruped_params_file, filtered_data_log, override_velocity=True)
            }
        }
    
    def python_power(self):
        # Generate a set of filtered data
        plt.figure()
        plt.title('Python Power')
        plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.power_trajectory(), label='Synthetic Python Power')
        plt.plot(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.power_trajectory(), label='Raw Python Power')
        plt.plot(self.model_dict['filt']['py'].quadruped_data.timelist, self.model_dict['filt']['py'].quadruped_data.power_trajectory(), label='Filtered Python Power')
        plt.legend()
    
    def debug(self):
        def synthetic_energy(self):
            plt.figure()
            plt.title('Synthetic Energy')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.energy_trajectory(), label='Synthetic Python Power')
            plt.legend()
        
        def manual_vel_test(self):
            
            self.model_dict['raw']['py'].quadruped_data.refresh()
            
            plt.figure()
            plt.title('Velocity Noise Comparison')
            plt.plot(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.leg_list[1].dt2, label='Python Computed Angular Velocity')

            self.model_dict['raw']['py'].manual_velocity_override()
            
            plt.plot(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.leg_list[1].dt2, label='Gazebo Computed Angluar Velocity')
            plt.legend()
            
        def synthetic_position(self):
            plt.figure()
            plt.title('Synthetic Kinematics')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].t1, label='Synthetic Thigh Position')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].t2, label='Synthetic Calf Position')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt1, label='Synthetic Thigh Velocity')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt2, label='Synthetic Calf Velocity')
            plt.legend()
            
        synthetic_position(self)
        # synthetic_energy(self)
        # manual_vel_test(self)
            
        
    
        
    
def main():
    # It seems that the thight timesteps + the noise is resulting in nonsense data in the python model since power is calculated as d/dt (energy)
    # The solution should involve a filtered position. This means that signal analysis needs to be done on the positional data   
    power_plots = PowerPlots()
    # power_plots.python_power()
    power_plots.debug()
    plt.show()

if __name__ == "__main__":
    main()