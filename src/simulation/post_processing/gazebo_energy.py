from scipy.integrate import cumtrapz
from simulation.post_processing.csv_data_processer import CSVDataProcesser
import simulation.parameters as sim_params

class GazeboEnergy(CSVDataProcesser):
    def __init__(self, file: str):
        self.data_dict = self.csv_to_dict(file)
        self.calc_power()
        self.calc_total_energy()

    # Takes data dictionary containing torque and velocity and calculates applied power at each index
    def calc_power(self):
        # Create 
        for leg in ['FR', 'FL', 'RL', 'RR']:
            for joint in ['hip', 'thigh', 'calf']:
                key = leg + "_" + joint + "_pow"
                self.data_dict[key] = []
                
        self.data_dict['power'] = []
        for i, value in enumerate(self.data_dict['time']):
            
            for leg in ['FR', 'FL', 'RL', 'RR']:
                for joint in ['hip', 'thigh', 'calf']:
                    prefix = leg + "_" + joint + "_"
                    self.data_dict[prefix + 'pow'].append(self.data_dict[prefix + 'tor'][i] + self.data_dict[prefix + 'vel'][i])
                    
            # Calculate instantaneous power consumption in total quadruped
            inst_power = (
                self.data_dict['FL_hip_pow'][i] +
                self.data_dict['FL_thigh_pow'][i] +
                self.data_dict['FL_calf_pow'][i] +
                self.data_dict['FR_hip_pow'][i] +
                self.data_dict['FR_thigh_pow'][i] +
                self.data_dict['FR_calf_pow'][i] +
                self.data_dict['RL_hip_pow'][i] +
                self.data_dict['RL_thigh_pow'][i] +
                self.data_dict['RL_calf_pow'][i] +
                self.data_dict['RR_hip_pow'][i] +
                self.data_dict['RR_thigh_pow'][i] +
                self.data_dict['RR_calf_pow'][i]
            )
            self.data_dict['power'].append(inst_power)

    def calc_total_energy(self):
        self.total_energy = cumtrapz(self.data_dict['power'], x=self.data_dict['time'])

def main():
    gazebo_energy_data = GazeboEnergy(sim_params.current_log_directory)
        
if __name__ == "__main__":
    main()