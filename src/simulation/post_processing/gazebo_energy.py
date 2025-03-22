from scipy.integrate import cumtrapz
from simulation.post_processing.csv_data_processer import CSVDataProcesser
import simulation.parameters as sim_params

class GazeboEnergy(CSVDataProcesser):
    def __init__(self, file):
        self.data_dict = self.csv_to_dict(file)
        self.calc_power()
        self.calc_total_energy()

    # Takes data dictionary containing torque and velocity and calculates applied power at each index
    def calc_power(self):
        self.data_dict['power'] = []
        for i, value in enumerate(self.data_dict['time']):
            # Calculate instantaneous power consumption
            inst_power = (
                self.data_dict['FL_hip_tor'][i] * self.data_dict['FL_hip_vel'][i] + 
                self.data_dict['FL_thigh_tor'][i] * self.data_dict['FL_thigh_vel'][i] + 
                self.data_dict['FL_calf_tor'][i] * self.data_dict['FL_calf_vel'][i] +
                self.data_dict['FR_hip_tor'][i] * self.data_dict['FR_hip_vel'][i] + 
                self.data_dict['FR_thigh_tor'][i] * self.data_dict['FR_thigh_vel'][i] + 
                self.data_dict['FR_calf_tor'][i] * self.data_dict['FR_calf_vel'][i] +
                self.data_dict['RL_hip_tor'][i] * self.data_dict['RL_hip_vel'][i] + 
                self.data_dict['RL_thigh_tor'][i] * self.data_dict['RL_thigh_vel'][i] + 
                self.data_dict['RL_calf_tor'][i] * self.data_dict['RL_calf_vel'][i] +
                self.data_dict['RR_hip_tor'][i] * self.data_dict['RR_hip_vel'][i] + 
                self.data_dict['RR_thigh_tor'][i] * self.data_dict['RR_thigh_vel'][i] + 
                self.data_dict['RR_calf_tor'][i] * self.data_dict['RR_calf_vel'][i]
            )
            self.data_dict['power'].append(inst_power)

    def calc_total_energy(self):
        self.total_energy = cumtrapz(self.data_dict['power'], x=self.data_dict['time'])

def main():
    gazebo_energy_data = GazeboEnergy(sim_params.current_log_directory)
        
if __name__ == "__main__":
    main()