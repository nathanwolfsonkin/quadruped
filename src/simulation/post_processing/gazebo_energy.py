import numpy as np
from scipy.integrate import cumtrapz
import yaml

from simulation.post_processing.csv_data_processer import CSVDataProcesser


class GazeboEnergy(CSVDataProcesser):
    def __init__(self, datafile: str, paramfile: str):
        self.quadruped_param_file = paramfile
        self.data_dict = self.csv_to_dict(datafile)
        self.mass = self.calc_total_mass()
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
                    self.data_dict[prefix + 'pow'].append(self.data_dict[prefix + 'tor'][i] * self.data_dict[prefix + 'vel'][i])
                    
            # Calculate instantaneous power consumption in total quadruped
            inst_power = (
                np.abs(self.data_dict['FL_hip_pow'][i]) +
                np.abs(self.data_dict['FL_thigh_pow'][i]) +
                np.abs(self.data_dict['FL_calf_pow'][i]) +
                np.abs(self.data_dict['FR_hip_pow'][i]) +
                np.abs(self.data_dict['FR_thigh_pow'][i]) +
                np.abs(self.data_dict['FR_calf_pow'][i]) +
                np.abs(self.data_dict['RL_hip_pow'][i]) +
                np.abs(self.data_dict['RL_thigh_pow'][i]) +
                np.abs(self.data_dict['RL_calf_pow'][i]) +
                np.abs(self.data_dict['RR_hip_pow'][i]) +
                np.abs(self.data_dict['RR_thigh_pow'][i]) +
                np.abs(self.data_dict['RR_calf_pow'][i])
            )
            self.data_dict['power'].append(inst_power)

    def calc_total_energy(self):
        self.total_energy = cumtrapz(self.data_dict['power'], x=self.data_dict['time'])
    
    def calc_total_mass(self):
        with open(self.quadruped_param_file, 'r') as file:
            quadruped_params = yaml.safe_load(file)
        
        mass = (quadruped_params['base']['m'] + 
                # FR
                quadruped_params['FR']['m1'] + 
                quadruped_params['FR']['m2'] + 
                quadruped_params['FR']['m3'] + 
                quadruped_params['FR']['m4'] + 
                # FL
                quadruped_params['FL']['m1'] + 
                quadruped_params['FL']['m2'] + 
                quadruped_params['FL']['m3'] + 
                quadruped_params['FL']['m4'] + 
                # RR
                quadruped_params['RR']['m1'] + 
                quadruped_params['RR']['m2'] + 
                quadruped_params['RR']['m3'] + 
                quadruped_params['RR']['m4'] + 
                # RL
                quadruped_params['RL']['m1'] + 
                quadruped_params['RL']['m2'] + 
                quadruped_params['RL']['m3'] + 
                quadruped_params['RL']['m4'])
        
        return mass
        
    def cost_of_transport(self):
        cot = self.total_energy[-1] / (self.mass * self.data_dict['distance'][-1])
        return cot

def main():
    pass
        
if __name__ == "__main__":
    main()