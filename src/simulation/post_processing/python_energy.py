import numpy as np
import yaml
import matplotlib.pyplot as plt

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData

from simulation.post_processing.csv_data_processer import CSVDataProcesser
import simulation.parameters as sim_params

class PythonEnergy(CSVDataProcesser):
    def __init__(self, quadruped_params_file: str, gait_data_file: str, override_velocity: bool):
        
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
        self.data_dict = self.csv_to_dict(gait_data_file)
        
        timelist = self.data_dict['time']
        
        gait_data_pos = [
            [self.data_dict['FR_thigh_pos'], self.data_dict['FR_calf_pos']],
            [self.data_dict['FL_thigh_pos'], self.data_dict['FL_calf_pos']],
            [self.data_dict['RL_thigh_pos'], self.data_dict['RL_calf_pos']],
            [self.data_dict['RR_thigh_pos'], self.data_dict['RR_calf_pos']]
            ]
        
        self.quadruped_data = QuadrupedData(quadruped=quadruped, timelist=timelist, gait_data=gait_data_pos)
        
        if override_velocity == True:
            self.manual_velocity_override()
        
    def manual_velocity_override(self):
        gait_data_vel = [
            [self.data_dict['FR_thigh_vel'], self.data_dict['FR_calf_vel']],
            [self.data_dict['FL_thigh_vel'], self.data_dict['FL_calf_vel']],
            [self.data_dict['RL_thigh_vel'], self.data_dict['RL_calf_vel']],
            [self.data_dict['RR_thigh_vel'], self.data_dict['RR_calf_vel']]
            ]
        self.quadruped_data.manual_set_velocity(gait_data_vel)