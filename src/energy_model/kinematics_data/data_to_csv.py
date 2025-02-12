import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
import csv

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData
from energy_model.kinematics_data.data_post_process import VideoDataProcess
from energy_model.freq_analysis import *

class DataToCSV:
    def __init__(self, leg_params: dict, body_params: dict, gait_data: list, timelist: list):

        # Contains quadrupedal parameters and state
        self.quadruped = Quadruped(leg_params=leg_params, body_params=body_params)

        # Contains data state trajectory
        self.quadruped_traj = QuadrupedData(self.quadruped, timelist=timelist, gait_data=gait_data)

        # Populate quadruped_traj with appropriate trajectory data
        for i, leg in enumerate(gait_data):
            self.quadruped_traj.leg_list[i].t1, self.quadruped_traj.leg_list[i].t2 = leg

        # Define high res timelist
        res_multiplier = 4
        high_res_timelist = np.linspace(0,timelist[-1], len(timelist)*res_multiplier)

        # Contains state trajctory approximation based on fourier analysis
        self.quadruped_traj_fourier = QuadrupedData(quadruped=self.quadruped, timelist=high_res_timelist)

        # Populate quadruped_traj_fourier with appropriate trajectory data
        for i, leg in enumerate(gait_data):
            t1_approx = fourier_approx(to_timeseries(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[i].t1), resolution_multiplier=res_multiplier)
            t2_approx = fourier_approx(to_timeseries(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[i].t2), resolution_multiplier=res_multiplier)
            self.quadruped_traj_fourier.leg_list[i].t1 = t1_approx
            self.quadruped_traj_fourier.leg_list[i].t2 = t2_approx
        
        # Propogate changes throughout class
        self.quadruped_traj_fourier.refresh()
        
    def data_to_cv(self):
        # Create a list of csv headers
        headers = [
            "timelist",
            "FL_hip",
            "FL_thigh",
            "FL_calf",
            "RL_hip",
            "RL_thigh", 
            "RL_calf",
            "RR_hip",
            "RR_thigh",
            "RR_calf",
            "FR_hip",
            "FR_thigh", 
            "FR_calf",   
        ]
        
        # Prepare the data
        raw_data = []
        for i, time in enumerate(self.quadruped_traj.timelist):
            row = [
                time,
                0.0,
                self.quadruped_traj.leg_list[0].t1[i],
                self.quadruped_traj.leg_list[0].t2[i],
                0.0,
                self.quadruped_traj.leg_list[1].t1[i],
                self.quadruped_traj.leg_list[1].t2[i],
                0.0,
                self.quadruped_traj.leg_list[0].t1[i],
                self.quadruped_traj.leg_list[0].t2[i],
                0.0,
                self.quadruped_traj.leg_list[1].t1[i],
                self.quadruped_traj.leg_list[1].t2[i]
            ]
            raw_data.append(row)

        approx_data = []
        for i, time in enumerate(self.quadruped_traj_fourier.timelist):
            row = [
                time,
                0.0,
                self.quadruped_traj_fourier.leg_list[0].t1[i],
                self.quadruped_traj_fourier.leg_list[0].t2[i],
                0.0,
                self.quadruped_traj_fourier.leg_list[1].t1[i],
                self.quadruped_traj_fourier.leg_list[1].t2[i],
                0.0,
                self.quadruped_traj_fourier.leg_list[0].t1[i],
                self.quadruped_traj_fourier.leg_list[0].t2[i],
                0.0,
                self.quadruped_traj_fourier.leg_list[1].t1[i],
                self.quadruped_traj_fourier.leg_list[1].t2[i]
            ]
            approx_data.append(row)

        # Write data to CSV file
        with open('/workspace/src/energy_pkg/gait_trajectory/raw_gait_traj.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)
            writer.writerows(raw_data)

        with open('/workspace/src/energy_pkg/gait_trajectory/approx_gait_traj.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(headers)
            writer.writerows(approx_data)

    def in_phase(self):
        # Define the 
        
        # Define 8 dimensional state for leg positions (ignores hip)
        state0 = []
        for leg in self.quadruped_traj_fourier.leg_list:
            state0.append(leg.t1[0])
            state0.append(leg.t2[0])
        
        vect0 = np.array(state0)
        opt_error = 100 #pick big number to initialize
        
        for index, value in enumerate(self.quadruped_traj_fourier.leg_list[0].t1):
            state = []
            for leg in self.quadruped_traj_fourier.leg_list:
                state.append(leg.t1[index])
                state.append(leg.t2[index])
            
            vect = np.array(state)
            
            if index > 20:
                error = np.linalg.norm(vect - vect0)
                if error < opt_error:
                    best_index = index
                    opt_error = error
        
        self.quadruped_traj_fourier.timelist = self.quadruped_traj_fourier.timelist[:best_index]    
        self.quadruped_traj_fourier.leg_list[0].t1 = self.quadruped_traj_fourier.leg_list[0].t1[:best_index]
        self.quadruped_traj_fourier.leg_list[0].t2 = self.quadruped_traj_fourier.leg_list[0].t2[:best_index]
        self.quadruped_traj_fourier.leg_list[1].t1 = self.quadruped_traj_fourier.leg_list[1].t1[:best_index]
        self.quadruped_traj_fourier.leg_list[1].t2 = self.quadruped_traj_fourier.leg_list[1].t2[:best_index]
        self.quadruped_traj_fourier.leg_list[2].t1 = self.quadruped_traj_fourier.leg_list[2].t1[:best_index]
        self.quadruped_traj_fourier.leg_list[2].t2 = self.quadruped_traj_fourier.leg_list[2].t2[:best_index]
        self.quadruped_traj_fourier.leg_list[3].t1 = self.quadruped_traj_fourier.leg_list[3].t1[:best_index]
        self.quadruped_traj_fourier.leg_list[3].t2 = self.quadruped_traj_fourier.leg_list[3].t2[:best_index]        
            
        
def main():
    # Define quadruped parameters
    unitree_a1_leg_params = {'l': [.2, .2], 'I': [0.0055, 0.003], 'm': [1.013, 0.166]}
    unitree_a1_body_params = {'l': 2*0.1805, 'I': 0.01, 'm': 5.660, 'origin': [0, 0], 'orientation': 0}
    
    # Collect data from video
    dataset = VideoDataProcess("unitree_a1")
    gait_data = dataset.get_angle_lists()
    gait_data = gait_data + gait_data # Concatenate the gaits such that 1=3 & 2=4 for a trot

    # Get timing information
    frame_time, total_time, total_frames = dataset.get_frame_data()
    timelist = np.linspace(0, total_time, total_frames)
    
    data_to_csv = DataToCSV(leg_params=unitree_a1_leg_params, 
                      body_params=unitree_a1_body_params,
                      gait_data=gait_data,
                      timelist=timelist)
    
    data_to_csv.in_phase()

    data_to_csv.data_to_cv()
    

if __name__ == "__main__":
    main()