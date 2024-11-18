import numpy as np

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData
from energy_model.kinematics_data.data_post_process import VideoDataProcess

from energy_model.freq_analysis import *


def main():
    # Define quadruped parameters
    unitree_a1_leg_params = {'l': [.2, .2], 'I': [0.0055, 0.003], 'm': [1.013, 0.166]}
    unitree_a1_body_params = {'l': 2*0.1805, 'I': 0.01, 'm': 5.660, 'origin': [0, 0], 'orientation': 0}
    quadruped = Quadruped(unitree_a1_leg_params, unitree_a1_body_params)

    # Collect data from video
    dataset = VideoDataProcess("unitree_a1")
    gait_data = dataset.get_angle_lists()

    # Define resolution multiplier
    res_multiplier = 4

    # Get timing information
    frame_time, total_time, total_frames = dataset.get_frame_data()
    timelist = np.linspace(0, total_time, total_frames)

    # Leg 1 hip
    leg1_hip = fourier_approx(to_timeseries(timelist, gait_data[0][0]), resolution_multiplier=res_multiplier)
   
    # Leg 1 knee
    leg1_knee = fourier_approx(to_timeseries(timelist, gait_data[0][1]), resolution_multiplier=res_multiplier)
    
    # Leg 2 hip
    leg2_hip = fourier_approx(to_timeseries(timelist, gait_data[1][0]), resolution_multiplier=res_multiplier)
    
    # Leg 2 knee
    leg2_knee = fourier_approx(to_timeseries(timelist, gait_data[1][1]), resolution_multiplier=res_multiplier)

    gait_data_approx = [[leg1_hip,leg1_knee],[leg2_hip,leg2_knee]]
    gait_data_approx = gait_data_approx + gait_data_approx # Concatenate the gaits such that 1=3 & 2=4 for a trot

    # Define updated timelist
    timelist = np.linspace(0, total_time, total_frames*res_multiplier)

    gait_info = QuadrupedData(quadruped, timelist, gait_data_approx)

    cot = gait_info.cost_of_transport()
    print(cot)

if __name__ == "__main__":
    main()