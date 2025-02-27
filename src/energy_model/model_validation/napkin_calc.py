import numpy as np

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData
from energy_model.kinematics_data.data_post_process import VideoDataProcess
from energy_model.freq_analysis import *  

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
    
    # Contains quadrupedal parameters and state
    quadruped = Quadruped(unitree_a1_leg_params, unitree_a1_body_params)

    # Contains data state trajectory
    quadruped_traj = QuadrupedData(quadruped, timelist=timelist, gait_data=gait_data)

    # Populate quadruped_traj with appropriate trajectory data
    for i, leg in enumerate(gait_data):
        quadruped_traj.leg_list[i].t1, quadruped_traj.leg_list[i].t2 = leg

    # Define high res timelist
    res_multiplier = 4
    high_res_timelist = np.linspace(0,timelist[-1], len(timelist)*res_multiplier)

    # Contains state trajctory approximation based on fourier analysis
    quadruped_traj_fourier = QuadrupedData(quadruped=quadruped, timelist=high_res_timelist)

    # Populate quadruped_traj_fourier with appropriate trajectory data
    for i, leg in enumerate(gait_data):
        t1_approx = fourier_approx(to_timeseries(quadruped_traj.timelist, quadruped_traj.leg_list[i].t1), resolution_multiplier=res_multiplier)
        t2_approx = fourier_approx(to_timeseries(quadruped_traj.timelist, quadruped_traj.leg_list[i].t2), resolution_multiplier=res_multiplier)
        quadruped_traj_fourier.leg_list[i].t1 = t1_approx
        quadruped_traj_fourier.leg_list[i].t2 = t2_approx
    
    # Propogate changes throughout class
    quadruped_traj_fourier.refresh()
    
    work = quadruped_traj_fourier.calc_work_done() # work done over a 2 second period
    
    # convert to power rate per hour
    work_per_hr = work * 60 * 60 / 2

    # convert 2 seconds to 2.5 hours
    max_runtime_hr = 2.5
    work_over_2_5_hours = work_per_hr * max_runtime_hr
    work_over_2_5_hours_kj = work_over_2_5_hours/1000.

    # Total battery capacity in watthours
    total_capacity_wh = 90.72
    joules_per_wh = 3600

    # Total battery capacity in joules
    total_capacity_j = total_capacity_wh * joules_per_wh
    total_capacity_kj = total_capacity_j/1000

    # Total time for approximated gait to expend battery capacity
    approx_time = total_capacity_j / work_per_hr
    
    print('\n\nTotal work done in joules over 2.5 hours for approximated gait:', work_over_2_5_hours_kj)
    print('Actual battery capacity in joules:                             ', total_capacity_kj)

    print('\n\nTotal time for approximated gait to expend battery capacity:   ', approx_time)
    print('Actual maximum runtime in hours:                               ', max_runtime_hr)
    
    print('\n\n')

if __name__ == "__main__":
    main()