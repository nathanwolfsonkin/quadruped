import numpy as np

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData
from energy_model.kinematics_data.data_post_process import VideoDataProcess

from energy_model.freq_analysis import *

def main():
    script_calcs()
    hand_calcs()


def hand_calcs():
    # Set parameters 
    quad_m = 9
    leg_m = 1
    l = 1
    I = 1
    g = 9.81
    
    # State trajectory
    t_0 = 0
    t_f = np.pi/4
    time_f = 1
    res = 1000
    timelist = np.linspace(0, time_f, res)
    theta = np.linspace(t_0, t_f, res)
    dtheta = np.gradient(theta,timelist)

    # Calculate total work done
    work = 0
    for i in range(len(theta) - 1):
        work += np.abs(calc_energy(leg_m, l, I, theta[i+1], dtheta[i+1]) - calc_energy(leg_m, l, I, theta[i], dtheta[i]))

    # Account for 4 legs
    work *= 4

    # Calculate total distance moved
    dist = l *  np.abs(np.sin(theta[-1]) - np.sin(theta[0]))

    # Calculate Cost of Transport
    cot = work / (quad_m*g*dist)

    print('\n Script')
    print('W: ', work)
    print('d: ', dist)
    print('m: ', quad_m)
    print('cot: ', cot)

def calc_energy(m, l, I, theta, dtheta):
    g = 9.81 # [m/s^2]    
    
    # Define the energy in the system at any given time
    # Assume CoM is at l/2
    U_g = .5 * m * g * l * np.cos(theta)
    K_t = .25 * m * l * dtheta
    K_w = .5 * I * dtheta**2
    E = U_g + K_t + K_w

    return E

def script_calcs():
    # Define quadruped parameters
    leg_params = {'l': [1, 1], 'I': [1, 1], 'm': [1, 1]}
    body_params = {'l': 1, 'I': 1, 'm': 1, 'origin': [0, 0], 'orientation': 0}
    quadruped = Quadruped(leg_params, body_params)

    # Generate unit test trajectory
    res = 100
    timelist = np.linspace(0, 1, res)

    # Create constant velocity trajector
    unit_traj = [[],[]]
    unit_traj[0] = np.linspace(0, 0, res).tolist()
    unit_traj[1] = np.linspace(0, -np.pi/4, res).tolist()

    gait_data = [unit_traj, unit_traj, unit_traj, unit_traj]

    quad_gait = QuadrupedData(quadruped, timelist, gait_data)

    print('\n Script')
    print('W: ', quad_gait.calc_work_done())
    print('d: ', quad_gait.calc_distance())
    print('velocity: ', quad_gait.calculate_vel())
    print('m: ', quad_gait.quadruped.m)
    print('cot: ', quad_gait.cost_of_transport())


if __name__ == "__main__":
    main()