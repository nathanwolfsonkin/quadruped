import numpy as np
import matplotlib.pyplot as plt

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData

from energy_model.freq_analysis import *

def main():
    # State trajectory
    theta_0 = 0
    theta_f = -np.pi/4
    time_f = 1
    res = 100
    timelist = np.linspace(0, time_f, res)
    theta = np.linspace(theta_0, theta_f, res)
    
    theta = np.sin(theta)

    dtheta = np.gradient(theta,timelist)
    
    test_range = 100

    # Set parameters 
    body_m = 1
    leg_m = 1
    l = 1
    I = 1

    test_index_list = [-1]
    er_W_list = []
    er_d_list = []
    er_m_list = []
    er_cot_list = []

    for body_m_i in range(test_range):
        body_m = body_m_i
        er_W, er_d, er_m, er_cot = calc_error(body_m, leg_m, l, I, theta, dtheta, timelist)
        
        test_index_list.append(test_index_list[-1] + 1)
        er_W_list.append(er_W)
        er_d_list.append(er_d)
        er_m_list.append(er_m)
        er_cot_list.append(er_cot)

    for leg_m_i in range(test_range):
        leg_m = leg_m_i
        er_W, er_d, er_m, er_cot = calc_error(body_m, leg_m, l, I, theta, dtheta, timelist)
        
        test_index_list.append(test_index_list[-1] + 1)
        er_W_list.append(er_W)
        er_d_list.append(er_d)
        er_m_list.append(er_m)
        er_cot_list.append(er_cot)

    for l_i in range(1,test_range+1):
        l = l_i
        er_W, er_d, er_m, er_cot = calc_error(body_m, leg_m, l, I, theta, dtheta, timelist)
        
        test_index_list.append(test_index_list[-1] + 1)
        er_W_list.append(er_W)
        er_d_list.append(er_d)
        er_m_list.append(er_m)
        er_cot_list.append(er_cot)

    for I_i in range(test_range):
        I = I_i
        er_W, er_d, er_m, er_cot = calc_error(body_m, leg_m, l, I, theta, dtheta, timelist)
        
        test_index_list.append(test_index_list[-1] + 1)
        er_W_list.append(er_W)
        er_d_list.append(er_d)
        er_m_list.append(er_m)
        er_cot_list.append(er_cot)

    test_index_list.pop(0)

    plt.figure()
    plt.plot(test_index_list, er_W_list, label='Work Error')
    plt.plot(test_index_list, er_d_list, label='Dist Error')
    plt.plot(test_index_list, er_m_list, label='Mass Error')
    plt.plot(test_index_list, er_cot_list, label='CoT Error')
    plt.title('Unit Test Error Across Parameter Variations')
    plt.xlabel('Test Number (0-100 - Body Mass) (101-200 - Leg Mass) (201-300 - Leg Length) (301-400 - Leg Inertia)')
    plt.ylabel('Error')
    plt.legend()
    plt.show()


def calc_error(body_m, leg_m, l, I, theta, dtheta, timelist):
    # Hand Calcs (work calc via direct change in energy between timesteps, energy calculated via pendulum energy equation)
    W_hand, d_hand, m_hand, cot_hand = hand_calcs(body_m, leg_m, l, I, theta, dtheta)

    # Script Calcs (work calc via integration of power trajectory)
    W_script, d_script, m_script, cot_script = script_calcs(body_m, leg_m, l, I, theta, timelist)

    er_W = abs(W_hand - W_script)
    er_d = abs(d_hand - d_script)
    er_m = abs(m_hand - m_script)
    er_cot = abs(cot_hand - cot_script)
    
    return er_W, er_d, er_m, er_cot

def hand_calcs(body_m, leg_m, l, I, theta, dtheta):
    g = 9.81
    quad_m = body_m + 8*leg_m

    # Calculate total work done
    work = 0
    for i in range(len(theta) - 1):
        work += np.abs(calc_energy(leg_m, l, I, theta[i+1], dtheta[i+1]) - calc_energy(leg_m, l, I, theta[i], dtheta[i]))

    # Account for 4 legs doing the same movement
    work *= 4

    dist = l *  np.abs(np.sin(theta[-1]) - np.sin(theta[0]))


    # Calculate Cost of Transport
    cot = work / (quad_m*g*dist)

    # print('\n Hand')
    # print('W: ', work)
    # print('d: ', dist)
    # print('m: ', quad_m)
    # print('cot: ', cot)

    return work, dist, quad_m, cot

def calc_energy(m, l, I, theta, dtheta):
    g = 9.81 # [m/s^2]    
    
    # Define the energy in the system at any given time
    # Assume CoM is at l/2
    U_g = .5 * m * g * l * np.cos(theta)
    K_t = .125 * m * (l**2) * dtheta**2
    K_w = .5 * I * dtheta**2
    E = U_g + K_t + K_w

    return E

def script_calcs(quad_m, leg_m, l, I, theta, timelist):
    
    # Define quadruped parameters
    leg_params = {'l': [l, l], 'I': [I, I], 'm': [leg_m, leg_m]}
    body_params = {'l': l, 'I': I, 'm': quad_m, 'origin': [0, 0], 'orientation': 0}
    quadruped = Quadruped(leg_params, body_params)

    # Create constant velocity trajector
    res = len(theta)
    unit_traj = [[],[]]
    unit_traj[0] = np.linspace(0, 0, res).tolist()
    unit_traj[1] = theta.tolist()

    gait_data = [unit_traj, unit_traj, unit_traj, unit_traj]

    quad_gait = QuadrupedData(quadruped, timelist, gait_data)

    W = quad_gait.calc_work_done()
    d = quad_gait.calc_distance()
    m = quad_gait.quadruped.m
    cot = quad_gait.cost_of_transport()

    # print('\n Script')
    # print('W: ', W)
    # print('d: ', d)
    # print('m: ', m)
    # print('cot: ', cot)
    return W, d, m, cot


if __name__ == "__main__":
    main()