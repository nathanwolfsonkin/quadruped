import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData
from energy_model.kinematics_data.data_post_process import DataPostProcess
from energy_model.freq_analysis import *

def main():
    # Body parameters gathered from the Unitree A1 official URDF
    leg_params = {'l': [.2, .2], 'I': [0.0055, 0.003], 'm': [1.013, 0.166]}
    body_params = {'l': 2*0.1805, 'I': 0.01, 'm': 5.660, 'origin': [0, 0], 'orientation': 0}
    quadruped = Quadruped(leg_params=leg_params, body_params=body_params)

    dataset = DataPostProcess("unitree_a1")
    
    # Get angle data
    leg_angle_list = dataset.get_angle_lists()
    leg_angle_list = leg_angle_list + leg_angle_list # Assume trot (leg1 = leg3; leg2 = leg4)

    # Get timing information
    frame_time, total_time, total_frames = dataset.get_frame_data()
    timelist = np.linspace(0, total_time, total_frames)

    quadruped_traj = QuadrupedData(timelist=timelist, data=leg_angle_list)

    leg1_dt1 = np.gradient(leg_angle_list[0][0], timelist)
    leg1_dt2 = np.gradient(leg_angle_list[0][1], timelist)
    leg2_dt1 = np.gradient(leg_angle_list[1][0], timelist)
    leg2_dt2 = np.gradient(leg_angle_list[1][1], timelist)
    leg_vel_list = [[leg1_dt1,leg1_dt2],[leg2_dt1,leg2_dt2]]
    leg_vel_list = leg_vel_list + leg_vel_list


    leg_energy = [[],[],[],[]]
    quad_energy = []

    # For each time instance
    for time_index, time in enumerate(timelist):
        # For each leg
        for leg_index, leg in enumerate(quadruped.leg_list):
            # Update current state
            leg.t1 = quadruped_traj.leg_list[leg_index].t1[time_index]
            leg.t2 = quadruped_traj.leg_list[leg_index].t2[time_index]
            leg.dt1 = quadruped_traj.leg_list[leg_index].dt1[time_index]
            leg.dt2 = quadruped_traj.leg_list[leg_index].dt2[time_index]

            leg_energy[leg_index].append(leg.total_energy())

        quad_energy.append(quadruped.total_energy())

    # Create a figure
    fig = plt.figure()

    # Create a GridSpec with 3 rows and 2 columns
    gs = gridspec.GridSpec(3, 2, figure=fig)

    # First subplot: spans the top two grid sections (i.e., top-left 2 blocks)
    ax1 = fig.add_subplot(gs[0, :])  # This spans the first row, all columns
    ax1.plot(timelist, quad_energy)
    ax1.set_title("Total Quadruped Energy")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Energy (j)")

    # Second subplot: bottom-left
    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(timelist, leg_energy[0])
    ax2.set_title("Front Leg 1 Energy")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Energy (j)")

    # Third subplot: bottom-right
    ax3 = fig.add_subplot(gs[1, 1])
    ax3.plot(timelist, leg_energy[1])
    ax3.set_title("Front Leg 2 Energy")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Energy (j)")

    # Fourth subplot: bottom-left (this could be a new plot or reused if needed)
    ax4 = fig.add_subplot(gs[2, 0])
    ax4.plot(timelist, leg_energy[2])
    ax4.set_title("Rear Leg 1 Energy")
    ax4.set_xlabel("Time (s)")
    ax4.set_ylabel("Energy (j)")

    # Fifth subplot: bottom-right
    ax5 = fig.add_subplot(gs[2, 1])
    ax5.plot(timelist, leg_energy[3])
    ax5.set_title("Rear Leg 2 Energy")
    ax5.set_xlabel("Time (s)")
    ax5.set_ylabel("Energy (j)")

    # Adjust layout for better spacing
    plt.tight_layout()

    # Show the plot
    plt.show()
    

if __name__ == "__main__":
    main()