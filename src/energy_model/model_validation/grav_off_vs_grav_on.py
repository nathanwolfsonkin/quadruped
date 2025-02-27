import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
import time

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData
from energy_model.kinematics_data.data_post_process import VideoDataProcess
from energy_model.freq_analysis import *

class ModelGaitPlots:
    def __init__(self, leg_params: dict, body_params: dict, gait_data: list, timelist: list, gravity: float):

        # Contains quadrupedal parameters and state
        self.quadruped = Quadruped(leg_params=leg_params, body_params=body_params, gravity=gravity)

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

    def all_energy_plot(self, fig=plt.figure(), new_fig = False, ax = []):
        leg_energy = [[],[],[],[]]
        quad_energy = []

        # For each time instance
        for time_index, time in enumerate(self.quadruped_traj_fourier.timelist):
            # For each leg
            for leg_index, leg in enumerate(self.quadruped.leg_list):
                # Update current state
                leg.t1 = self.quadruped_traj_fourier.leg_list[leg_index].t1[time_index]
                leg.t2 = self.quadruped_traj_fourier.leg_list[leg_index].t2[time_index]
                leg.dt1 = self.quadruped_traj_fourier.leg_list[leg_index].dt1[time_index]
                leg.dt2 = self.quadruped_traj_fourier.leg_list[leg_index].dt2[time_index]

                # Calculate current energy
                leg_energy[leg_index].append(leg.total_energy())

            quad_energy.append(self.quadruped.total_energy())

         # Create a GridSpec with 3 rows and 2 columns
        gs = gridspec.GridSpec(3, 2, figure=fig)

        # Create a figure
        if new_fig == True:
            # First subplot: spans the top two grid sections (i.e., top-left 2 blocks)
            ax1 = fig.add_subplot(gs[0, :])  # This spans the first row, all columns
            ax1.plot(self.quadruped_traj_fourier.timelist, quad_energy, label="gravity on")
            ax1.set_title("Total Quadruped Energy")
            ax1.set_xlabel("Time (s)")
            ax1.set_ylabel("Energy (j)")

            # Second subplot: bottom-left
            ax2 = fig.add_subplot(gs[1, 0])
            ax2.plot(self.quadruped_traj_fourier.timelist, leg_energy[0])
            ax2.set_title("Front Leg 1 Energy")
            ax2.set_xlabel("Time (s)")
            ax2.set_ylabel("Energy (j)")

            # Third subplot: bottom-right
            ax3 = fig.add_subplot(gs[1, 1])
            ax3.plot(self.quadruped_traj_fourier.timelist, leg_energy[1])
            ax3.set_title("Front Leg 2 Energy")
            ax3.set_xlabel("Time (s)")
            ax3.set_ylabel("Energy (j)")

            # Fourth subplot: bottom-left (this could be a new plot or reused if needed)
            ax4 = fig.add_subplot(gs[2, 0])
            ax4.plot(self.quadruped_traj_fourier.timelist, leg_energy[2])
            ax4.set_title("Rear Leg 1 Energy")
            ax4.set_xlabel("Time (s)")
            ax4.set_ylabel("Energy (j)")

            # Fifth subplot: bottom-right
            ax5 = fig.add_subplot(gs[2, 1])
            ax5.plot(self.quadruped_traj_fourier.timelist, leg_energy[3])
            ax5.set_title("Rear Leg 2 Energy")
            ax5.set_xlabel("Time (s)")
            ax5.set_ylabel("Energy (j)")
        else:
            ax1, ax2, ax3, ax4, ax5 = ax
            # First subplot: spans the top two grid sections (i.e., top-left 2 blocks)
            ax1.plot(self.quadruped_traj_fourier.timelist, quad_energy, label='gravity off')
            ax1.set_title("Total Quadruped Energy")
            ax1.set_xlabel("Time (s)")
            ax1.set_ylabel("Energy (j)")
            ax1.legend()

            # Second subplot: bottom-left
            ax2.plot(self.quadruped_traj_fourier.timelist, leg_energy[0])
            ax2.set_title("Front Leg 1 Energy")
            ax2.set_xlabel("Time (s)")
            ax2.set_ylabel("Energy (j)")

            # Third subplot: bottom-right
            ax3.plot(self.quadruped_traj_fourier.timelist, leg_energy[1])
            ax3.set_title("Front Leg 2 Energy")
            ax3.set_xlabel("Time (s)")
            ax3.set_ylabel("Energy (j)")

            # Fourth subplot: bottom-left (this could be a new plot or reused if needed)
            ax4.plot(self.quadruped_traj_fourier.timelist, leg_energy[2])
            ax4.set_title("Rear Leg 1 Energy")
            ax4.set_xlabel("Time (s)")
            ax4.set_ylabel("Energy (j)")

            # Fifth subplot: bottom-right
            ax5.plot(self.quadruped_traj_fourier.timelist, leg_energy[3])
            ax5.set_title("Rear Leg 2 Energy")
            ax5.set_xlabel("Time (s)")
            ax5.set_ylabel("Energy (j)")
        # Adjust layout for better spacing
        plt.tight_layout()
        
        return fig, [ax1, ax2, ax3, ax4, ax5]
            

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
    
    grav_on = ModelGaitPlots(leg_params=unitree_a1_leg_params, 
                               body_params=unitree_a1_body_params,
                               gait_data=gait_data,
                               timelist=timelist,
                               gravity=9.81)
    
    grav_off = ModelGaitPlots(leg_params=unitree_a1_leg_params, 
                            body_params=unitree_a1_body_params,
                            gait_data=gait_data,
                            timelist=timelist,
                            gravity=0.0)
    
    # Generate plots
    fig, ax = grav_on.all_energy_plot(new_fig=True)    
    grav_off.all_energy_plot(fig, new_fig=False, ax=ax)
    plt.show()
    

if __name__ == "__main__":
    main()