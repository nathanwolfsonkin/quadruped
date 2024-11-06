import numpy as np
import matplotlib.pyplot as plt

from energy_model.quadruped_energy import Quadruped
from energy_model.kinematics_data.data_post_process import DataPostProcess
from energy_model.freq_analysis import *


class KinematicsPlots:
    def __init__(self, model: str):

        leg_params = {'l': [.2, .2], 'I': [0.0055, 0.003], 'm': [1.013, 0.166]}
        body_params = {'l': 2*0.1805, 'I': 0.01, 'm': 5.660, 'origin': [0, 0], 'orientation': 0}
        self.quadruped = Quadruped(leg_params=leg_params, body_params=body_params)
        dataset = DataPostProcess(model)
    
        # Get angle data
        leg1, leg2 = dataset.get_angle_lists()
        self.leg1_t1, self.leg1_t2 = leg1
        self.leg2_t1, self.leg2_t2 = leg2

        # Get timing information
        frame_time, total_time, total_frames = dataset.get_frame_data()
        self.timelist = np.linspace(0, total_time, total_frames)

        # Get velocity data
        self.leg1_dt1 = np.gradient(self.leg1_t1, self.timelist)
        self.leg1_dt2 = np.gradient(self.leg1_t2, self.timelist)
        self.leg2_dt1 = np.gradient(self.leg2_t1, self.timelist)
        self.leg2_dt2 = np.gradient(self.leg2_t2, self.timelist)

        # Calculate fourier approximations
        self.fourier_leg1_t1 = fourier_approx(to_timeseries(self.timelist, self.leg1_t1))
        self.fourier_leg1_t2 = fourier_approx(to_timeseries(self.timelist, self.leg1_t2))
        self.fourier_leg2_t1 = fourier_approx(to_timeseries(self.timelist, self.leg2_t1))
        self.fourier_leg2_t2 = fourier_approx(to_timeseries(self.timelist, self.leg2_t2))

        self.fourier_leg1_dt1 = fourier_approx(to_timeseries(self.timelist, self.leg1_dt1))
        self.fourier_leg1_dt2 = fourier_approx(to_timeseries(self.timelist, self.leg1_dt2))
        self.fourier_leg2_dt1 = fourier_approx(to_timeseries(self.timelist, self.leg2_dt1))
        self.fourier_leg2_dt2 = fourier_approx(to_timeseries(self.timelist, self.leg2_dt2))
    
    def leg1_data_approx(self):
        # Leg 1
        plt.figure()
        plt.plot(self.timelist, self.leg1_t1, label='θ_1', color='b')
        plt.plot(self.timelist, self.fourier_leg1_t1, label='θ_1 Fourier Approximation', linestyle='--', color='b')
        plt.plot(self.timelist, self.leg1_t2, label='θ_2', color='r')
        plt.plot(self.timelist, self.fourier_leg1_t2, label='θ_2 Fourier Approximation', linestyle='--', color='r')
        plt.title('Leg 1')
        plt.legend()
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')

    def leg2_data_approx(self):
        # Leg 2
        plt.figure()
        plt.plot(self.timelist, self.leg2_t1, label='θ_1', color='g')
        plt.plot(self.timelist, self.fourier_leg2_t1, label='θ_1 Fourier Approximation', linestyle='--', color='g')
        plt.plot(self.timelist, self.leg2_t2, label='θ_2', color='y')
        plt.plot(self.timelist, self.fourier_leg2_t2, label='θ_2 Fourier Approximation', linestyle='--', color='y')
        plt.title('Leg 2')
        plt.legend()
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')       

    def all_angles_approx(self): 
        # All angles plot
        plt.figure()
        plt.plot(self.timelist, self.leg1_t1, label='leg1 θ_1', color='b')
        plt.plot(self.timelist, self.fourier_leg1_t1, label='leg1 θ_1 Fourier Approximation', linestyle='--', color='b')
        plt.plot(self.timelist, self.leg1_t2, label='leg1 θ_2', color='r')
        plt.plot(self.timelist, self.fourier_leg1_t2, label='leg1 θ_2 Fourier Approximation', linestyle='--', color='r')
        plt.plot(self.timelist, self.leg2_t1, label='leg2 θ_1', color='g')
        plt.plot(self.timelist, self.fourier_leg2_t1, label='leg2 θ_1 Fourier Approximation', linestyle='--', color='g')
        plt.plot(self.timelist, self.leg2_t2, label='leg2 θ_2', color='y')
        plt.plot(self.timelist, self.fourier_leg2_t2, label='leg2 θ_2 Fourier Approximation', linestyle='--', color='y')
        plt.title('All Angles')
        plt.legend()
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')

    def leg1_t1_t2_phase_protrait(self):
        # Phase plot
        plt.figure()
        plt.plot(self.leg1_t1, self.leg1_t2, label='θ_1')
        plt.plot(self.fourier_leg1_t1, self.fourier_leg1_t2, label='θ_2', linestyle='--')
        plt.title('Leg 1 Phase Plot')
        plt.legend()
        plt.xlabel('θ_1 (rad))')
        plt.ylabel('θ_2 (rad)')

    def leg2_t1_t2_phase_portrait(self):
        # Phase plot
        plt.figure()
        plt.plot(self.leg2_t1, self.leg2_t2, label='θ_1')
        plt.plot(self.fourier_leg2_t1, self.fourier_leg2_t2, label='θ_2', linestyle='--')
        plt.title('Leg 2 Phase Plot')
        plt.legend()
        plt.xlabel('θ_1 (rad))')
        plt.ylabel('θ_2 (rad)')

    def leg1_t1_dt1(self):
        # Create the figure
        plt.figure()

        # Create the first axis for θ_1
        ax1 = plt.gca()  # Get the current axes
        ax1.plot(self.timelist, self.fourier_leg1_t1, label='θ_1', color='tab:blue')
        ax1.set_xlabel('Time (s)')  # Label for the x-axis
        ax1.set_ylabel('θ_1', color='tab:blue')  # Label for the first y-axis
        ax1.tick_params(axis='y', labelcolor='tab:blue')  # Color the ticks

        # Create the second axis for dθ_1 (using twinx)
        ax2 = ax1.twinx()  # Create a second y-axis
        ax2.plot(self.timelist, self.fourier_leg1_dt1, label='dθ_1', color='tab:red')
        ax2.set_ylabel('dθ_1', color='tab:red')  # Label for the second y-axis
        ax2.tick_params(axis='y', labelcolor='tab:red')  # Color the ticks

        # Add the title and legend
        plt.title('θ_1 and dθ_1 vs Time')
        ax1.legend(loc='upper left')
        ax2.legend(loc='upper right')

    def leg1_vb_vs_time(self):
        for i, _ in enumerate(self.leg1_t1):
            self.quadruped.leg_list[0].t1 = self.leg1_t1[i]
            self.quadruped.leg_list[0].t2 = self.leg1_t2[i]
            self.quadruped.leg_list[0].dt1 = self.leg1_dt1[i]
            self.quadruped.leg_list[0].dt2 = self.leg1_dt2[i]
            self.quadruped.leg_list[0].get_vB()


def main():
    kin_plot = KinematicsPlots("unitree_a1")
    kin_plot.leg1_data_approx()
    kin_plot.leg2_data_approx()
    kin_plot.all_angles_approx()
    kin_plot.leg1_t1_t2_phase_protrait()
    kin_plot.leg2_t1_t2_phase_portrait()
    kin_plot.leg1_t1_dt1()
    plt.show()
    

if __name__ == "__main__":
    main()