import numpy as np
import matplotlib.pyplot as plt
from matplotlib import gridspec
import csv

from energy_model.quadruped_energy import Quadruped
from energy_model.quadruped_energy import QuadrupedData
from energy_model.kinematics_data.data_post_process import VideoDataProcess
from energy_model.freq_analysis import *

class ModelGaitPlots:
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

    def leg1_data_approx(self):
        # Leg 1
        plt.figure()
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t1, label='θ_1', color='b')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].t1, label='θ_1 Fourier Approximation', linestyle='--', color='b')
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t2, label='θ_2', color='r')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].t2, label='θ_2 Fourier Approximation', linestyle='--', color='r')
        plt.title('Leg 1')
        plt.legend()
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')

    def leg2_data_approx(self):
        # Leg 2
        plt.figure()
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t1, label='θ_1', color='g')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[1].t1, label='θ_1 Fourier Approximation', linestyle='--', color='g')
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t2, label='θ_2', color='y')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[1].t2, label='θ_2 Fourier Approximation', linestyle='--', color='y')
        plt.title('Leg 2')
        plt.legend()
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')       

    def all_angles_approx(self): 
        # All angles plot
        plt.figure()
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t1, label='leg1 θ_1', color='b')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].t1, label='leg1 θ_1 Fourier Approximation', linestyle='--', color='b')
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t2, label='leg1 θ_2', color='r')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].t2, label='leg1 θ_2 Fourier Approximation', linestyle='--', color='r')
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t1, label='leg2 θ_1', color='g')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[1].t1, label='leg2 θ_1 Fourier Approximation', linestyle='--', color='g')
        plt.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t2, label='leg2 θ_2', color='y')
        plt.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[1].t2, label='leg2 θ_2 Fourier Approximation', linestyle='--', color='y')
        plt.title('All Angles')
        plt.legend()
        plt.xlabel('time (s)')
        plt.ylabel('angle (rad)')

    def leg1_t1_t2_phase_protrait(self):
        # Phase plot
        plt.figure()
        plt.plot(self.quadruped_traj.leg_list[0].t1, self.quadruped_traj.leg_list[0].t2, label='Video Data')
        plt.plot(self.quadruped_traj_fourier.leg_list[0].t1, self.quadruped_traj_fourier.leg_list[0].t2, label='Fourier Approximation', linestyle='--')
        plt.title('Leg 1 Phase Plot')
        plt.legend()
        plt.xlabel('θ_1 (rad))')
        plt.ylabel('θ_2 (rad)')

    def leg2_t1_t2_phase_portrait(self):
        # Phase plot
        plt.figure()
        plt.plot(self.quadruped_traj.leg_list[1].t1, self.quadruped_traj.leg_list[1].t2, label='Video Data')
        plt.plot(self.quadruped_traj_fourier.leg_list[1].t1, self.quadruped_traj_fourier.leg_list[1].t2, label='Fourier Approximation', linestyle='--')
        plt.title('Leg 2 Phase Plot')
        plt.legend()
        plt.xlabel('θ_1 (rad))')
        plt.ylabel('θ_2 (rad)')

    def leg1_t1_dt1(self):
        # Create the figure
        plt.figure()

        # Create the first axis for θ_1
        ax1 = plt.gca()  # Get the current axes
        ax1.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].t1, label='θ_1', color='tab:blue')
        ax1.set_xlabel('Time (s)')  # Label for the x-axis
        ax1.set_ylabel('θ_1', color='tab:blue')  # Label for the first y-axis
        ax1.tick_params(axis='y', labelcolor='tab:blue')  # Color the ticks

        # Create the second axis for dθ_1 (using twinx)
        ax2 = ax1.twinx()  # Create a second y-axis
        ax2.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].dt1, label='dθ_1', color='tab:red')
        ax2.set_ylabel('dθ_1', color='tab:red')  # Label for the second y-axis
        ax2.tick_params(axis='y', labelcolor='tab:red')  # Color the ticks

        # Add the title and legend
        plt.title('θ_1 and dθ_1 vs Time')
        ax1.legend(loc='upper left')
        ax2.legend(loc='upper right')

    def leg1_pfoot_vs_time(self):
        pB_list = []
        for i, _ in enumerate(self.quadruped_traj_fourier.leg_list[0].t1):
            self.quadruped.leg_list[0].t1 = self.quadruped_traj_fourier.leg_list[0].t1[i]
            self.quadruped.leg_list[0].t2 = self.quadruped_traj_fourier.leg_list[0].t2[i]
            self.quadruped.leg_list[0].dt1 = self.quadruped_traj_fourier.leg_list[0].dt1[i]
            self.quadruped.leg_list[0].dt2 = self.quadruped_traj_fourier.leg_list[0].dt2[i]
            pB_list.append(self.quadruped.leg_list[0].get_pB()[0])
        
        plt.figure()
        plt.plot(self.quadruped_traj_fourier.timelist, pB_list, label='P_foot')
        plt.title('Leg 1 Foot Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Foot Position (horizontal direction)')

    def leg1_vfoot_vs_time(self):
        vB_list = []
        for i, _ in enumerate(self.quadruped_traj_fourier.leg_list[0].t1):
            self.quadruped.leg_list[0].t1 = self.quadruped_traj_fourier.leg_list[0].t1[i]
            self.quadruped.leg_list[0].t2 = self.quadruped_traj_fourier.leg_list[0].t2[i]
            self.quadruped.leg_list[0].dt1 = self.quadruped_traj_fourier.leg_list[0].dt1[i]
            self.quadruped.leg_list[0].dt2 = self.quadruped_traj_fourier.leg_list[0].dt2[i]
            vB_list.append(self.quadruped.leg_list[0].get_vB()[0])
        
        plt.figure()
        plt.plot(self.quadruped_traj_fourier.timelist, vB_list, label='V_foot')
        plt.title('Leg 1 Foot Velocity')
        plt.xlabel('Time (s)')
        plt.ylabel('Foot Velocity (horizontal direction)')

    def all_energy_plot(self):
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

        # Create a figure
        fig = plt.figure()

        # Create a GridSpec with 3 rows and 2 columns
        gs = gridspec.GridSpec(3, 2, figure=fig)

        # First subplot: spans the top two grid sections (i.e., top-left 2 blocks)
        ax1 = fig.add_subplot(gs[0, :])  # This spans the first row, all columns
        ax1.plot(self.quadruped_traj_fourier.timelist, quad_energy)
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

        # Adjust layout for better spacing
        plt.tight_layout()

        # Show the plot
        plt.show()

    def power_spectrum_comparison(self):
        data_dict = {
            'front' : {
                'thigh':{},
                'calf': {}
            },
            'rear' : {
                'thigh':{},
                'calf': {}
            }
        }
        
        # Front Thigh
        positive_freqs, positive_power_spectrum, dominant_frequencies, dominant_powers = get_power_spectrum_plot_data(to_timeseries(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t1))
        data_dict['front']['thigh']['positive_freqs'] = positive_freqs
        data_dict['front']['thigh']['positive_power_spectrum'] = positive_power_spectrum
        data_dict['front']['thigh']['dominant_frequencies'] = dominant_frequencies
        data_dict['front']['thigh']['dominant_powers'] = dominant_powers

        positive_freqs, positive_power_spectrum, dominant_frequencies, dominant_powers = get_power_spectrum_plot_data(to_timeseries(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t2))
        data_dict['front']['calf']['positive_freqs'] = positive_freqs
        data_dict['front']['calf']['positive_power_spectrum'] = positive_power_spectrum
        data_dict['front']['calf']['dominant_frequencies'] = dominant_frequencies
        data_dict['front']['calf']['dominant_powers'] = dominant_powers

        positive_freqs, positive_power_spectrum, dominant_frequencies, dominant_powers = get_power_spectrum_plot_data(to_timeseries(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t1))
        data_dict['rear']['thigh']['positive_freqs'] = positive_freqs
        data_dict['rear']['thigh']['positive_power_spectrum'] = positive_power_spectrum
        data_dict['rear']['thigh']['dominant_frequencies'] = dominant_frequencies
        data_dict['rear']['thigh']['dominant_powers'] = dominant_powers

        positive_freqs, positive_power_spectrum, dominant_frequencies, dominant_powers = get_power_spectrum_plot_data(to_timeseries(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t2))
        data_dict['rear']['calf']['positive_freqs'] = positive_freqs
        data_dict['rear']['calf']['positive_power_spectrum'] = positive_power_spectrum
        data_dict['rear']['calf']['dominant_frequencies'] = dominant_frequencies
        data_dict['rear']['calf']['dominant_powers'] = dominant_powers


        # Thigh Comparison
        plt.figure()
        plt.plot(data_dict['front']['thigh']['positive_freqs'], data_dict['front']['thigh']['positive_power_spectrum'], label="Front Thigh")
        plt.scatter(data_dict['front']['thigh']['dominant_frequencies'], data_dict['front']['thigh']['dominant_powers'], color='red')

        plt.plot(data_dict['rear']['thigh']['positive_freqs'], data_dict['rear']['thigh']['positive_power_spectrum'], label="Rear Thigh")
        plt.scatter(data_dict['rear']['thigh']['dominant_frequencies'], data_dict['rear']['thigh']['dominant_powers'], color='red')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Power')
        plt.legend()
        # plt.title('Power Spectrum Density')

        # Calf Comparison
        plt.figure()
        plt.plot(data_dict['front']['calf']['positive_freqs'], data_dict['front']['calf']['positive_power_spectrum'], label="Front calf")
        plt.scatter(data_dict['front']['calf']['dominant_frequencies'], data_dict['front']['calf']['dominant_powers'], color='red')
        
        plt.plot(data_dict['rear']['calf']['positive_freqs'], data_dict['rear']['calf']['positive_power_spectrum'], label="Rear calf")
        plt.scatter(data_dict['rear']['calf']['dominant_frequencies'], data_dict['rear']['calf']['dominant_powers'], color='red')
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Power')
        plt.legend()
        # plt.title('Power Spectrum Density')

    def thigh_calf_data_comparison(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        # ax.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t1, label='Front Thigh Raw', color='b')
        ax.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].t1, label='Front Thigh Position', lw=3)#, linestyle='--', color='b')
        # ax.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t1, label='Rear Thigh Raw', color='g')
        ax.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[1].t1, label='Rear Thigh Position', lw=3)#, linestyle='--', color='g')
        # ax.title('Thigh Angle')
        ax.set_xlabel('Time (s)', fontsize=14)
        ax.set_ylabel('Angle (rad)', fontsize=14)
        ax.tick_params(axis='both', which='major', width=2.5)
        ax.legend(loc=4, fontsize=14)



        fig = plt.figure()
        ax = fig.add_subplot(111)
        # ax.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[0].t2, label='Front Claf Raw', color='r')
        ax.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[0].t2, label='Front Calf Position', lw=3)#, linestyle='--', color='r')
        # ax.plot(self.quadruped_traj.timelist, self.quadruped_traj.leg_list[1].t2, label='Rear Raw', color='y')
        ax.plot(self.quadruped_traj_fourier.timelist, self.quadruped_traj_fourier.leg_list[1].t2, label='Rear Calf Position', lw=3)#, linestyle='--', color='y')
        # ax.title('Calf Angle')
        ax.set_xlabel('Time (s)', fontsize=14)
        ax.set_ylabel('Angle (rad)', fontsize=14)
        ax.tick_params(axis='both', which='major', width=2.5)
        ax.legend(loc=4, fontsize=14)

    def data_to_cv(self):
        # Create a list of csv headers
        raw_headers = [
            "Raw Timelist",
            "Leg1_t1",
            "Leg1_t2",
            "Leg2_t1", 
            "Leg2_t2",
        ]

        approx_headers = [
            "Approximation Timelist",
            "Leg1_t1_approximation",
            "Leg1_t2_approximation",
            "Leg2_t1_approximation", 
            "Leg2_t2_approximation",
        ]

        # Prepare the data
        raw_data = []
        for i, time in enumerate(self.quadruped_traj.timelist):
            row = [
                time,
                self.quadruped_traj.leg_list[0].t1[i],
                self.quadruped_traj.leg_list[0].t2[i],
                self.quadruped_traj.leg_list[1].t1[i],
                self.quadruped_traj.leg_list[1].t2[i]
            ]
            raw_data.append(row)

        approx_data = []
        for i, time in enumerate(self.quadruped_traj_fourier.timelist):
            row = [
                time,
                self.quadruped_traj_fourier.leg_list[0].t1[i],
                self.quadruped_traj_fourier.leg_list[0].t2[i],
                self.quadruped_traj_fourier.leg_list[1].t1[i],
                self.quadruped_traj_fourier.leg_list[1].t2[i]
            ]
            approx_data.append(row)

        # Write data to CSV file
        with open('/workspace/src/energy_model/kinematics_data/unitree_a1/raw_leg_traj_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(raw_headers)
            writer.writerows(raw_data)

        with open('/workspace/src/energy_model/kinematics_data/unitree_a1/approx_leg_traj_data.csv', mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(approx_headers)
            writer.writerows(approx_data)
            
            

        

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
    
    plots = ModelGaitPlots(leg_params=unitree_a1_leg_params, 
                               body_params=unitree_a1_body_params,
                               gait_data=gait_data,
                               timelist=timelist)
    
    # Generate plots
    # plots.leg1_data_approx()
    # plots.leg2_data_approx()
    # plots.all_angles_approx()
    # plots.leg1_t1_t2_phase_protrait()
    # plots.leg2_t1_t2_phase_portrait()
    # plots.leg1_t1_dt1()
    # plots.leg1_pfoot_vs_time()
    # plots.leg1_vfoot_vs_time()
    # plots.all_energy_plot()
    # plots.power_spectrum_comparison()
    plots.thigh_calf_data_comparison()

    plt.show()

    # plots.data_to_cv()
    

if __name__ == "__main__":
    main()