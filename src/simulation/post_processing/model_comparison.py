import matplotlib.pyplot as plt
import numpy as np

import simulation.parameters as sim_params
from simulation.post_processing.synthetic_energy import SyntheticEnergy
from simulation.post_processing.gazebo_energy import GazeboEnergy
from simulation.post_processing.python_energy import PythonEnergy
from simulation.post_processing.low_pass_filter import Filter
from simulation.post_processing.freq_analysis import plot_power_spectrum

class ModelComparison:
    def __init__(self, raw_data_log, quad_param_file):
        self.tsi = 400
        self.left_stance_time = .28
        self.left_swing_time = .55
        self.right_stance_time = 0.530
        self.right_swing_time = 0.77

        self.left_stance_time2 = self.left_stance_time + .5
        self.right_stance_time2 = self.right_stance_time + .5
        
        # Desired Data Analysis
        # desired_data_log = sim_params.get_latest_log(sim_params.desired_logging_directory)
        
        # Filtering of position, velocity, and torque
        # Generate filtered CSV log
        # Filter.generate_filtered_csv(raw_data_log)
        # filtered_data_log = sim_params.get_latest_log(sim_params.postprocess_filtered_logging_directory)
        timelist = GazeboEnergy(raw_data_log, quad_param_file).data_dict['time']

        self.model_dict = {
            'synth': {
                'gz': None,
                'py': SyntheticEnergy(quad_param_file, sim_params.analytical_gait_params_file, timelist)
            },
            'raw': {
                'gz': GazeboEnergy(raw_data_log, quad_param_file),
                'py': PythonEnergy(quad_param_file, raw_data_log, override_velocity=True)
            },
            'filt': {
                # 'gz': GazeboEnergy(filtered_data_log),
                # 'py': PythonEnergy(quad_param_file, filtered_data_log, override_velocity=True)
            },
        }
    
    def position(self):
        def generate_subplot(self, fig, index, leg_joint):
            ax = fig.add_subplot(index)
            
            match leg_joint:
                case "FL Thigh":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[0].t1[:self.tsi], 
                        label=r'$\theta_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[0].t1[:self.tsi], 
                        label=r'$\theta$', lw=2)
                    stance_time = self.left_stance_time
                    swing_time = self.left_swing_time
                    stance_time2 = self.left_stance_time2
                case "FR Thigh":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[1].t1[:self.tsi], 
                        label=r'$\theta_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[1].t1[:self.tsi], 
                        label=r'$\theta$', lw=2)
                    stance_time = self.right_stance_time
                    swing_time = self.right_swing_time
                    stance_time2 = self.right_stance_time2
                case "FL Calf":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[0].t2[:self.tsi], 
                        label=r'$\theta_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[0].t2[:self.tsi], 
                        label=r'$\theta$', lw=2)
                    stance_time = self.left_stance_time
                    swing_time = self.left_swing_time
                    stance_time2 = self.left_stance_time2
                case "FR Calf":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[1].t2[:self.tsi], 
                        label=r'$\theta_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[1].t2[:self.tsi], 
                        label=r'$\theta$', lw=2)
                    stance_time = self.right_stance_time
                    swing_time = self.right_swing_time
                    stance_time2 = self.right_stance_time2

            ax.axvline(x=stance_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=swing_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=stance_time2, color='red', linestyle='--', linewidth=1)

            ax.text(stance_time+.05, .85, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())

            ax.text(swing_time+.05, .85, 'Swing Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            
            ax.text(stance_time2+.05, .85, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            
            ax.tick_params(axis='both', which='major', width=2.5)
            ax.set_xlabel('Time (s)', fontsize=14)
            ax.set_ylabel(leg_joint + ' Position (rad)', fontsize=14)
            ax.legend(loc=4, fontsize=10)

        # fig = plt.figure()
        # generate_subplot(self, fig, 221, "FL Thigh")
        # generate_subplot(self, fig, 222, "FR Thigh")
        # generate_subplot(self, fig, 223, "FL Calf")
        # generate_subplot(self, fig, 224, "FR Calf")
    
        fig = plt.figure()
        generate_subplot(self, fig, 111, "FL Thigh")
        fig = plt.figure()
        generate_subplot(self, fig, 111, "FL Calf")



    def test_position(self):
        fig = plt.figure()
        ax = fig.add_subplot(211)
        ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
            self.model_dict['synth']['py'].quadruped_data.leg_list[0].t1[:self.tsi], 
            lw=2, label='FL Calf')
        
        ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
            self.model_dict['synth']['py'].quadruped_data.leg_list[1].t1[:self.tsi], 
            lw=2, label='FR Thigh')
        
        ax.tick_params(axis='both', which='major', width=2.5)
        ax.set_xlabel('Time (s)', fontsize=14)
        ax.set_ylabel('Thigh Position (rad)', fontsize=14)
        ax.legend(loc=4, fontsize=10)
        
        ax = fig.add_subplot(212)
        ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
            self.model_dict['synth']['py'].quadruped_data.leg_list[0].t2[:self.tsi], 
            lw=2, label='FL Calf')
        ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
            self.model_dict['synth']['py'].quadruped_data.leg_list[1].t2[:self.tsi], 
            lw=2, label='FR Calf')

        ax.tick_params(axis='both', which='major', width=2.5)
        ax.set_xlabel('Time (s)', fontsize=14)
        ax.set_ylabel('Calf Position (rad)', fontsize=14)
        ax.legend(loc=4, fontsize=10)

    def velocity(self):
        def generate_subplot(self, fig, index, leg_joint):
            ax = fig.add_subplot(index)
            
            match leg_joint:
                case "FL Thigh":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[1].dt1[:self.tsi], 
                        label=r'$\dot{\theta}_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[1].dt1[:self.tsi], 
                        label=r'$\dot{\theta}$', lw=2)
                    stance_time = self.left_stance_time
                    swing_time = self.left_swing_time
                    stance_time2 = self.left_stance_time2
                case "FR Thigh":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt1[:self.tsi], 
                        label=r'$\dot{\theta}_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[0].dt1[:self.tsi], 
                        label=r'$\dot{\theta}$', lw=2)
                    stance_time = self.right_stance_time
                    swing_time = self.right_swing_time
                    stance_time2 = self.right_stance_time2
                case "FL Calf":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[1].dt2[:self.tsi], 
                        label=r'$\dot{\theta}_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[1].dt2[:self.tsi], 
                        label=r'$\dot{\theta}$', lw=2)
                    stance_time = self.left_stance_time
                    swing_time = self.left_swing_time
                    stance_time2 = self.left_stance_time2
                case "FR Calf":
                    ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt2[:self.tsi], 
                        label=r'$\dot{\theta}_r$', lw=2)
                    ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                        self.model_dict['raw']['py'].quadruped_data.leg_list[0].dt2[:self.tsi], 
                        label=r'$\dot{\theta}$', lw=2)
                    stance_time = self.right_stance_time
                    swing_time = self.right_swing_time
                    stance_time2 = self.right_stance_time2
            ax.axvline(x=stance_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=swing_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=stance_time2, color='red', linestyle='--', linewidth=1)
            ax.text(stance_time+.025, .05, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            ax.text(swing_time+.025, .75, 'Swing Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            ax.text(stance_time2+.025, .05, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            ax.tick_params(axis='both', which='major', width=2.5)
            ax.set_xlabel('Time (s)', fontsize=14)
            ax.set_ylabel(leg_joint + ' Velocity (rad/s)', fontsize=14)
            ax.legend(loc=4, fontsize=10)
        
        # fig = plt.figure()
        # generate_subplot(self, fig, 221, "FL Thigh")
        # generate_subplot(self, fig, 222, "FR Thigh")
        # generate_subplot(self, fig, 223, "FL Calf")
        # generate_subplot(self, fig, 224, "FR Calf")
        
        fig = plt.figure()
        generate_subplot(self, fig, 111, "FL Thigh")
        fig = plt.figure()
        generate_subplot(self, fig, 111, "FL Calf")

    def power(self):
        def thigh(self):
            fig = plt.figure()
            ax = fig.add_subplot(111)
            
            # Synthetic
            thigh_power_traj, calf_power_traj = self.model_dict['synth']['py'].quadruped_data.leg_list[0].power_trajectory()
            power_traj = np.abs(thigh_power_traj)
            ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Synthetic', lw=2)
            
            # Kinematic
            power_traj, _ = self.model_dict['raw']['py'].quadruped_data.leg_list[0].power_trajectory()
            power_traj = np.abs(power_traj)
            ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Kinematic', lw=2)

            # Dynamic
            power_traj = self.model_dict['raw']['gz'].data_dict['FR_thigh_pow']
            power_traj = np.abs(power_traj)
            ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Dynamic', lw=2)

            
            ax.tick_params(axis='both', which='major', width=2.5)
            ax.set_xlabel('Time (s)', fontsize=14)
            ax.set_ylabel('Power Consumption (W)', fontsize=14)
            ax.legend(fontsize=10)
            ax.set_title('Thigh Power')

            ax.axvline(x=self.right_stance_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=self.right_swing_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=self.right_stance_time2, color='red', linestyle='--', linewidth=1)

            ax.text(self.right_stance_time+.05, .75, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())

            ax.text(self.right_swing_time+.05, .75, 'Swing Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            
            ax.text(self.right_stance_time2+.05, .75, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())

        def calf(self):
            fig = plt.figure()
            ax = fig.add_subplot(111)

            # Synthetic
            thigh_power_traj, calf_power_traj = self.model_dict['synth']['py'].quadruped_data.leg_list[0].power_trajectory()
            power_traj = np.abs(calf_power_traj)
            ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Synthetic', lw=2)
            
            # Kinematic
            thigh_power_traj, calf_power_traj = self.model_dict['raw']['py'].quadruped_data.leg_list[0].power_trajectory()
            power_traj = np.abs(calf_power_traj)
            ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Kinematic', lw=2)

            # Dynamic
            power_traj = self.model_dict['raw']['gz'].data_dict['FR_calf_pow']
            power_traj = np.abs(power_traj)
            ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Dynamic', lw=2)
            
            ax.tick_params(axis='both', which='major', width=2.5)
            ax.set_xlabel('Time (s)', fontsize=14)
            ax.set_ylabel('Power Consumption (W)', fontsize=14)
            ax.legend(fontsize=10)
            ax.set_title('Calf Power')

            ax.axvline(x=self.right_stance_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=self.right_swing_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=self.right_stance_time2, color='red', linestyle='--', linewidth=1)

            ax.text(self.right_stance_time+.05, .75, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())

            ax.text(self.right_swing_time+.05, .75, 'Swing Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            
            ax.text(self.right_stance_time2+.05, .75, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())

        def leg(self):
            fig = plt.figure()
            ax = fig.add_subplot(111)

            # Sizing
            thigh_power_traj = self.model_dict['raw']['gz'].data_dict['FR_thigh_pow']
            calf_power_traj = self.model_dict['raw']['gz'].data_dict['FR_calf_pow']
            power_traj = np.abs(thigh_power_traj) + np.abs(calf_power_traj)
            ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], 
                    power_traj[:self.tsi], 
                    label='', color='w', lw=2)
            
            # Synthetic
            thigh_power_traj, calf_power_traj = self.model_dict['synth']['py'].quadruped_data.leg_list[0].power_trajectory()
            power_traj = np.abs(thigh_power_traj) + np.abs(calf_power_traj)
            ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Synthetic', lw=2)
            
            # Kinematic
            thigh_power_traj, calf_power_traj = self.model_dict['raw']['py'].quadruped_data.leg_list[0].power_trajectory()
            power_traj = np.abs(thigh_power_traj) + np.abs(calf_power_traj)
            ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Kinematic', lw=2)

            # Dynamic
            thigh_power_traj = self.model_dict['raw']['gz'].data_dict['FR_thigh_pow']
            calf_power_traj = self.model_dict['raw']['gz'].data_dict['FR_calf_pow']
            power_traj = np.abs(thigh_power_traj) + np.abs(calf_power_traj)
            ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Dynamic', lw=2)
            
            
            ax.tick_params(axis='both', which='major', width=2.5)
            ax.set_xlabel('Time (s)', fontsize=14)
            ax.set_ylabel('Power Consumption (W)', fontsize=14)
            ax.legend(fontsize=10)
            ax.set_title('Leg Power')

            ax.axvline(x=self.right_stance_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=self.right_swing_time, color='red', linestyle='--', linewidth=1)
            ax.axvline(x=self.right_stance_time2, color='red', linestyle='--', linewidth=1)

            ax.text(self.right_stance_time+.05, .75, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())

            ax.text(self.right_swing_time+.05, .75, 'Swing Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())
            
            ax.text(self.right_stance_time2+.05, .75, 'Stance Phase', color='red', rotation=90,
                    va='bottom', ha='left', transform=ax.get_xaxis_transform())

        def quadruped(self):
            fig = plt.figure()
            ax = fig.add_subplot(111)

            # Synthetic
            power_traj = self.model_dict['synth']['py'].quadruped_data.power_trajectory()
            ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Synthetic', lw=2)
            
            # Kinematic
            power_traj = self.model_dict['raw']['py'].quadruped_data.power_trajectory()
            ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist[:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Kinematic', lw=2)

            # Dynamic
            power_traj = self.model_dict['raw']['gz'].data_dict['power']
            ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], 
                    power_traj[:self.tsi], 
                    label='Dynamic', lw=2)
            
            ax.tick_params(axis='both', which='major', width=2.5)
            ax.set_xlabel('Time (s)', fontsize=14)
            ax.set_ylabel('Power Consumption (W)', fontsize=14)
            ax.legend(fontsize=10)
            ax.set_title('Quadruped Power')
    
        thigh(self)
        calf(self)
        leg(self)
        quadruped(self)

    def distance(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)

        # Synthetic
        ax.plot(self.model_dict['synth']['py'].quadruped_data.timelist, 
                self.model_dict['synth']['py'].dist_list, 
                label='Synthetic', lw=2)
        
        # # Kinematic
        ax.plot(self.model_dict['raw']['py'].quadruped_data.timelist, 
                self.model_dict['raw']['py'].dist_list, 
                label='Kinematic', lw=2)

        # Dynamic
        ax.plot(self.model_dict['raw']['gz'].data_dict['time'], 
                self.model_dict['raw']['gz'].data_dict['distance'], 
                label='Dynamic', lw=2)
        
        ax.tick_params(axis='both', which='major', width=2.5)
        ax.set_xlabel('Time (s)', fontsize=14)
        ax.set_ylabel('Distance (m)', fontsize=14)
        ax.legend(fontsize=10)

        print('Synthetic: ', self.model_dict['synth']['py'].dist_list[-1])
        print('Kinematic: ', self.model_dict['raw']['py'].dist_list[-1])
        print('Dynamic:   ', self.model_dict['raw']['gz'].data_dict['distance'][-1])

    def cost_of_transport(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        synthetic_cot = self.model_dict['synth']['py'].quadruped_data.cost_of_transport()
        kinematic_cot = self.model_dict['raw']['py'].quadruped_data.cost_of_transport()
        dynamic_cot = self.model_dict['raw']['gz'].cost_of_transport()

        ax.bar(['Synthetic','Kinematic','Dynamic'], [synthetic_cot, kinematic_cot, dynamic_cot])
        
        ax.tick_params(axis='both', which='major', width=2.5)
        ax.set_ylabel('Cost of Transport', fontsize=14)

    def torque_velocity_power(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        
        ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], self.model_dict['raw']['gz'].data_dict['FR_calf_pos'][:self.tsi], 
                label='FR Calf Velocity', lw=2)
        ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], self.model_dict['raw']['gz'].data_dict['FR_calf_tor'][:self.tsi], 
                label='FR Calf Torque', lw=2)
        ax.plot(self.model_dict['raw']['gz'].data_dict['time'][:self.tsi], np.abs(self.model_dict['raw']['gz'].data_dict['FR_calf_pow'][:self.tsi]), 
                label='FR Calf Power', lw=2)
        
        ax.tick_params(axis='both', which='major', width=2.5)
        ax.set_xlabel('Time (s)', fontsize=14)
        ax.set_ylabel(r'Angular Velocity (rad/s) / Torque (N$\cdot$m) / Power (W)', fontsize=14)
        ax.legend(fontsize=10)

        ax.axvline(x=self.right_stance_time, color='red', linestyle='--', linewidth=1)
        ax.axvline(x=self.right_swing_time, color='red', linestyle='--', linewidth=1)
        ax.axvline(x=self.right_stance_time2, color='red', linestyle='--', linewidth=1)

        ax.text(self.right_stance_time+.05, .75, 'Stance Phase', color='red', rotation=90,
                va='bottom', ha='left', transform=ax.get_xaxis_transform())

        ax.text(self.right_swing_time+.05, .75, 'Swing Phase', color='red', rotation=90,
                va='bottom', ha='left', transform=ax.get_xaxis_transform())
        
        ax.text(self.right_stance_time2+.05, .75, 'Stance Phase', color='red', rotation=90,
                va='bottom', ha='left', transform=ax.get_xaxis_transform())
        
    
    def power_spectrum_comparison(self):
        def position_spectra(self):
            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_pos'], title="Raw Gazebo FR_thigh position")
        
        def velocity_spectra(self):
            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_vel'], title="Raw Gazebo FR_thigh Velocity")

        def torque_spectra(self):
            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_tor'], title="Raw FR_thigh Torque")

        def power_spectra(self):

            # Kinematic
            thigh_power_traj, calf_power_traj = self.model_dict['raw']['py'].quadruped_data.leg_list[0].power_trajectory()
            kinematic_power_traj = np.abs(thigh_power_traj) + np.abs(calf_power_traj)

            plot_power_spectrum(self.model_dict['raw']['py'].quadruped_data.timelist, thigh_power_traj, title="Raw Python Power")

            # Dynamic
            thigh_power_traj = self.model_dict['raw']['gz'].data_dict['FR_thigh_pow']
            calf_power_traj = self.model_dict['raw']['gz'].data_dict['FR_calf_pow']
            dynamic_power_traj = np.abs(thigh_power_traj) + np.abs(calf_power_traj)

            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], thigh_power_traj, title="Raw Gazebo Power")
        
        # position_spectra(self)
        # velocity_spectra(self)
        # torque_spectra(self)
        power_spectra(self)

def main():
    raw_data_log = sim_params.get_latest_log(sim_params.raw_logging_directory)
    model_comparison = ModelComparison(raw_data_log, sim_params.quadruped_params_file)
    # model_comparison.position()
    # model_comparison.velocity()
    # model_comparison.power()
    # model_comparison.distance()
    model_comparison.cost_of_transport()
    # model_comparison.torque_velocity_power()
    # model_comparison.power_spectrum_comparison()
    # model_comparison.test_position()
    plt.show()

if __name__ == "__main__":
    main()