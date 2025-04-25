import matplotlib.pyplot as plt

import simulation.parameters as sim_params
from simulation.post_processing.synthetic_energy import SyntheticEnergy
from simulation.post_processing.gazebo_energy import GazeboEnergy
from simulation.post_processing.python_energy import PythonEnergy
from simulation.post_processing.low_pass_filter import Filter
from simulation.post_processing.freq_analysis import plot_power_spectrum

class PowerAnalysis:
    def __init__(self):
        
        # Desired Data Analysis
        # desired_data_log = sim_params.get_latest_log(sim_params.desired_logging_directory)
        
        # Raw Data Analysis
        raw_data_log = sim_params.get_latest_log(sim_params.raw_logging_directory)
        
        # Debug Specific Logs
        pre_derivative_change_log = "/workspace/src/simulation/data_logs/derivative_kick.csv"
        no_damping_no_saturation = "/workspace/src/simulation/data_logs/no_damping_no_saturation.csv"
        no_damping_yes_saturation = "/workspace/src/simulation/data_logs/no_damping_yes_saturation.csv"
        yes_damping_no_saturation = "/workspace/src/simulation/data_logs/yes_damping_no_saturation.csv"
        yes_damping_yes_saturation = "/workspace/src/simulation/data_logs/yes_damping_yes_saturation.csv"
        
        # Filtering of position, velocity, and torque
        # Generate filtered CSV log
        Filter.generate_filtered_csv(raw_data_log)
        filtered_data_log = sim_params.get_latest_log(sim_params.postprocess_filtered_logging_directory)
        
        timelist = GazeboEnergy(raw_data_log).data_dict['time']

        self.model_dict = {
            'synth': {
                'gz': None,
                'py': SyntheticEnergy(sim_params.quadruped_params_file, sim_params.analytical_gait_params_file, timelist)
            },
            'raw': {
                'gz': GazeboEnergy(raw_data_log),
                'py': PythonEnergy(sim_params.quadruped_params_file, raw_data_log, override_velocity=True)
            },
            'filt': {
                'gz': GazeboEnergy(filtered_data_log),
                'py': PythonEnergy(sim_params.quadruped_params_file, filtered_data_log, override_velocity=True)
            },
            'debug': {
                'pre_derivative_change': GazeboEnergy(pre_derivative_change_log),
                'no_damping_no_saturation': GazeboEnergy(no_damping_no_saturation),
                'no_damping_yes_saturation': GazeboEnergy(no_damping_yes_saturation),
                'yes_damping_no_saturation': GazeboEnergy(yes_damping_no_saturation),
                'yes_damping_yes_saturation': GazeboEnergy(yes_damping_yes_saturation),
                'py_computed_vel': PythonEnergy(sim_params.quadruped_params_file, raw_data_log, override_velocity=False)
            }
        }
    
    def power_comparison(self):
        def raw_power_comparison(self: PowerAnalysis):
            plt.figure()
            plt.title('Raw Data Computed Power')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.power_trajectory(), label='Synthetic Python Power')
            plt.plot(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.power_trajectory(), label='Raw Python Power')
            plt.plot(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['power'], label='Raw Gazebo Power')
            plt.legend()
            
        def filt_power_comparison(self: PowerAnalysis):
            plt.figure()
            plt.title('Filtered Data Computed Power')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.power_trajectory(), label='Synthetic Python Power')
            plt.plot(self.model_dict['filt']['py'].quadruped_data.timelist, self.model_dict['filt']['py'].quadruped_data.power_trajectory(), label='Filtered Python Power')
            plt.plot(self.model_dict['filt']['gz'].data_dict['time'], self.model_dict['filt']['gz'].data_dict['power'], label='Filtered Gazebo Power')
            plt.legend()
            
        raw_power_comparison(self)
        filt_power_comparison(self)
    
    def power_spectrum_comparison(self):
        def power_spectra(self):
            def synthetic_power_spectrum(self: PowerAnalysis):
                plot_power_spectrum(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.power_trajectory(), title="Synthetic Python Power")

            def raw_power_spectra(self: PowerAnalysis):
                plot_power_spectrum(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.power_trajectory(), title="Raw Python Power")
                plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['power'], title="Raw Gazebo Power")
            
            def filtered_power_spectra(self: PowerAnalysis):
                plot_power_spectrum(self.model_dict['filt']['py'].quadruped_data.timelist, self.model_dict['filt']['py'].quadruped_data.power_trajectory(), title="Filtered Python Power")
                plot_power_spectrum(self.model_dict['filt']['gz'].data_dict['time'], self.model_dict['filt']['gz'].data_dict['power'], title="Filtered Gazebo Power")
            
            synthetic_power_spectrum(self)
            raw_power_spectra(self)
            filtered_power_spectra(self)
        
        def velocity_spectra(self):
            def synthetic_velocity_spectrum(self: PowerAnalysis):
                plot_power_spectrum(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt1, title="Synthetic Python FR_thigh Velocity")

            def raw_velocity_spectra(self: PowerAnalysis):
                plot_power_spectrum(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.leg_list[0].dt1, title="Raw Python FR_thigh Velocity")
                plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_vel'], title="Raw Gazebo FR_thigh Velocity")
            
            def filtered_velocity_spectra(self: PowerAnalysis):
                plot_power_spectrum(self.model_dict['filt']['py'].quadruped_data.timelist, self.model_dict['filt']['py'].quadruped_data.leg_list[0].dt1, title="Filtered Python FR_thigh Velocity")
                plot_power_spectrum(self.model_dict['filt']['gz'].data_dict['time'], self.model_dict['filt']['gz'].data_dict['FR_thigh_vel'], title="Filtered Gazebo FR_thigh Velocity")
            
            synthetic_velocity_spectrum(self)
            raw_velocity_spectra(self)
            filtered_velocity_spectra(self)
            
        def torque_spectra(self):
            def synthetic_torque_spectrum(self: PowerAnalysis):
                pass
                # plot_power_spectrum(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt1, title="Synthetic Python FR_thigh Torque")

            def raw_torque_spectra(self: PowerAnalysis):
                # plot_power_spectrum(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.leg_list[0].dt1, title="Raw Python FR_thigh Torque")
                plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_tor'], title="Raw FR_thigh Torque")
            
            def filtered_torque_spectra(self: PowerAnalysis):
                # plot_power_spectrum(self.model_dict['filt']['py'].quadruped_data.timelist, self.model_dict['filt']['py'].quadruped_data.leg_list[0].dt1, title="Filtered Python FR_thigh Torque")
                plot_power_spectrum(self.model_dict['filt']['gz'].data_dict['time'], self.model_dict['filt']['gz'].data_dict['FR_thigh_tor'], title="Filtered Gazebo FR_thigh Torque")
            
            synthetic_torque_spectrum(self)
            raw_torque_spectra(self)
            filtered_torque_spectra(self)
        
        def tor_vel_pow_comparison(self: PowerAnalysis):
            # plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_tor'], title="Raw FR_thigh Torque")
            # plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_vel'], title="Raw FR_thigh Velocity")
            # plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_pow'], title="Raw FR_thigh Power")

            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_pos'], title="Raw FR_calf Position")
            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_tor'], title="Raw FR_calf Torque")
            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_vel'], title="Raw FR_calf Velocity")
            plot_power_spectrum(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_pow'], title="Raw FR_calf Power")

        
        power_spectra(self)
        # velocity_spectra(self)
        # torque_spectra(self) # Torque is not tracked by the python model directly, only changes in energy
        # tor_vel_pow_comparison(self)
        
    def tracking_error(self):
        def position_tracking_error(self: PowerAnalysis):
            plt.figure()
            plt.title('Desired Position vs Achieved Position')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].t1, label='Desired FR_thigh Position')
            plt.plot(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_pos'], label='Achieved FR_thigh Position')
            plt.legend()
        
        def velocity_tracking_error(self: PowerAnalysis):
            plt.figure()
            plt.title('Desired Velocity vs Achieved Velocity')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt1, label='Desired FR_thigh Velocity')
            plt.plot(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_thigh_vel'], label='Achieved FR_thigh Velocity')
            plt.legend()
        
        position_tracking_error(self)
        velocity_tracking_error(self)
        
    
    def debug(self):
        # Plot used to visualize the resolution of the energy as computed by the python model with synthetic parameters
        # Illustrates why synthetic power was highly noisy due to resolution constraints
        def synthetic_energy(self: PowerAnalysis):
            plt.figure()
            plt.title('Synthetic Energy')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.energy_trajectory(), label='Synthetic Python Power')
            plt.legend()
        
        # Plots used to compare joint velocities based on finite difference in the python model vs importing directly from gazebo
        def manual_vel_test(self: PowerAnalysis):
            self.model_dict['raw']['py'].quadruped_data.refresh()
            
            plt.figure()
            plt.title('Velocity Noise Comparison')
            plt.plot(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.leg_list[1].dt2, label='Python Computed Angular Velocity')

            self.model_dict['raw']['py'].manual_velocity_override()
            
            plt.plot(self.model_dict['raw']['py'].quadruped_data.timelist, self.model_dict['raw']['py'].quadruped_data.leg_list[1].dt2, label='Gazebo Computed Angluar Velocity')
            plt.legend()
            
        # Plots used to compare the commanded joint positions and velocities through pure analytical methods vs sampling the command through the ROS node
        def synthetic_kinematics(self: PowerAnalysis):
            plt.figure()
            plt.title('Synthetic Position Comparison')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].t1, label='Synthetic Thigh Pos')
            # plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].t2, label='Synthetic Calf Pos')
            plt.plot(self.model_dict['debug']['py_computed_vel'].quadruped_data.timelist, self.model_dict['debug']['py_computed_vel'].quadruped_data.leg_list[0].t1, label='Synthetic Thigh Pos (Num)')
            # plt.plot(self.model_dict['debug']['py_computed_vel'].quadruped_data.timelist, self.model_dict['debug']['py_computed_vel'].quadruped_data.leg_list[0].t2, label='Synthetic Calf Pos (Num)')
            plt.legend()
            
            plt.figure()
            plt.title('Synthetic Velocity Comparison')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt1, label='Synthetic Thigh Velocity')
            # plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].dt2, label='Synthetic Calf Velocity')
            plt.plot(self.model_dict['debug']['py_computed_vel'].quadruped_data.timelist, self.model_dict['debug']['py_computed_vel'].quadruped_data.leg_list[0].dt1, label='Synthetic Thigh Vel (Num)')
            # plt.plot(self.model_dict['debug']['py_computed_vel'].quadruped_data.timelist, self.model_dict['debug']['py_computed_vel'].quadruped_data.leg_list[0].dt2, label='Synthetic Calf Vel (Num)')
            plt.legend()
        
        # Plots used to compare the torque inputs before and after removal of derivitive kick
        def plugin_noise_analysis(self: PowerAnalysis):
            fig = plt.figure()
            ax = fig.add_subplot(111)
            ax.tick_params(axis='both', which='major', width=2.5)
            # plt.title('Plugin Noise Analysis')
            plt.plot(self.model_dict['debug']['pre_derivative_change'].data_dict['time'], self.model_dict['debug']['pre_derivative_change'].data_dict['FR_calf_tor'], label='Error Derivative Based Signal')
            plt.plot(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_tor'], label='Position Derivative Based Signal')
            plt.legend(fontsize=14)
            plt.xlabel('Time (s)', fontsize=14)
            plt.ylabel(r'Applied Torque (N$\cdot$m)', fontsize=14)
        
        def power_spectra_and_signals(self):
            plot_power_spectrum(self.model_dict['debug']['no_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_no_saturation'].data_dict['power'], title="no_damping_no_saturation")
            plot_power_spectrum(self.model_dict['debug']['no_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_yes_saturation'].data_dict['power'], title="no_damping_yes_saturation")
            plot_power_spectrum(self.model_dict['debug']['yes_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_no_saturation'].data_dict['power'], title="yes_damping_no_saturation")
            plot_power_spectrum(self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['power'], title="yes_damping_yes_saturation")

            plt.figure()
            plt.title('Power Signals')
            plt.plot(self.model_dict['debug']['no_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_no_saturation'].data_dict['power'], label='no_damping_no_saturation')
            plt.plot(self.model_dict['debug']['no_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_yes_saturation'].data_dict['power'], label='no_damping_yes_saturation')
            plt.plot(self.model_dict['debug']['yes_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_no_saturation'].data_dict['power'], label='yes_damping_no_saturation')
            plt.plot(self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['power'], label='yes_damping_yes_saturation')
            plt.legend()

            plt.figure()
            plt.title('Position Signals')
            plt.plot(self.model_dict['synth']['py'].quadruped_data.timelist, self.model_dict['synth']['py'].quadruped_data.leg_list[0].t1, label='Synthetic')
            plt.plot(self.model_dict['debug']['no_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_no_saturation'].data_dict['FR_thigh_pos'], label='no_damping_no_saturation')
            plt.plot(self.model_dict['debug']['no_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_yes_saturation'].data_dict['FR_thigh_pos'], label='no_damping_yes_saturation')
            plt.plot(self.model_dict['debug']['yes_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_no_saturation'].data_dict['FR_thigh_pos'], label='yes_damping_no_saturation')
            plt.plot(self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['FR_thigh_pos'], label='yes_damping_yes_saturation')
            plt.legend()

            plt.figure()
            plt.title('Torque Signals')
            plt.plot(self.model_dict['debug']['no_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_no_saturation'].data_dict['FR_thigh_tor'], label='no_damping_no_saturation')
            plt.plot(self.model_dict['debug']['no_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['no_damping_yes_saturation'].data_dict['FR_thigh_tor'], label='no_damping_yes_saturation')
            plt.plot(self.model_dict['debug']['yes_damping_no_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_no_saturation'].data_dict['FR_thigh_tor'], label='yes_damping_no_saturation')
            plt.plot(self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['time'], self.model_dict['debug']['yes_damping_yes_saturation'].data_dict['FR_thigh_tor'], label='yes_damping_yes_saturation')
            plt.legend()

        
        # synthetic_energy(self)        
        synthetic_kinematics(self)
        # manual_vel_test(self)
        # plugin_noise_analysis(self)
        # power_spectra_and_signals(self)
        
    def temp(self):
        plt.figure()
        plt.title('Torque and Omega')
        plt.plot(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_vel'], label='FR_calf Velocity')
        plt.plot(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_tor'], label='FR_calf Torque')
        plt.plot(self.model_dict['raw']['gz'].data_dict['time'], self.model_dict['raw']['gz'].data_dict['FR_calf_pow'], label='FR_calf Power')
        plt.legend()
            

def main():
    power_analysis = PowerAnalysis()
    # power_analysis.power_comparison()
    power_analysis.power_spectrum_comparison()
    # power_analysis.power()
    # power_analysis.tracking_error()
    # power_analysis.debug()
    # power_analysis.temp()
    plt.show()

if __name__ == "__main__":
    main()