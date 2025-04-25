import csv
import numpy as np
from scipy.signal import butter, filtfilt
from pathlib import Path

from simulation.post_processing.csv_data_processer import CSVDataProcesser
import simulation.parameters as sim_params


class Filter:
    def __init__(self):
        pass
    
    @staticmethod
    def butterworth_filter(time: list, signal: list, cutoff_freq: float, order: int = 4) -> list:
        """
        Apply a Butterworth low-pass filter to time series data.

        Parameters:
            timeseries (np.ndarray): A (n, 2) array where the first column is time and the second is the signal.
            cutoff_freq (float): The cutoff frequency for the low-pass filter.
            order (int): The order of the Butterworth filter (default is 4).

        Returns:
            np.ndarray: Filtered time series with the same time component.
        """

        # Estimate sampling rate from time data
        dt = np.mean(np.diff(time))  # Average time step
        if dt <= 0:
            raise ValueError("Time values must be strictly increasing.")
        
        sampling_rate = 1.0 / dt  # Derived from time step

        # Compute normalized cutoff frequency
        nyquist = 0.5 * sampling_rate
        if not (0 < cutoff_freq < nyquist):
            raise ValueError(f"Cutoff frequency must be between 0 and {nyquist} Hz.")

        normal_cutoff = cutoff_freq / nyquist

        # Design Butterworth filter
        b, a = butter(order, normal_cutoff, btype='low', analog=False)

        # Apply zero-phase filtering
        filtered_signal = filtfilt(b, a, signal)

        return filtered_signal.tolist()
    
    @staticmethod
    def generate_filtered_csv(filepath):
        
        data_dict = CSVDataProcesser.csv_to_dict(filepath)
        
        # Filter 
        for leg in ['FR', 'FL', 'RL', 'RR']:
            for joint in ['hip', 'thigh', 'calf']:
                for state in ['pos', 'vel', 'tor']:
                    key = leg + '_' + joint + '_' + state
                    match state:
                        case 'pos':
                            data_dict[key] = Filter.butterworth_filter(data_dict['time'], data_dict[key], sim_params.pos_filt_cutoff_freq)
                        case 'vel':
                            data_dict[key] = Filter.butterworth_filter(data_dict['time'], data_dict[key], sim_params.vel_filt_cutoff_freq)
                        case 'tor':
                            data_dict[key] = Filter.butterworth_filter(data_dict['time'], data_dict[key], sim_params.tor_filt_cutoff_freq)
                        case _:
                            raise ValueError(f"State <{state}> is not recognized. State must be 'pos' 'vel' 'tor'")
                    
                    
                    
        # Output filename
        filtered_filename = Path(filepath).name
        filtered_filepath = sim_params.postprocess_filtered_logging_directory + filtered_filename
        
        # Open CSV file and write headers
        with open(filtered_filepath, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([
                "time",
                "FL_hip_pos", "FL_thigh_pos", "FL_calf_pos",
                "FR_hip_pos", "FR_thigh_pos", "FR_calf_pos",
                "RL_hip_pos", "RL_thigh_pos", "RL_calf_pos",
                "RR_hip_pos", "RR_thigh_pos", "RR_calf_pos",
                "FL_hip_vel", "FL_thigh_vel", "FL_calf_vel",
                "FR_hip_vel", "FR_thigh_vel", "FR_calf_vel",
                "RL_hip_vel", "RL_thigh_vel", "RL_calf_vel",
                "RR_hip_vel", "RR_thigh_vel", "RR_calf_vel",
                "FL_hip_tor", "FL_thigh_tor", "FL_calf_tor",
                "FR_hip_tor", "FR_thigh_tor", "FR_calf_tor",
                "RL_hip_tor", "RL_thigh_tor", "RL_calf_tor",
                "RR_hip_tor", "RR_thigh_tor", "RR_calf_tor",
                "distance"
                ])
            
            for i, value in enumerate(data_dict['time']):
                writer.writerow([
                    data_dict['time'][i],
                    data_dict['FL_hip_pos'][i], data_dict['FL_thigh_pos'][i], data_dict['FL_calf_pos'][i],
                    data_dict['FR_hip_pos'][i], data_dict['FR_thigh_pos'][i], data_dict['FR_calf_pos'][i],
                    data_dict['RL_hip_pos'][i], data_dict['RL_thigh_pos'][i], data_dict['RL_calf_pos'][i],
                    data_dict['RR_hip_pos'][i], data_dict['RR_thigh_pos'][i], data_dict['RR_calf_pos'][i],
                    data_dict['FL_hip_vel'][i], data_dict['FL_thigh_vel'][i], data_dict['FL_calf_vel'][i],
                    data_dict['FR_hip_vel'][i], data_dict['FR_thigh_vel'][i], data_dict['FR_calf_vel'][i],
                    data_dict['RL_hip_vel'][i], data_dict['RL_thigh_vel'][i], data_dict['RL_calf_vel'][i],
                    data_dict['RR_hip_vel'][i], data_dict['RR_thigh_vel'][i], data_dict['RR_calf_vel'][i],
                    data_dict['FL_hip_tor'][i], data_dict['FL_thigh_tor'][i], data_dict['FL_calf_tor'][i],
                    data_dict['FR_hip_tor'][i], data_dict['FR_thigh_tor'][i], data_dict['FR_calf_tor'][i],
                    data_dict['RL_hip_tor'][i], data_dict['RL_thigh_tor'][i], data_dict['RL_calf_tor'][i],
                    data_dict['RR_hip_tor'][i], data_dict['RR_thigh_tor'][i], data_dict['RR_calf_tor'][i],
                    data_dict['distance'][i]
                ])


def main():    
    filepath = sim_params.get_latest_log(sim_params.raw_logging_directory)
    
    Filter.generate_filtered_csv(filepath)
               
if __name__ == "__main__":
    main()

