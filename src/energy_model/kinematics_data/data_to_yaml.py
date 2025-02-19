import numpy as np
import yaml
import math

from energy_model.kinematics_data.data_post_process import VideoDataProcess
from energy_model.freq_analysis import get_dominant_sine_waves

# Helper function to find the GCD of floating-point frequencies
def find_gcd_of_floats(frequencies):
    scale_factor = 10**6  # Scale factor for precision
    scaled_frequencies = [int(f * scale_factor) for f in frequencies]
    
    gcd_freq = scaled_frequencies[0]
    for freq in scaled_frequencies[1:]:
        gcd_freq = math.gcd(gcd_freq, freq)
    
    # Return the scaled GCD as a float
    return gcd_freq / scale_factor

def main():
    dataset = VideoDataProcess("unitree_a1")

    # Get angle data for the legs
    leg1, leg2 = dataset.get_angle_lists()
    zero_list = [0] * len(leg1[0])
    
    test_case = False
    
    if test_case:
        filename = '/workspace/src/energy_pkg/gait_trajectory/gait_sin_waves_test_case.yaml'
        leg_angles = {
            'FL_hip': zero_list,
            'FL_thigh': leg1[0], 
            'FL_calf': leg1[1],
            'RL_hip': zero_list,
            'RL_thigh': leg1[0], 
            'RL_calf': leg1[1],
            'RR_hip': zero_list,
            'RR_thigh': leg1[0], 
            'RR_calf': leg1[1],
            'FR_hip': zero_list,
            'FR_thigh': leg1[0], 
            'FR_calf': leg1[1],
        }
    else:
        filename = '/workspace/src/energy_pkg/gait_trajectory/gait_sin_waves.yaml'
        leg_angles = {
            'FL_hip': zero_list,
            'FL_thigh': leg1[0], 
            'FL_calf': leg1[1],
            'RL_hip': zero_list,
            'RL_thigh': leg2[0], 
            'RL_calf': leg2[1],
            'RR_hip': zero_list,
            'RR_thigh': leg1[0], 
            'RR_calf': leg1[1],
            'FR_hip': zero_list,
            'FR_thigh': leg2[0], 
            'FR_calf': leg2[1],
        }
    
    
    # Get timing information
    frame_time, total_time, total_frames = dataset.get_frame_data()
    # Define number of peaks
    num_peaks = 2
    
    leg_angles_sinwave = {}
    # Create timeseries with time in the first column and the angle in the second column
    for key in leg_angles:
        timeseries = np.zeros([total_frames, 2])
        timeseries[:, 0] = np.linspace(0, total_time, total_frames)
        timeseries[:, 1] = leg_angles[key]
        
        freq, amplitudes, phases, _ = get_dominant_sine_waves(timeseries, num_peaks)
        freq = freq.tolist()
        amplitudes = amplitudes.tolist()
        phases = phases.tolist()
        
        leg_name, joint_name = key.split("_")
        if leg_name not in leg_angles_sinwave:
            leg_angles_sinwave[leg_name] = {}
            
        if joint_name not in leg_angles_sinwave[leg_name]:
            leg_angles_sinwave[leg_name][joint_name] = {}
        
        for index in range(len(freq)):
            key_name = f"freq_amp_phase{index+1}"
            leg_angles_sinwave[leg_name][joint_name][key_name] = [
                freq[index],
                amplitudes[index],
                phases[index]
            ]

    if test_case:
        for leg in leg_angles_sinwave:
            if leg == 'RL' or leg == 'FR':
                for joint in leg_angles_sinwave[leg]:
                    freq_list = []
                    for freq_amp_phase in leg_angles_sinwave[leg][joint]:
                        freq_list.append(leg_angles_sinwave[leg][joint][freq_amp_phase][0])
                    
                    d = .25
                        
                    pass                    
                    for freq_amp_phase in leg_angles_sinwave[leg][joint]:
                        leg_angles_sinwave[leg][joint][freq_amp_phase][2] += (2*np.pi*leg_angles_sinwave[leg][joint][freq_amp_phase][0]*d)

    # Write data to YAML file
    with open(filename, mode='w') as file:
        yaml.dump(leg_angles_sinwave, file, default_flow_style=False)


if __name__ == "__main__":
    main()
