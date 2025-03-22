import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks

import simulation.parameters as sim_params
from simulation.post_processing.csv_data_processer import CSVDataProcesser

from simulation.post_processing.low_pass_filter import Filter

def plot_power_spectrum(time, signal, title=""):
    total_frames = len(time)
    frame_time = time[1] - time[0]
    
    time = np.array(time)
    signal = np.array(signal)
    
    # Perform FFT
    fft_values = np.fft.fft(signal)
    fft_freqs = np.fft.fftfreq(total_frames, d=frame_time)
    
    # Compute power spectrum
    power_spectrum = np.abs(fft_values) ** 2
    
    # Sort frequencies and power spectrum from lowest to highest frequency
    sorted_indices = np.argsort(fft_freqs)
    sorted_fft_freqs = fft_freqs[sorted_indices]
    sorted_power_spectrum = power_spectrum[sorted_indices]
    
    # Only keep the positive half of the spectrum
    positive_half_n = total_frames // 2
    positive_freqs = sorted_fft_freqs[positive_half_n:]
    positive_power_spectrum = sorted_power_spectrum[positive_half_n:]
    
    # Find peaks in the power spectrum
    peaks, _ = find_peaks(positive_power_spectrum)
    
    # Plot power spectrum density
    plt.figure(figsize=(10, 5))
    plt.plot(positive_freqs, np.log10(positive_power_spectrum), label="Power Spectrum Density")
    plt.scatter(positive_freqs[peaks], np.log10(positive_power_spectrum[peaks]), color='red', label="Peaks")
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Log Power')
    plt.title(f"PSD - {title}")
    plt.legend()
    plt.grid()

def main():
    csv_dir = sim_params.get_latest_log(sim_params.raw_logging_directory)
    
    # Extract data
    data_dict = CSVDataProcesser.csv_to_dict(csv_dir)
    pid = ' P-I-D = 20-0-1'
    
    filt_RL_thigh_pos = Filter.butterworth_filter(data_dict['time'], data_dict['FL_thigh_pos'], sim_params.pos_filt_cutoff_freq)
    
    plot_power_spectrum(data_dict['time'], data_dict['FL_thigh_pos'], title='raw_FL_thigh_pos' + pid)
    plot_power_spectrum(data_dict['time'], filt_RL_thigh_pos, title='filt_FL_thigh_pos' + pid)

    plt.show()


if __name__ == "__main__":
   main()