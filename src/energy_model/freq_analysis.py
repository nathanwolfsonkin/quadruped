import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from energy_model.kinematics_data.data_post_process import VideoDataProcess

def to_timeseries(timelist, datalist):
    if len(timelist) != len(datalist):
        raise ValueError("The vectors are not of equal size")
    
    return np.column_stack((timelist, datalist))

def fourier_approx(timeseries, N=2):
    # Extract data from timeseries
    total_frames = len(timeseries[:, 0])
    time_list = timeseries[:, 0]
    frame_time = time_list[1] - time_list[0]
    
    # Extract the values (angle data) for FFT analysis
    signal = timeseries[:, 1]
    
    # Perform FFT
    fft_values = np.fft.fft(signal)
    fft_freqs = np.fft.fftfreq(total_frames, d=frame_time)
    
    # Compute power spectrum
    power_spectrum = np.abs(fft_values) ** 2
    
    # Sort frequencies and power spectrum from lowest to highest frequency
    sorted_indices = np.argsort(fft_freqs)
    sorted_fft_freqs = fft_freqs[sorted_indices]
    sorted_power_spectrum = power_spectrum[sorted_indices]
    
    # Only keep the positive half of the spectrum for peak detection
    positive_half_n = total_frames // 2
    positive_freqs = sorted_fft_freqs[positive_half_n:]
    positive_power_spectrum = sorted_power_spectrum[positive_half_n:]
    fft_values_positive = fft_values[sorted_indices][positive_half_n:]
    
    # Check the zero frequency (DC component) manually
    dc_power = sorted_power_spectrum[positive_half_n]
    
    # Find peaks in the power spectrum excluding the DC component
    peaks, _ = find_peaks(positive_power_spectrum[1:])  # Exclude the zero frequency
    peaks += 1  # Adjust indices for the offset
    
    # Sort peaks by power and select the top N
    dominant_peaks = peaks[np.argsort(positive_power_spectrum[peaks])[-N:]]
    dominant_frequencies = positive_freqs[dominant_peaks]
    dominant_powers = positive_power_spectrum[dominant_peaks]
    
    # Include the DC component in the results
    dominant_frequencies = np.insert(dominant_frequencies, 0, positive_freqs[0])
    dominant_powers = np.insert(dominant_powers, 0, dc_power)
    
    # Construct continuous approximation function from dominant frequencies
    approximation = np.zeros_like(time_list)
    
    # Add DC component
    approximation += fft_values_positive[0].real / total_frames  # Average of the signal (DC component)
    
    # Sum of sinusoids for each dominant frequency
    for i, freq in enumerate(dominant_frequencies[1:]):  # Exclude DC, start from 1
        index = dominant_peaks[i]
        amplitude = (2 * np.abs(fft_values_positive[index])) / total_frames  # Multiply by 2 for positive frequencies
        phase = np.angle(fft_values_positive[index])
        
        # Add sinusoidal component to the approximation
        approximation += amplitude * np.cos(2 * np.pi * freq * time_list + phase)

    if __name__ == "__main__":
        # Plot the power spectrum with dominant frequencies marked
        plt.figure()
        plt.plot(sorted_fft_freqs, np.log10(sorted_power_spectrum))
        plt.plot(dominant_frequencies, np.log10(dominant_powers), "x", label="Dominant Frequencies")
        plt.xlabel('Frequency (Hz)')
        plt.ylabel('Power')
        plt.title('Power Spectrum')
        plt.legend()
        
        # Plot the original signal vs. its approximation
        plt.figure()
        plt.plot(time_list, signal, label="Original Signal")
        plt.plot(time_list, approximation, label="Approximated Signal", linestyle="--")
        plt.xlabel('Time (s)')
        plt.ylabel('Signal')
        plt.title('Original Signal vs. Fourier Approximation')
        plt.legend()
        plt.show()
    else:
        return approximation

def main():
    dataset = VideoDataProcess("unitree_a1")

    # Get angle data for the legs
    leg1, leg2 = dataset.get_angle_lists()
    leg1_t1 = leg1[0]
    
    # Get timing information
    frame_time, total_time, total_frames = dataset.get_frame_data()
    
    # Create timeseries with time in the first column and the angle in the second column
    timeseries = np.zeros([total_frames, 2])
    timeseries[:, 0] = np.linspace(0, total_time, total_frames)
    timeseries[:, 1] = leg1_t1

    # Define number of peaks dominant frequencies to utilize (plus DC gain)
    N = 2
    fourier_approx(timeseries, N)

if __name__ == "__main__":
    main()
