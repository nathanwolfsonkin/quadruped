import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from energy_model.kinematics_data.data_post_process import VideoDataProcess

def to_timeseries(timelist, datalist):
    if len(timelist) != len(datalist):
        raise ValueError("The vectors are not of equal size")
    
    return np.column_stack((timelist, datalist))

def get_dominant_sine_waves(timeseries, N=2):
    """
    Analyze the timeseries and extract the N dominant sine waves including the DC component.
    
    Parameters:
        timeseries: np.ndarray
            A 2D array with time in the first column and signal in the second column.
        N: int
            The number of dominant frequencies (plus DC component) to include.
    
    Returns:
        dominant_frequencies: np.ndarray
            The frequencies of the dominant sine waves.
        dominant_amplitudes: np.ndarray
            The amplitudes of the dominant sine waves.
        dominant_phases: np.ndarray
            The phases of the dominant sine waves.
        total_frames: int
            The total number of frames in the original signal (used for scaling).
    """
    total_frames = len(timeseries[:, 0])
    time_list = timeseries[:, 0]
    frame_time = time_list[1] - time_list[0]
    
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
    
    # Check the zero frequency (DC component)
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
    
    # Compute amplitudes and phases for the dominant sine waves
    dominant_amplitudes = []
    dominant_phases = []
    
    for i, freq in enumerate(dominant_frequencies):
        if i == 0:
            # DC component
            amplitude = fft_values_positive[0].real / total_frames
            phase = 0
        else:
            index = dominant_peaks[i - 1]  # Adjust for DC
            amplitude = (2 * np.abs(fft_values_positive[index])) / total_frames  # Multiply by 2 for positive frequencies
            phase = np.angle(fft_values_positive[index])
        
        dominant_amplitudes.append(amplitude)
        dominant_phases.append(phase)
    
    return dominant_frequencies, np.array(dominant_amplitudes), np.array(dominant_phases), total_frames

def fourier_approx(timeseries, N=2, resolution_multiplier=1):
    """
    Generate an Nth-order Fourier approximation of the given time series with the desired resolution.
    
    Parameters:
        timeseries: np.ndarray
            A 2D array with time in the first column and signal in the second column.
        N: int
            The number of dominant frequencies (plus DC component) to include.
        resolution_multiplier: int
            Factor to increase the resolution of the output time series.
    
    Returns:
        approximation: np.ndarray
            The approximated signal.
        high_res_time_list: np.ndarray
            The time array with increased resolution.
    """
    # Extract original time data
    original_time = timeseries[:, 0]
    total_time = original_time[-1]
    total_frames = len(original_time)
    frame_time = original_time[1] - original_time[0]
    
    # Compute dominant sine waves
    dominant_frequencies, dominant_amplitudes, dominant_phases, _ = get_dominant_sine_waves(timeseries, N)
    
    # Generate high-resolution time vector
    high_res_total_frames = total_frames * resolution_multiplier
    high_res_time_list = np.linspace(0, total_time, high_res_total_frames)
    
    # Construct the approximation
    approximation = np.zeros_like(high_res_time_list)
    
    # Add sine wave components
    for i, freq in enumerate(dominant_frequencies):
        approximation += dominant_amplitudes[i] * np.cos(2 * np.pi * freq * high_res_time_list + dominant_phases[i])
    
    return approximation.tolist()

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

    # Define number of peaks and resolution multiplier
    N = 2
    resolution_multiplier = 4  # Increase resolution by a factor of 4
    
    # Generate Fourier approximation
    high_res_timeseries = fourier_approx(timeseries, N, resolution_multiplier)
    
    # Plot results
    plt.figure()
    plt.plot(timeseries[:, 0], timeseries[:, 1], label="Original Signal")
    plt.plot(high_res_timeseries[:, 0], high_res_timeseries[:, 1], label="High-Resolution Approximation", linestyle="--")
    plt.xlabel('Time (s)')
    plt.ylabel('Signal')
    plt.title('Original Signal vs. Fourier Approximation')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
