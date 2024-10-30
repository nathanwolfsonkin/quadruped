import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import find_peaks
from energy_model.kinematics_data.angle_converter import get_angle_lists
from energy_model.kinematics_data.ground_velocity import get_frame_time

def main():
    # Get angle data for the legs
    leg1, leg2 = get_angle_lists()
    leg1_t1 = leg1[0]
    
    # Get timing information
    frame_time, total_time, total_frames = get_frame_time()
    
    # Create timeseries with time in the first column and the angle in the second column
    timeseries = np.zeros([total_frames, 2])
    timeseries[:, 0] = np.linspace(0, total_time, total_frames)
    timeseries[:, 1] = leg1_t1
    
    # Extract the values (angle data) for FFT analysis
    signal = timeseries[:, 1]
    
    # Perform FFT
    fft_values = np.fft.fft(signal)
    fft_freqs = np.fft.fftfreq(total_frames, d=frame_time)
    
    # Compute power spectrum
    power_spectrum = np.abs(fft_values) ** 2
    
    # Only keep the positive half of the spectrum
    half_n = total_frames // 2
    fft_freqs = fft_freqs[:half_n]
    power_spectrum = power_spectrum[:half_n]
    fft_values = fft_values[:half_n]  # Also take the positive half of FFT values for amplitudes and phases
    
    # Check the zero frequency (DC component) manually
    dc_power = power_spectrum[0]
    print(f"DC Component: {fft_freqs[0]:.2f} Hz with power {dc_power:.2f}")
    
    # Find peaks in the power spectrum excluding the DC component
    peaks, _ = find_peaks(power_spectrum[1:])  # Exclude the zero frequency
    peaks += 1  # Adjust indices for the offset
    
    # Sort peaks by power and select the top N
    N = 4  # Number of dominant frequencies to display
    dominant_peaks = peaks[np.argsort(power_spectrum[peaks])[-N:]]
    dominant_frequencies = fft_freqs[dominant_peaks]
    dominant_powers = power_spectrum[dominant_peaks]
    
    # Include the DC component in the results
    dominant_frequencies = np.insert(dominant_frequencies, 0, fft_freqs[0])
    dominant_powers = np.insert(dominant_powers, 0, dc_power)
    
    # Print the dominant frequencies and their powers
    for i, freq in enumerate(dominant_frequencies):
        print(f"Frequency {i + 1}: {freq:.2f} Hz with power {dominant_powers[i]:.2f}")
    
    # Construct continuous approximation function from dominant frequencies
    time_continuous = np.linspace(0, total_time, 1000)  # High-resolution time for smooth plotting
    approximation = np.zeros_like(time_continuous)
    
    # Add DC component
    approximation += fft_values[0].real / total_frames  # Average of the signal (DC component)
    
    # Sum of sinusoids for each dominant frequency
    for i, freq in enumerate(dominant_frequencies[1:]):  # Exclude DC, start from 1
        index = dominant_peaks[i]
        amplitude = (2 * np.abs(fft_values[index])) / total_frames  # Multiply by 2 for positive frequencies
        phase = np.angle(fft_values[index])
        
        # Add sinusoidal component to the approximation
        approximation += amplitude * np.cos(2 * np.pi * freq * time_continuous + phase)

    
    # Plot the power spectrum with dominant frequencies marked
    plt.figure()
    plt.plot(fft_freqs, power_spectrum)
    plt.plot(dominant_frequencies, dominant_powers, "x", label="Dominant Frequencies")
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Power')
    plt.title('Power Spectrum')
    plt.legend()
    
    # Plot the original signal vs. its approximation
    plt.figure()
    plt.plot(timeseries[:, 0], signal, label="Original Signal")
    plt.plot(time_continuous, approximation, label="Approximated Signal", linestyle="--")
    plt.xlabel('Time (s)')
    plt.ylabel('Signal')
    plt.title('Original Signal vs. Fourier Approximation')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    main()
