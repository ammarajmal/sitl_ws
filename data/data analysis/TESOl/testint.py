#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import decimate, butter, filtfilt, cheby1, ellip, freqz, correlate, periodogram

# Experiment No. 1
# Load the camera data from the uploaded CSV file
camera_data_file_path = 'data_Exp1_20s_2024-07_16_09_24_03.csv'
# Load the LDV data from the uploaded CSV file
ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-00.721.csv'
exp = 1

# # # Experiment No. 2
# camera_data_file_path = 'data_Exp1_20s_2024_07__16_09_24_56.csv'
# ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-54.529.csv'
# exp = 2

# Experiment No. 3
# camera_data_file_path = 'data_Exp1_20s_2024-07-16_09-22-15.csv'
# ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-22-12.684.csv'
# exp = 3

camera_data = pd.read_csv(camera_data_file_path)
ldv_data = pd.read_csv(ldv_data_file_path, skiprows=6, delimiter=';')

# Rename columns and clean data
ldv_data.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']

# Convert epoch_time_ms to numeric
ldv_data['epoch_time_ms'] = pd.to_numeric(ldv_data['epoch_time_ms'], errors='coerce')

# Drop any rows with missing values
ldv_data = ldv_data.dropna(subset=['epoch_time_ms', 'distance_mm'])

# Convert epoch time to match the camera data time format
ldv_data['epoch_time'] = ldv_data['epoch_time_ms'] / 1000  # Convert to seconds

# Find the first camera timestamp in the camera data
first_camera_timestamp = camera_data['Time (s)'].iloc[0]

# Discard the LDV data before the first camera timestamp
ldv_data = ldv_data[ldv_data['epoch_time'] >= first_camera_timestamp]


# Convert camera data from meters to millimeters
camera_data['Cam1 Position X'] = camera_data['Cam1 Position X'] * 1000
camera_data['Cam2 Position X'] = camera_data['Cam2 Position X'] * 1000
camera_data['Cam3 Position X'] = camera_data['Cam3 Position X'] * 1000

# Calculate the sampling frequency of the camera data
camera_sampling_frequency = 1 / np.mean(np.diff(camera_data['Time (s)']))
print(f'Camera Sampling Frequency: {camera_sampling_frequency:.2f} Hz')

# Calculate the sampling frequency of the LDV data
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv_data['epoch_time']))
print(f'Fin LDV Sampling Frequency: {ldv_sampling_frequency:.2f} Hz')


# Low-pass filter the LDV data to prevent aliasing
nyquist = 60 / 2  # Nyquist frequency for the target rate
cutoff = nyquist - 10  # Cut-off frequency slightly below the Nyquist frequency

# Uncomment the desired filter design

# Butterworth Filter
# order = 10
# b, a = butter(order, cutoff / (1000 / 2), btype='low')

# Chebyshev Type I Filter
order = 10
rp = 0.5  # Maximum ripple in the passband (dB)
b, a = cheby1(order, rp, cutoff / (1000 / 2), btype='low')

# Elliptic Filter
# order = 10
# rp = 0.5  # Maximum ripple in the passband (dB)
# rs = 40  # Minimum attenuation in the stopband (dB)
# b, a = ellip(order, rp, rs, cutoff / (1000 / 2), btype='low')

# Apply the filter to the LDV data
ldv_displacement_filtered = filtfilt(b, a, ldv_data['distance_mm'].values)

# Downsample LDV data from 1000 Hz to 60 Hz using decimate
decimation_factor = 1000 // 60
ldv_displacement_resampled = decimate(ldv_displacement_filtered, decimation_factor)




# Print the number of samples of LDV before and after resampling
print(f'Number of samples of LDV before resampling: {len(ldv_displacement_filtered)}')
print(f'Number of samples of LDV after resampling: {len(ldv_displacement_resampled)}')
print(f'Number of samples of Camera 1: {len(camera_data)}')

# Frequency Plot Comparison of LDV before and after resampling
f_ldv, Pxx_ldv = periodogram(ldv_displacement_filtered, fs=1000)
f_ldv_resampled, Pxx_ldv_resampled = periodogram(ldv_displacement_resampled, fs=60)

# Extract the camera displacement data
cam1_trans_x = camera_data['Cam1 Position X'].values
cam2_trans_x = camera_data['Cam2 Position X'].values
cam3_trans_x = camera_data['Cam3 Position X'].values

# Align the data with zero by subtracting the first data point
cam1_trans_x_aligned = cam1_trans_x - cam1_trans_x[0]
cam2_trans_x_aligned = cam2_trans_x - cam2_trans_x[0]
cam3_trans_x_aligned = cam3_trans_x - cam3_trans_x[0]
ldv_displacement_resampled_aligned = ldv_displacement_resampled 

# Function to apply cross-correlation and shift with windowing and fine-tuning
def align_signals(camera_data, ldv_data, window_size=100):
    # Apply a window to the signals to focus on the significant parts
    camera_windowed = camera_data[window_size:-window_size]
    ldv_windowed = ldv_data[window_size:-window_size]

    correlation = correlate(camera_windowed, ldv_windowed)
    lag = np.argmax(correlation) - (len(ldv_windowed) + 60)
    
    # Adjust for the lag
    if lag > 0:
        camera_data_aligned = camera_data[lag:]
        ldv_data_aligned = ldv_data[:len(camera_data_aligned)]
    else:
        ldv_data_aligned = ldv_data[-lag:]
        camera_data_aligned = camera_data[:len(ldv_data_aligned)]
    
    return camera_data_aligned, ldv_data_aligned, lag

# Apply the refined alignment
cam1_aligned, ldv1_aligned, lag1 = align_signals(cam1_trans_x_aligned, ldv_displacement_resampled_aligned)
cam2_aligned, ldv2_aligned, lag2 = align_signals(cam2_trans_x_aligned, ldv_displacement_resampled_aligned)
cam3_aligned, ldv3_aligned, lag3 = align_signals(cam3_trans_x_aligned, ldv_displacement_resampled_aligned)

def samples_to_time(samples, fs):
    return samples / fs



# Sampling frequency of the cameras
print(f'Camera Sampling Frequency: {camera_sampling_frequency:.2f} Hz'
      f' (1 sample every {1 / camera_sampling_frequency:.2f} seconds)'
        f' (1 sample every {camera_sampling_frequency:.2f} Hz)')
print(f'LDV Sampling Frequency: {ldv_sampling_frequency:.2f} Hz'
        f' (1 sample every {1 / ldv_sampling_frequency:.2f} seconds)'
            f' (1 sample every {ldv_sampling_frequency:.2f} Hz)')
print(f'LDV Resampled Sampling Frequency: {60:.2f} Hz'
        f' (1 sample every {1 / 60:.2f} seconds)'
            f' (1 sample every {60:.2f} Hz)')

print(f'Camera 1 Lag: {lag1} samples ({lag1 / camera_sampling_frequency:.2f} seconds)')
print(f'Camera 2 Lag: {lag2} samples ({lag2 / camera_sampling_frequency:.2f} seconds)')
print(f'Camera 3 Lag: {lag3} samples ({lag3 / camera_sampling_frequency:.2f} seconds)')
print(f'Camera 1 Duration: {len(cam1_aligned) / camera_sampling_frequency:.2f} seconds')
print(f'Camera 2 Duration: {len(cam2_aligned) / camera_sampling_frequency:.2f} seconds')
print(f'Camera 3 Duration: {len(cam3_aligned) / camera_sampling_frequency:.2f} seconds')

# Function to apply FFT and compute frequency domain plot
def plot_fft(ax, time_data, signal_data, label, fs, color):
    # Compute the FFT of the signal
    fft_data = np.fft.fft(signal_data)
    # Compute the frequency axis
    freqs = np.fft.fftfreq(len(signal_data), d=1/fs)
    # Compute the magnitude of the FFT
    fft_magnitude = np.abs(fft_data)
    # Plot the magnitude of the FFT
    ax.semilogy(freqs[:len(freqs)//2], fft_magnitude[:len(fft_magnitude)//2], label=label, color=color)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Magnitude')
    ax.legend()
    ax.grid(True, which='both', linestyle='--')
# Plot the displacement comparison of LDV before and after resampling
fig, axs = plt.subplots(3, 2, figsize=(14, 18))

time_cam1 = samples_to_time(np.arange(len(cam1_aligned)), camera_sampling_frequency)
time_cam2 = samples_to_time(np.arange(len(cam2_aligned)), camera_sampling_frequency)
time_cam3 = samples_to_time(np.arange(len(cam3_aligned)), camera_sampling_frequency)
time_ldv1 = samples_to_time(np.arange(len(ldv1_aligned)), 60)
time_ldv2 = samples_to_time(np.arange(len(ldv2_aligned)), 60)
time_ldv3 = samples_to_time(np.arange(len(ldv3_aligned)), 60)

axs[0, 0].plot(time_cam1, cam1_aligned, label='Camera 1', color='red')
axs[0, 0].plot(time_ldv1, ldv1_aligned, label='LDV', color='blue', linestyle='dotted')
axs[0, 0].set_ylabel('Displacement (mm)')
axs[0, 0].set_xlabel('Time (s)')
axs[0, 0].legend()
axs[0, 0].grid(True, which='both', linestyle='--',  linewidth=0.5)
axs[0, 0].set_title(f'Displacement Comparison of Camera 1')

axs[1, 0].plot(time_cam2, cam2_aligned, label='Camera 2', color='green')
axs[1, 0].plot(time_ldv2, ldv2_aligned, label='LDV', color='blue', linestyle='dotted')
axs[1, 0].set_ylabel('Displacement (mm)')
axs[1, 0].set_xlabel('Time (s)')
axs[1, 0].legend()
axs[1, 0].grid(True, which='both', linestyle='--')
axs[1, 0].set_title(f'Displacement Comparison of Camera 2')

axs[2, 0].plot(time_cam3, cam3_aligned, label='Camera 3', color='orange')
axs[2, 0].plot(time_ldv3, ldv3_aligned, label='LDV', color='blue', linestyle='dotted')
axs[2, 0].set_ylabel('Displacement (mm)')
axs[2, 0].set_xlabel('Time (s)')
axs[2, 0].legend()
axs[2, 0].grid(True, which='both', linestyle='--')
axs[2, 0].set_title(f'Displacement Comparison of Camera 3')

# Plot the frequency plots using FFT
plot_fft(axs[0, 1], time_cam1, cam1_aligned, 'Camera 1', 60, 'red')
plot_fft(axs[0, 1], time_ldv1, ldv1_aligned, 'LDV', 60, 'gray')
axs[0, 1].set_title('Frequency Plot (Camera 1 and LDV)')

plot_fft(axs[1, 1], time_cam2, cam2_aligned, 'Camera 2', 60, 'green')
plot_fft(axs[1, 1], time_ldv2, ldv2_aligned, 'LDV', 60, 'gray')
axs[1, 1].set_title('Frequency Plot (Camera 2 and LDV)')

plot_fft(axs[2, 1], time_cam3, cam3_aligned, 'Camera 3', 60, 'orange')
plot_fft(axs[2, 1], time_ldv3, ldv3_aligned, 'LDV', 60, 'gray')
axs[2, 1].set_title('Frequency Plot (Camera 3 and LDV)')

plt.tight_layout(rect=[0, 0, 1, 0.95])
fig.subplots_adjust(hspace=0.4)  # Adjust the hspace to create vertical gaps
frequency_plot_path = "frequency_comparison_fft"
plt.savefig(f'{frequency_plot_path}_exp{exp}.png', dpi=300)
plt.show()

plt.close(fig)  # Close the figure to avoid reusing the same canvas

# Print the highest frequency of the LDV data before and after resampling
print(f'Highest frequency of LDV before resampling: {f_ldv[np.argmax(Pxx_ldv)]:.2f} Hz')
print(f'Highest frequency of LDV after resampling: {f_ldv_resampled[np.argmax(Pxx_ldv_resampled)]:.2f} Hz')

# print(f'Highest frequency of Camera 1: {f_cam1[np.argmax(Pxx_cam1)]:0.3f} Hz')
# print(f'Highest frequency of Camera 2: {f_cam2[np.argmax(Pxx_cam2)]:0.3f} Hz')
# print(f'Highest frequency of Camera 3: {f_cam3[np.argmax(Pxx_cam3)]:0.3f} Hz')

# # Plot the displacement comparison of LDV before and after resampling
# fig, axs = plt.subplots(3, 2, figsize=(14, 18))

# time_cam1 = samples_to_time(np.arange(len(cam1_aligned)), camera_sampling_frequency)
# time_cam2 = samples_to_time(np.arange(len(cam2_aligned)), camera_sampling_frequency)
# time_cam3 = samples_to_time(np.arange(len(cam3_aligned)), camera_sampling_frequency)
# time_ldv1 = samples_to_time(np.arange(len(ldv1_aligned)), 60)
# time_ldv2 = samples_to_time(np.arange(len(ldv2_aligned)), 60)
# time_ldv3 = samples_to_time(np.arange(len(ldv3_aligned)), 60)

# axs[0, 0].plot(time_cam1, cam1_aligned, label='Camera 1', color='red')
# axs[0, 0].plot(time_ldv1, ldv1_aligned, label='LDV', color='blue', linestyle='dotted')
# axs[0, 0].set_ylabel('Displacement (mm)')
# axs[0, 0].set_xlabel('Time (s)')
# axs[0, 0].legend()
# axs[0, 0].grid(True, which='both', linestyle='--', color='gray', linewidth=0.5)
# axs[0, 0].set_title(f'Displacement Comparison of Camera 1)')



# axs[1, 0].plot(time_cam2, cam2_aligned, label='Camera 2', color='green')
# axs[1, 0].plot(time_ldv2, ldv2_aligned, label='LDV', color='blue', linestyle='dotted')
# axs[1, 0].set_ylabel('Displacement (mm)')
# axs[1, 0].set_xlabel('Time (s)')
# axs[1, 0].legend()
# axs[1, 0].grid(True, which='both', linestyle='--', color='gray')
# axs[1, 0].set_title(f'Displacement Comparison of Camera 2)')

# axs[2, 0].plot(time_cam3, cam3_aligned, label='Camera 3', color='orange')
# axs[2, 0].plot(time_ldv3, ldv3_aligned, label='LDV', color='blue', linestyle='dotted')
# axs[2, 0].set_ylabel('Displacement (mm)')
# axs[2, 0].set_xlabel('Time (s)')
# axs[2, 0].legend()
# axs[2, 0].grid(True, which='both', linestyle='--', color='gray')
# axs[2, 0].set_title(f'Displacement Comparison of Camera 3)')

# # Plot the frequency plots of both the signals on top of each other
# f_cam1, Pxx_cam1 = periodogram(cam1_aligned, fs=60)
# f_ldv1, Pxx_ldv1 = periodogram(ldv1_aligned, fs=60)
# axs[0, 1].semilogy(f_cam1, Pxx_cam1, label='Camera 1', color='red')
# axs[0, 1].semilogy(f_ldv1, Pxx_ldv1, label='LDV', color='gray')
# axs[0, 1].set_title('Frequency Plot (Camera 1 and LDV)')
# axs[0, 1].set_xlabel('Frequency (Hz)')
# axs[0, 1].set_ylabel('Power Spectral Density')
# axs[0, 1].legend()
# axs[0, 1].grid(True, which='both', linestyle='--', color='gray')

# f_cam2, Pxx_cam2 = periodogram(cam2_aligned, fs=60)
# f_ldv2, Pxx_ldv2 = periodogram(ldv2_aligned, fs=60)
# axs[1, 1].semilogy(f_cam2, Pxx_cam2, label='Camera 2', color='green')
# axs[1, 1].semilogy(f_ldv2, Pxx_ldv2, label='LDV', color='gray')
# axs[1, 1].set_title('Frequency Plot (Camera 2 and LDV)')
# axs[1, 1].set_xlabel('Frequency (Hz)')
# axs[1, 1].set_ylabel('Power Spectral Density')
# axs[1, 1].legend()
# axs[1, 1].grid(True, which='both', linestyle='--', color='gray')

# f_cam3, Pxx_cam3 = periodogram(cam3_aligned, fs=60)
# f_ldv3, Pxx_ldv3 = periodogram(ldv3_aligned, fs=60)
# axs[2, 1].semilogy(f_cam3, Pxx_cam3, label='Camera 3', color='orange')
# axs[2, 1].semilogy(f_ldv3, Pxx_ldv3, label='LDV', color='gray')
# axs[2, 1].set_title('Frequency Plot (Camera 3 and LDV)')
# axs[2, 1].set_xlabel('Frequency (Hz)')
# axs[2, 1].set_ylabel('Power Spectral Density')
# axs[2, 1].legend()
# axs[2, 1].grid(True, which='both', linestyle='--', color='gray')

# plt.tight_layout(rect=[0, 0, 1, 0.95])
# fig.subplots_adjust(hspace=0.4)  # Adjust the hspace to create vertical gaps
# frequency_plot_path = "frequency_comparison"
# plt.savefig(f'{frequency_plot_path}_exp{exp}.png', dpi=300)
# plt.show()

# plt.close(fig)  # Close the figure to avoid reusing the same canvas

# # Print the highest frequency of the LDV data before and after resampling
# print(f'Highest frequency of LDV before resampling: {f_ldv[np.argmax(Pxx_ldv)]:.2f} Hz')
# print(f'Highest frequency of LDV after resampling: {f_ldv_resampled[np.argmax(Pxx_ldv_resampled)]:.2f} Hz')
# print(f'Highest frequency of Camera 1: {f_cam1[np.argmax(Pxx_cam1)]:0.3f} Hz')
# print(f'Highest frequency of Camera 2: {f_cam2[np.argmax(Pxx_cam2)]:0.3f} Hz')
# print(f'Highest frequency of Camera 3: {f_cam3[np.argmax(Pxx_cam3)]:0.3f} Hz')