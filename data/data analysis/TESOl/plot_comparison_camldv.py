#!/usr/bin/env python3

#!/usr/bin/env python3
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import decimate, butter, filtfilt, cheby1, ellip, freqz, correlate, periodogram
from matplotlib.ticker import AutoMinorLocator, MultipleLocator

# Experiment No. 1
# Load the camera data from the uploaded CSV file
# camera_data_file_path = 'data_Exp1_20s_2024-07_16_09_24_03.csv'
# # Load the LDV data from the uploaded CSV file
# ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-00.721.csv'
# exp = 1

# # Experiment No. 2
camera_data_file_path = 'data_Exp1_20s_2024_07__16_09_24_56.csv'
ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-54.529.csv'
exp = 2

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


camera_sampling_frequency = 1 / np.mean(np.diff(camera_data['Time (s)']))
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv_data['epoch_time']))


# now upsample the cameras to match the LDV sampling frequency
cam_upsampled = pd.DataFrame()
cam_upsampled['Time (s)'] = np.arange(first_camera_timestamp, camera_data['Time (s)'].iloc[-1], 1/ldv_sampling_frequency)
cam_upsampled['Cam1 Position X'] = np.interp(cam_upsampled['Time (s)'], camera_data['Time (s)'], camera_data['Cam1 Position X'])
cam_upsampled['Cam2 Position X'] = np.interp(cam_upsampled['Time (s)'], camera_data['Time (s)'], camera_data['Cam2 Position X'])
cam_upsampled['Cam3 Position X'] = np.interp(cam_upsampled['Time (s)'], camera_data['Time (s)'], camera_data['Cam3 Position X'])

cam_upsampled_sampling_frequency = 1 / np.mean(np.diff(cam_upsampled['Time (s)']))

print(f'Camera        Sampling Frequency: {camera_sampling_frequency:.2f} Hz')
print(f'LDV           Sampling Frequency: {ldv_sampling_frequency:.2f} Hz')
print(f'Cam Upsampled Sampling Frequency: {cam_upsampled_sampling_frequency:.2f} Hz')

# now comapre the start time of the camera and the ldv data
print(f'Camera        Start Time: {pd.to_datetime(camera_data["Time (s)"].iloc[0], unit="s")}')
print(f'LDV           Start Time: {pd.to_datetime(ldv_data["epoch_time"].iloc[0], unit="s")}')
print(f'Cam Upsampled Start Time: {pd.to_datetime(cam_upsampled["Time (s)"].iloc[0], unit="s")}')

# compare the number of samples of the camera and the ldv data
print(f'Camera        Number of Samples: {len(camera_data)}')
print(f'LDV           Number of Samples: {len(ldv_data)}')
print(f'Cam Upsampled Number of Samples: {len(cam_upsampled)}')

# compare the duration of the camera and the ldv data
print(f'Camera        Duration: {camera_data["Time (s)"].iloc[-1] - camera_data["Time (s)"].iloc[0]} s')
print(f'LDV           Duration: {ldv_data["epoch_time"].iloc[-1] - ldv_data["epoch_time"].iloc[0]} s')
print(f'Cam Upsampled Duration: {cam_upsampled["Time (s)"].iloc[-1] - cam_upsampled["Time (s)"].iloc[0]} s')

# Extract Camera and LDV Displacement Data
cam1_disp_x = cam_upsampled['Cam1 Position X'].values
cam2_disp_x = cam_upsampled['Cam2 Position X'].values
cam3_disp_x = cam_upsampled['Cam3 Position X'].values
ldv_disp_x = -ldv_data['distance_mm'].values
# Align the camera and ldv data with the zero by subtracting the first data point
cam1_disp_x_alligned = cam1_disp_x - cam1_disp_x[0]
cam2_disp_x_alligned = cam2_disp_x - cam2_disp_x[0]
cam3_disp_x_alligned = cam3_disp_x - cam3_disp_x[0]
ldv_disp_x = ldv_disp_x - np.mean(ldv_disp_x)



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

# Apply the refined alignment function to the camera and LDV data
cam1_aligned, ldv1_aligned, lag1 = align_signals(cam1_disp_x_alligned, ldv_disp_x)
cam2_aligned, ldv2_aligned, lag2 = align_signals(cam2_disp_x_alligned, ldv_disp_x)
cam3_aligned, ldv3_aligned, lag3 = align_signals(cam3_disp_x_alligned, ldv_disp_x)

def samples_to_time(samples, fs):
    return samples / fs
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
# fig, axs = plt.subplots(3, 2, figsize=(14, 18))
fig, axs = plt.subplots( figsize=(14, 18))
time_cam1 = samples_to_time(np.arange(len(cam1_aligned)), cam_upsampled_sampling_frequency)
time_cam2 = samples_to_time(np.arange(len(cam2_aligned)), cam_upsampled_sampling_frequency)
time_cam3 = samples_to_time(np.arange(len(cam3_aligned)), cam_upsampled_sampling_frequency)
time_ldv1 = samples_to_time(np.arange(len(ldv1_aligned)), ldv_sampling_frequency)
time_ldv2 = samples_to_time(np.arange(len(ldv2_aligned)), ldv_sampling_frequency)
time_ldv3 = samples_to_time(np.arange(len(ldv3_aligned)), ldv_sampling_frequency)

# plot half of the data
# time_cam1 = time_cam1[:len(time_cam1)//2]
plt.plot(time_cam1[:len(time_cam1)//2], cam1_aligned[:len(time_cam1)//2], label='Camera 1', color='b')
plt.plot(time_cam2[:len(time_cam1)//2], cam2_aligned[:len(time_cam1)//2], label='Camera 2', color='r')
plt.plot(time_cam3[:len(time_cam1)//2], cam3_aligned[:len(time_cam1)//2], label='Camera 3', color='g')
plt.plot(time_ldv1[:len(time_cam1)//2], ldv1_aligned[:len(time_cam1)//2], label='LDV 1', color='y')
plt.title(f'Exp {exp}: Camera 1 vs LDV 1 _{camera_data_file_path}')
plt.xlabel('Time (s)')
plt.ylabel('Displacement (mm)')
plt.legend()
plt.grid(True, which='both', linestyle='--')
plt.minorticks_on()
plt.grid(which='minor', linestyle=':', linewidth='0.5', color='lightgray')
plt.tick_params(axis='both', which='both', direction='in', length=6)
plt.tick_params(axis='both', which='minor', direction='in', length=4, width=0.5)
# set minor axes locator to 2
# set minor axes ticks on
plt.show()



# # For the first subplot, plot the displacement data of Camera 1 and LDV 1
# axs[0, 0].plot(time_cam1, cam1_aligned, label='Camera 1', color='b')
# axs[0, 0].plot(time_ldv1, ldv1_aligned, label='LDV 1', color='y')
# axs[0, 0].set_title('Camera 1 vs LDV 1')
# axs[0, 0].set_xlabel('Time (s)')
# axs[0, 0].set_ylabel('Displacement (mm)')
# axs[0, 0].legend()
# axs[0, 0].grid(True, which='both', linestyle='--')
# axs[0, 0].minorticks_on()
# axs[0, 0].grid(which='minor', linestyle=':', linewidth='0.5', color='lightgray')
# axs[0, 0].tick_params(axis='both', which='both', direction='in', length=6)
# axs[0, 0].tick_params(axis='both', which='minor', direction='in', length=4, width=0.5)
# axs[0, 0].xaxis.set_major_locator(MultipleLocator(2))  # Major ticks every 2 seconds
# axs[0, 0].yaxis.set_major_locator(MultipleLocator(1.5))  # Major ticks every 1 mm
# axs[0, 0].xaxis.set_minor_locator(AutoMinorLocator(2))  # 4 minor ticks per major tick
# axs[0, 0].yaxis.set_minor_locator(AutoMinorLocator(4))  # 4 minor ticks per major tick

# # For the second subplot, plot the displacement data of Camera 2 and LDV 2
# axs[1, 0].plot(time_cam2, cam2_aligned, label='Camera 2', color='r')
# axs[1, 0].plot(time_ldv2, ldv2_aligned, label='LDV 2', color='y')
# axs[1, 0].set_title('Camera 2 vs LDV 2')
# axs[1, 0].set_xlabel('Time (s)')
# axs[1, 0].set_ylabel('Displacement (mm)')
# axs[1, 0].legend()
# axs[1, 0].grid(True, which='both', linestyle='--')
# axs[1, 0].minorticks_on()
# axs[1, 0].grid(which='minor', linestyle=':', linewidth='0.5', color='lightgray')
# axs[1, 0].tick_params(axis='both', which='both', direction='in', length=6)
# axs[1, 0].tick_params(axis='both', which='minor', direction='in', length=4, width=0.5)
# axs[1, 0].xaxis.set_major_locator(MultipleLocator(2))  # Major ticks every 2 seconds
# axs[1, 0].yaxis.set_major_locator(MultipleLocator(1.5))  # Major ticks every 1 mm
# axs[1, 0].xaxis.set_minor_locator(AutoMinorLocator(2))  # 4 minor ticks per major tick
# axs[1, 0].yaxis.set_minor_locator(AutoMinorLocator(4))  # 4 minor ticks per

# # For the third subplot, plot the displacement data of Camera 3 and LDV 3
# axs[2, 0].plot(time_cam3, cam3_aligned, label='Camera 3', color='g', linewidth=2.5)
# axs[2, 0].plot(time_ldv3, ldv3_aligned, label='LDV 3', color='y',linewidth=1.5)
# axs[2, 0].set_title('Camera 3 vs LDV 3')
# axs[2, 0].set_xlabel('Time (s)')
# axs[2, 0].set_ylabel('Displacement (mm)')
# axs[2, 0].legend()
# axs[2, 0].grid(True, which='both', linestyle='--')
# axs[2, 0].minorticks_on()
# axs[2, 0].grid(which='minor', linestyle=':', linewidth='0.5', color='lightgray')
# axs[2, 0].tick_params(axis='both', which='both', direction='in', length=6)
# axs[2, 0].tick_params(axis='both', which='minor', direction='in', length=4, width=0.5)
# axs[2, 0].xaxis.set_major_locator(MultipleLocator(2))  # Major ticks every 2 seconds
# axs[2, 0].yaxis.set_major_locator(MultipleLocator(1.5))  # Major ticks every 1 mm
# axs[2, 0].xaxis.set_minor_locator(AutoMinorLocator(2))  # 4 minor ticks per major tick
# axs[2, 0].yaxis.set_minor_locator(AutoMinorLocator(4))  # 4 minor ticks per major tick


# # Plot the frequency plots using FFT
# plot_fft(axs[0, 1], time_cam1, cam1_aligned, 'Camera 1', cam_upsampled_sampling_frequency, 'b')
# plot_fft(axs[1, 1], time_cam2, cam2_aligned, 'Camera 2', cam_upsampled_sampling_frequency, 'r')
# plot_fft(axs[2, 1], time_cam3, cam3_aligned, 'Camera 3', cam_upsampled_sampling_frequency, 'g')

# plot_fft(axs[0, 1], time_ldv1, ldv1_aligned, 'LDV 1', ldv_sampling_frequency, 'y')
# plot_fft(axs[1, 1], time_ldv2, ldv2_aligned, 'LDV 2', ldv_sampling_frequency, 'y')
# plot_fft(axs[2, 1], time_ldv3, ldv3_aligned, 'LDV 3', ldv_sampling_frequency, 'y')

# axs[0, 1].set_title('Frequency Plot (Camera 1 and LDV)')
# axs[1, 1].set_title('Frequency Plot (Camera 2 and LDV)')
# axs[2, 1].set_title('Frequency Plot (Camera 3 and LDV)')










# plt.show()



