import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import decimate, butter, filtfilt, periodogram

# Experiment No. 1
# Load the camera data from the uploaded CSV file
camera_data_file_path = 'data_Exp1_20s_2024-07_16_09_24_03.csv'
# Load the LDV data from the uploaded CSV file
ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-00.721.csv'
exp = 1

# # Experiment No. 2
# camera_data_file_path = 'data_Exp1_20s_2024_07__16_09_24_56.csv'
# ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-54.529.csv'
# exp = 2

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

# Convert camera data from meters to millimeters
camera_data['Cam1 Position X'] = camera_data['Cam1 Position X'] * 1000
camera_data['Cam2 Position X'] = camera_data['Cam2 Position X'] * 1000
camera_data['Cam3 Position X'] = camera_data['Cam3 Position X'] * 1000

# Calculate the sampling frequency of the camera data
camera_sampling_frequency = 1 / np.mean(np.diff(camera_data['Time (s)']))
print(f'Camera Sampling Frequency: {camera_sampling_frequency:.2f} Hz')

# Calculate the sampling frequency of the LDV data
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv_data['epoch_time']))
print(f'LDV Sampling Frequency: {ldv_sampling_frequency:.2f} Hz')


# # Upsample the camera data to 1000 Hz using linear interpolation
# camera_data_upsampled = pd.DataFrame()
# camera_data_upsampled['Time (s)'] = np.linspace(camera_data['Time (s)'].iloc[0], camera_data['Time (s)'].iloc[-1], len(camera_data) * 1000 // 60)
# camera_data_upsampled['Cam1 Position X'] = np.interp(camera_data_upsampled['Time (s)'], camera_data['Time (s)'], camera_data['Cam1 Position X'])
# camera_data_upsampled['Cam2 Position X'] = np.interp(camera_data_upsampled['Time (s)'], camera_data['Time (s)'], camera_data['Cam2 Position X'])
# camera_data_upsampled['Cam3 Position X'] = np.interp(camera_data_upsampled['Time (s)'], camera_data['Time (s)'], camera_data['Cam3 Position X'])

# # Now again calculating the frequency of the upsampled camera data
# camera_upsampled_sampling_frequency = 1 / np.mean(np.diff(camera_data_upsampled['Time (s)']))
# print(f'Upsampled Camera Sampling Frequency: {camera_upsampled_sampling_frequency:.2f} Hz')


# Low-pass filter the LDV data to prevent aliasing
nyquist = 60 / 2  # Nyquist frequency for the target rate
cutoff = nyquist - 10  # Cut-off frequency slightly below the Nyquist frequency
b, a = butter(4, cutoff / (1000 / 2), btype='low')
ldv_displacement_filtered = filtfilt(b, a, ldv_data['distance_mm'].values)

# Downsample LDV data from 1000 Hz to 60 Hz using decimate
decimation_factor = 1000 // 60
ldv_displacement_resampled = decimate(ldv_displacement_filtered, decimation_factor)

# Print the number of samples of LDV before and after resampling
print(f'Number of samples of LDV before resampling: {len(ldv_displacement_filtered)}')
print(f'Number of samples of LDV after resampling: {len(ldv_displacement_resampled)}')
print(f'Number of samples of Camera 1: {len(camera_data)}')


# find the first timestamp of the camera data and then try to find that in the ldv data timestamp
# then get the index of that timestamp and then get the index of the last timestamp of the camera data
# then use those indices to get the ldv data between those timestamps
# then resample the ldv data to match the camera data timestamps
# then plot the displacement comparison of the camera data and the ldv data
cam_ts_first = camera_data['Time (s)'].iloc[0]
cam_ts_last = camera_data['Time (s)'].iloc[-1]
print(f'Starting time of camera recording in human readable format: {pd.to_datetime(cam_ts_first, unit="s")}')
print(f'Ending time of camera recording in human readable format: {pd.to_datetime(cam_ts_last, unit="s")}')



ldv_ts_first = ldv_data['epoch_time'].iloc[0]
ldv_ts_last = ldv_data['epoch_time'].iloc[-1]
print(f'Starting time of LDV recording in human readable format: {pd.to_datetime(ldv_ts_first, unit="s")}')
print(f'Ending time of LDV recording in human readable format: {pd.to_datetime(ldv_ts_last, unit="s")}')


# now try to find the index in ldv_data which is most nearer to the cam_ts_first timestamp
for i, ts in enumerate(ldv_data['epoch_time']):
    if ts >= cam_ts_first:
        ldv_start_index = i
        break
print(f'Index of the first timestamp of camera data in LDV data: {ldv_start_index}')
# now display the timestamp of cam_ts_first and ldv_data[ldv_start_index] to see if they are close enough
cam_first = pd.to_datetime(cam_ts_first, unit='s')
ldv_first = pd.to_datetime(ldv_data["epoch_time"].iloc[ldv_start_index], unit='s')
print(f'Timestamp of the first timestamp of camera data in human readable: {cam_first}')
print(f'Timestamp of the first timestamp of LDV    data in human readable: {ldv_first}')




# Interpolate the camera data to align with the LDV timestamps
ldv_resampled_timestamps = np.linspace(ldv_data['epoch_time'].iloc[0], ldv_data['epoch_time'].iloc[-1], len(ldv_displacement_resampled))

camera_data_interp = pd.DataFrame()
camera_data_interp['Time (s)'] = ldv_resampled_timestamps
camera_data_interp['Cam1 Position X'] = np.interp(ldv_resampled_timestamps, camera_data['Time (s)'], camera_data['Cam1 Position X'])
camera_data_interp['Cam2 Position X'] = np.interp(ldv_resampled_timestamps, camera_data['Time (s)'], camera_data['Cam2 Position X'])
camera_data_interp['Cam3 Position X'] = np.interp(ldv_resampled_timestamps, camera_data['Time (s)'], camera_data['Cam3 Position X'])

# Align the data with zero by subtracting the first data point
cam1_trans_x_aligned = camera_data_interp['Cam1 Position X'] - camera_data_interp['Cam1 Position X'][0]
cam2_trans_x_aligned = camera_data_interp['Cam2 Position X'] - camera_data_interp['Cam2 Position X'][0]
cam3_trans_x_aligned = camera_data_interp['Cam3 Position X'] - camera_data_interp['Cam3 Position X'][0]
ldv_displacement_resampled_aligned = ldv_displacement_resampled - ldv_displacement_resampled[0]

# Plot the displacement comparison of LDV before and after resampling
fig, axs = plt.subplots(3, 2, figsize=(14, 18))

axs[0, 0].plot(ldv_resampled_timestamps, cam1_trans_x_aligned, label='Camera 1', color='red')
axs[0, 0].plot(ldv_resampled_timestamps, ldv_displacement_resampled_aligned, label='LDV', color='blue')
axs[0, 0].set_ylabel('Displacement (mm)')
axs[0, 0].legend()
axs[0, 0].grid(True, which='both', linestyle='--', color='gray')
axs[0, 0].set_title('Displacement Comparison of Camera 1 and LDV')

axs[1, 0].plot(ldv_resampled_timestamps, cam2_trans_x_aligned, label='Camera 2', color='green')
axs[1, 0].plot(ldv_resampled_timestamps, ldv_displacement_resampled_aligned, label='LDV', color='blue')
axs[1, 0].set_ylabel('Displacement (mm)')
axs[1, 0].legend()
axs[1, 0].grid(True, which='both', linestyle='--', color='gray')
axs[1, 0].set_title('Displacement Comparison of Camera 2 and LDV')

axs[2, 0].plot(ldv_resampled_timestamps, cam3_trans_x_aligned, label='Camera 3', color='orange')
axs[2, 0].plot(ldv_resampled_timestamps, ldv_displacement_resampled_aligned, label='LDV', color='blue')
axs[2, 0].set_ylabel('Displacement (mm)')
axs[2, 0].set_xlabel('Time (s)')
axs[2, 0].legend()
axs[2, 0].grid(True, which='both', linestyle='--', color='gray')
axs[2, 0].set_title('Displacement Comparison of Camera 3 and LDV')

# Plot the frequency plots of both the signals on top of each other
f_cam1, Pxx_cam1 = periodogram(cam1_trans_x_aligned, fs=60)
f_ldv1, Pxx_ldv1 = periodogram(ldv_displacement_resampled_aligned, fs=60)
axs[0, 1].semilogy(f_cam1, Pxx_cam1, label='Camera 1', color='red')
axs[0, 1].semilogy(f_ldv1, Pxx_ldv1, label='LDV', color='blue')
axs[0, 1].set_title('Frequency Plot (Camera 1 and LDV)')
axs[0, 1].set_xlabel('Frequency (Hz)')
axs[0, 1].set_ylabel('Power Spectral Density')
axs[0, 1].legend()
axs[0, 1].grid(True, which='both', linestyle='--', color='gray')

f_cam2, Pxx_cam2 = periodogram(cam2_trans_x_aligned, fs=60)
f_ldv2, Pxx_ldv2 = periodogram(ldv_displacement_resampled_aligned, fs=60)
axs[1, 1].semilogy(f_cam2, Pxx_cam2, label='Camera 2', color='green')
axs[1, 1].semilogy(f_ldv2, Pxx_ldv2, label='LDV', color='blue')
axs[1, 1].set_title('Frequency Plot (Camera 2 and LDV)')
axs[1, 1].set_xlabel('Frequency (Hz)')
axs[1, 1].set_ylabel('Power Spectral Density')
axs[1, 1].legend()
axs[1, 1].grid(True, which='both', linestyle='--', color='gray')

f_cam3, Pxx_cam3 = periodogram(cam3_trans_x_aligned, fs=60)
f_ldv3, Pxx_ldv3 = periodogram(ldv_displacement_resampled_aligned, fs=60)
axs[2, 1].semilogy(f_cam3, Pxx_cam3, label='Camera 3', color='orange')
axs[2, 1].semilogy(f_ldv3, Pxx_ldv3, label='LDV', color='blue')
axs[2, 1].set_title('Frequency Plot (Camera 3 and LDV)')
axs[2, 1].set_xlabel('Frequency (Hz)')
axs[2, 1].set_ylabel('Power Spectral Density')
axs[2, 1].legend()
axs[2, 1].grid(True, which='both', linestyle='--', color='gray')

plt.tight_layout(rect=[0, 0, 1, 0.95])
fig.subplots_adjust(hspace=0.4)  # Adjust the hspace to create vertical gaps
frequency_plot_path = "frequency_comparison"
plt.savefig(f'{frequency_plot_path}_exp{exp}.png', dpi=300)
plt.show()

plt.close(fig)  # Close the figure to avoid reusing the same canvas

# # Print the highest frequency of the LDV data before and after resampling
# print(f'Highest frequency of LDV before resampling: {f_ldv[np.argmax(Pxx_ldv)]:.2f} Hz')
# print(f'Highest frequency of LDV after resampling: {f_ldv_resampled[np.argmax(Pxx_ldv_resampled)]:.2f} Hz')
