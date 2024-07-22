#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

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

cam = pd.read_csv(camera_data_file_path)
ldv = pd.read_csv(ldv_data_file_path, skiprows=6, delimiter=';')

# LDV
ldv.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']  # Rename the columns
ldv['epoch_time_ms'] = pd.to_numeric(ldv['epoch_time_ms'], errors='coerce')  # Convert epoch_time_ms to numeric
ldv = ldv.dropna(subset=['epoch_time_ms', 'distance_mm'])  # Drop any rows with missing values
ldv['epoch_time'] = ldv['epoch_time_ms'] / 1000  # Convert to seconds

# Calculate the original sampling frequency
ldv_sampling_frequency = 1 / np.mean(np.diff(ldv['epoch_time']))
print("Original sampling frequency:", ldv_sampling_frequency)

# Set the new sampling frequency
ldv_new_sampling_frequency = 60

# Butterworth filter
b, a = butter(5, 20/500, btype='low')
ldvf = filtfilt(b, a, ldv['distance_mm'].ravel())  # Filter the data

# Calculate the number of samples for the new sampling frequency
num_original_samples = len(ldvf)
num_new_samples = int(num_original_samples * ldv_new_sampling_frequency / ldv_sampling_frequency)
print("Number of samples after downsampling:", num_new_samples)

# Resample the data to the new sampling frequency
ldvd = resample(ldvf, num_new_samples)

# Create a new time axis for the resampled data
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Calculate the new sampling frequency for verification
ldv_new_sampling_frequency_calculated = 1 / np.mean(np.diff(resampled_time))
print("New sampling frequency calculated from resampled data:", ldv_new_sampling_frequency_calculated)

# Plotting (Optional)
plt.figure(1)
plt.plot(resampled_time, ldvd)
plt.title(f'Filtered and Resampled LDV Data (Experiment {exp})')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True, which='both', linestyle='--')
plt.savefig(f'_Filtered_resampled_ldv_data - Exp {exp}.png')
# plt.show()

# Cam Interpolation
# now, save time axes from the camera data into time with header 'Time (s)'
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
data = cam[['Cam1 Position X', 'Cam2 Position X', 'Cam3 Position X']].values

# interpolate the data to 1 KHz
tc1 = np.arange(0, time.iloc[-1], 1/1000)
datau = np.zeros((len(tc1), 3))
for i in range(3):
    datau[:, i] = np.interp(tc1, time, data[:, i])

# Resample to 60 Hz
cam_resampled = resample(datau, 60 * len(datau)//1000)

# Time vector for resampled data
tc_resampled = np.arange(len(cam_resampled)) / 60
camd = (cam_resampled - cam_resampled[9, :]) * 1000

# Trim the LDV data to match the length of the camera data
ldvd = ldvd[:len(cam_resampled)]
resampled_time = resampled_time[:len(cam_resampled)]

# Plotting
plt.figure(2)
plt.plot(tc_resampled, camd)
plt.title(f'CAM Data (Experiment {exp})')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True, which='both', linestyle='--')
plt.legend(['CAM1', 'CAM2', 'CAM3'])
plt.savefig(f'_CAM_data - Exp {exp}.png')
# plt.show()

# Adjust time and plot both CAM and LDV data
plt.figure(3)
plt.plot(tc_resampled[:-12], camd[12:, :])
plt.plot(resampled_time, -ldvd)
plt.legend(['CAM1', 'CAM2', 'CAM3', 'LDV'])
plt.title(f'Adjusted Time Data (Experiment {exp})')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.grid(True, which='both', linestyle='--')
plt.savefig(f'_Adjusted_time_data - Exp {exp}.png')
# use minor grid and use more ytick labels
plt.minorticks_on()
plt.yticks(np.arange(-10, 11, 1.5))

# FFT
Nfft = 2**11
fs = 60
f_ldv, Pldv = welch(ldvd, fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam, Pc1 = welch(camd[:, 0], fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam, Pc2 = welch(camd[:, 1], fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam, Pc3 = welch(camd[:, 2], fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
plt.figure(4)
plt.semilogy(f_ldv, Pldv, label='LDV')
plt.semilogy(f_cam, Pc1, label='CAM1')
plt.semilogy(f_cam, Pc2, label='CAM2')
plt.semilogy(f_cam, Pc3, label='CAM3')
plt.title(f'LDV vs. Camera Data in Frequency Domain - (Experiment {exp})')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.legend(['LDV', 'CAM1', 'CAM2', 'CAM3'])
plt.grid(True, which='both', linestyle='--')
plt.savefig(f'_Frequency_domain_data - Exp {exp}.png')
plt.show()

