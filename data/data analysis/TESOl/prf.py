import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, resample, welch

# Load the camera data
camera_data_file_path = 'data_Exp1_20s_2024-07_16_09_24_03.csv'
ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-00.721.csv'

cam = pd.read_csv(camera_data_file_path)
ldv = pd.read_csv(ldv_data_file_path, skiprows=6, delimiter=';')

# LDV Processing
ldv.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']
ldv['epoch_time_ms'] = pd.to_numeric(ldv['epoch_time_ms'], errors='coerce')
ldv = ldv.dropna(subset=['epoch_time_ms', 'distance_mm'])
ldv['epoch_time'] = ldv['epoch_time_ms'] / 1000

ldv_sampling_frequency = 1 / np.mean(np.diff(ldv['epoch_time']))

ldv_new_sampling_frequency = 60
b, a = butter(5, 20/500, btype='low')
ldvf = filtfilt(b, a, ldv['distance_mm'].ravel())

num_original_samples = len(ldvf)
num_new_samples = int(num_original_samples * ldv_new_sampling_frequency / ldv_sampling_frequency)

ldvd = resample(ldvf, num_new_samples)
resampled_time = np.linspace(0, len(ldvd) / ldv_new_sampling_frequency, num_new_samples)

# Camera Data Processing
time = cam['Time (s)'] - cam['Time (s)'].iloc[0]
data = cam[['Cam1 Position X', 'Cam2 Position X', 'Cam3 Position X']].values

tc1 = np.arange(0, time.iloc[-1], 1/1000)
datau = np.zeros((len(tc1), 3))
for i in range(3):
    datau[:, i] = np.interp(tc1, time, data[:, i])

cam_resampled = resample(datau, 60 * len(datau)//1000)
tc_resampled = np.arange(len(cam_resampled)) / 60
camd = (cam_resampled - cam_resampled[9, :]) * 1000

# Diagnostic Outputs
print("LDV Resampled Time Vector:")
print(resampled_time)

print("Camera Resampled Time Vector:")
print(tc_resampled)

print("Length of LDV Data:", len(ldvd))
print("Length of Camera Data:", len(camd))

# Plotting in the Time Domain
plt.figure(1)
plt.plot(resampled_time, ldvd, label='LDV')
plt.plot(tc_resampled, camd[:, 0], label='CAM1')
plt.plot(tc_resampled, camd[:, 1], label='CAM2')
plt.plot(tc_resampled, camd[:, 2], label='CAM3')
plt.title('LDV vs Camera Data in Time Domain')
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.legend()

# Frequency Domain Analysis using FFT
Nfft = 2**11
fs = 60

# Welch's method for LDV data
f_ldv, Pldv = welch(ldvd, fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Welch's method for Camera data (assuming comparing first camera data)
f_cam, Pc1 = welch(camd[:, 0], fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam, Pc2 = welch(camd[:, 1], fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)
f_cam, Pc3 = welch(camd[:, 2], fs=fs, nperseg=Nfft//2, noverlap=Nfft//4, nfft=Nfft)

# Plotting in the Frequency Domain
plt.figure(2)
plt.semilogy(f_ldv, Pldv, label='LDV')
plt.semilogy(f_cam, Pc1, label='CAM1')
plt.semilogy(f_cam, Pc2, label='CAM2')
plt.semilogy(f_cam, Pc3, label='CAM3')
plt.title('LDV vs Camera Data in Frequency Domain')
plt.xlabel('Frequency (Hz)')
plt.ylabel('Power Spectral Density')
plt.legend()

plt.show()
