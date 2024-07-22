import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import decimate, butter, filtfilt, correlate
from matplotlib.ticker import MaxNLocator

# Load the camera data from the uploaded CSV file

file_path = 'data_Exp1_20s_2024-07_16_09_24_03.csv'
# file_path = 'data_Exp1_20s_2024_07__16_09_24_56.csv'
data = pd.read_csv(file_path)

# # Load the LDV data from the uploaded CSV file
ldv_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-00.721.csv'
# ldv_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-54.529.csv'
ldv_data = pd.read_csv(ldv_file_path, skiprows=6, delimiter=';')

# Rename columns and clean data
ldv_data.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']

# # Convert epoch_time_ms to numeric
ldv_data['epoch_time_ms'] = pd.to_numeric(ldv_data['epoch_time_ms'], errors='coerce')

# # Drop any rows with missing values
ldv_data = ldv_data.dropna(subset=['epoch_time_ms', 'distance_mm'])

# # Convert epoch time to match the camera data time format
ldv_data['epoch_time'] = ldv_data['epoch_time_ms'] / 1000  # Convert to seconds

# # Convert camera data from meters to millimeters
data['Cam1 Position X'] = data['Cam1 Position X'] * 1000
data['Cam2 Position X'] = data['Cam2 Position X'] * 1000
data['Cam3 Position X'] = data['Cam3 Position X'] * 1000

# Low-pass filter the LDV data to prevent aliasing
nyquist = 60 / 2  # Nyquist frequency for the target rate
cutoff = nyquist - 10  # Cut-off frequency slightly below the Nyquist frequency
b, a = butter(4, cutoff / (1000 / 2), btype='low')
ldv_displacement_filtered = filtfilt(b, a, ldv_data['distance_mm'].values)

# Downsample LDV data from 1000 Hz to 60 Hz using decimate
decimation_factor = 1000 // 60
ldv_displacement_resampled = decimate(ldv_displacement_filtered, decimation_factor)

# Extract the camera displacement data
cam1_trans_x = data['Cam1 Position X'].values
cam2_trans_x = data['Cam2 Position X'].values
cam3_trans_x = data['Cam3 Position X'].values

# Align the data with zero by subtracting the first data point
cam1_trans_x_aligned = cam1_trans_x - cam1_trans_x[0]
cam2_trans_x_aligned = cam2_trans_x - cam2_trans_x[0]
cam3_trans_x_aligned = cam3_trans_x - cam3_trans_x[0]
ldv_displacement_resampled_aligned = ldv_displacement_resampled - ldv_displacement_resampled[0]
# cam1_trans_x_aligned = cam1_trans_x 
# cam2_trans_x_aligned = cam2_trans_x 
# cam3_trans_x_aligned = cam3_trans_x 
# ldv_displacement_resampled_aligned = ldv_displacement_resampled


# # Function to apply cross-correlation and shift
def align_signals(camera_data, ldv_data):
    correlation = correlate(camera_data, ldv_data)
    lag = np.argmax(correlation) - (len(ldv_data) + 80)
    if lag > 0:
        camera_data_aligned = camera_data[lag:]
        ldv_data_aligned = ldv_data[:len(camera_data_aligned)]
    else:
        ldv_data_aligned = ldv_data[-lag:]
        camera_data_aligned = camera_data[:len(ldv_data_aligned)]
    return camera_data_aligned, ldv_data_aligned

# Align signals
cam1_aligned, ldv1_aligned = align_signals(cam1_trans_x_aligned, ldv_displacement_resampled_aligned)
cam2_aligned, ldv2_aligned = align_signals(cam2_trans_x_aligned, ldv_displacement_resampled_aligned)
cam3_aligned, ldv3_aligned = align_signals(cam3_trans_x_aligned, ldv_displacement_resampled_aligned)

# Plotting the comparisons between LDV and Cameras 1, 2, and 3 data
fig, axs = plt.subplots(3, 1, figsize=(14, 18))

axs[0].plot(cam1_aligned, label='Camera 1', color='red')
axs[0].plot(ldv1_aligned, label='LDV', color='blue')
axs[0].set_ylabel('Displacement (mm)')
axs[0].legend()
axs[0].grid(True, which='both', linestyle='--', color='gray')
axs[0].set_title('Displacement Comparison of Camera 1 and LDV')
axs[0].yaxis.set_major_locator(MaxNLocator(integer=True))


axs[1].plot(cam2_aligned, label='Camera 2', color='green')
axs[1].plot(ldv2_aligned, label='LDV', color='blue')
axs[1].set_ylabel('Displacement (mm)')
axs[1].legend()
axs[1].grid(True, which='both', linestyle='--', color='gray')
axs[1].set_title('Displacement Comparison of Camera 2 and LDV')

axs[2].plot(cam3_aligned, label='Camera 3', color='orange')
axs[2].plot(ldv3_aligned, label='LDV', color='blue')
axs[2].set_ylabel('Displacement (mm)')
axs[2].set_xlabel('Sample Index')
axs[2].legend()
axs[2].grid(True, which='both', linestyle='--', color='gray')
axs[2].set_title('Displacement Comparison of Camera 3 and LDV')

plt.tight_layout(rect=[0, 0, 1, 0.95])
fig.subplots_adjust(hspace=0.4)  # Adjust the hspace to create vertical gaps


plt.savefig('comparison.png', dpi=300)
plt.show()