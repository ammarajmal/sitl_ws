import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import decimate, correlate
from scipy.signal import decimate, butter, filtfilt, correlate

from matplotlib.ticker import MaxNLocator


# Load the camera data from the uploaded CSV file
file_path = 'CAMERA.csv'
data = pd.read_csv(file_path)

# Load the LDV data from the uploaded CSV file
ldv_file_path = 'LDV.csv'
ldv_data = pd.read_csv(ldv_file_path, skiprows=6, delimiter=';')

# Rename columns and clean data
ldv_data.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']

# Convert epoch_time_ms to numeric
ldv_data['epoch_time_ms'] = pd.to_numeric(ldv_data['epoch_time_ms'], errors='coerce')

# Drop any rows with missing values
ldv_data = ldv_data.dropna(subset=['epoch_time_ms', 'distance_mm'])

# Convert epoch time to match the camera data time format
ldv_data['epoch_time'] = ldv_data['epoch_time_ms'] / 1000  # Convert to seconds

# Convert camera data from meters to millimeters
data['cam1_trans_x'] = data['cam1_trans_x'] * 1000

# Low-pass filter the LDV data to prevent aliasing
nyquist = 60 / 2  # Nyquist frequency for the target rate
cutoff = nyquist - 10  # Cut-off frequency slightly below the Nyquist frequency
b, a = butter(4, cutoff / (1000 / 2), btype='low')
ldv_displacement_filtered = filtfilt(b, a, ldv_data['distance_mm'].values)

# Downsample LDV data from 1000 Hz to 60 Hz using decimate
decimation_factor = 1000 // 60
ldv_displacement_resampled = decimate(ldv_displacement_filtered, decimation_factor)

# Extract the camera displacement data
cam1_trans_x = data['cam1_trans_x'].values

# Align the data with zero by subtracting the first data point
cam1_trans_x_aligned = cam1_trans_x - cam1_trans_x[0]
ldv_displacement_resampled_aligned = ldv_displacement_resampled - ldv_displacement_resampled[0]

# Use cross-correlation to find the best shift between the camera and LDV data
correlation = correlate(cam1_trans_x_aligned, ldv_displacement_resampled_aligned)
lag = np.argmax(correlation) - (len(ldv_displacement_resampled_aligned) - 1)

# Apply the shift
if lag > 0:
    cam1_trans_x_aligned = cam1_trans_x_aligned[lag:]
    ldv_displacement_resampled_aligned = ldv_displacement_resampled_aligned[:len(cam1_trans_x_aligned)]
else:
    ldv_displacement_resampled_aligned = ldv_displacement_resampled_aligned[-lag:]
    cam1_trans_x_aligned = cam1_trans_x_aligned[:len(ldv_displacement_resampled_aligned)]

# Plotting the comparison between Camera 1 and LDV data
fig, ax = plt.subplots(figsize=(14, 8))

ax.plot(cam1_trans_x_aligned, label='Camera 1', color='red')
ax.plot(ldv_displacement_resampled_aligned, label='LDV', color='blue')

# Set y-axis to show displacement in millimeters
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_yticklabels(['{:.0f}'.format(x) for x in ax.get_yticks()])

# Set x-axis to show sample index
ax.set_xlabel('Sample Index')
ax.set_ylabel('Displacement (mm)')
ax.legend()
ax.grid(True, which='both', linestyle='--', color='gray')
ax.set_title('Displacement Comparison of Camera 1 and LDV')

plt.tight_layout()
plt.show()
