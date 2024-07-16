import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import correlate
from matplotlib.ticker import MaxNLocator

# Load the camera data from the uploaded CSV file
file_path = 'CAMERA.csv'
data = pd.read_csv(file_path)

# Load the LDV data from the uploaded CSV file
ldv_file_path = 'LDV.csv'
ldv_data = pd.read_csv(ldv_file_path, skiprows=6, delimiter=';')


# Rename columns and clean data
ldv_data.columns = ['acquisition_time', 'epoch_time_ms', 'distance_mm']

# Convert epoch_time_ms to numeric (it should already be numeric, but we'll coerce any errors just in case)
ldv_data['epoch_time_ms'] = pd.to_numeric(ldv_data['epoch_time_ms'], errors='coerce')

# Drop any rows with missing values
ldv_data = ldv_data.dropna(subset=['epoch_time_ms', 'distance_mm'])

# Convert epoch time to match the camera data time format
ldv_data['epoch_time'] = ldv_data['epoch_time_ms'] / 1000  # Convert to seconds

# Aligning the LDV and camera data on the time axis
# We'll use the 'time' column from the camera data and the 'epoch_time' column from the LDV data

# Merging the camera and LDV data on the time axis
merged_data = pd.merge_asof(data.sort_values('time'), 
                            ldv_data[['epoch_time', 'distance_mm']].sort_values('epoch_time'), 
                            left_on='time', right_on='epoch_time', direction='nearest')

# Extracting the relevant columns for x-axis displacement from all three cameras and the LDV data
cam1_trans_x = merged_data['cam1_trans_x']
cam2_trans_x = merged_data['cam2_trans_x']
cam3_trans_x = merged_data['cam3_trans_x']
ldv_displacement = merged_data['distance_mm']

# Aligning the data with zero by subtracting the first data point
cam1_trans_x_aligned = cam1_trans_x - cam1_trans_x.iloc[0]
cam2_trans_x_aligned = cam2_trans_x - cam2_trans_x.iloc[0]
cam3_trans_x_aligned = cam3_trans_x - cam3_trans_x.iloc[0]
ldv_displacement_aligned = ldv_displacement - ldv_displacement.iloc[0]



# Plotting the comparisons with the specified colors and aligned LDV data for all three cameras
fig, ax = plt.subplots(figsize=(14, 8))

ax.plot(cam1_trans_x_aligned, label='Camera 1', color='red')
ax.plot(cam2_trans_x_aligned, label='Camera 2', color='orange')
ax.plot(cam3_trans_x_aligned, label='Camera 3', color='green')


# Set y-axis to show millimeters
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_yticklabels(['{:.0f}'.format(x * 1000) for x in ax.get_yticks()])

# Set x-axis to show seconds
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_xticklabels(['{:.0f}'.format(x/60) for x in ax.get_xticks()])

ax.set_ylabel('Displacement (mm)')
ax.set_xlabel('Time (s)')
ax.legend()
ax.grid(True, which='both', linestyle='--', color='gray')
ax.set_title('Displacement Comparison of Three Cameras')


plt.tight_layout()
plt.show()