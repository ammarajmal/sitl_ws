import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import correlate
from matplotlib.ticker import MaxNLocator

# Load the camera data from the uploaded CSV file
test_name = 'Beam Test 8i'
file_path = 'Data_Exp 7_60s_2024-06-26_20-30-00.csv'
data = pd.read_csv(file_path)
# now get the cam1_trans_x,cam2_trans_x, and cam3_trans_x from the data
cam1_trans_x = data['cam1_trans_x']
cam2_trans_x = data['cam2_trans_x']
cam3_trans_x = data['cam3_trans_x']

# Aligning the data with zero by subtracting the first data point
cam1_trans_x_aligned = cam1_trans_x - cam1_trans_x.iloc[0]
cam2_trans_x_aligned = cam2_trans_x - cam2_trans_x.iloc[0]
cam3_trans_x_aligned = cam3_trans_x - cam3_trans_x.iloc[0]

# Creating a figure with 4 subplots (1 row for comparison, 3 rows for individual cameras)
fig, axs = plt.subplots(4, 1, figsize=(14, 20))  # Increase the figure size for better clarity

# Main comparison plot
axs[0].plot(cam1_trans_x_aligned, label='Camera 1', color='red')
axs[0].plot(cam2_trans_x_aligned, label='Camera 2', color='orange')
axs[0].plot(cam3_trans_x_aligned, label='Camera 3', color='green')
axs[0].set_title(f'Displacement Comparison of Three Cameras - {test_name}')
axs[0].set_ylabel('Displacement (mm)')
axs[0].legend()
axs[0].grid(True, which='both', linestyle='--', color='gray')

# Individual camera plots
cameras = [cam1_trans_x_aligned, cam2_trans_x_aligned, cam3_trans_x_aligned]
colors = ['red', 'orange', 'green']
for i in range(3):
    axs[i+1].plot(cameras[i], color=colors[i])
    axs[i+1].set_title(f'Camera {i+1} Displacement')
    axs[i+1].set_ylabel('Displacement (mm)')
    axs[i+1].set_xlabel('Time (s)')
    axs[i+1].grid(True, which='both', linestyle='--', color='gray')

# Adjust tick labels on y-axis to show millimeters and x-axis to show seconds
for ax in axs:
    ax.yaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_yticklabels(['{:.4f}'.format(x * 1000) for x in ax.get_yticks()])
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xticklabels(['{:.0f}'.format(x/60) for x in ax.get_xticks()])
    # INCREASE VERTICAL SPACE BETWEEN SUBPLOTS
plt.subplots_adjust(hspace=0.5)
# Save the figure
fig.savefig(f'Camera_Comparison - {test_name}.png')

# plt.tight_layout()
plt.show()

