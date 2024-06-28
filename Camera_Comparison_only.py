import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import correlate
from matplotlib.ticker import MaxNLocator

# Load the camera data from the uploaded CSV file
test_name = 'Beam Test 7'
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



# Plotting the comparisons with the specified colors and aligned LDV data for all three cameras
fig, ax = plt.subplots(figsize=(14, 8))

ax.plot(cam1_trans_x_aligned, label='Camera 1', color='red')
ax.plot(cam2_trans_x_aligned, label='Camera 2', color='orange')
ax.plot(cam3_trans_x_aligned, label='Camera 3', color='green')


# Set y-axis to show millimeters
ax.yaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_yticklabels(['{:.4f}'.format(x * 1000) for x in ax.get_yticks()])

# Set x-axis to show seconds
ax.xaxis.set_major_locator(MaxNLocator(integer=True))
ax.set_xticklabels(['{:.0f}'.format(x/60) for x in ax.get_xticks()])


ax.set_ylabel('Displacement (mm)')
ax.set_xlabel('Time (s)')
ax.legend()

ax.grid(True, which='both', linestyle='--', color='gray')
ax.set_title('Displacement Comparison of Three Cameras')


plt.tight_layout()

ax.grid(True, which='both', linestyle='--', color='gray')
ax.set_title(f'Displacement Comparison of Three Cameras - {test_name}')
plt.show()
# now save the plot in the file
fig.savefig(f'Camera_Comparison - {test_name}_onePlot.png')

