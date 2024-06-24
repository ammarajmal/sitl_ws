import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
# Load the CSV file
camera_name = 'cam2'
file_path = f'{camera_name}.csv'
data = pd.read_csv(file_path)
data = (data - data.iloc[0]) * 1000
# Rename columns and clean data 
data.columns = ['time', 'translation_x']

# Write a function to plot the data
def plot_data(data):
    time_data = data['time'].to_numpy()
    translation_x_data = data['translation_x'].to_numpy()
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.plot(time_data, translation_x_data, linestyle='-', color='b', label=f'{camera_name} Translation X')
    ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    ax.set_xticklabels(['{:.0f}'.format(x/1000) for x in ax.get_xticks()])
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Translation X (mm)')
    ax.set_title(f'{camera_name} Translation X vs. Time')
    ax.grid(True, which='both', linestyle='--', color='gray')
    ax.legend()
    plt.show()
    

# Call the function to plot the data
plot_data(data)
