#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May  4 14:45:00 2020

"""
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import decimate, butter, filtfilt, correlate, periodogram

# Experiment No. 1
# Load the camera data from the uploaded CSV file
camera_data_file_path = 'data_Exp1_20s_2024-07_16_09_24_03.csv'
# Load the LDV data from the uploaded CSV file
ldv_data_file_path = 'protocol_optoNCDT-ILD1420_2024-07-16_09-24-00.721.csv'
exp = 1

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

# Filter specifications
order = 4
cutoff = 25  # Cutoff frequency in Hz

# Sampling frequency of original signal
fs = 1000

# Normalize the cutoff frequency with respect to Nyquist frequency of the original signal
normalized_cutoff = cutoff / (fs / 2)

# Design the Butterworth filter
b, a = butter(order, normalized_cutoff, btype='low', analog=False)

# Frequency response
w, h = freqz(b, a, worN=8000)
plt.plot(0.5 * fs * w / np.pi, np.abs(h), 'b')
plt.plot(cutoff, 0.5 * np.sqrt(2), 'ko')
plt.axvline(cutoff, color='k')
plt.xlim(0, 0.5 * fs)
plt.title("Butterworth Lowpass Filter Frequency Response")
plt.xlabel('Frequency [Hz]')
plt.ylabel('Gain')
plt.grid()
plt.show()
