import numpy as np
import scipy.signal as signal
import matplotlib.pyplot as plt

# Generate a sample signal: sine wave at 50 Hz sampled at 1000 Hz
fs = 1000  # Original sampling rate
t = np.linspace(0, 1, fs, endpoint=False)  # One second of data
freq = 7  # Frequency of the sine wave
original_signal = np.sin(2 * np.pi * freq * t)

# Step 1: Apply low-pass filter to limit the frequency content to below 30 Hz
nyquist = 60 / 2  # Nyquist frequency for the target rate
cutoff = nyquist - 10  # Cut-off frequency slightly below the Nyquist frequency
b, a = signal.butter(4, cutoff / (fs / 2), btype='low')
filtered_signal = signal.filtfilt(b, a, original_signal)

# Step 2: Downsample the signal to a rate close to 60 Hz
downsample_factor = int(fs / 60)  # Integer factor to approximate 60 Hz
downsampled_signal = filtered_signal[::downsample_factor]

# Calculate the actual new sampling rate
actual_new_fs = fs / downsample_factor

# Step 3: Resample the downsampled signal to exactly 60 Hz using resample_poly
resampled_signal = signal.resample_poly(downsampled_signal, 60, int(actual_new_fs))

# Create time arrays for plotting
t_downsampled = np.arange(len(downsampled_signal)) / actual_new_fs
t_resampled = np.arange(len(resampled_signal)) / 60

# Plotting the signals for comparison
plt.figure(figsize=(14, 10))

plt.subplot(3, 1, 1)
plt.plot(t, original_signal, label='Original Signal (1000 Hz)')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t_downsampled, downsampled_signal, label='Downsampled Signal (~{} Hz)'.format(int(actual_new_fs)))
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t_resampled, resampled_signal, label='Resampled Signal (60 Hz)')
plt.legend()

plt.tight_layout()
plt.show()
