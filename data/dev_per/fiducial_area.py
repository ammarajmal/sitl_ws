#!/usr/bin/env python3
import numpy as np
import cv2
import glob
import yaml
# Given values
marker_length = 0.02  # in meters
distance_to_marker = 1.0263737267020026  # from the z translation component

# Load calibration data from ost.yaml
with open('calibrationdata/ost.yaml') as file:
    calibration_data = yaml.safe_load(file)
camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape((3, 3))
print(f"Camera matrix:\n{camera_matrix}")
# Camera matrix (example values)
# camera_matrix = np.array([[1778.24048, 0, 363.58088],
#                           [0, 2369.28224, 213.32711],
#                           [0, 0, 1]])

# Extract focal length (assuming fx and fy are similar, taking average)
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]
focal_length = (fx + fy) / 2

# Calculate the projected size of the marker side in the image
s_image = (marker_length * focal_length) / distance_to_marker

# Expected area in the image
expected_area = s_image ** 2
print(f"Expected fiducial area in image: {expected_area:.2f} pixels^2")

# Detected fiducial area from the provided output
detected_area = 1646.2274196650842
print(f"Detected fiducial area in image: {detected_area:.2f} pixels^2")

# Calculate percentage error
percentage_error = abs((detected_area - expected_area) / expected_area) * 100
print(f"Percentage error: {percentage_error:.2f}%")

# Determine accuracy
tolerance = 10  # assuming 10% tolerance for accuracy
if percentage_error <= tolerance:
    print("Fiducial area is accurate within tolerance.")
else:
    print("Fiducial area is not accurate within tolerance.")
