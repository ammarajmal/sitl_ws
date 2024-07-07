#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import yaml
# Number of successful images: 48
# Total reprojection error: 0.07610157420833856
# Load calibration data from ost.yaml
with open('calibrationdata/ost.yaml') as file:
    calibration_data = yaml.safe_load(file)

image_width = calibration_data['image_width']
image_height = calibration_data['image_height']
camera_matrix = np.array(calibration_data['camera_matrix']['data']).reshape((3, 3))
distortion_coefficients = np.array(calibration_data['distortion_coefficients']['data']).reshape((1, 5))

# Define the chessboard size
chessboard_size = (5, 6)  # Update based on your chessboard

# Define real world coordinates for points in the chessboard
objp = np.zeros((np.prod(chessboard_size), 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)

# List of calibration images
images = glob.glob('calibrationdata/*.png')

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# For debugging: count the number of images where corners are found
successful_images = 0

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    # If found, add object points, image points (after refining them)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)
        successful_images += 1

        # Draw and display the corners for debugging
        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard Corners', img)
        cv2.waitKey(100)

cv2.destroyAllWindows()

print(f"Number of successful images: {successful_images}")
if successful_images == 0:
    print("No chessboard corners were found in any of the images.")
else:
    # Perform camera calibration to get the rotation and translation vectors
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], camera_matrix, distortion_coefficients)

    # Calculate reprojection error
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    mean_error /= len(objpoints)
    print(f"Total reprojection error: {mean_error}")
