#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon May 27 11:21:00 2024
Finding the time difference between three topics
"""
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import pandas as pd
import matplotlib.pyplot as plt
import time
import numpy as np

# Initialize variables to store the latest timestamps and the DataFrame
latest_time1 = None
latest_time2 = None
latest_time3 = None
data = {'time': [], 'cam1_cam2': [], 'cam2_cam3': [], 'cam1_cam3': []}
df = pd.DataFrame(data)

# Flag to stop collecting data after 30 seconds
stop_collecting = False

def callback_cam1(msg):
    global latest_time1
    latest_time1 = msg.header.stamp
    calculate_time_differences()

def callback_cam2(msg):
    global latest_time2
    latest_time2 = msg.header.stamp
    calculate_time_differences()

def callback_cam3(msg):
    global latest_time3
    latest_time3 = msg.header.stamp
    calculate_time_differences()

def calculate_time_differences():
    if stop_collecting:
        return

    global df

    current_time = rospy.Time.now().to_sec()

    diff1_2 = None
    if latest_time1 is not None and latest_time2 is not None:
        diff1_2 = abs((latest_time1 - latest_time2).to_sec())

    diff2_3 = None
    if latest_time2 is not None and latest_time3 is not None:
        diff2_3 = abs((latest_time2 - latest_time3).to_sec())

    diff1_3 = None
    if latest_time1 is not None and latest_time3 is not None:
        diff1_3 = abs((latest_time1 - latest_time3).to_sec())

    # Append to DataFrame using pd.concat
    new_row = pd.DataFrame({'time': [current_time], 'cam1_cam2': [diff1_2], 'cam2_cam3': [diff2_3], 'cam1_cam3': [diff1_3]})
    df = pd.concat([df, new_row], ignore_index=True)

def main():
    global stop_collecting
    global df

    rospy.init_node('timestamp_difference_calculator')

    # Subscribers
    rospy.Subscriber('/sony_cam1_detect/fiducial_transforms', FiducialTransformArray, callback_cam1)
    rospy.Subscriber('/sony_cam2_detect/fiducial_transforms', FiducialTransformArray, callback_cam2)
    rospy.Subscriber('/sony_cam3_detect/fiducial_transforms', FiducialTransformArray, callback_cam3)

    # Start a timer to stop collecting data after 30 seconds
    start_time = time.time()
    rospy.loginfo("Starting to collect timestamp differences for 30 seconds...")

    while not rospy.is_shutdown() and time.time() - start_time < 30:
        rospy.sleep(0.1)

    stop_collecting = True
    rospy.loginfo("Finished collecting timestamp differences.")

    # Replace None with NaN
    df = df.replace({None: np.nan})

    # Drop rows with NaN values
    df = df.dropna()

    # Save the DataFrame to a CSV file (optional)
    df.to_csv('timestamp_differences.csv', index=False)

    # Convert DataFrame columns to numpy arrays before plotting
    time_array = df['time'].to_numpy()
    cam1_cam2_array = df['cam1_cam2'].to_numpy()
    cam2_cam3_array = df['cam2_cam3'].to_numpy()
    cam1_cam3_array = df['cam1_cam3'].to_numpy()

    # Plot the data
    plt.figure(figsize=(10, 6))
    plt.plot(time_array, cam1_cam2_array, label='cam1_cam2')
    plt.plot(time_array, cam2_cam3_array, label='cam2_cam3')
    plt.plot(time_array, cam1_cam3_array, label='cam1_cam3')
    plt.xlabel('Time (s)')
    plt.ylabel('Time Difference (s)')
    plt.title('Timestamp Differences Between Cameras')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
