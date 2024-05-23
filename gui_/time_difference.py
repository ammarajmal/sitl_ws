#!/usr/bin/env python3
import rospy
import message_filters
from fiducial_msgs.msg import FiducialTransformArray
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import time
import subprocess
import itertools

class TimeSyncChecker:
    def __init__(self):
        self.timer = 10 # 10 seconds
        self.camera_topics = {
            "camera 1": "/sony_cam1_detect/fiducial_transforms",
            "camera 2": "/sony_cam2_detect/fiducial_transforms",
            "camera 3": "/sony_cam3_detect/fiducial_transforms"
        }
        rospy.init_node('timesync_checker', anonymous=False)
        # Create Time Synchronizer
        self.subs = [message_filters.Subscriber(topic, FiducialTransformArray) for topic in self.camera_topics.values()]
        ats = message_filters.ApproximateTimeSynchronizer(self.subs, 10, slop=0.01)
        ats.registerCallback(self.callback)
        
        self.last_timestamp_cam1 = None
        self.last_timestamp_cam2 = None
        self.last_timestamp_cam3 = None
        

        # Create TimeSynchronizer
        sub_cam1 = message_filters.Subscriber("/sony_cam1_detect/fiducial_transforms", FiducialTransformArray)
        sub_cam2 = message_filters.Subscriber("/sony_cam2_detect/fiducial_transforms", FiducialTransformArray)
        sub_cam3 = message_filters.Subscriber("/sony_cam3_detect/fiducial_transforms", FiducialTransformArray)
        # ts = message_filters.TimeSynchronizer([sub_cam1, sub_cam2], 10)
        ts = message_filters.ApproximateTimeSynchronizer([sub_cam1, sub_cam2], 10, slop=0.01)  # slop = 0.01 seconds (10 ms)

        ts.registerCallback(self.callback)

        self.start_time = time.time()
        self.unsynced_data = []
        self.synced_data = []

        # Plotting setup
        self.fig, self.axes = plt.subplots(4, 1, figsize=(10, 8))
        self.line1, = self.axes[0].plot([], [])
        self.line2, = self.axes[1].plot([], [])
        self.line3, = self.axes[2].plot([], [])
        self.line4, = self.axes[3].plot([], [])
        self.time_diffs = []
        self.axes[0].set_title("Unsynchronized Time Difference (Camera 1 and Camera 2)")
        self.axes[1].set_title("Synchronized Time Difference (Camera 1 and Camera 2)")
        for ax in self.axes:
            ax.set_xlabel("Time (seconds)")
            ax.set_ylabel("Time Difference (seconds)")
            ax.grid(True)

    def callback(self, msg1, msg2):
        unsynced_time_diff = abs(msg1.header.stamp - msg2.header.stamp).to_sec()
        rospy.loginfo("Unsynchronized time difference: %.6f seconds", unsynced_time_diff)
        self.unsynced_data.append({'time': time.time(), 'time_diff': unsynced_time_diff})

        synced_time_diff = 0.0
        rospy.loginfo("Synchronized time difference: %.6f seconds", synced_time_diff)
        self.synced_data.append({'time': time.time(), 'time_diff': synced_time_diff})

        # Check if self.timer seconds have passed and plot
        if time.time() - self.start_time >= self.timer:
            self.plot_and_save_data()
            rospy.signal_shutdown("Data collection and plotting complete")

    def plot_time_diff(self, time_diff):
        self.time_diffs.append(time_diff)
        self.times.append(time.time() - self.start_time)

        # ... (plotting logic remains the same)

    def plot_and_save_data(self):
        # Create DataFrames from collected data
        df_unsynced = pd.DataFrame(self.unsynced_data)
        df_synced = pd.DataFrame(self.synced_data)

        # Save data to CSV files
        df_unsynced.to_csv("unsynced_time_differences.csv", index=False)
        df_synced.to_csv("synced_time_differences.csv", index=False)

        # Shift time values to start from 0
        df_unsynced['time'] -= df_unsynced['time'].iloc[0]
        df_synced['time'] -= df_synced['time'].iloc[0]

        # Convert time differences to milliseconds
        df_unsynced['time_diff'] *= 1000
        df_synced['time_diff'] *= 1000

        # Determine shared y-axis limits
        min_time_diff = min(df_unsynced['time_diff'].min(), df_synced['time_diff'].min())
        max_time_diff = max(df_unsynced['time_diff'].max(), df_synced['time_diff'].max())


        # Plot the time differences in subplots
        self.line1.set_data(df_unsynced['time'], df_unsynced['time_diff'])
        self.line2.set_data(df_synced['time'], df_synced['time_diff'])
        for ax in self.axes:
            ax.relim()
            ax.autoscale_view()
            ax.set_xlim([0, self.timer])  # Set x-axis limits
            ax.set_ylabel("Time Difference (ms)")
                    # Set shared y-axis limits
            
        # Adjust subplot spacing
        plt.subplots_adjust(hspace=0.5)  # Adjust this value for desired spacing
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        # Save the figure after the timer duration
        plt.savefig("time_difference_plots.png")

        # Open the plot image
        subprocess.Popen(["xdg-open", "time_difference_plots.png"])

if __name__ == '__main__':
    try:
        TimeSyncChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
