#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose
import csv
import os
import time

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator

class PoseDataSaver:
    def __init__(self):
        self.type = 'x-axis'
        self.pose_data = []
        self.start_time = time.time()
        self.duration = 20  # Duration to collect data in seconds
        self.last_print_time = self.start_time  # Track the last time we printed the remaining time
        self.initial_pose = None

        rospy.init_node('pose_data_saver', anonymous=True)
        self.pose_sub = rospy.Subscriber('/aruco_detect/pose', Pose, self.pose_callback)
        # get the time and save it into the file name using time.strftime
        self.name_time = time.strftime("%Y%m%d_%H-%M-%S")
        # get current directory and save file into it.
        cur_dir = os.path.dirname(os.path.realpath(__file__))
        self.csv_file_path = os.path.expanduser(f'{cur_dir}/pose_data_{self.name_time}.csv')

    def pose_callback(self, data):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        remaining_time = self.duration - elapsed_time
        
        if current_time - self.last_print_time >= 1:
            print(f"Time left: {int(remaining_time)} seconds")
            self.last_print_time = current_time

        if remaining_time > 0:
            if self.initial_pose is None:
                self.initial_pose = (data.position.x, data.position.y, data.position.z, data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
            displacement_x = data.position.x - self.initial_pose[0]
            displacement_y = data.position.y - self.initial_pose[1]
            displacement_z = data.position.z - self.initial_pose[2]
            displacement_rot_x = data.orientation.x - self.initial_pose[3]
            displacement_rot_y = data.orientation.y - self.initial_pose[4]
            displacement_rot_z = data.orientation.z - self.initial_pose[5] 

            self.pose_data.append([
                elapsed_time,
                displacement_x,
                displacement_y,
                displacement_z,
                displacement_rot_x,
                displacement_rot_y,
                displacement_rot_z,
            ])
        else:
            self.pose_sub.unregister()
            self.save_to_csv()
            self.plot_data()
            rospy.signal_shutdown("Data collection complete")

    def save_to_csv(self):
        with open(self.csv_file_path, 'w', newline='') as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerow(['time', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z'])
            csvwriter.writerows(self.pose_data)
        rospy.loginfo(f"Data saved to {self.csv_file_path}")

    def plot_data(self):
        times = [row[0] for row in self.pose_data]
        pos_x = [row[1] for row in self.pose_data]
        pos_y = [row[2] for row in self.pose_data]
        pos_z = [row[3] for row in self.pose_data]

        plt.figure(figsize=(10, 6))
        # give the title to the plot as the experiment name
        plt.suptitle(f'Exp_{self.type}_{self.name_time}', fontsize=12)
        ax1 = plt.subplot(3, 1, 1)
        ax1.plot(times, pos_x, label='Position X')
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('X (mm)')
        ax1.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax1.set_yticklabels(['{:.2f}'.format(x * 1000) for x in ax1.get_yticks()])
        ax1.legend()

        ax2 = plt.subplot(3, 1, 2)
        ax2.plot(times, pos_y, label='Position Y')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Y (mm)')
        ax2.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax2.set_yticklabels(['{:.2f}'.format(x * 1000) for x in ax2.get_yticks()])
        ax2.legend()

        ax3 = plt.subplot(3, 1, 3)
        ax3.plot(times, pos_z, label='Position Z')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Z (mm)')
        ax3.yaxis.set_major_locator(MaxNLocator(integer=True))
        ax3.set_yticklabels(['{:.2f}'.format(x * 1000) for x in ax3.get_yticks()])
        ax3.legend()

        plt.tight_layout()
        plot_file_path = self.csv_file_path.replace('.csv', '.png')
        plt.savefig(plot_file_path)
        # plt.show()
        plt.close()
        rospy.loginfo(f"Plot saved to {plot_file_path}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    saver = PoseDataSaver()
    saver.run()
