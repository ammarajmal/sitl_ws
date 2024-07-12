#!/usr/bin/env python3

import rospy
import csv
import time
from geometry_msgs.msg import Pose
from fiducial_msgs.msg import FiducialTransformArray
time_dur = 5
camera_name = 'cam1'
class FiducialTransformRecorder:
    def __init__(self):
        rospy.init_node('data_saving', anonymous=True)
        self.sub = rospy.Subscriber(f'/sony_{camera_name}_detect/pose_world', Pose, self.callback)
        self.start_time = time.time()
        self.data = []

    def callback(self, msg):
        current_time = time.time()
        if current_time - self.start_time <= time_dur:
            trans_x = msg.position.x
            trans_y = msg.position.y
            trans_z = msg.position.z
            rot_x = msg.orientation.x
            rot_y = msg.orientation.y
            rot_z = msg.orientation.z
            rot_w = msg.orientation.w
            # append the time and trans and rot data to self.data
            self.data.append([current_time - self.start_time, trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w])
        else:
            rospy.signal_shutdown('Data collection complete')

    def save_to_csv(self):
        with open(f'{camera_name}_pose_w.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'position_x', 'position_y', 'position_z', 'orientation_x', 'orientation_y', 'orientation_z', 'orientation_w'])
            writer.writerows(self.data)

if __name__ == '__main__':
    recorder = FiducialTransformRecorder()
    rospy.spin()
    recorder.save_to_csv()
    print(f'Data saved to {camera_name}.csv')
