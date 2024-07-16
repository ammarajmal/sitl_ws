#!/usr/bin/env python

import rospy
import csv
import time
from geometry_msgs.msg import TransformStamped
from fiducial_msgs.msg import FiducialTransformArray
time_dur = 60
camera_name = 'cam2'
class FiducialTransformRecorder:
    def __init__(self):
        rospy.init_node('fiducial_transform_recorder', anonymous=True)
        self.sub = rospy.Subscriber(f'/sony_{camera_name}_detect/fiducial_transforms', FiducialTransformArray, self.callback)
        self.start_time = time.time()
        self.data = []

    def callback(self, msg):
        current_time = time.time()
        if current_time - self.start_time <= time_dur:
            for transform in msg.transforms:
                translation_x = transform.transform.translation.x
                self.data.append([current_time - self.start_time, translation_x])
                print('Time: {:.2f}s, Translation X: {:.2f}'.format(current_time - self.start_time, translation_x))
        else:
            rospy.signal_shutdown('Data collection complete')

    def save_to_csv(self):
        with open(f'{camera_name}.csv', 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time (s)', 'Translation X'])
            writer.writerows(self.data)

if __name__ == '__main__':
    recorder = FiducialTransformRecorder()
    rospy.spin()
    recorder.save_to_csv()
    print(f'Data saved to {camera_name}.csv')
