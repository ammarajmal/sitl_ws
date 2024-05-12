#!/usr/bin/env python3
import subprocess
import rospy
number_of_cameras = 3
def topic_check(this_topic):
    try:
        all_topics = rospy.get_published_topics()
        return this_topic in [topic[0] for topic in all_topics]
    except Exception as e:
        return False
def check_cameras():
    print('Checking cameras...')
    cameras = {(i, f'/sony_cam{i}/image_raw') for i in range(1, number_of_cameras+1)}
    for camera in cameras:
        # print('Checking camera', camera[0])
        if topic_check(camera[1]):
            print('Camera', camera[0], 'is available')
        else:
            print('Camera', camera[0], 'is not available')
    # for i in range(1, number_of_cameras+1):
        # print('Checking camera', i)
        
        
if __name__ == '__main__':
    # Check if the topic is available
    check_cameras()
    # if topic_check('/sony_cam3/image_raw'):
    #     print('Topic is available')
    # else:
    #     print('Topic is not available')
        
        