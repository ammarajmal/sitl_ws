#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image

def callback(data):
    # Image resolution is in data.height and data.width
    rospy.loginfo("Received an image of resolution: %d x %d", data.width, data.height)

def listener():
    rospy.init_node('image_subscriber', anonymous=True)
    rospy.Subscriber("/sony_cam1_node/image_raw", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
