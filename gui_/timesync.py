#!/usr/bin/env python3
import rospy
import message_filters
from fiducial_msgs.msg import FiducialTransformArray

def sync_topics():
    c_name = 'Sony'
    rospy.init_node('timesync', anonymous=False)
    rospy.loginfo("Starting timesync node")
    camera_topics = {
        f"{c_name}1": "/sony_cam1_detect/fiducial_transforms",
        f"{c_name}2": "/sony_cam2_detect/fiducial_transforms",
        f"{c_name}3": "/sony_cam3_detect/fiducial_transforms"
    }
    camera_topics_subs = {}
    for camera_name, topic in camera_topics.items():
        print(f"Subscribing to {topic} for {camera_name}")
        # rospy.Subscriber(topic, FiducialTransformArray, fiducial_callback)
        camera_topics_subs[camera_name] = message_filters.Subscriber(topic, FiducialTransformArray)
    ats = message_filters.ApproximateTimeSynchronizer(list(camera_topics_subs.values()), queue_size=10, slop=0.01) 
    print("Registering callback")
    ats.registerCallback(fiducial_callback)
    rospy.spin()
def fiducial_callback(*args):
    rospy.loginfo("Callback called with %d args", len(args))  # Add logging here
    for arg in args:
        rospy.loginfo(arg.header.stamp)                    # Log timestamps
        # ... rest of your callback logic ...
    # rospy.loginfo("Callback called with %d args", len(args))  # Add logging here
    # for arg in args:
    #     rospy.loginfo(arg.header.stamp)                    # Log timestamps
    
if __name__ == '__main__':
    sync_topics()
    