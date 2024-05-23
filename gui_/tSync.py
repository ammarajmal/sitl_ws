#!/usr/bin/env python

import rospy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from fiducial_msgs.msg import FiducialTransformArray

def callback(cam1, cam2, cam3):
    sync_end_time = rospy.get_time()
    rospy.loginfo("Received synchronized messages from all cameras")
    
    # Log the timestamps of the messages after synchronization
    rospy.loginfo("After synchronization - Timestamps: cam1: {}, cam2: {}, cam3: {}".format(
        cam1.header.stamp.to_sec(),
        cam2.header.stamp.to_sec(),
        cam3.header.stamp.to_sec()
    ))
    rospy.loginfo("Time after synchronization: {:.6f}".format(sync_end_time))

    # Process the messages from all three cameras here
    # if cam1.transforms:
    #     rospy.loginfo("Camera 1 Transform: {}".format(cam1.transforms[0]))
    # if cam2.transforms:
    #     rospy.loginfo("Camera 2 Transform: {}".format(cam2.transforms[0]))
    # if cam3.transforms:
    #     rospy.loginfo("Camera 3 Transform: {}".format(cam3.transforms[0]))

def main():
    rospy.init_node('fiducial_tracking', anonymous=True)
    rospy.loginfo("Node initialized")

    # Create subscribers for each camera topic
    cam1_sub = Subscriber('/sony_cam1_detect/fiducial_transforms', FiducialTransformArray)
    cam2_sub = Subscriber('/sony_cam2_detect/fiducial_transforms', FiducialTransformArray)
    cam3_sub = Subscriber('/sony_cam3_detect/fiducial_transforms', FiducialTransformArray)

    # Set up the approximate time synchronizer
    ats = ApproximateTimeSynchronizer([cam1_sub, cam2_sub, cam3_sub], queue_size=10, slop=0.4)
    rospy.loginfo("ApproximateTimeSynchronizer initialized with slop of 0.4 seconds")

    # Register a callback that also logs the message timestamps before synchronization
    def pre_sync_callback(msg1, msg2, msg3):
        pre_sync_time = rospy.get_time()
        rospy.loginfo("Before synchronization - Timestamps: cam1: {}, cam2: {}, cam3: {}".format(
            msg1.header.stamp.to_sec(),
            msg2.header.stamp.to_sec(),
            msg3.header.stamp.to_sec()
        ))
        rospy.loginfo("Time before synchronization: {:.6f}".format(pre_sync_time))
        callback(msg1, msg2, msg3)
    
    ats.registerCallback(pre_sync_callback)
    rospy.loginfo("Callback registered")

    rospy.spin()
    rospy.loginfo("ROS spin started")

if __name__ == '__main__':
    main()
