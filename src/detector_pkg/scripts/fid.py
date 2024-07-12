#!/usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from geometry_msgs.msg import Transform
import numpy as np
import cv2

def rvec_to_quaternion(R):
    q = np.zeros(4)
    trace = np.trace(R)
    
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2
        q[3] = 0.25 * S
        q[0] = (R[2, 1] - R[1, 2]) / S
        q[1] = (R[0, 2] - R[2, 0]) / S
        q[2] = (R[1, 0] - R[0, 1]) / S
    else:
        if (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
            S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q[3] = (R[2, 1] - R[1, 2]) / S
            q[0] = 0.25 * S
            q[1] = (R[0, 1] + R[1, 0]) / S
            q[2] = (R[0, 2] + R[2, 0]) / S
        elif R[1, 1] > R[2, 2]:
            S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q[3] = (R[0, 2] - R[2, 0]) / S
            q[0] = (R[0, 1] + R[1, 0]) / S
            q[1] = 0.25 * S
            q[2] = (R[1, 2] + R[2, 1]) / S
        else:
            S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q[3] = (R[1, 0] - R[0, 1]) / S
            q[0] = (R[0, 2] + R[2, 0]) / S
            q[1] = (R[1, 2] + R[2, 1]) / S
            q[2] = 0.25 * S
    
    return q

def publish_fiducial_transforms(rvecs, tvecs):
    # Initialize the ROS node
    rospy.init_node('fiducial_transform_publisher', anonymous=True)
    
    # Create a publisher for FiducialTransformArray
    pub = rospy.Publisher('/fiducial_transforms', FiducialTransformArray, queue_size=10)
    
    # Set the publishing rate
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Create a FiducialTransformArray message
        fiducial_transform_array = FiducialTransformArray()
        
        # Populate the FiducialTransformArray with rvecs and tvecs
        for rvec, tvec in zip(rvecs, tvecs):
            fiducial_transform = FiducialTransform()
            transform = Transform()

            # Convert rvec and tvec to a Transform message
            transform.translation.x = tvec[0]
            transform.translation.y = tvec[1]
            transform.translation.z = tvec[2]
            
            # Convert rvec to rotation matrix
            rmat, _ = cv2.Rodrigues(np.array(rvec, dtype=np.float64))
            # Convert rotation matrix to quaternion
            quaternion = rvec_to_quaternion(rmat)

            transform.rotation.x = quaternion[0]
            transform.rotation.y = quaternion[1]
            transform.rotation.z = quaternion[2]
            transform.rotation.w = quaternion[3]
            
            fiducial_transform.transform = transform
            
            # Append the FiducialTransform to the array
            fiducial_transform_array.transforms.append(fiducial_transform)
        
        # Publish the FiducialTransformArray
        rospy.loginfo("Publishing FiducialTransformArray with {} transforms".format(len(fiducial_transform_array.transforms)))
        pub.publish(fiducial_transform_array)
        
        # Sleep to maintain the publishing rate
        rate.sleep()

if __name__ == '__main__':
    try:
        # Example rvecs and tvecs
        rvecs = [[0, 0, 0], [0.1, 0.2, 0.3]]
        tvecs = [[1, 2, 3], [4, 5, 6]]
        
        publish_fiducial_transforms(rvecs, tvecs)
    except rospy.ROSInterruptException:
        pass