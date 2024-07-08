#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Pose

class ArucoDetector:
    def __init__(self) -> None:
        self.bridge = CvBridge()
        
        # Load ROS parameters
        self.camera_name = rospy.get_param('camera_name', 'sony_cam1')
        aruco_dict_name = rospy.get_param('aruco_dict', 'DICT_4X4_50')
        self.aruco_marker_size = rospy.get_param('aruco_marker_size', 0.02)
        self.experiment_name = rospy.get_param('experiment_name', 'Exp1')
        
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)

        # Initialize pose publisher
        self.node_name = rospy.get_name()
        self.pose_pub = rospy.Publisher(f"{self.node_name}/pose", Pose, queue_size=10)
        
        self.camera_matrix = None
        self.dist_coeffs = None
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, aruco_dict_name, aruco.DICT_4X4_50))
        self.parameters = self.setup_aruco_parameters()
        
        # Register shutdown hook
        rospy.on_shutdown(self.cleanup)

    def setup_aruco_parameters(self):
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10
        parameters.adaptiveThreshWinSizeMax = 23
        parameters.adaptiveThreshWinSizeMin = 3
        parameters.adaptiveThreshWinSizeStep = 10
        parameters.minMarkerPerimeterRate = 0.02
        parameters.maxMarkerPerimeterRate = 4.0
        parameters.polygonalApproxAccuracyRate = 0.05
        parameters.errorCorrectionRate = 0.6
        return parameters

    def image_callback(self, data):
        if self.camera_matrix is None or self.dist_coeffs is None:
            rospy.logwarn("Camera matrix or distortion coefficients are not available yet.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")
            return

        # Apply Gaussian blur to reduce noise
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # gray = cv2.GaussianBlur(gray, (5, 5), 0)
        
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            # Refine corners for sub-pixel accuracy
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(5, 5),
                                 zeroZone=(-1, -1),
                                 criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01))  # Increased accuracy

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
            for rvec, tvec in zip(rvecs, tvecs):
                self.publish_pose(cv_image, rvec, tvec)

        cv_image_resized = cv2.resize(cv_image, (960, 540))
        cv2.imshow("Image window", cv_image_resized)
        cv2.waitKey(3)

    def publish_pose(self, cv_image, rvec, tvec):
        aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        pose_msg = Pose()
        pose_msg.position.x = tvec[0][0]
        pose_msg.position.y = tvec[0][1]
        pose_msg.position.z = tvec[0][2]
        rmat, _ = cv2.Rodrigues(rvec)
        pose_msg.orientation.w, pose_msg.orientation.x, pose_msg.orientation.y, pose_msg.orientation.z = self.rotation_matrix_to_quaternion(rmat)
        self.pose_pub.publish(pose_msg)

    def camera_info_callback(self, data):
        self.camera_matrix = np.array(data.K).reshape(3, 3)
        self.dist_coeffs = np.array(data.D)

    def rotation_matrix_to_quaternion(self, R):
        q = np.empty((4,))
        t = np.trace(R)
        if t > 0:
            S = np.sqrt(t + 1.0) * 2
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

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("aruco_detector")
    aruco_detector = ArucoDetector()
    rospy.spin()
