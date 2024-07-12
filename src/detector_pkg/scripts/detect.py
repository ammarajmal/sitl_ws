#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped

class ArucoDetector:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        # Load ROS parameters
        self.camera_name = rospy.get_param('~camera_name', 'sony_cam1')
        aruco_dict_name = rospy.get_param('~aruco_dict', 'DICT_4X4_50')
        self.aruco_marker_size = rospy.get_param('~aruco_marker_size', 0.02)
        self.experiment_name = rospy.get_param('~experiment_name', 'Exp1')

        # Set up subscribers and publishers
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        rospy.sleep(5)
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        
        # Initialize pose publishers
        self.node_name = rospy.get_name()
        self.pose_pub = rospy.Publisher(f"{self.node_name}/pose", PoseStamped, queue_size=10)
        self.pose_w_pub = rospy.Publisher(f"{self.node_name}/pose_world", PoseStamped, queue_size=10)

        # Initialize camera matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None

        # Set up ArUco dictionary and parameters
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, aruco_dict_name, aruco.DICT_4X4_50))
        self.parameters = self.setup_aruco_parameters()

        # Flags to check the reception of valid messages
        self.image_received = False
        self.camera_info_received = False

        # Register shutdown hook
        rospy.on_shutdown(self.cleanup)

    def setup_aruco_parameters(self):
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10  # Threshold constant for adaptive thresholding
        parameters.adaptiveThreshWinSizeMax = 23  # Maximum window size for adaptive thresholding
        parameters.adaptiveThreshWinSizeMin = 3  # Minimum window size for adaptive thresholding
        parameters.adaptiveThreshWinSizeStep = 10  # Step size for adaptive thresholding window
        parameters.minMarkerPerimeterRate = 0.02  # Minimum marker perimeter rate
        parameters.maxMarkerPerimeterRate = 4.0  # Maximum marker perimeter rate
        parameters.polygonalApproxAccuracyRate = 0.05  # Polygonal approximation accuracy rate
        parameters.errorCorrectionRate = 0.6  # Error correction rate
        return parameters

    def image_callback(self, data):
        if not self.camera_info_received:
            rospy.logwarn("Camera info not received yet. Skipping image processing.")
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(f"Could not convert image: {e}")
            return

        if self.image_received and self.camera_info_received:
            self.process_image(cv_image)

    def camera_info_callback(self, data):
        try:
            self.camera_matrix = np.array(data.K).reshape(3, 3)
            self.dist_coeffs = np.array(data.D)
            self.camera_info_received = True
        except Exception as e:
            rospy.logerr(f"Could not convert camera info: {e}")
            return

    def process_image(self, cv_image):
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            for corner in corners:
                cv2.cornerSubPix(gray, corner,
                                 winSize=(5, 5),
                                 zeroZone=(-1, -1),
                                 criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 30, 0.01))

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)
            for rvec, tvec in zip(rvecs, tvecs):
                R_w, T_w = self.camera_to_world_pose(rvec, tvec)
                self.publish_pose(cv_image, rvec, tvec, R_w, T_w)
            # show the image with the markers
            cv2.imshow(f"Camera {self.camera_name[-1]}", cv_image)
            cv2.waitKey(1)

    def publish_pose(self, cv_image, rvec, tvec, R_w, T_w):
        aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = self.camera_name
        pose_msg.pose.position.x = tvec[0][0]
        pose_msg.pose.position.y = tvec[0][1]
        pose_msg.pose.position.z = tvec[0][2]
        rmat, _ = cv2.Rodrigues(rvec)
        pose_msg.pose.orientation.w, pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z = self.rotation_matrix_to_quaternion(rmat)
        self.pose_pub.publish(pose_msg)

        pose_w_msg = PoseStamped()
        pose_w_msg.header.stamp = rospy.Time.now()
        pose_w_msg.header.frame_id = "world"
        pose_w_msg.pose.position.x = T_w[0][0]
        pose_w_msg.pose.position.y = T_w[1][0]
        pose_w_msg.pose.position.z = T_w[2][0]
        pose_w_msg.pose.orientation.w, pose_w_msg.pose.orientation.x, pose_w_msg.pose.orientation.y, pose_w_msg.pose.orientation.z = self.rotation_matrix_to_quaternion(R_w)
        self.pose_w_pub.publish(pose_w_msg)

    def camera_to_world_pose(self, rvec, tvec):
        # Convert rotation vector to rotation matrix
        R_c, _ = cv2.Rodrigues(rvec)
        
        # Inverse of the rotation matrix
        R_w = R_c.T
        
        # Adjust the translation vector
        T_c = np.array(tvec).reshape(-1, 1)
        T_w = -np.dot(R_w, T_c)
        
        return R_w, T_w

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
            elif (R[1, 1] > R[2, 2]):
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
    rospy.init_node("aruco_detector", anonymous=True)
    aruco_detector = ArucoDetector()
    rospy.spin()
