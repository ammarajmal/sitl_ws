#!/usr/bin/env python3

import cv2
import cv2.aruco as aruco
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from fiducial_msgs.msg import FiducialTransform, FiducialTransformArray

class ArucoDetector:
    def __init__(self) -> None:
        self.bridge = CvBridge()

        # Load ROS parameters
        self.camera_name = rospy.get_param('~camera_name', 'sony_cam1')
        self.aruco_dict_name = rospy.get_param('~aruco_dict', 'DICT_4X4_50')
        self.aruco_marker_size = rospy.get_param('~aruco_marker_size', 0.02)
        self.experiment_name = rospy.get_param('~experiment_name', 'Exp1')

        # Set up subscribers and publishers
        self.camera_info_sub = rospy.Subscriber(f"/{self.camera_name}/camera_info", CameraInfo, self.camera_info_callback)
        rospy.sleep(0.5)
        self.image_sub = rospy.Subscriber(f"/{self.camera_name}/image_raw", Image, self.image_callback)
        
        # Initialize pose publisher
        self.node_name = rospy.get_name()
        self.transform_pub = rospy.Publisher(f"{self.node_name}/fiducial_transforms", FiducialTransformArray, queue_size=10)

        # Initialize camera matrix and distortion coefficients
        self.camera_matrix = None
        self.dist_coeffs = None

        # Set up ArUco dictionary and parameters
        self.aruco_dict = aruco.Dictionary_get(getattr(aruco, self.aruco_dict_name, aruco.DICT_4X4_50))
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
                                winSize=(10, 10),
                                zeroZone=(-1, -1),
                                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 0.001))

            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.aruco_marker_size, self.camera_matrix, self.dist_coeffs)

            transform_array_msg = FiducialTransformArray()
            transform_array_msg.header.stamp = rospy.Time.now()
            transform_array_msg.header.frame_id = self.camera_name

            for rvec, tvec, corner, fid in zip(rvecs, tvecs, corners, ids):
                transform = FiducialTransform()
                transform.fiducial_id = int(fid)

                transform.transform.translation.x = tvec[0][0]
                transform.transform.translation.y = tvec[0][1]
                transform.transform.translation.z = tvec[0][2]

                rmat, _ = cv2.Rodrigues(rvec)
                qw, qx, qy, qz = self.rotation_matrix_to_quaternion(rmat)
                transform.transform.rotation.x = qx
                transform.transform.rotation.y = qy
                transform.transform.rotation.z = qz
                transform.transform.rotation.w = qw

                image_error = self.calculate_reprojection_error(rvec, tvec, corner)
                transform.image_error = image_error

                fiducial_area = cv2.contourArea(corner)
                transform.fiducial_area = fiducial_area

                if len(corner) == 4:  # Check if there are 4 corners detected
                    transform.object_error = (image_error / cv2.norm(corner[0] - corner[2])) * (np.linalg.norm(tvec) / self.aruco_marker_size)
                else:
                    transform.object_error = -1

                transform_array_msg.transforms.append(transform)

                # Draw the detected marker and the axis for visualization
                aruco.drawDetectedMarkers(cv_image, corners)
                aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.1)

            self.transform_pub.publish(transform_array_msg)

            # Show the image with detected markers
            cv2.imshow(f"Image window {self.camera_name}", cv_image)
            cv2.waitKey(3)

    def calculate_reprojection_error(self, rvec, tvec, corner):
        marker_points = np.array([
            [-self.aruco_marker_size/2, self.aruco_marker_size/2, 0],
            [self.aruco_marker_size/2, self.aruco_marker_size/2, 0],
            [self.aruco_marker_size/2, -self.aruco_marker_size/2, 0],
            [-self.aruco_marker_size/2, -self.aruco_marker_size/2, 0]
        ])

        projected_points, _ = cv2.projectPoints(marker_points, rvec, tvec, self.camera_matrix, self.dist_coeffs)
        corner = np.squeeze(corner).astype(np.float32)
        projected_points = np.squeeze(projected_points).astype(np.float32)
        error = cv2.norm(corner, projected_points, cv2.NORM_L2) / len(projected_points)

        return error

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
        return q

    def cleanup(self):
        cv2.destroyAllWindows()

if __name__ == "__main__":
    rospy.init_node("aruco_detector", anonymous=True)
    aruco_detector = ArucoDetector()
    rospy.spin()
