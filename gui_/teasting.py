#!/usr/bin/env python3
import rospy
from fiducial_msgs.msg import FiducialTransformArray
camera_data = {}
def camera_callback(data, camera_name):
    global camera_data
    camera_data[camera_name] = data
    print(camera_data["Camera 1"].transforms[0].transform.translation.x)
    
def subscribe_to_camera_topics(camera_topics, duration):
    start_time = rospy.Time.now()
    end_time = start_time + duration
    for camera_name, topic in camera_topics.items():
      # print(camera_name, topic)
      rospy.Subscriber(topic,
                       FiducialTransformArray,
                       callback=camera_callback,
                       callback_args=camera_name)
    while rospy.Time.now() < end_time and rospy.is_shutdown() is False:
        rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node('data_processor', anonymous=False)
    camera_topics = {
      "Camera 1":"/sony_cam1_detect/fiducial_transforms",
      "Camera 2":"/sony_cam2_detect/fiducial_transforms",
      "Camera 3":"/sony_cam3_detect/fiducial_transforms"
      }
    duration = rospy.Duration(5) # 5 seconds
    subscribe_to_camera_topics(camera_topics, duration)
    print("Time's up!")
