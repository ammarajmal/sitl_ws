#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Quaternion
from influxdb_client import InfluxDBClient, Point, WritePrecision, WriteOptions
import os

# InfluxDB details - Load from environment and config
TOKEN = os.getenv('OMEN_TOKEN')
ORG = 'CAU'
BUCKET = 'SITL'
URL = 'http://localhost:8086'
USERNAME = 'ammar'
PASSWORD = 'sitl1234'

# Updated authentication setup
client = InfluxDBClient(url=URL, token=TOKEN, org=ORG, username=USERNAME, password=PASSWORD)

def callback(data):
    for transform in data.transforms:
        point = Point("Sony_Cam_Data") \
            .tag("fiducial_id", transform.fiducial_id) \
            .field("x", transform.transform.translation.x) \
            .field("y", transform.transform.translation.y) \
            .field("z", transform.transform.translation.z) \
            .field("rot_x", transform.transform.rotation.x) \
            .field("rot_y", transform.transform.rotation.y) \
            .field("rot_z", transform.transform.rotation.z) \
            .field("rot_w", transform.transform.rotation.w) \
            .field("image_error", transform.image_error) \
            .field("object_error", transform.object_error) \
            .field("fiducial_area", transform.fiducial_area)
            # .time("timestamp", data.header.stamp.secs, write_precision=WritePrecision.S)

        write_api = client.write_api(write_options=WriteOptions(batch_size=500, flush_interval=10_000, jitter_interval=2_000, retry_interval=5_000, max_retries=5, max_retry_delay=30_000, exponential_base=2))
        write_api.write(bucket=BUCKET, org=ORG, record=point)

def listener():
    rospy.init_node('influx', anonymous=True)
    rospy.Subscriber('/sony_cam1_detect/fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
