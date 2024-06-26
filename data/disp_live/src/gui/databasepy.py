#!/usr/bin/env python

import rospy
import os
import time
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from fiducial_msgs.msg import FiducialTransformArray

# InfluxDB 2.0 setup
token = os.environ.get("INFLUXDB_TOKEN")
org = "Chung-Ang University"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

def callback_nuc1(data):
    process_data(data, 'nuc1')

def callback_nuc2(data):
    process_data(data, 'nuc2')

def callback_nuc3(data):
    process_data(data, 'nuc3')

def process_data(data, nuc_name):
    for transform in data.transforms:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        save_to_influxdb(nuc_name, translation, rotation)

def save_to_influxdb(nuc_name, translation, rotation):
    point = Point("fiducial_transforms") \
        .tag("nuc", nuc_name) \
        .field("translation_x", translation.x) \
        .field("translation_y", translation.y) \
        .field("translation_z", translation.z) \
        .field("rotation_x", rotation.x) \
        .field("rotation_y", rotation.y) \
        .field("rotation_z", rotation.z) \
        .field("rotation_w", rotation.w) \
        .time(time.time_ns(), WritePrecision.NS)
    write_api.write(bucket="SITL", record=point)

if __name__ == '__main__':
    rospy.init_node('data_processor', anonymous=True)
    rospy.Subscriber("/nuc1/fiducial_transforms", FiducialTransformArray, callback_nuc1)
    rospy.Subscriber("/nuc2/fiducial_transforms", FiducialTransformArray, callback_nuc2)
    rospy.Subscriber("/nuc3/fiducial_transforms", FiducialTransformArray, callback_nuc3)
    try:
        rospy.spin()
    finally:
        write_api.close()
        write_client.close()
