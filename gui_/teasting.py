#!/usr/bin/env python3
import rospy
import csv
import os
import time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from fiducial_msgs.msg import FiducialTransformArray
from functools import partial

dur = 60
# InfluxDB 2.0 setup
token = os.environ.get("TES_TOKEN")
org = "Chung-Ang University"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)
experiment_name = "test1"
camera_data = {}


def initialize_camera_data(camera_names):
    for camera_name in camera_names:
        camera_data[camera_name] = {
            'first_translation_x': None,
            'first_translation_y': None,
            'first_translation_z': None
        }


def camera_callback(start_time, end_time, camera_name, data):
    if camera_name not in camera_data:
        rospy.logwarn(f"Received data for unknown camera: {camera_name}")
        return
    first_data = camera_data[camera_name]
    # print(camera_data["Camera 1"].transforms[0].transform.translation.x)
    for transform in data.transforms:
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        if first_data['first_translation_x'] is None:
            print("First data point found for camera:", camera_name, 'at', translation.x, translation.y, translation.z)
            first_data['first_translation_x'] = translation.x
            first_data['first_translation_y'] = translation.y
            first_data['first_translation_z'] = translation.z
        normalized_translation_x = translation.x - first_data['first_translation_x']
        normalized_translation_y = translation.y - first_data['first_translation_y']
        normalized_translation_z = translation.z - first_data['first_translation_z']
        save_to_influxdb(camera_name, normalized_translation_x, normalized_translation_y, normalized_translation_z,
                         rotation, data, start_time, end_time)
        



def save_to_influxdb(camera_name, translation_x, translation_y, translation_z, rotation, data, start_time, end_time):
    point = Point("ind_fiducial_transforms") \
        .tag("camera", camera_name) \
        .tag("experiment", experiment_name) \
        .field("translation_x", translation_x) \
        .field("translation_y", translation_y) \
        .field("translation_z", translation_z) \
        .field("rotation_x", rotation.x) \
        .field("rotation_y", rotation.y) \
        .field("rotation_z", rotation.z) \
        .field("rotation_w", rotation.w) \
        .field("fiducial_id", data.transforms[0].fiducial_id) \
        .field("image_error", data.transforms[0].image_error) \
        .field("fiducial_area", data.transforms[0].fiducial_area) \
        .field("time_cam", data.header.stamp.to_nsec()) \
        .time(rospy.Time.now().to_nsec(), WritePrecision.NS)
    write_api.write(bucket="TEST_BUCKET", record=point)
    

def save_influxdb_data_to_csv(start_time, end_time):
    with open("influxdb_data.csv", mode="w") as file:
        writer = csv.writer(file, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL)
        query = f'from(bucket: "TEST_BUCKET") |> range(start: {int(start_time.to_sec())}s, stop: {int(end_time.to_sec())}s) |> pivot(rowKey:["_time"], columnKey: ["_field"], valueColumn: "_value")'
        result = write_client.query_api().query_data_frame(org=org, query=query)
        for table in result:
            for record in table.records:
                writer.writerow([record.get_field(), record.get_value(), record.get_time()])


def subscribe_to_camera_topics(camera_topics, duration):
    start_time = rospy.Time.now()
    end_time = start_time + duration
    for camera_name, topic in camera_topics.items():
        callback_with_params = partial(camera_callback, start_time, end_time, camera_name)
        rospy.Subscriber(topic,
                         FiducialTransformArray,
                         callback=callback_with_params)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        rospy.sleep(1)


def main_loop():
    rospy.init_node('data_processor', anonymous=False)
    camera_topics = {
        "Camera 1": "/sony_cam1_detect/fiducial_transforms",
        "Camera 2": "/sony_cam2_detect/fiducial_transforms",
        "Camera 3": "/sony_cam3_detect/fiducial_transforms"
    }
    initialize_camera_data(camera_topics.keys())
    duration = rospy.Duration(dur)  # 5 seconds
    subscribe_to_camera_topics(camera_topics, duration)
    print("Time's up!")


if __name__ == "__main__":
    main_loop()
