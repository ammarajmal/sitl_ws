#!/usr/bin/env python3
import rospy
import csv
import os
import pandas as pd
import time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from influxdb_client.client.flux_table import FluxRecord
from fiducial_msgs.msg import FiducialTransformArray
from functools import partial
from datetime import datetime


dur = 5
start_time_ = None
end_time_ = None
c_name = 'Sony'
# InfluxDB 2.0 setup
token = os.environ.get("TES_TOKEN")
org = "Chung-Ang University"
url = "http://localhost:8086"
client = InfluxDBClient(url=url, token=token, org=org)
query_api = client.query_api()

write_api = client.write_api(write_options=SYNCHRONOUS)
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
    # print(f"Writing data to InfluxDB: {point.to_line_protocol()}")
    write_api.write(bucket="TEST_BUCKET", record=point)
    

def save_influxdb_data_to_csv(start_time, end_time, experiment_name, camera_name):
    print(f'Saving data to CSV for experiment {experiment_name} and camera {camera_name}.')

    # Construct the Flux Query
    query = f'''
    from(bucket: "TEST_BUCKET")
        |> range(start: -10m)  // Adjust the range as needed
        |> filter(fn: (r) => r["_measurement"] == "ind_fiducial_transforms")
        |> filter(fn: (r) => r["camera"] == "{camera_name}")
        |> filter(fn: (r) => r["experiment"] == "{experiment_name}")
        |> pivot(rowKey: ["_time", "camera"], columnKey: ["_field"], valueColumn: "_value")

    '''

    # Execute the Query
    try:
        tables = query_api.query(org=org, query=query)
    except Exception as e:
        print(f"Query failed: {e}")
        print("No data returned from the query.")
        return
    finally:
        print("Data read successfully.")
    dataframe = []
    for table in tables:
        records = []
        for record in table.records[1:]:
            records.append(record.values) # Get the values from Flux Record
        # Determine the actual column names from the first record
        column_names = list(records[0].keys())
        df = pd.DataFrame(records, columns=column_names)
        dataframe.append(df)
    combined_df = pd.concat(dataframe, ignore_index=True)
    # Dispaly the Dataframe for validation
    print(combined_df.head())
    
    # Save to CSV
    csv_filename = f"fiducial_transforms_{experiment_name}_{camera_name}_{start_time}_{end_time}.csv"  # Enhanced filename
    combined_df.to_csv(csv_filename, index=False)  # Save without row indices
    print(f"Data saved to {csv_filename}")



    print(f"Data saved to influxdb_data.csv")


def subscribe_to_camera_topics(camera_topics, duration):
    global start_time_, end_time_
    start_time = rospy.Time.now()
    end_time = start_time + duration
    start_time_ = start_time
    end_time_ = end_time
    # print(datetime.fromtimestamp(start_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))
    # print(datetime.fromtimestamp(end_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))

    tim = (start_time, end_time)
    for camera_name, topic in camera_topics.items():
        print(f"Subscribing to {topic} for {camera_name}")
        callback_with_params = partial(camera_callback, start_time, end_time, camera_name)
        rospy.Subscriber(topic,
                         FiducialTransformArray,
                         callback=callback_with_params)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        rospy.sleep(1)
    print("finally up.!")
    return tim
    # print(tim)


def main_loop():
    rospy.init_node('data_processor', anonymous=False)
    camera_topics = {
        f"{c_name}1": "/sony_cam1_detect/fiducial_transforms",
        f"{c_name}2": "/sony_cam2_detect/fiducial_transforms",
        f"{c_name}3": "/sony_cam3_detect/fiducial_transforms"
    }
    initialize_camera_data(camera_topics.keys())
    duration = rospy.Duration(dur)  # 5 seconds
    st, en = subscribe_to_camera_topics(camera_topics, duration)
    # print(datetime.fromtimestamp(st.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))
    # print(datetime.fromtimestamp(en.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))
    save_influxdb_data_to_csv(st, en, experiment_name, camera_name=f"{c_name}1")
    print("Time's up!")


if __name__ == "__main__":
    main_loop()
    print('After the main loop')

    print(datetime.fromtimestamp(start_time_.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))
    print(datetime.fromtimestamp(end_time_.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))
    
