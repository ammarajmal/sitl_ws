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

message_counts = {}


def initialize_camera_data(camera_names):
    for camera_name in camera_names:
        camera_data[camera_name] = {
            'first_translation_x': None,
            'first_translation_y': None,
            'first_translation_z': None
        }
        message_counts[camera_name] = {
            'count': 0,
            'start_time': None,
            'end_time': None
        }


def calculate_frequency(camera_name):
    count_data = message_counts[camera_name]
    duration = (count_data['end_time'] - count_data['start_time']).to_sec()
    if duration > 0:
        frequency = count_data['count'] / duration
    else:
        frequency = 0
    print(f"Frequency for {camera_name}: {frequency} Hz")
    return frequency


def camera_callback(start_time, end_time, camera_name, data):
    if camera_name not in camera_data:
        rospy.logwarn(f"Received data for unknown camera: {camera_name}")
        return
    first_data = camera_data[camera_name]

    count_data = message_counts[camera_name]
    if count_data['start_time'] is None:
        count_data['start_time'] = rospy.Time.now()
    count_data['count'] += 1
    count_data['end_time'] = rospy.Time.now()

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
        .time(rospy.Time.now().to_sec())
    # print(f"Writing data to InfluxDB: {point.to_line_protocol()}")
    write_api.write(bucket="TEST_BUCKET", record=point)


def save_influxdb_data_to_csv(start_time, end_time, experiment_name, camera_names):
    print(f'Saving data to CSV for experiment {experiment_name}.')
    start_time_ns = start_time.to_nsec()
    end_time_ns = end_time.to_nsec()
    # Construct the Flux Query
    query = f'''
    from(bucket: "TEST_BUCKET")
        |> range(start: {start_time_ns}, stop: {end_time_ns})
        |> filter(fn: (r) => r["_measurement"] == "ind_fiducial_transforms")
        |> filter(fn: (r) => {" or ".join([f'r["camera"] == "{camera}"' for camera in camera_names])})
        |> filter(fn: (r) => r["experiment"] == "{experiment_name}")
        |> pivot(rowKey: ["_time", "camera"], columnKey: ["_field"], valueColumn: "_value")
    '''
    print(query)

    # Execute the Query
    try:
        tables = query_api.query(org=org, query=query)
        dataframe = []
        for table in tables:
            records = []
            for record in table.records[1:]:
                records.append(record.values)  # Get the values from Flux Record
            # Determine the actual column names from the first record
            column_names = list(records[0].keys())
            df = pd.DataFrame(records, columns=column_names)
            dataframe.append(df)
        combined_df = pd.concat(dataframe, ignore_index=True)
        # Display the Dataframe for validation
        print(combined_df.head())

        # Save to CSV
        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"fiducial_transforms_{experiment_name}_{timestamp_str}.csv"  # Enhanced filename
        combined_df.to_csv(csv_filename, index=False)  # Save without row indices
        print(f"Data saved to {csv_filename}")
        
        # Check the frequency of saved messages
        check_influxdb_frequency(combined_df, camera_names)

    except Exception as e:
        print(f"Query failed: {e}")
        print("No data returned from the query.")
        return
    finally:
        print("function completed.")


def check_influxdb_frequency(df, camera_names):
    for camera_name in camera_names:
        camera_df = df[df['camera'] == camera_name]
        if not camera_df.empty:
            time_diffs = camera_df['_time'].diff().dropna().apply(pd.to_timedelta)
            average_interval = time_diffs.mean().total_seconds()
            if average_interval > 0:
                frequency = 1 / average_interval
            else:
                frequency = 0
            print(f"Frequency of saved messages for {camera_name} in InfluxDB: {frequency} Hz")
        else:
            print(f"No data found in InfluxDB for camera: {camera_name}")


def subscribe_to_camera_topics(camera_topics, duration):
    global start_time_, end_time_
    start_time = rospy.Time.now()
    end_time = start_time + duration
    # print(datetime.fromtimestamp(start_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))
    # print(datetime.fromtimestamp(end_time.to_sec()).strftime('%Y-%m-%d %H:%M:%S'))

    tim = (start_time, end_time)
    for camera_name, topic in camera_topics.items():
        print(f"Subscribing to {topic} for {camera_name}")
        callback_with_params = partial(camera_callback, start_time, end_time, camera_name)
        rospy.Subscriber(topic, FiducialTransformArray, callback=callback_with_params)
    while rospy.Time.now() < end_time and not rospy.is_shutdown():
        rospy.sleep(1)
    print("finally up.!")
    return tim


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
    # Calculate and display the frequency of incoming messages
    for camera_name in camera_topics.keys():
        calculate_frequency(camera_name)
    # Save data and calculate the frequency of saved messages
    save_influxdb_data_to_csv(st, en, experiment_name, camera_names=list(camera_topics.keys()))
    print("Time's up!")


if __name__ == "__main__":
    main_loop()
    print('After the main loop')
