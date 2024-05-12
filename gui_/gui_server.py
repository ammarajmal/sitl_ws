import tkinter as tk
import customtkinter
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Quaternion
from influxdb_client import InfluxDBClient, Point, WritePrecision, WriteOptions
import subprocess
import os
import re
themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red': ("#fa5f5a", "#ba3732")
          }
COLOR_SELECT = list(themes.keys())[1]
customtkinter.set_appearance_mode("System")
customtkinter.set_default_color_theme(COLOR_SELECT)


# InfluxDB details
TOKEN = os.getenv('')
ORG = 'Chung-Ang University'
BUCKET = 'SITL'
URL = 'http://localhost:8086'
USERNAME = 'tesol'
PASSWORD = '00000000'
# Updated authentication setup
client = InfluxDBClient(url=URL, token=TOKEN, org=ORG, username=USERNAME, password=PASSWORD)


class ServerGUI(customtkinter.CTk):
    def __init__(self, master=None):
        super().__init__(master)
        self.title('VIDEO MEASUREMENT SYSTEM')
        self.geometry('740x400')
        self.resizable(False, False)
        self.number_of_cameras = 3
        self.csv_check = tk.StringVar(value="on")
        self.camera_status = {}
        for i in range(1, self.number_of_cameras+1):
            self.camera_status[f'Camera {i}'] = {
                'status': '-',
                'topic': f'/sony_cam{i}/image_raw',
                'cam_fps': 0,
                'detect_topic': f'/sony_cam{i}_detect/fiducial_transforms',
                'detect_status': '-',
                'detect_fps': 0
            }
        self.create_widgets()
        
        self.init_ros_node()
        
    def init_ros_node(self):
        print('Initializing ROS Node')
        
    def create_widgets(self):
        self.create_title_frame()
        self.create_left_frame()
        self.create_middle_frame()
        self.create_right_frame()
        self.exit_button = customtkinter.CTkButton(
            self, text='Exit', command=self.quit, fg_color=themes['red'][1], font=('calibri', 14))
        self.exit_button.place(relx=0.34, rely=0.9, relwidth=0.32)
        
    def create_title_frame(self):
        gap = 0.01
        frame_width = 1-2*gap
        frame_height = 0.1
        self.title_frame = customtkinter.CTkFrame(self, border_width=1)
        self.title_frame.place(relx=gap, rely=gap, relwidth=frame_width, relheight=frame_height)
        self.title_frame_widgets()
        
    def title_frame_widgets(self):
        self.title_frame_label = customtkinter.CTkLabel(
            self.title_frame, text='DATA INGESTION DASHBOARD', font=('calibri', 14))
        self.title_frame_label.place(relx=0.35, rely=0.2)
        
    def create_left_frame(self):
        gap = 0.01
        frame_width = (1/3)-(1.35*gap)
        frame_height = 0.8-5*gap
        self.left_frame = customtkinter.CTkFrame(self, border_width=1)
        self.left_frame.place(relx=gap, rely=0.13, relwidth=frame_width, relheight=frame_height)
        self.left_frame_widgets()
        
    def left_frame_widgets(self)-> None:
        self.create_frame_label = customtkinter.CTkLabel(
            self.left_frame, text='DATA INGESTION', font=('calibri', 14))
        self.camera_check = customtkinter.CTkButton(
            self.left_frame,
            text='Check Camera',
            command=self.check_camera,
            fg_color=themes['blue'])
        self.detection_check = customtkinter.CTkButton(
            self.left_frame,
            text='Check Detection',
            command=self.check_detection,
            fg_color=themes['blue'])
        self.server_start = customtkinter.CTkButton(
            self.left_frame,
            text='Start Server',
            command=self.start_server,
            fg_color=themes['red'][1])
        self.server_stop = customtkinter.CTkButton(
            self.left_frame,
            text='Stop Server',
            command=self.stop_server,
            fg_color=themes['red'][1])
        self.create_frame_label.place(relx=0.5, rely=0.08, anchor='center')
        self.camera_check.place(relx=0.5, rely=0.25, anchor='center')
        self.detection_check.place(relx=0.5, rely=0.4, anchor='center')
        self.server_start.place(relx=0.5, rely=0.6, anchor='center')
        self.server_stop.place(relx=0.5, rely=0.75, anchor='center')
        
    def create_middle_frame(self):
        gap = 0.01
        frame_width = (1/3)-(1.35*gap)
        frame_height = 0.8-5*gap
        self.middle_frame = customtkinter.CTkFrame(self, border_width=1)
        self.middle_frame.place(relx=(2*gap)+frame_width, rely=0.13, relwidth=frame_width, relheight=frame_height)
        self.middle_frame_widgets()
        
    def middle_frame_widgets(self):
        self.middle_frame_label = customtkinter.CTkLabel(
            self.middle_frame, text='EXPERIMENT SETUP', font=('calibri', 14))
        self.middle_frame_exp_label = customtkinter.CTkLabel(
            self.middle_frame, text='Experiment Name', font=('Arial', 13))
        self.middle_frame_exp_entry = customtkinter.CTkEntry(self.middle_frame)
        self.middle_frame_exp_duration_label = customtkinter.CTkLabel(
            self.middle_frame, text='Experiment Duration', font=('Arial', 13))
        self.middle_frame_exp_duration_entry = customtkinter.CTkEntry(self.middle_frame)
        self.middle_frame_exp_duration_entry.insert(0, '10')
        self.middle_frame_csv_option_checkbox = customtkinter.CTkCheckBox(
            self.middle_frame,
            text='Save Data to CSV',
            command=self.csv_checkbox_event,
            variable=self.csv_check,
            onvalue="on",
            offvalue="off",
            checkbox_height=18, checkbox_width=18)
        self.middle_frame_start_data_recording = customtkinter.CTkButton(
            self.middle_frame,
            text='Start Data Recording',
            command=self.start_data_recording,
            fg_color=themes['red'][1])
        self.middle_frame_stop_data_recording = customtkinter.CTkButton(
            self.middle_frame,
            text='Stop Data Recording',
            command=self.stop_data_recording,
            fg_color=themes['red'][1])
        
        self.middle_frame_label.place(relx=0.5, rely=0.08, anchor='center')
        self.middle_frame_exp_label.place(relx=0.08, rely=0.2)
        self.middle_frame_exp_entry.place(relx=0.65, rely=0.2, relwidth=0.3)
        self.middle_frame_exp_duration_label.place(relx=0.08, rely=0.31)
        self.middle_frame_exp_duration_entry.place(relx=0.65, rely=0.31, relwidth=0.3)
        self.middle_frame_csv_option_checkbox.place(relx=0.25, rely=0.43)
        self.middle_frame_start_data_recording.place(relx=0.5, rely=0.6, anchor='center')
        self.middle_frame_stop_data_recording.place(relx=0.5, rely=0.75, anchor='center')
        
    def csv_checkbox_event(self):
        print('checkbox toggled, current value:', self.csv_check.get())
        
    def start_data_recording(self):
        print('Starting Data Recording')
        
    def stop_data_recording(self):
        print('Stopping Data Recording')
        
    def create_right_frame(self):
        gap = 0.01
        frame_width = (1/3)-(1.35*gap)
        frame_height = 0.8-5*gap
        self.right_frame = customtkinter.CTkFrame(self, border_width=1)
        self.right_frame.place(relx=(3*gap)+(2*frame_width), rely=0.13, relwidth=frame_width, relheight=frame_height)
        self.create_right_frame_widgets()
        
    def create_right_frame_widgets(self):
        self.right_frame_label = customtkinter.CTkLabel(
            self.right_frame, text='SYSTEM STATUS', font=('calibri', 14))
        self.right_frame_label.place(relx=0.5, rely=0.08, anchor='center')
        
        self.camera_status_labels = {}
        self.detection_status_labels = {}
        
        y_offset = 0.05
        for i in range(1, self.number_of_cameras+1):
            self.camera_status_labels[f'Camera {i}'] = customtkinter.CTkLabel(
                self.right_frame,
                text=f'Camera {i}: {self.camera_status[f"Camera {i}"]["status"]}',
                font=('calibri', 12))
            self.detection_status_labels[f'Camera {i}'] = customtkinter.CTkLabel(
                self.right_frame,
                text=f'Detection {i}: {self.camera_status[f"Camera {i}"]["detect_status"]}',
                font=('calibri', 12))
            self.camera_status_labels[f'Camera {i}'].place(relx=0.07, rely=y_offset + i * 0.1)
            self.detection_status_labels[f'Camera {i}'].place(relx=0.07, rely=y_offset + (i+3) * 0.1)
            self.database_status_label = customtkinter.CTkLabel(
                self.right_frame,
                text='Database Status: Not Connected',
                font=('calibri', 12))
            self.database_status_label.place(relx=0.5, rely=0.8, anchor='center')

    def start_server(self):
        rospy.init_node('influx', anonymous=True)
        for i in range(1, 4):
            topic = f'/sony_cam{i}/image_raw'
            if rospy.get_published_topics()[0]:
                self.camera_status[f'Camera {i}'] = "Running"
            else:
                self.camera_status[f'Camera {i}'] = "Idle"
        self.update_status_labels()
    def stop_server(self):
        rospy.signal_shutdown('Server stopped')
        for key in self.camera_status:
            self.camera_status[key] = "Idle"
        for key in self.detection_status:
            self.detection_status[key] = "Idle"
        self.update_status_labels()
    def get_topic_frequency(self, topic):
        proc = subprocess.Popen(['rostopic', 'hz', topic],
                                stdout=subprocess.PIPE,
                                stderr=subprocess.STDOUT)
        rospy.sleep(0.5)  # wait a bit to get some output
        proc.terminate()
        try:
            output, _ = proc.communicate(timeout=2)
            output = output.decode('utf-8')
        except subprocess.TimeoutExpired:
            proc.kill()
            output, _ = proc.communicate()
        # extract frequency from output using regular expression
        match = re.search(r'average rate: (.*)', output)
        # match = re.search(r'average rate: ([\d\.]+)', output)
        if match:
            # return float with 0 decimal places by truncating
            return float(match.group(1)).__trunc__()
            # return float(match.group(1))
        else:
            return 0

    def topic_check(self, this_topic):
        try:
            all_topics = rospy.get_published_topics()
            return this_topic in [topic[0] for topic in all_topics]
        except Exception as e:
            return False
    def check_camera(self):
        for i in range(1, self.number_of_cameras + 1):
            camera = self.camera_status[f'Camera {i}']
            if self.topic_check(camera['topic']):
                camera['status'] = "Running"
                camera['cam_fps'] = self.get_topic_frequency(camera['topic'])
            else:
                camera['status'] = "Idle"
                camera['cam_fps'] = 0
        self.update_status_labels()

    def check_detection(self):
        for i in range(1, self.number_of_cameras + 1):
            camera = self.camera_status[f'Camera {i}']
            if self.topic_check(camera['detect_topic']):
                camera['detect_status'] = "Running"
                camera['detect_fps'] = self.get_topic_frequency(camera['detect_topic'])
            else:
                camera['detect_status'] = "Idle"
                camera['detect_fps'] = 0
        self.update_status_labels()

    def update_status_labels(self):
        for camera, info in self.camera_status.items():
            self.camera_status_labels[camera].configure(
                text=f'{camera}: {info["status"]} at {info["cam_fps"]} Hz')
            self.detection_status_labels[camera].configure(
                text=f'Detection {camera.split()[1]}: {info["detect_status"]} at {info["detect_fps"]} Hz')

        
    def callback(self, data):
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
    

if __name__ == "__main__":
    root = ServerGUI()
    root.mainloop()
    print('Done.!')

