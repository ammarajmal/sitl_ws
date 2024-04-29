#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import subprocess
import tkinter as tk
from typing import Tuple
import customtkinter
import rospy
import rospkg
import roslaunch
import _backend_

from fiducial_msgs.msg import FiducialTransformArray
from fast_cam.msg import CameraSpecs

themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red': ("#fa5f5a", "#ba3732")
          }

# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[0]

# Modes: "System" (standard), "Dark", "Light"
customtkinter.set_appearance_mode("System")
# Themes: "blue" (standard), "green", "dark-blue"
customtkinter.set_default_color_theme(COLOR_SELECT)

class NodeGUI(customtkinter.CTk):
    def __init__(self) -> None:
        super().__init__()

        self.nuc_number = '1'
        self.package = 'dslr_cam'
        self.node_name = f"sony_cam{self.nuc_number}"
        # gui settings
        self.title(f"CAMERA {self.nuc_number} Dashboard")
        self.geometry("960x500")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)
        # detection & calibration parameters
        self.board_size = "4x4"  # dimensions for calibration
        self.square_size = "0.01725"  # in meters for calibration
        self.marker_dim = "0.0200"  # in meters for ARUCO marker
        self.marker_dict = "0"  # (DICT_4X4_50)
        self.var_dictionary = tk.StringVar(self, "0")  # dict 5x5 (1000)
        # path management
        self.launch_path = rospkg.RosPack().get_path(self.package) + '/launch/'
        # self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.local_nuc_launch = f'{self.launch_path}use.launch'
        self.view_launch = f"{self.launch_path}viewcamera.launch"
        self.calib_launch = f"{self.launch_path}calib.launch"
        # self.detect_launch = f"{self.detect_launch_path}local_detect_cam.launch"

        # process management
        self.running_processes = {}

        # gui widgets initialization
        self.create_widgets()

    def create_widgets(self) -> None:
        """Starts the GUI widgets"""
        self.create_left_frame()
        # self.create_middle_frame()
        # self.create_right_frame()
    def create_left_frame(self) -> None:
        """Creates the left frame of the GUI"""
        print("Creating left frame")
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.left_frame.place(relx=0, rely=0, relwidth=0.33, relheight=1)
        self.create_left_top_frame()
        self.create_left_bottom_frame()
        self.create_left_exit_button()
    def create_left_top_frame(self) -> None:
        """Creates the top frame of the left frame"""
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(
            relx=0.10, rely=0.08, relwidth=0.8, relheight=0.38)
        self.create_left_top_frame_content()
    def create_left_top_frame_content(self) -> None:
        """Creates the content of the top frame of the left frame"""
        # Camera Start Stop and View
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text=f"START CAMERA - NUC {self.nuc_number}")
        self.left_top_frame_label.place(relx=0.5, rely=0.17, anchor="center")

        self.left_top_frame_start_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera",
            command=lambda: self._start_cam_button_event(self.nuc_number, False))
        self.left_top_frame_start_cam_button.place(
            relx=0.5, rely=0.35, anchor="center")

        self.left_top_frame_start_cam_label_number = customtkinter.CTkLabel(
            self.left_top_frame, text="①", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_top_frame_start_cam_label_number.place(
            relx=0.08, rely=0.35, anchor="center")

        self.left_top_frame_view_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="View Camera", fg_color='gray',
            command=lambda: self._view_cam_button_event(self.nuc_number))
        self.left_top_frame_view_cam_button.place(
            relx=0.5, rely=0.55, anchor="center")

        self.left_top_frame_view_cam_label_number = customtkinter.CTkLabel(
            self.left_top_frame, text="②", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_top_frame_view_cam_label_number.place(
            relx=0.08, rely=0.55, anchor="center")

        self.left_top_frame_start_view_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start & View Camera", border_width=2,
            border_color=themes['red'][0],
            command=lambda: self._start_cam_button_event(self.nuc_number, True))
        self.left_top_frame_start_view_cam_button.place(
            relx=0.5, rely=0.80, anchor="center")

    def create_left_bottom_frame(self) -> None:
        """Creates the bottom frame of the left frame"""
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(
            relx=0.10, rely=0.50, relwidth=0.8, relheight=0.32)
        # self._create_left_bottom_frame_content()
    def create_left_exit_button(self) -> None:
        """Creates the exit button of the left frame"""
        print("Creating exit button")
        """ Exit Button """
        self.left_exit_button = customtkinter.CTkButton(self.left_frame, text="Exit Program", fg_color=themes["red"], command=self.destroy_routine)
        self.left_exit_button.place(relx=0.5, rely=0.90, anchor="center")


    def _start_cam_button_event(self, nuc_number: str, view: bool) -> None:
        """Starts the camera node"""
        camera_topic = f'/{self.node_name}/image_raw'
        topic_check = self.check_active_topic(camera_topic)
        if topic_check == "True":
            print(f"Camera {nuc_number} already running, Now stopping it")
            return
        if topic_check == "False":
            print(f"Camera {nuc_number} is not running, Now starting it")
            # self.start_camera(nuc_number, view)
        else:
            print(f"{topic_check}! Could not perform action.")
        
        # if view:
        #     print(f"Starting camera {nuc_number} and viewing")
        # else:
        #     print(f"Starting camera {nuc_number}")
    def check_active_topic(self, topic_name):
        """Checks whether a topic is currently running/active or not.. """
        try:
            all_topics = rospy.get_published_topics()
            if topic_name in [topic[0] for topic in all_topics]:
                return "True"
            else:
                return "False"
        except:
            return "Error"
    
    def _view_cam_button_event(self, nuc_number: str) -> None:
        """Starts the camera view node"""
        print(f"Viewing camera {nuc_number}")



        # self.create_camera_specs()
        # self.create_camera_control()
        # self.create_camera_view()
        # self.create_camera_calib()
    def destroy_routine(self) -> None:
        """Destroys the GUI and all the running processes"""
        for process in self.running_processes.values():
            process.kill()
        self.quit()

if __name__ == "__main__":
    print("Starting Camera Dashboard")
    root = NodeGUI()
    root.mainloop()
    print("Color theme: ", COLOR_SELECT)