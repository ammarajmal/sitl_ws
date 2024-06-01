#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# import subprocess

import tkinter as tk
import customtkinter
import subprocess
import re
import psutil
import rospy
import rospkg
import roslaunch
# import _backend_

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

        self.package = 'dslr_cam'
        self.image_width = "640"
        self.image_height = "480"
        self.cam_resolution = f'{self.image_width}x{self.image_height}'
        self.cam_fps = "60"
        self.update_interval = 1000  # in milliseconds
        self.is_detection_active = False
        self.detection_rate_timeout = 5 # Timeout for detection rate calculation for rostopic hz
        # gui settings
        self.title(f"Displacement Measurement Dashboard")
        self.geometry("900x500")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)
        # detection & calibration parameters
        self.board_size = "6x5"  # dimensions for calibration
        self.square_size = "0.025"  # in meters for calibration
        self.marker_dim = "0.020"  # in meters for ARUCO marker
        self.marker_dict = "0"  # (DICT_4X4_50)
        self.var_dictionary = tk.StringVar(self, "0")  # dict 5x5 (1000)
        # path management
        self.launch_path = rospkg.RosPack().get_path(self.package) + '/launch/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.cam_launch = f'{self.launch_path}use.launch'
        self.view_launch = f"{self.launch_path}use_viewcamera.launch"
        self.calib_launch = f"{self.launch_path}calib.launch"
        self.detect_launch = f"{self.detect_launch_path}use_aruco.launch"

        # process management
        self.running_processes = {}

        # gui widgets initialization
        self.create_widgets()

    def create_widgets(self) -> None:
        """Starts the GUI widgets"""
        self.create_left_frame()
        self.create_middle_frame()
        self.create_right_frame()
    def create_right_frame(self) -> None:
        """Creates the right frame of the GUI"""
        self.right_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.right_frame.place(relx=0.66, rely=0, relwidth=0.34, relheight=1)
        self.create_right_top_frame()
        self.create_right_bottom_frame()
    def create_right_bottom_frame(self) -> None:
        """Creates the bottom frame of the right frame"""
        self.right_bottom_frame = customtkinter.CTkFrame(self.right_frame, fg_color=themes[COLOR_SELECT][0])
        self.right_bottom_frame.place(relx=0.06, rely=0.2, relwidth=0.8, relheight=0.66)
        self.create_right_bottom_frame_content()
    def create_right_bottom_frame_content(self) -> None:
        """ Camera Information """
        label_height = 0.13  # This determines the vertical space each label set uses
        label_color = 'white'  # Set the text color for all labels

        self.right_bottom_frame_cam_name_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="Camera Name:", text_color=label_color)
        self.right_bottom_frame_cam_name_label.place(relx=0.1, rely=0.05)
        self.right_bottom_frame_cam_name_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text=f'Sony Camera {1}', text_color=label_color)
        self.right_bottom_frame_cam_name_result_label.place(relx=0.6, rely=0.05)

        self.right_bottom_frame_camera_status_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="Camera Status:", text_color=label_color)
        self.right_bottom_frame_camera_status_label.place(relx=0.1, rely=0.05 + label_height * 1)
        self.right_bottom_frame_camera_status_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="IDLE", text_color='red')  # Keep specific status colors if needed
        self.right_bottom_frame_camera_status_result_label.place(relx=0.6, rely=0.05 + label_height * 1)

        self.right_bottom_frame_camera_resolution_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="Camera Resolution:", text_color=label_color)
        self.right_bottom_frame_camera_resolution_label.place(relx=0.1, rely=0.05 + label_height * 2)
        self.right_bottom_frame_camera_resolution_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text='-', text_color=label_color)
        self.right_bottom_frame_camera_resolution_result_label.place(relx=0.6, rely=0.05 + label_height * 2)

        self.right_bottom_frame_camera_fps_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="Camera FPS:", text_color=label_color)
        self.right_bottom_frame_camera_fps_label.place(relx=0.1, rely=0.05 + label_height * 3)
        self.right_bottom_frame_camera_fps_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color=label_color)
        self.right_bottom_frame_camera_fps_result_label.place(relx=0.6, rely=0.05 + label_height * 3)

        self.right_bottom_frame_camera_detect_status_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="Detection Status:", text_color=label_color)
        self.right_bottom_frame_camera_detect_status_label.place(relx=0.1, rely=0.05 + label_height * 4)
        self.right_bottom_frame_camera_detect_status_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="IDLE", text_color='red')
        self.right_bottom_frame_camera_detect_status_result_label.place(relx=0.6, rely=0.05 + label_height * 4)

        self.right_bottom_frame_camera_detect_rate_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="Detection Rate:", text_color=label_color)
        self.right_bottom_frame_camera_detect_rate_label.place(relx=0.1, rely=0.05 + label_height * 5)
        self.right_bottom_frame_camera_detect_rate_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color=label_color)
        self.right_bottom_frame_camera_detect_rate_result_label.place(relx=0.6, rely=0.05 + label_height * 5)

        self.right_bottom_frame_camera_detect_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="Detection Result:", text_color=label_color)
        self.right_bottom_frame_camera_detect_result_label.place(relx=0.1, rely=0.05 + label_height * 6)
        self.right_bottom_frame_camera_detect_result_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="None", text_color=label_color)
        self.right_bottom_frame_camera_detect_result_result_label.place(relx=0.6, rely=0.05 + label_height * 6)
    def create_right_top_frame(self) -> None:
        """ SYSTEM IDENTIFICATION """
        self.right_top_frame = customtkinter.CTkFrame(
            self.right_frame, fg_color=themes[COLOR_SELECT][0])
        self.right_top_frame.place(relx=0.06, rely=0.08, relwidth=0.8, relheight=0.08)
        self.create_right_top_frame_content()
    def create_right_top_frame_content(self) -> None:
        """ System Identification """
        self.right_top_frame_label = customtkinter.CTkLabel(
            self.right_top_frame, text="CAMERA INFORMATION",
            text_color='white')
        self.right_top_frame_label.place(relx=0.5, rely=0.5, anchor="center")
    def create_middle_frame(self) -> None:
        """Creates the middle frame of the GUI"""
        self.middle_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.middle_frame.place(relx=0.33, rely=0, relwidth=0.33, relheight=1)
        self.create_middle_top_frame()
        self.create_middle_center_frame()
        self.create_middle_bottom_frame()
    def create_middle_top_frame(self) -> None:
        """ Calibration & Detection Parameters Label """
        self.middle_top_frame = customtkinter.CTkFrame(self.middle_frame)
        self.middle_top_frame.place(
            relx=0.06, rely=0.08, relwidth=0.8, relheight=0.08)
        self.create_middle_top_frame_content()
    def create_middle_top_frame_content(self) -> None:
        """ Calibration & Detection Parameters """
        self.middle_top_frame_label = customtkinter.CTkLabel(
            self.middle_top_frame, text="SETTING PARAMETERS")
        self.middle_top_frame_label.place(relx=0.5, rely=0.5, anchor="center")
    def create_middle_center_frame(self) -> None:
        """Creates the center frame of the middle frame"""
        self.middle_center_frame = customtkinter.CTkFrame(self.middle_frame)
        self.middle_center_frame.place(
            relx=0.06, rely=0.20, relwidth=0.8, relheight=0.30)
        self.create_middle_center_frame_content()
    def create_middle_center_frame_content(self) -> None:
        """ Setting the Calibration Parameters """
        self.middle_center_frame_label = customtkinter.CTkLabel(
            self.middle_center_frame, text=f"CAMERA CALIBRATION")
        self.middle_center_frame_label.place(relx=0.5, rely=0.13, anchor="center")
        self.middle_center_frame_square_size_label = customtkinter.CTkLabel(
            self.middle_center_frame, text="Square Size: (m)")
        self.middle_center_frame_square_size_label.place(relx=0.1, rely=0.22)
        self.middle_center_frame_square_size_entry = customtkinter.CTkEntry(
            self.middle_center_frame,
            placeholder_text=self.square_size,
            placeholder_text_color="gray")
        self.middle_center_frame_square_size_entry.place(
            relx=0.62, rely=0.22, relwidth=0.25)
        self.middle_center_frame_chessboard_label = customtkinter.CTkLabel(
            self.middle_center_frame, text="Chessboard Size: (m)")
        self.middle_center_frame_chessboard_label.place(
            relx=0.1, rely=0.42)
        self.middle_center_frame_chessboard_entry = customtkinter.CTkEntry(
            self.middle_center_frame,
            placeholder_text=self.board_size,
            placeholder_text_color="gray")
        self.middle_center_frame_chessboard_entry.place(
            relx=0.62, rely=0.42, relwidth=0.25)
        self.middle_center_frame_calib_update_button = customtkinter.CTkButton(
            self.middle_center_frame, text='Update',
            command=lambda: self._update_calib_params())
        self.middle_center_frame_calib_update_button.place(
            relx=0.5, rely=0.75, relwidth=0.4, anchor="center")
        self.middle_center_frame_calib_update_label = customtkinter.CTkLabel(
            self.middle_center_frame, text="", text_color='green',
            font=customtkinter.CTkFont(size=20, weight="bold"))
        self.middle_center_frame_calib_update_label.place(
            relx=0.8, rely=0.75, anchor="center")
    def _update_calib_params(self) -> None:
        """ Updates the calibration parameters """
        square_size_entry = self.middle_center_frame_square_size_entry.get()
        chessboard_size_entry = self.middle_center_frame_chessboard_entry.get()
        if not chessboard_size_entry and not square_size_entry:
            print("Nothing updated")
            print(f'Original square size: {self.square_size}')
            print(f'Original board size: {self.board_size}')
            print('Please enter new calibration parameters')
            return
        if square_size_entry:
            self.square_size = square_size_entry
            print(f'Updated square size: {self.square_size}')
        else:
            print(f'Original square size: {self.square_size}')
        if chessboard_size_entry:
            self.board_size = chessboard_size_entry
            print(f'Updated board size: {self.board_size}')
        else:
            print(f'Original board size: {self.board_size}')
        print('Checkerboard parameters updated successfully')
        self.middle_center_frame_calib_update_label.configure(
            text="☑", text_color='green')
    def _start_camera_calibration(self,nuc_number) -> None:
        """ Starts the camera calibration """
        if self.running_processes.get(f'sony_cam{nuc_number}_calib_driver') is not None:    # Calib node running
            rospy.loginfo("Calibration already running, Now stopping it")
            if self.running_processes.get(f'sony_cam{nuc_number}_cam_driver') is not None:  # Camera running
                try:
                    self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_cam_driver')
                except KeyError:
                    print("Error: Could not stop camera.")
                else:
                    self.left_top_frame_start_cam1_button.configure(
                        text="Start Camera 1", fg_color=themes[COLOR_SELECT][0])
            try:
                self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_calib_driver')
            except KeyError:
                print("Error: Calibration driver not found.")
            else:
                self.left_bottom_frame_calib_button.configure(
                    text="Start Calibration", fg_color=themes[COLOR_SELECT][0])
        else:
            rospy.loginfo(f"Starting calibration for camera {nuc_number}")
            self._calibrate_camera(nuc_number)
            self.left_bottom_frame_calib_button.configure(
                text="Stop Calibration", fg_color=themes['red'])
    def _calibrate_camera(self, nuc_number) -> None:
        print("*** Starting Camera Calibration ***")
        print(f'Board size: {self.board_size}')
        print(f'Square size: {self.square_size}')
        if self.running_processes.get(f'sony_cam{nuc_number}_cam_driver') is None:
            self._start_camera(nuc_number)
        
        try:
            self.calib_process = roslaunch.parent.ROSLaunchParent(
                self.uuid, [self.calib_launch])
            self.calib_process.start()
            self.running_processes[f'sony_cam{nuc_number}_calib_driver'] = self.calib_process
            rospy.loginfo("Calibration started successfully")
            rospy.sleep(0.5)
        except roslaunch.RLException as e:
            print(f"Error: Failed to launch camera calibration: {str(e)}")
            

    def create_middle_bottom_frame(self) -> None:
        """Creates the bottom frame of the middle frame"""
        self.middle_bottom_frame = customtkinter.CTkFrame(self.middle_frame)
        self.middle_bottom_frame.place(
            relx=0.06, rely=0.54, relwidth=0.8, relheight=0.32)
        self.create_middle_bottom_frame_content()
    def create_middle_bottom_frame_content(self) -> None:
        """ Setting the Detection Parameters """
        self.middle_bottom_frame_label = customtkinter.CTkLabel(
            self.middle_bottom_frame, text=f"CAMERA DETECTION")
        self.middle_bottom_frame_label.place(relx=0.5, rely=0.13, anchor="center")
        self.middle_bottom_frame_marker_dim_label = customtkinter.CTkLabel(
            self.middle_bottom_frame, text="Marker Dimension: (m)")
        self.middle_bottom_frame_marker_dim_label.place(relx=0.1, rely=0.22)
        self.middle_bottom_frame_marker_dim_entry = customtkinter.CTkEntry(
            self.middle_bottom_frame,
            placeholder_text=self.marker_dim,
            placeholder_text_color="gray")
        self.middle_bottom_frame_marker_dim_entry.place(relx=0.63, rely=0.22, relwidth=0.25)
        self.middle_bottom_frame_marker_dict_label = customtkinter.CTkLabel(
            self.middle_bottom_frame, text="Marker Dictionary:")
        self.middle_bottom_frame_marker_dict_label.place(relx=0.1, rely=0.42)
        self.middle_bottom_frame_marker_dict_entry = customtkinter.CTkEntry(
            self.middle_bottom_frame,
            placeholder_text="0",
            placeholder_text_color="gray")
        self.middle_bottom_frame_marker_dict_entry.place(relx=0.63, rely=0.42, relwidth=0.25)
        self.middle_bottom_frame_marker_update_button = customtkinter.CTkButton(
            self.middle_bottom_frame, text='Update',
            command=self.update_marker_dict)
        self.middle_bottom_frame_marker_update_button.place(
            relx=0.5, rely=0.75, relwidth=0.4, anchor="center")
        self.middle_bottom_frame_marker_update_button_label = customtkinter.CTkLabel(
            self.middle_bottom_frame, text="", text_color='green',
            font=customtkinter.CTkFont(size=20, weight="bold"))
        self.middle_bottom_frame_marker_update_button_label.place(
            relx=0.8, rely=0.75, anchor="center")
    def update_marker_dict(self) -> None:
        """ Updates the marker dictionary """
        marker_dim_entry = self.middle_bottom_frame_marker_dim_entry.get()
        marker_dict_entry = self.middle_bottom_frame_marker_dict_entry.get()
        if not marker_dim_entry and not marker_dict_entry:
            print("Nothing updated")
            print(f'Original marker dimension: {self.marker_dim}')
            print(f'Original marker dictionary: {self.marker_dict}')
            print('Please enter new detection parameters')
            return
        if marker_dim_entry:
            self.marker_dim = marker_dim_entry
            print(f'Updated marker dimension: {self.marker_dim}')
        else:
            print(f'Original marker dimension: {self.marker_dim}')
        if marker_dict_entry:
            self.marker_dict = marker_dict_entry
            print(f'Updated marker dictionary: {self.marker_dict}')
        else:
            print(f'Original marker dictionary: {self.marker_dict}')
        print('Detection parameters updated successfully')
        self.middle_bottom_frame_marker_update_button_label.configure(
            text="☑", text_color='green')
    def create_left_frame(self) -> None:
        """Creates the left frame of the GUI"""
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.left_frame.place(relx=0, rely=0, relwidth=0.33, relheight=1)
        self.create_left_top_frame()
        self.create_left_center_frame()
        # self.create_left_bottom_frame()
        self.create_left_exit_button()
    def create_left_top_frame(self) -> None:
        """Start Camera and Detection"""
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(
            relx=0.1, rely=0.08, relwidth=0.8, relheight=0.08)
        self.create_left_top_frame_content()
    def create_left_top_frame_content(self) -> None:
        """ Starting and Viewing Camera """
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text="START CAMERAS")
        self.left_top_frame_label.place(relx=0.5, rely=0.5, anchor="center")
    def create_left_center_frame(self) -> None:
        self.left_center_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_center_frame.place(relx=0.10 , rely=0.20, relwidth=0.8, relheight=0.32)
        self.create_left_center_frame_content()
    def create_left_center_frame_content(self) -> None:
        """ Camera Control """
        self.left_top_frame_start_cam_label = customtkinter.CTkLabel(
            self.left_center_frame, text="Start Camera")
        self.left_top_frame_start_cam1_button = customtkinter.CTkButton(
            self.left_center_frame, text="  1  ",
            command=lambda: self._start_cam_button_event(1), width=2)
        self.left_top_frame_start_cam2_button = customtkinter.CTkButton(
            self.left_center_frame, text="  2  ",
            command=lambda: self._start_cam_button_event(2), width=2)
        self.left_top_frame_start_cam3_button = customtkinter.CTkButton(
            self.left_center_frame, text="  3  ",
            command=lambda: self._start_cam_button_event(3), width=2)
        self.left_top_frame_view_cam_label = customtkinter.CTkLabel(
            self.left_center_frame, text="View Camera")
        self.left_top_frame_view_cam1_button = customtkinter.CTkButton(
            self.left_center_frame, text="  1  ", fg_color='gray', width=2,
            command=lambda: self._view_cam_button_event(1))
        self.left_top_frame_view_cam2_button = customtkinter.CTkButton(
            self.left_center_frame, text="  2  ", fg_color='gray', width=2,
            command=lambda: self._view_cam_button_event(2))
        self.left_top_frame_view_cam3_button = customtkinter.CTkButton(
            self.left_center_frame, text="  3  ", fg_color='gray', width=2,
            command=lambda: self._view_cam_button_event(3))
        self.left_top_frame_start_view_label = customtkinter.CTkLabel(
            self.left_center_frame, text="Start & View")
        self.left_top_frame_start_view_cam1_button = customtkinter.CTkButton(
            self.left_center_frame, text="  1  ", border_width=2,
            border_color=themes['red'][0], width=2,
            command=lambda: self._startnview_cam_button_event(1))
        self.left_top_frame_start_view_cam2_button = customtkinter.CTkButton(
            self.left_center_frame, text="  2  ", border_width=2,
            border_color=themes['red'][0], width=2,
            command=lambda: self._startnview_cam_button_event(2))
        self.left_top_frame_start_view_cam3_button = customtkinter.CTkButton(
            self.left_center_frame, text="  3  ", border_width=2,
            border_color=themes['red'][0], width=2,
            command=lambda: self._startnview_cam_button_event(3))

        self.left_top_frame_start_cam_label.place(
            relx=0.06, rely=0.15)
        self.left_top_frame_start_cam1_button.place(
            relx=0.44, rely=0.15)
        self.left_top_frame_start_cam2_button.place(
            relx=0.61, rely=0.15)
        self.left_top_frame_start_cam3_button.place(
            relx=0.78, rely=0.15)

        self.left_top_frame_view_cam_label.place(
            relx=0.06, rely=0.4)
        self.left_top_frame_view_cam1_button.place(
            relx=0.44, rely=0.4)
        self.left_top_frame_view_cam2_button.place(
            relx=0.61, rely=0.4)
        self.left_top_frame_view_cam3_button.place(
            relx=0.78, rely=0.4)
        self.left_top_frame_start_view_label.place(
            relx=0.06, rely=0.65)
        self.left_top_frame_start_view_cam1_button.place(
            relx=0.44, rely=0.65)
        self.left_top_frame_start_view_cam2_button.place(
            relx=0.61, rely=0.65)
        self.left_top_frame_start_view_cam3_button.place(
            relx=0.78, rely=0.65)

    def create_left_bottom_frame(self) -> None:
        """Creates the bottom frame of the left frame"""
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(
            relx=0.10, rely=0.50, relwidth=0.8, relheight=0.32)
        self._create_left_bottom_frame_content()
    def _create_left_bottom_frame_content(self) -> None:
        """ Detection of the Marker """
        self.left_bottom_frame_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="DETECT & CALIBRATE")
        self.left_bottom_frame_label.place(relx=0.5, rely=0.17, anchor="center")
        self.left_bottom_frame_detect_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Start Detection", fg_color=themes[COLOR_SELECT][0],
            command=lambda: self._detect_marker_button_event(1))
        self.left_bottom_frame_detect_button.place(
            relx=0.5, rely=0.40, anchor="center")
        self.left_bottom_frame_detect_label_number = customtkinter.CTkLabel(
            self.left_bottom_frame, text="③", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_bottom_frame_detect_label_number.place(
            relx=0.1, rely=0.40, anchor="center")
        self.left_bottom_frame_calib_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Calibrate Camera", fg_color=themes[COLOR_SELECT][0],
            command=self._start_camera_calibration)
        self.left_bottom_frame_calib_button.place(
            relx=0.5, rely=0.7, anchor="center")
        self.left_bottom_frame_calib_label_number = customtkinter.CTkLabel(
            self.left_bottom_frame, text="④", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_bottom_frame_calib_label_number.place(
            relx=0.1, rely=0.7, anchor="center")
    def _detect_marker_button_event(self, nuc_number: str) -> None:
        """Starts the marker detection node"""
        print("Detect Marker Button Clicked")
        if self.running_processes.get(f'sony_cam{nuc_number}_detect_driver') is not None:
            rospy.loginfo("Detection already running, Now stopping it")
            try:
                self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_detect_driver')
            except KeyError:
                print("Error: Detection driver not found.")
            else:
                self.left_bottom_frame_detect_button.configure(
                    text="Start Detection", fg_color=themes[COLOR_SELECT][0])
                # Update the detection specific labels
                self.right_bottom_frame_camera_detect_status_result_label.configure(
                    text="IDLE", text_color='red')
        else:
            if self.running_processes.get(f'sony_cam{nuc_number}_cam_driver') is None:
                print(f"Camera {nuc_number} is not running, Please start the camera first")
                return
            rospy.loginfo(f"Starting detection for camera {nuc_number}")
            self._detect_marker(nuc_number)
            self.left_bottom_frame_detect_button.configure(
                text="Stop Detection", fg_color=themes['red'])
            # Update the detection specific labels
            self.right_bottom_frame_camera_detect_status_result_label.configure(
                text="ACTIVE", text_color='yellow')

    def create_left_exit_button(self) -> None:
        """Creates the exit button of the left frame"""
        self.left_exit_button = customtkinter.CTkButton(self.left_frame, text="Exit Program", fg_color=themes["red"], command=self.destroy_routine)
        self.left_exit_button.place(relx=0.5, rely=0.90, anchor="center")
    def _detect_marker(self, nuc_number) -> None:
        """Start detection node and set up rate calculation."""
        try:
            detect_launch_args = [
                f"{self.detect_launch}",
                f"launch_nuc:=sony_cam{nuc_number}",
                f"fiducial_len:={self.marker_dim}",
                f"dictionary:={self.marker_dict}"]
            detect_roslaunch_file = [(
                roslaunch.rlutil.resolve_launch_arguments(detect_launch_args)[0],
                detect_launch_args[1:])]
            detect_driver = roslaunch.parent.ROSLaunchParent(
                self.uuid, detect_roslaunch_file)
            detect_driver.start()
            self.running_processes[f'sony_cam{nuc_number}_detect_driver'] = detect_driver
            rospy.loginfo("Detection started successfully")
            rospy.sleep(0.5)
            # Start the detection rate updater
            self.is_detection_active = True
            self.update_detection_rate(nuc_number)
        except roslaunch.RLException as e:
            print(f"Error: Failed to launch marker detection: {str(e)}")
            
    def _start_camera(self, nuc_number:str)-> None:
        cam_launch_args = [
            f"{self.cam_launch}",
            f"launch_nuc:=sony_cam{nuc_number}"]
        cam_roslaunch_file = [(
            roslaunch.rlutil.resolve_launch_arguments(cam_launch_args)[0],
            cam_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(
            self.uuid, cam_roslaunch_file)
        cam_driver.start()
        self.running_processes[f'sony_cam{nuc_number}_cam_driver'] = cam_driver
        rospy.loginfo(f"Camera {nuc_number} started successfully")
        rospy.sleep(0.5)
    def _view_camera(self, nuc_number:str)-> None:
        view_launch_args = [
            f"{self.view_launch}",
            f"launch_nuc:=sony_cam{nuc_number}"]
        view_roslaunch_file = [(
            roslaunch.rlutil.resolve_launch_arguments(view_launch_args)[0],
            view_launch_args[1:])]
        view_driver = roslaunch.parent.ROSLaunchParent(
            self.uuid, view_roslaunch_file)
        view_driver.start()
        self.running_processes[f'sony_cam{nuc_number}_view_driver'] = view_driver
        rospy.loginfo(f"Camera {nuc_number} view started successfully")
        rospy.sleep(0.5)
        
    def _start_cam_button_event(self, nuc_number: str) -> None:
        try:
            if self.running_processes.get(f'sony_cam{nuc_number}_cam_driver') is not None: # i.e., camera is running
                rospy.loginfo(f"Camera {nuc_number} already running, Now stopping it")
                # Check and stop detection process if running
                if self.running_processes.get(f'sony_cam{nuc_number}_detect_driver') is not None: # i.e., Detection is running
                    rospy.loginfo(f"Stopping detection {nuc_number}")
                    try:
                        self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_detect_driver')
                    except KeyError:
                        print("Error: Detection driver not found.")
                    else:
                        # self.left_bottom_frame_detect_button.configure(
                        #     text="Start Detection", fg_color=themes[COLOR_SELECT][0])
                        # Update the detection specific labels
                        self.right_bottom_frame_camera_detect_status_result_label.configure(
                            text="IDLE", text_color='red')
                        # self.right_bottom_frame_camera_detect_rate_result_label.configure(
                        #     text='detection rate running', text_color='white')
                    # Check and stop view process if running
                if self.running_processes.get(f'sony_cam{nuc_number}_view_driver') is not None: # i.e., view is running, so stop both
                    rospy.loginfo(f"Stopping camera view {nuc_number}")
                    try:
                        self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_view_driver')
                    except KeyError:
                        print("Error: View driver not found.")
                    else:
                        try:
                            self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_cam_driver')
                        except KeyError:
                            print("Error: Camera driver not found.")
                        else:
                            self.___start_cam_button_color_event(nuc_number, "IDLE")
                            # self.left_top_frame_start_cam1_button.configure(
                            #     text="Start Camera", fg_color=themes['blue'])
                            self.___view_cam_button_color_event(nuc_number, "IDLE")
                            
                            # self.left_top_frame_start_view_cam_button.configure(
                            #     text="Start & View Camera", fg_color=themes['blue'])
                            self.___startnview_cam_button_color_event(nuc_number, "IDLE")
                            # Update the camera specific labels
                            self.right_bottom_frame_camera_status_result_label.configure(
                                text="IDLE", text_color='red')
                            self.right_bottom_frame_camera_resolution_result_label.configure(
                                text='-', text_color='white')
                            self.right_bottom_frame_camera_fps_result_label.configure(
                                text='-', text_color='white')
                else: # i.e., camera is running but view is not running, so only stop the camera
                    try:
                        self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_cam_driver')
                    except KeyError:
                        print("Error: Camera driver not found.")
                    else:
                        self.___start_cam_button_color_event(nuc_number, "IDLE")
            else: # i.e., camera is not running
                rospy.loginfo(f"Camera {nuc_number} is not running, Now starting it")
                self._start_camera(nuc_number)
                self.___start_cam_button_color_event(nuc_number, "ACTIVE")
                # self.left_top_frame_start_cam1_button.configure(
                #     fg_color=themes['red'])
                # Update the camera status label
                self.right_bottom_frame_camera_status_result_label.configure(
                    text="ACTIVE", text_color='yellow')
                self.right_bottom_frame_camera_resolution_result_label.configure(
                    text=self.cam_resolution, text_color='white')
                self.right_bottom_frame_camera_fps_result_label.configure(
                    text=self.cam_fps, text_color='white')
        except roslaunch.RLException as e:
            print('Error! Close GUI and try again after launching ROS.')
            print(f"Error: Failed to launch camera: {str(e)}")

    def ___view_cam_button_color_event(self, nuc_number: str, status: str) -> None:
        if nuc_number == 1:
            if status == "ACTIVE":
                self.left_top_frame_view_cam1_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_view_cam1_button.configure(
                    fg_color='gray')
        elif nuc_number == 2:
            if status == "ACTIVE":
                self.left_top_frame_view_cam2_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_view_cam2_button.configure(
                    fg_color='gray')
        elif nuc_number == 3:
            if status == "ACTIVE":
                self.left_top_frame_view_cam3_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_view_cam3_button.configure(
                    fg_color='gray')
    def ___start_cam_button_color_event(self, nuc_number, status) -> None:
        if nuc_number == 1:
            if status == "ACTIVE":
                self.left_top_frame_start_cam1_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_start_cam1_button.configure(
                    fg_color=themes['blue'])
        elif nuc_number == 2:
            if status == "ACTIVE":
                self.left_top_frame_start_cam2_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_start_cam2_button.configure(
                fg_color=themes['blue'])
        elif nuc_number == 3:
            if status == "ACTIVE":
                self.left_top_frame_start_cam3_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_start_cam3_button.configure(
                fg_color=themes['blue'])
            
            # Update the camera status label
            # self.right_bottom_frame_camera_status_result_label.configure(
            #     text="IDLE", text_color='red')
            # self.right_bottom_frame_camera_resolution_result_label.configure(
            #     text='-', text_color='white')
            # self.right_bottom_frame_camera_fps_result_label.configure(
            #     text='-', text_color='white')
    def ___startnview_cam_button_color_event(self, nuc_number: str, status: str) -> None:
        if nuc_number == 1:
            if status == "ACTIVE":
                self.left_top_frame_start_view_cam1_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_start_view_cam1_button.configure(
                    fg_color=themes['blue'])
        elif nuc_number == 2:
            if status == "ACTIVE":
                self.left_top_frame_start_view_cam2_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_start_view_cam2_button.configure(
                    fg_color=themes['blue'])
        elif nuc_number == 3:
            if status == "ACTIVE":
                self.left_top_frame_start_view_cam3_button.configure(
                    fg_color=themes['red'])
            else:
                self.left_top_frame_start_view_cam3_button.configure(
                    fg_color=themes['blue'])
    def _startnview_cam_button_event(self, nuc_number: str) -> None:
        print("View Camera Button Clicked")
        if self.running_processes.get(f'sony_cam{nuc_number}_cam_driver') is not None:
            if self.running_processes.get(f'sony_cam{nuc_number}_view_driver') is not None:
                rospy.loginfo(f"Stopping camera {nuc_number} node and view")
                try:
                    self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_view_driver')
                except KeyError:
                    print("Error: View driver not found.")
                else:
                    self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_cam_driver')
                    # self.left_top_frame_start_view_cam_button.configure(
                    #     text="Start & View Camera", fg_color=themes['blue'])
                    self.___startnview_cam_button_color_event(nuc_number, "IDLE")
                    self.___start_cam_button_color_event(nuc_number, "IDLE")
                    # self.left_top_frame_start_cam1_button.configure(
                    #     text="Start Camera", fg_color=themes['blue'])
                    # self.left_top_frame_view_cam_button.configure(
                    #     text="View Camera", fg_color='gray')
                    self.___view_cam_button_color_event(nuc_number, "IDLE")
                    # Update the camera status label
                    self.right_bottom_frame_camera_status_result_label.configure(
                        text="IDLE", text_color='red')
                    self.right_bottom_frame_camera_resolution_result_label.configure(
                        text='-', text_color='white')
                    self.right_bottom_frame_camera_fps_result_label.configure(
                        text='-', text_color='white')
            else:
                rospy.loginfo(f"Viewing camera {nuc_number}")
                try:
                    self._view_camera(nuc_number)
                except roslaunch.RLException as e:
                    print(f"Error: Failed to launch camera view: {str(e)}")
                else:
                    # self.left_top_frame_start_view_cam_button.configure(
                    #     text="Stop View & Camera", fg_color=themes['red'])
                    self.___startnview_cam_button_color_event(nuc_number, "ACTIVE")
                    # self.left_top_frame_view_cam_button.configure(
                    #     text="Stop View Camera", fg_color=themes['red'])
                    self.___view_cam_button_color_event(nuc_number, "ACTIVE")
                    self.___start_cam_button_color_event(nuc_number, "ACTIVE")
                    # self.left_top_frame_start_cam1_button.configure(
                    #     text="Stop Camera", fg_color=themes['red'])
                    # Update the camera status label
                    self.right_bottom_frame_camera_status_result_label.configure(
                        text="ACTIVE", text_color='yellow')
                    self.right_bottom_frame_camera_resolution_result_label.configure(
                        text=self.cam_resolution, text_color='white')
                    self.right_bottom_frame_camera_fps_result_label.configure(
                        text=self.cam_fps, text_color='white')
        else:
            try:
                self._start_camera(nuc_number)
                self._view_camera(nuc_number)
            except roslaunch.RLException as e:
                print(f"Error: Failed to launch camera view: {str(e)}")
            else:
                # self.left_top_frame_start_view_cam_button.configure(
                #     text="Stop View & Camera", fg_color=themes['red'])
                self.___startnview_cam_button_color_event(nuc_number, "ACTIVE")
                # self.left_top_frame_view_cam_button.configure(
                #     text="Stop View Camera", fg_color=themes['red'])
                self.___view_cam_button_color_event(nuc_number, "ACTIVE")
                self.___start_cam_button_color_event(nuc_number, "ACTIVE")
                # self.left_top_frame_start_cam1_button.configure(
                #     text="Stop Camera", fg_color=themes['red'])
                # Update the camera status label
                self.right_bottom_frame_camera_status_result_label.configure(
                    text="ACTIVE", text_color='yellow')
                self.right_bottom_frame_camera_resolution_result_label.configure(
                    text=self.cam_resolution, text_color='white')
                self.right_bottom_frame_camera_fps_result_label.configure(
                    text=self.cam_fps, text_color='white')

    def _view_cam_button_event(self, nuc_number: str) -> None:
        """Starts the camera view node"""
        if self.running_processes.get(f'sony_cam{nuc_number}_cam_driver') is None:
            print(f"Camera {nuc_number} is not running, Please start the camera first")
            return
        if self.running_processes.get(f'sony_cam{nuc_number}_view_driver') is None:
            rospy.loginfo(f"Viewing camera {nuc_number}")
            try:
                self._view_camera(nuc_number)
            except roslaunch.RLException as e:
                print(f"Error: Failed to launch camera view: {str(e)}")
            else:
                # self.left_top_frame_view_cam_button.configure(
                #     text="Stop View Camera", fg_color=themes['red'])
                self.___view_cam_button_color_event(nuc_number, "ACTIVE")
        else:
            rospy.loginfo(f"Stopping camera view {nuc_number}")
            try:
                self._cleanup_processes(nuc_number, f'sony_cam{nuc_number}_view_driver')
            except KeyError:
                print("Error: View driver not found.")
            else:
                self.___view_cam_button_color_event(nuc_number, "IDLE")
                # self.left_top_frame_view_cam_button.configure(
                #     text="View Camera", fg_color=themes['blue'])
    def destroy_routine(self) -> None:
        """destroys the GUI and quits the program"""
        self.destroy()
        self.quit()
    def _cleanup_processes(self, nuc_number, node_process:str) -> None:
        """ Cleans up the processes """
        try:
            self.running_processes[node_process].shutdown()
            del self.running_processes[node_process]
            # Check if it's the detection node being cleaned up
            if node_process == f'sony_cam{nuc_number}_detect_driver':
                self.is_detection_active = False
        except KeyError:
            print(f"Error: {node_process} not found in running processes.")
        else:
            print(f"{node_process} stopped successfully.")
            rospy.sleep(0.5)

        
    def update_detection_rate(self, nuc_number) -> None:
        """Periodically update the detection rate and result labels."""
        detect_topic = f"/sony_cam{nuc_number}_detect/fiducial_transforms"
        if self.is_detection_active:
            frequency = get_ros_topic_frequency(detect_topic)
            if frequency is not None:
                self.right_bottom_frame_camera_detect_rate_result_label.configure(
                    text=f'{frequency:.2f} Hz', text_color='white')
                self.right_bottom_frame_camera_detect_result_result_label.configure(
                    text="Detected", text_color=themes['green'][0])
            else:
                self.right_bottom_frame_camera_detect_rate_result_label.configure(
                    text="-", text_color='white')
                self.right_bottom_frame_camera_detect_result_result_label.configure(
                    text="Not Detected", text_color='red')
        else:
            self.right_bottom_frame_camera_detect_rate_result_label.configure(
                text="-", text_color='white')
            self.right_bottom_frame_camera_detect_result_result_label.configure(
                text="Not Detected", text_color='red')

        self.after(self.update_interval, self.update_detection_rate(nuc_number))

    def update_camera_fps(self, nuc_number) -> None:
        """Periodically update the camera FPS label."""
        camera_topic = f"/sony_cam{nuc_number}/image_raw"
        if self.running_processes.get(f'sony_cam{nuc_number}_cam_driver') is not None:
            frequency = get_ros_topic_frequency(camera_topic)
            if frequency is not None:
                self.right_bottom_frame_camera_fps_result_label.configure(
                    text=f'{frequency:.2f} Hz', text_color='white')
            else:
                self.right_bottom_frame_camera_fps_result_label.configure(
                    text="-", text_color='white')
        else:
            self.right_bottom_frame_camera_fps_result_label.configure(
                text="-", text_color='white')

        self.after(self.update_interval, self.update_camera_fps(nuc_number))

        
        
        
def get_ros_topic_frequency(topic):
    """Get the frequency of a ROS topic using 'rostopic hz' command."""
    process = subprocess.Popen(['rostopic', 'hz', topic], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Wait for a while to collect data
    rospy.sleep(0.9)

    # Terminate the process
    process.terminate()

    # Read the output
    try:
        output, _ = process.communicate(timeout=2)
        output = output.decode('utf-8')
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()

    # Extract frequency from the output using regular expression
    match = re.search(r'average rate: ([\d\.]+)', output)
    if match:
        return float(match.group(1))
    else:
        return None
def check_cpu_load():
    return psutil.cpu_percent(interval=1)
if __name__ == "__main__":
    root = NodeGUI()
    root.mainloop()
    print("Color theme: ", COLOR_SELECT)