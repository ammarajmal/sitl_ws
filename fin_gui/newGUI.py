#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import datetime
import tkinter as tk
import customtkinter as ctk
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import subprocess
import csv
import message_filters
from fiducial_msgs.msg import FiducialTransformArray
import re
import psutil
import rospy
import rospkg
import roslaunch
from fast_cam.msg import CameraSpecs
themes = {'blue': ("#3B8ED0", "#1F6AA5", "#1f82d1"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red': ("#fa5f5a", "#ba3732")
          }
# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[0]
# Modes: "System" (standard), "Dark", "Light"
ctk.set_appearance_mode("System")
# Themes: "blue" (standard), "green", "dark-blue"
ctk.set_default_color_theme(COLOR_SELECT)

class NodeGUI(ctk.CTk):
    def __init__(self, *args, **kwargs):
        super(NodeGUI, self).__init__(*args, **kwargs)
        self.title("Displacement Measurement System")
        self.geometry("1200x580")
        self.resizable(1, 1)
        self.protocol("WM_DELETE_WINDOW", self.on_closing)

        self.sub1 = None
        self.sub2 = None
        self.sub3 = None
        self.ats = None
        self.cam1 = False
        self.cam2 = False
        self.cam3 = False

        self.experiment_name = 'Exp1'
        self.experiment_dur = 60 # seconds
        self.exp_name_var = tk.StringVar(self, self.experiment_name)
        self.exp_dur_var = tk.StringVar(self, self.experiment_dur)

        self.file_name = None

        self.image_width = '640'
        self.image_height = '480'
        self.camera_resolution = self.image_width + 'x' + self.image_height
        self.camera_fps = '60'

        self.update_interval = 1000 # ms
        self.detection_rate_timeout = 5 # timeout for detection rate calculation

        self.is_detection_active = False
        self.is_data_collection_active = False

        self.board_size = '6x5' # default board size for calibration
        self.square_size = '0.025' # default square size for calibration in meters

        self.marker_dim = '0.020' # ARUCO marker dimension in meters
        self.marker_dict = '0' #  ARUCO marker dictionary (DICT_4X4_50)
        self.marker_dict_var = tk.StringVar(self, self.marker_dict)

        self.cam_pkg = 'dslr_cam'
        self.detect_pkg = 'disp_6dof'
        try:
            self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
            self.cam_launch_path = rospkg.RosPack().get_path(self.cam_pkg) + '/launch/'
            self.detect_launch_path = rospkg.RosPack().get_path(self.detect_pkg) + '/launch/'
            self.cam_launch_file = f'{self.cam_launch_path}use.launch'
            self.cam_view_launch_file = f'{self.cam_launch_path}use_viewcamera.launch'
            self.cam_calib_launch_file = f'{self.cam_launch_path}calib.launch'
            self.detect_launch_file = f'{self.detect_launch_path}use_aruco.launch'
        except rospkg.common.ResourceNotFound:
            print('ROS packages not found')
            self.cam_launch_file = None
            self.cam_view_launch_file = None
            self.cam_calib_launch_file = None
            self.detect_launch_file = None
        
        
        self.label_height = 0.055
        self.ver_space = 0.06
        self.frame_width = 1 - (self.ver_space*4)
        self.frame_height = 0.2725

        # process management
        self.running_processes = {}

        # gui widgets initialization
        self.create_widgets()
        # self.update()
        self.left_frame = None
        
        

        
        # self.after(1000, self.update)
        # self.after(1000, self.update_cpu)
        # self.after(1000, self.update_memory)
        # self.after(1000, self.update_temp)
        # self.after(1000, self.update_disk)
        # self.after(1000, self.update_network)
        # self.after(1000, self.update_camera)
        # self.after(1000, self.update_fiducial)
    def create_widgets(self)-> None:
        ''' Starts the GUI widgets creation '''

        self.create_left_frame()
        self.create_middle_first_frame()
        self.create_middle_second_frame()
        self.create_right_frame()
    def create_right_frame(self)-> None:
        ''' Creates the right frame for displaying camera images '''
        self.right_frame = tk.Frame(self, bg=themes[COLOR_SELECT][2])
        self.right_frame.place(relx=0.75, rely=0, relwidth=0.25, relheight=1)
        self.create_right_top_frame()
        self.create_right_bottom_frame()
    def create_right_top_frame(self)-> None:
        ''' Creates the top frame in the right frame '''
        self.right_top_frame = ctk.CTkFrame(self.right_frame,fg_color=themes[COLOR_SELECT][0])
        self.right_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_right_top_frame_widgets()
    def create_right_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the right frame '''
        self.right_top_frame_label = ctk.CTkLabel(self.right_top_frame, text='SYSTEM INFORMATION', text_color='white')
        self.right_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_right_bottom_frame(self)-> None:
        ''' Creates the bottom frame in the right frame '''
        self.right_bottom_frame = ctk.CTkFrame(self.right_frame, fg_color=themes[COLOR_SELECT][0])
        self.right_bottom_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=1-(self.ver_space*3)-self.label_height, anchor='n')
        # self.create_right_bottom_frame_widgets()
    def create_middle_second_frame(self)-> None:
        ''' Creates the middle frame for setting system parameters '''
        self.middle_second_frame = tk.Frame(self, bg=themes[COLOR_SELECT][0])
        self.middle_second_frame.place(relx=0.5, rely=0, relwidth=0.25, relheight=1)
        self.create_middle_second_top_frame()
        self.create_middle_second_center_frame()
        self.create_middle_second_bottom_frame()
    def create_middle_second_top_frame(self)-> None:
        ''' Creates the top frame in the middle second frame '''
        self.middle_second_top_frame = ctk.CTkFrame(self.middle_second_frame)
        self.middle_second_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_middle_second_top_frame_widgets()
    def create_middle_second_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the middle second frame '''
        self.middle_second_top_frame_label = ctk.CTkLabel(self.middle_second_top_frame, text='RECORD & PLOT DATA')
        self.middle_second_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_middle_second_center_frame(self)-> None:
        ''' Creates the center frame in the middle second frame '''
        self.middle_second_center_frame = ctk.CTkFrame(self.middle_second_frame)
        self.middle_second_center_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=self.frame_height+0.1, anchor='n')
        self.create_middle_second_center_frame_widgets()
    def create_middle_second_center_frame_widgets(self)-> None:
        ''' Creates the widgets in the center frame in the middle second frame '''
        self.middle_second_center_record_label = ctk.CTkLabel(self.middle_second_center_frame, text='RECORD DATA')
        self.middle_second_center_record_label.place(relx=0.5, rely=0.05, anchor='n')
        self.middle_second_center_exp_label = ctk.CTkLabel(self.middle_second_center_frame, text='Experiment Name: ')
        self.middle_second_center_exp_label.place(relx=0.09, rely=0.2)
        self.middle_second_center_exp_entry = ctk.CTkEntry(self.middle_second_center_frame, textvariable=self.exp_name_var)
        self.middle_second_center_exp_entry.place(relx=0.75, rely=0.2, anchor='n', relwidth=0.3)
        self.middle_second_center_dur_label = ctk.CTkLabel(self.middle_second_center_frame, text='Experiment Duration(s):')
        self.middle_second_center_dur_label.place(relx=0.09, rely=0.37)
        self.middle_second_center_dur_entry = ctk.CTkEntry(self.middle_second_center_frame, textvariable=self.exp_dur_var)
        self.middle_second_center_dur_entry.place(relx=0.8, rely=0.37, anchor='n', relwidth=0.2)
        self.middle_second_center_rec1_button = ctk.CTkButton(self.middle_second_center_frame, text='1', command=lambda:self.record_data(1))
        self.middle_second_center_rec1_button.place(relx=0.1, rely=0.58, relwidth=0.2)
        self.middle_second_center_rec2_button = ctk.CTkButton(self.middle_second_center_frame, text='2', command=lambda:self.record_data(2))
        self.middle_second_center_rec2_button.place(relx=0.4, rely=0.58, relwidth=0.2)
        self.middle_second_center_rec3_button = ctk.CTkButton(self.middle_second_center_frame, text='3', command=lambda:self.record_data(3))
        self.middle_second_center_rec3_button.place(relx=0.7, rely=0.58, relwidth=0.2)
        self.middle_second_center_recall_button = ctk.CTkButton(self.middle_second_center_frame, text='RECALL', command=self.recall_data)
        self.middle_second_center_recall_button.place(relx=0.5, rely=0.8, anchor='n')
    def recall_data(self):
        ''' Recalls the data '''
        print('Recall Data')
        
        
    def record_data(self, num):
        ''' Records the data '''
        self.experiment_name = self.exp_name_var.get()
        self.experiment_dur = int(self.exp_dur_var.get())
        self.file_name = f'{self.experiment_name}_{datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")}.csv'
        self.is_data_collection_active = True
        self.record_data_process()
    def record_data_process(self):
        ''' Records the data '''
        if self.is_data_collection_active:
            self.data_collection_process()
            self.after(self.experiment_dur*1000, self.record_data_process)
    def data_collection_process(self):
        ''' Collects the data '''
        print('Data Collection Process')
    def create_middle_second_bottom_frame(self)-> None:
        ''' Creates the bottom frame in the middle second frame '''
        self.middle_second_bottom_frame = ctk.CTkFrame(self.middle_second_frame)
        self.middle_second_bottom_frame.place(relx=0.5, rely=(self.ver_space*3)+self.label_height+self.frame_height+0.1, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        self.create_middle_second_bottom_frame_widgets()
    def create_middle_second_bottom_frame_widgets(self)-> None:
        ''' Creates the widgets in the bottom frame in the middle second frame '''
        self.middle_second_bottom_frame_label = ctk.CTkLabel(self.middle_second_bottom_frame, text='PLOT DATA')
        self.middle_second_bottom_frame_label.place(relx=0.5, rely=0.1, anchor='n')
        self.middle_second_bottom_frame_button = ctk.CTkButton(self.middle_second_bottom_frame, text='PLOT', command=self.plot_data)
        self.middle_second_bottom_frame_button.place(relx=0.5, rely=0.5, anchor='n')
    def plot_data(self):
        ''' Plots the data '''
        print('Plot Data')
        
    def create_middle_first_frame(self)-> None:
        ''' Creates the middle frame for setting system parameters '''
        self.middle_first_frame = tk.Frame(self, bg=themes['blue'][2])
        self.middle_first_frame.place(relx=0.25, rely=0, relwidth=0.25, relheight=1)
        self.create_middle_first_top_frame()
        self.create_middle_first_center_frame()
        self.create_middle_first_bottom_frame()
        self.exit_button_frame()
    def create_middle_first_top_frame(self)-> None:
        ''' Creates the top frame in the middle first frame '''
        self.middle_first_top_frame = ctk.CTkFrame(self.middle_first_frame)
        self.middle_first_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_middle_first_top_frame_widgets()
    def create_middle_first_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the middle first frame '''
        self.middle_first_top_frame_label = ctk.CTkLabel(self.middle_first_top_frame, text='SYSTEM PARAMETERS')
        self.middle_first_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_middle_first_center_frame(self)-> None:
        ''' Creates the center frame in the middle first frame '''
        self.middle_first_center_frame = ctk.CTkFrame(self.middle_first_frame)
        self.middle_first_center_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        # self.create_middle_first_center_frame_widgets()
    def create_middle_first_bottom_frame(self)-> None:
        ''' Creates the bottom frame in the middle first frame '''
        self.middle_first_bottom_frame = ctk.CTkFrame(self.middle_first_frame)
        self.middle_first_bottom_frame.place(relx=0.5, rely=(self.ver_space*3)+self.label_height+self.frame_height, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        # self.create_middle_first_bottom_frame_widgets()
    def exit_button_frame(self)-> None:
        ''' Creates the exit button frame '''
        self.exit_button = ctk.CTkButton(self.middle_first_frame, text='EXIT', command=self.on_closing, fg_color=themes["red"])
        self.exit_button.place(relx=0.5, rely=(self.ver_space*4)+self.label_height+(self.frame_height*2), relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        
        
        
        
        
    def create_left_frame(self)-> None:
        ''' Creates the left frame to start Camera and Detection '''
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][0])
        self.left_frame.place(relx=0, rely=0, relwidth=0.25, relheight=1)
        self.create_left_top_frame()
        self.create_left_center_first_frame()
        self.create_left_center_second_frame()
        self.create_left_bottom_frame()
    def create_left_top_frame(self)-> None:
        ''' Creates the top frame in the left frame '''
        self.left_top_frame = ctk.CTkFrame(self.left_frame)
        self.left_top_frame.place(relx=0.5, rely=self.ver_space, relwidth=self.frame_width, relheight=self.label_height, anchor='n')
        self.create_left_top_frame_widgets()
    def create_left_center_first_frame(self)-> None:
        ''' Starting Cameras '''
        self.left_center_first_frame = ctk.CTkFrame(self.left_frame)
        self.left_center_first_frame.place(relx=0.5, rely=(self.ver_space*2)+self.label_height, relwidth=self.frame_width, relheight=self.frame_height, anchor='n')
        self.create_left_center_first_frame_widgets()
    def create_left_center_second_frame(self)-> None:
        ''' Starting Detection '''
        self.left_center_second_frame = ctk.CTkFrame(self.left_frame)
        self.left_center_second_frame.place(relx=0.5, rely=(self.ver_space*3)+self.label_height+self.frame_height, relwidth=self.frame_width, relheight=0.15, anchor='n')
        self.create_left_center_second_frame_widgets()
    def create_left_bottom_frame(self)-> None:
        ''' Starting Data Collection '''
        self.left_bottom_frame = ctk.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(relx=0.5, rely=(self.ver_space*4)+self.label_height+self.frame_height+0.15, relwidth=self.frame_width, relheight=0.20, anchor='n')
        self.create_left_bottom_frame_widgets()
    def create_left_top_frame_widgets(self)-> None:
        ''' Creates the widgets in the top frame in the left frame '''
        self.left_top_frame_label = ctk.CTkLabel(self.left_top_frame, text='START CAMERA & DETECTION')
        self.left_top_frame_label.place(relx=0.5, rely=0.5, anchor='center')
    def create_left_center_first_frame_widgets(self)-> None:
        ''' Starts individual cameras'''
        self.left_start_cam_label = ctk.CTkLabel(self.left_center_first_frame, text='Start Camera')
        self.left_start_cam1_button = ctk.CTkButton(self.left_center_first_frame, text='  1  ', command=lambda:self.start_camera_btn_event(1), width=2)
        self.left_start_cam2_button = ctk.CTkButton(self.left_center_first_frame, text='  2  ', command=lambda:self.start_camera_btn_event(2), width=2)
        self.left_start_cam3_button = ctk.CTkButton(self.left_center_first_frame, text='  3  ', command=lambda:self.start_camera_btn_event(3), width=2)

        self.left_start_view_label = ctk.CTkLabel(self.left_center_first_frame, text='View Camera')
        self.left_view1_button = ctk.CTkButton(self.left_center_first_frame, text='  1  ', command=lambda:self.start_view(1), width=2, fg_color='gray')
        self.left_view2_button = ctk.CTkButton(self.left_center_first_frame, text='  2  ', command=lambda:self.start_view(2), width=2, fg_color='gray')
        self.left_view3_button = ctk.CTkButton(self.left_center_first_frame, text='  3  ', command=lambda:self.start_view(3), width=2, fg_color='gray')

        self.left_startnview_label = ctk.CTkLabel(self.left_center_first_frame, text='Start & View')
        self.left_startnview1_button = ctk.CTkButton(self.left_center_first_frame, text='  1  ', command=lambda:self.start_camera_view(1), width=2)
        self.left_startnview2_button = ctk.CTkButton(self.left_center_first_frame, text='  2  ' , command=lambda:self.start_camera_view(2), width=2)
        self.left_startnview3_button = ctk.CTkButton(self.left_center_first_frame, text='  3  ', command=lambda:self.start_camera_view(3), width=2)

        v_space1 = 0.15
        v_space2 = 0.42
        v_space3 = 0.69
        h_space = 0.25

        start1_pos = 0.52
        start2_pos = 0.69
        start3_pos = 0.86
        self.left_start_cam_label.place(relx=h_space, rely=v_space1, anchor='n')
        self.left_start_cam1_button.place(relx=start1_pos, rely=v_space1, anchor='n')
        self.left_start_cam2_button.place(relx=start2_pos, rely=v_space1, anchor='n')
        self.left_start_cam3_button.place(relx=start3_pos, rely=v_space1, anchor='n')
        
        self.left_start_view_label.place(relx=h_space, rely=v_space2, anchor='n')
        self.left_view1_button.place(relx=start1_pos, rely=v_space2, anchor='n')
        self.left_view2_button.place(relx=start2_pos, rely=v_space2, anchor='n')
        self.left_view3_button.place(relx=start3_pos, rely=v_space2, anchor='n')
        
        self.left_startnview_label.place(relx=h_space, rely=v_space3, anchor='n')
        self.left_startnview1_button.place(relx=start1_pos, rely=v_space3, anchor='n')
        self.left_startnview2_button.place(relx=start2_pos, rely=v_space3, anchor='n')
        self.left_startnview3_button.place(relx=start3_pos, rely=v_space3, anchor='n')
    def create_left_center_second_frame_widgets(self)-> None:
        ''' Calibrate Cameras '''
        self.left_calib_cam_label = ctk.CTkLabel(self.left_center_second_frame, text='CALIBRATE CAMERA')
        self.left_calib_cam1_button = ctk.CTkButton(self.left_center_second_frame, text='1', command=lambda:self.calibrate_camera(1))
        self.left_calib_cam2_button = ctk.CTkButton(self.left_center_second_frame, text='2', command=lambda:self.calibrate_camera(2))
        self.left_calib_cam3_button = ctk.CTkButton(self.left_center_second_frame, text='3', command=lambda:self.calibrate_camera(3))
        self.left_calib_cam_label.place(relx=0.5, rely=0.1, anchor='n')
        self.left_calib_cam1_button.place(relx=0.2, rely=0.5, anchor='n', relwidth=0.2)
        self.left_calib_cam2_button.place(relx=0.5, rely=0.5, anchor='n', relwidth=0.2)
        self.left_calib_cam3_button.place(relx=0.8, rely=0.5, anchor='n', relwidth=0.2)
        
    def create_left_bottom_frame_widgets(self)-> None:
        self.left_start_detect_label = ctk.CTkLabel(self.left_bottom_frame, text='START DETECTION')
        self.left_detect1_button = ctk.CTkButton(self.left_bottom_frame, text='1', command=lambda:self.start_detection(1))
        self.left_detect2_button = ctk.CTkButton(self.left_bottom_frame, text='2', command=lambda:self.start_detection(2))
        self.left_detect3_button = ctk.CTkButton(self.left_bottom_frame, text='3', command=lambda:self.start_detection(3))
        self.left_detectall_button = ctk.CTkButton(self.left_bottom_frame, text='ALL CAMERAS', command=self.start_detection_all, fg_color='gray')
        
        self.left_start_detect_label.place(relx=0.5, rely=0.08, anchor='n')
        self.left_detect1_button.place(relx=0.2, rely=0.34, anchor='n', relwidth=0.2)
        self.left_detect2_button.place(relx=0.5, rely=0.34, anchor='n', relwidth=0.2)
        self.left_detect3_button.place(relx=0.8, rely=0.34, anchor='n', relwidth=0.2)
        self.left_detectall_button.place(relx=0.5, rely=0.66, anchor='n')
    def start_detection(self, cam_num):
        ''' Starts detection for the selected camera '''
        if cam_num == 1:
            self.start_detection_process(1)
        elif cam_num == 2:
            self.start_detection_process(2)
        elif cam_num == 3:
            self.start_detection_process(3)
    def start_detection_all(self):
        ''' Starts detection for all cameras '''
        self.start_detection_process(1)
        self.start_detection_process(2)
        self.start_detection_process(3)
        
    def start_camera_btn_event(self, cam_num):
        ''' Starts the camera '''
        try:
            if self.running_processes.get(f'sony_cam{cam_num}_cam_driver') is not None:
                # i.e., the camera is running, stop the camera
                rospy.loginfo(f'Stopping camera {cam_num}')
                # check and stop the detection process if running
                if self.running_processes.get(f'sony_cam{cam_num}_detect_driver') is not None:
                    # i.e., the detection process is running, stop the detection process
                    rospy.loginfo(f'Stopping detection for camera {cam_num}')
                    try:
                        self.cleanup_process(f'sony_cam{cam_num}_detect_driver')
                    except KeyError:
                        rospy.logerr(f'Detection process for camera {cam_num} not found')
                    else:
                        print(f'Camera {cam_num} detection stopped')
                        # update the detection status in the right pane
                        self._ui_detect_cam_btn(cam_num, "IDLE")
                if self.running_processes.get(f'sony_cam{cam_num}_view_driver') is not None:
                    # i.e., the view process is running, stop the view process
                    rospy.loginfo(f'Stopping view for camera {cam_num}')
                    try:
                        self.cleanup_process(f'sony_cam{cam_num}_view_driver')
                    except KeyError:
                        rospy.logerr(f'View process for camera {cam_num} not found')
                    else:
                        # update the view status in the right pane
                        print(f'Camera {cam_num} view stopped')
                        self._ui_start_cam_btn(cam_num, "IDLE")
                        self._ui_view_cam_btn(cam_num, "IDLE")
                        self._ui_start_view_cam_btn(cam_num, "IDLE")
                else:
                    # i.e., view and detect processes are not running, but camera is running so stop the camera
                    try:
                        self.cleanup_process(f'sony_cam{cam_num}_cam_driver')
                    except KeyError:
                        rospy.logerr(f'Camera {cam_num} not found')
                    else:
                        print(f'Camera {cam_num} stopped')
                        self._ui_start_cam_btn(cam_num, "IDLE")
            else:
                # i.e., the camera is not running, start the camera
                rospy.loginfo(f'Starting camera {cam_num}')
                self.start_camera_process(cam_num)
                rospy.loginfo(f'Camera {cam_num} started')
                self._ui_start_cam_btn(cam_num, "RUNNING")
        except rospy.ROSInterruptException:
            rospy.logerr('ROS Interrupted')
    def start_camera_process(self, cam_num):
        ''' Starts the camera process '''
        if cam_num == 1:
            self.start_camera(1)
        elif cam_num == 2:
            self.start_camera(2)
        elif cam_num == 3:
            self.start_camera(3)






            
    #     self.menu = tk.Menu(self)
    #     self.config(menu=self.menu)
    #     self.file_menu = tk.Menu(self.menu, tearoff=0)
    #     self.menu.add_cascade(label="File", menu=self.file_menu)
    #     self.file_menu.add_command(label="Exit", command=self.on_closing)
    #     self.help_menu = tk.Menu(self.menu, tearoff=0)
    #     self.menu.add_cascade(label="Help", menu=self.help_menu)
    #     self.help_menu.add_command(label="About", command=self.show_about)
    # def create_tabs(self):
    #     self.tabs = ctk.CTkNotebook(self)
    #     self.tabs.pack(fill="both", expand=True)
    #     self.create_tab_system()
    #     self.create_tab_camera()
    #     self.create_tab_fiducial()
    # def create_tab_system(self):
    #     self.tab_system = ctk.CTkFrame(self.tabs)
    #     self.tabs.add(self.tab_system, text="System")
    #     self.create_system_widgets()
    # def create_system_widgets(self):
    #     self.create_system_info()
    #     self.create_system_cpu()
    #     self.create_system_memory()
    #     self.create_system_temp()
    #     self.create_system_disk()
    #     self.create_system_network()
    # def create_system_info(self):
    #     self.frame_system_info = ctk.CTkFrame(self.tab_system)
    #     self.frame_system_info.pack(fill="both", expand=True)
    #     self.label_system_info = ctk.CTkLabel(self.frame_system_info, text="System Information", font=("Helvetica", 16))
    #     self.label_system_info.pack(fill="both", expand=True)
    #     self.label_system_info = ctk.CTkLabel(self.frame_system_info, text="System Information", font=("Helvetica", 16))
    #     self.label_system_info.pack(fill="both", expand=True)
    #     self.label_system_info = ctk.CTkLabel(self.frame_system_info, text="System Information", font=("Helvetica", 16))
    #     self.label_system_info.pack(fill="both", expand=True)
    #     self.label_system_info = ctk.CTkLabel(self.frame_system_info, text="System Information", font=("Helvetica", 16))
    #     self.label_system_info.pack(fill="both", expand=True)
    #     self.label_system_info = ctk.CTkLabel(self.frame_system_info, text="System Information", font=("Helvetica", 16))
    #     self.label_system_info.pack(fill="both", expand=True)
    #     self.label_system_info = ctk.CTkLabel(self.frame_system_info, text="System Information", font=("Helvetica", 16))
    #     self.label_system_info.pack(fill="both", expand=True)
    # def create_system_cpu(self):
    #     self.frame_system_cpu = ctk.CTkFrame(self.tab_system)
    #     self.frame_system_cpu.pack(fill="both", expand=True)
    #     self.label_system_cpu = ctk.CTkLabel(self.frame_system_cpu, text="CPU Usage", font=("Helvetica", 16))
    #     self.label_system_cpu.pack(fill="both", expand=True)
    #     self.label_system_cpu = ctk.CTkLabel(self.frame_system_cpu, text="CPU Usage", font=("Helvetica", 16))
    #     self.label_system_cpu.pack(fill="both", expand=True)
    #     self.label_system_cpu = ctk.CTkLabel(self.frame_system_cpu, text="CPU Usage", font=("Helvetica", 16))
    #     self.label_system_cpu.pack(fill="both", expand=True)
    #     self.label_system_cpu = ctk.CTkLabel(self.frame_system_cpu, text="CPU Usage", font=("Helvetica", 16))
    #     self.label_system_cpu.pack(fill="both", expand=True)
    #     self.label_system_cpu = ctk.CTkLabel(self.frame_system_cpu, text="CPU Usage", font=("Helvetica", 16))
    #     self.label_system_cpu.pack(fill="both", expand=True)
    #     self.label_system_cpu = ctk.CTkLabel(self.frame_system_cpu, text="CPU Usage", font=("Helvetica", 16))
    #     self.label_system_cpu.pack(fill="both", expand=True)
    # def create_system_memory(self):
    #     self.frame_system_memory = ctk.CTkFrame(self.tab_system)
    #     self.frame_system_memory.pack(fill="both", expand=True)
    #     self.label_system_memory = ctk.CTkLabel(self.frame_system_memory, text="Memory Usage", font=("Helvetica", 16))
    #     self.label_system_memory.pack(fill="both", expand=True)
    #     self.label_system_memory = ctk.CTkLabel(self.frame_system_memory, text="Memory Usage", font=("Helvetica", 16))
    #     self.label_system_memory.pack(fill="both", expand=True)
    #     self.label_system_memory = ctk.CTkLabel(self.frame_system_memory, text="Memory Usage", font=("Helvetica", 16))
    #     self.label_system_memory.pack(fill="both", expand=True)
    #     self.label_system_memory = ctk.CTkLabel(self.frame_system_memory, text="Memory Usage", font=("Helvetica", 16))
    #     self.label_system_memory.pack(fill="both", expand=True)
    #     self.label_system_memory = ctk.CTkLabel(self.frame_system_memory, text="Memory Usage", font=("Helvetica", 16))
    #     self.label_system_memory.pack(fill="both", expand=True)
    #     self.label_system_memory = ctk.CTkLabel(self.frame_system_memory, text="Memory Usage", font=("Helvetica", 16))
    #     self.label_system_memory.pack(fill="both", expand=True)
    def on_closing(self):
        self.quit()
    def show_about(self):
        ctk.messagebox.showinfo("About", "Displacement Measurement System\nVersion 1.0")
    def update(self):
        self.after(1000, self.update)
    def update_cpu(self):
        self.after(1000, self.update_cpu)
    def update_memory(self):
        self.after(1000, self.update_memory)
    def update_temp(self):
        self.after(1000, self.update_temp)
    def update_disk(self):
        self.after(1000, self.update_disk)
    def update_network(self):
        self.after(1000, self.update_network)
    def update_camera(self):
        self.after(1000, self.update_camera)
    def update_fiducial(self):
        self.after(1000, self.update_fiducial)
    def create_tab_camera(self):
        self.tab_camera = ctk.CTkFrame(self.tabs)
        self.tabs.add(self.tab_camera, text="Camera")
        self.create_camera_widgets()
    def create_camera_widgets(self):
        self.create_camera_info()
        self.create_camera_specs()
        self.create_camera_image()
    def create_camera_info(self):
        self.frame_camera_info = ctk.CTkFrame(self.tab_camera)
        self.frame_camera_info.pack(fill="both", expand=True)
        self.label_camera_info = ctk.CTkLabel(self.frame_camera_info, text="Camera Information", font=("Helvetica", 16))
        self.label_camera_info.pack(fill="both", expand=True)
if __name__ == "__main__":
    app = NodeGUI()
    app.mainloop()