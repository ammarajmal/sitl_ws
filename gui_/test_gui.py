#!/usr/bin/env python3
""" backend definitions for the gui"""
# import subprocess
import tkinter as tk
import customtkinter
import rospy
import rospkg
import roslaunch
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError
# import cv2
# import numpy as np

themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red':("#fa5f5a", "#ba3732")
          }



# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[2]

# Modes: "System" (standard), "Dark", "Light"
customtkinter.set_appearance_mode("System")
# Themes: "blue" (standard), "green", "dark-blue"
customtkinter.set_default_color_theme(COLOR_SELECT)

print("Starting GUI...")
PACKAGE = 'dslr_cam'
print("Package: ", PACKAGE)
launch_path = rospkg.RosPack().get_path(PACKAGE) + '/launch/'
detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
csv_path = launch_path.replace('launch/', 'csvfiles/')
bag_path = launch_path.replace('launch/', 'bagfiles/')



uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid)
cam_launch = f"{launch_path}camera.launch"
local_nuc_launch=f'{launch_path}local_nuc.launch'
view_launch = f"{launch_path}viewcam.launch"
calib_launch = f"{launch_path}calib.launch"

