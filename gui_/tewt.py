        
    def _start_cam_button_event(self, nuc_number: str, view: bool) -> None:
        try:
            camera_topic = f'/{self.node_name}/image_raw'
            camera_active_check = self.check_active_topic(camera_topic)
            if self.running_processes.get(f'{self.node_name}_cam_driver') is not None: # i.e., camera is running
            if camera_active_check == "True": # i.e., camera is running
                rospy.loginfo(f"Camera {nuc_number} already running, Now stopping it")
                if view:
                    try:
                        self._cleanup_processes(f'{self.node_name}_view_driver')
                    except KeyError:
                        print("Error: View driver not found.")
                    else:
                        self.left_top_frame_start_view_cam_button.configure(
                            text="Start & View Camera", fg_color=themes['blue'])
                else:
                    try:
                        self._cleanup_processes(f'{self.node_name}_cam_driver')
                    except KeyError:
                        print("Error: Camera driver not found.")
                    else:
                        self.left_top_frame_start_cam_button.configure(
                            text="Start Camera", fg_color=themes['blue'])
                        
            if camera_active_check == "False": # i.e., camera is not running
                rospy.loginfo(f"Camera {nuc_number} is not running, Now starting it")
                self._start_camera(nuc_number)
                if view:
                    rospy.loginfo(f"Viewing camera {nuc_number}")
                    try:
                        self._view_camera(nuc_number)
                    except roslaunch.RLException as e:
                        print(f"Error: Failed to launch camera view: {str(e)}")
                    else:
                        self.left_top_frame_start_view_cam_button.configure(
                            text="Stop View & Camera", fg_color=themes['red'])
                else:
                    self.left_top_frame_start_cam_button.configure(
                    text="Stop Camera", fg_color=themes['red'])
                # update the camera spceifications tab with the camera specs
                # self._update_camera_specs()
                # self._check_camera_fps()
            if camera_active_check == "Error": # i.e., error in checking the camera status (May be due to ROS not running)
                print(f"{camera_active_check}! Could not perform action.")
            # if view:
            #     print(f"Starting camera {nuc_number} and viewing")
            # else:
            #     print(f"Starting camera {nuc_number}")
        except Exception as e:
            print('Error! Close GUI and try again after launching ROS.')
            print(f"Error: {e}")