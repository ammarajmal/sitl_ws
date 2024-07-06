def single_record_data(cam):
        # Update the experiment name and duration
    if self.right_bottom_frame_exp_name_entry.get() is not None:
        self.experiment_name = self.right_bottom_frame_exp_name_entry.get()
    if self.right_bottom_frame_exp_duration_entry.get() is not None:
        self.experiment_duration = int(self.right_bottom_frame_exp_duration_entry.get())
    # File name for data collection
    # save current time in cur_time from rospy and convert it to seconds
    cur_time = rospy.get_time()
    cur_time = datetime.datetime.fromtimestamp(cur_time).strftime('%Y-%m-%d_%H-%M-%S')
    cwd = os.getcwd()
    # print('Current working directory:', cwd)
    cwd = os.path.join(cwd, 'data/data analysis/')
    
    self.file_name = f"Data_{self.experiment_name}_{self.experiment_duration}s_{cur_time}.csv"
    self.file_name = os.path.join(cwd, self.file_name)
    