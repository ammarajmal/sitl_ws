# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ammar/sitl_ws/src/fiducials/fiducial_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ammar/sitl_ws/build/fiducial_slam

# Utility rule file for fiducial_slam_generate_messages_py.

# Include the progress variables for this target.
include CMakeFiles/fiducial_slam_generate_messages_py.dir/progress.make

CMakeFiles/fiducial_slam_generate_messages_py: /home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/_AddFiducial.py
CMakeFiles/fiducial_slam_generate_messages_py: /home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/__init__.py


/home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/_AddFiducial.py: /opt/ros/noetic/lib/genpy/gensrv_py.py
/home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/_AddFiducial.py: /home/ammar/sitl_ws/src/fiducials/fiducial_slam/srv/AddFiducial.srv
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ammar/sitl_ws/build/fiducial_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating Python code from SRV fiducial_slam/AddFiducial"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/gensrv_py.py /home/ammar/sitl_ws/src/fiducials/fiducial_slam/srv/AddFiducial.srv -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p fiducial_slam -o /home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv

/home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/__init__.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/__init__.py: /home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/_AddFiducial.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/ammar/sitl_ws/build/fiducial_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Python srv __init__.py for fiducial_slam"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py -o /home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv --initpy

fiducial_slam_generate_messages_py: CMakeFiles/fiducial_slam_generate_messages_py
fiducial_slam_generate_messages_py: /home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/_AddFiducial.py
fiducial_slam_generate_messages_py: /home/ammar/sitl_ws/devel/.private/fiducial_slam/lib/python3/dist-packages/fiducial_slam/srv/__init__.py
fiducial_slam_generate_messages_py: CMakeFiles/fiducial_slam_generate_messages_py.dir/build.make

.PHONY : fiducial_slam_generate_messages_py

# Rule to build all files generated by this target.
CMakeFiles/fiducial_slam_generate_messages_py.dir/build: fiducial_slam_generate_messages_py

.PHONY : CMakeFiles/fiducial_slam_generate_messages_py.dir/build

CMakeFiles/fiducial_slam_generate_messages_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fiducial_slam_generate_messages_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fiducial_slam_generate_messages_py.dir/clean

CMakeFiles/fiducial_slam_generate_messages_py.dir/depend:
	cd /home/ammar/sitl_ws/build/fiducial_slam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ammar/sitl_ws/src/fiducials/fiducial_slam /home/ammar/sitl_ws/src/fiducials/fiducial_slam /home/ammar/sitl_ws/build/fiducial_slam /home/ammar/sitl_ws/build/fiducial_slam /home/ammar/sitl_ws/build/fiducial_slam/CMakeFiles/fiducial_slam_generate_messages_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fiducial_slam_generate_messages_py.dir/depend

