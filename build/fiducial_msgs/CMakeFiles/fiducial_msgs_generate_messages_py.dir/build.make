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
CMAKE_SOURCE_DIR = /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sitl1/sitl_ws/build/fiducial_msgs

# Utility rule file for fiducial_msgs_generate_messages_py.

# Include the progress variables for this target.
include CMakeFiles/fiducial_msgs_generate_messages_py.dir/progress.make

CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_Fiducial.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntry.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntryArray.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py
CMakeFiles/fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py


/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_Fiducial.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_Fiducial.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/Fiducial.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating Python from MSG fiducial_msgs/Fiducial"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/Fiducial.msg -Ifiducial_msgs:/home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p fiducial_msgs -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialArray.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/Fiducial.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py: /opt/ros/noetic/share/std_msgs/msg/Header.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating Python from MSG fiducial_msgs/FiducialArray"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialArray.msg -Ifiducial_msgs:/home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p fiducial_msgs -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialTransform.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py: /opt/ros/noetic/share/geometry_msgs/msg/Quaternion.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py: /opt/ros/noetic/share/geometry_msgs/msg/Vector3.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py: /opt/ros/noetic/share/geometry_msgs/msg/Transform.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating Python from MSG fiducial_msgs/FiducialTransform"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialTransform.msg -Ifiducial_msgs:/home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p fiducial_msgs -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialTransformArray.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py: /opt/ros/noetic/share/geometry_msgs/msg/Vector3.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialTransform.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py: /opt/ros/noetic/share/geometry_msgs/msg/Quaternion.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py: /opt/ros/noetic/share/geometry_msgs/msg/Transform.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating Python from MSG fiducial_msgs/FiducialTransformArray"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialTransformArray.msg -Ifiducial_msgs:/home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p fiducial_msgs -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntry.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntry.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialMapEntry.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating Python from MSG fiducial_msgs/FiducialMapEntry"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialMapEntry.msg -Ifiducial_msgs:/home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p fiducial_msgs -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntryArray.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntryArray.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialMapEntryArray.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntryArray.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialMapEntry.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating Python from MSG fiducial_msgs/FiducialMapEntryArray"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialMapEntryArray.msg -Ifiducial_msgs:/home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p fiducial_msgs -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py: /opt/ros/noetic/lib/genpy/gensrv_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/srv/InitializeMap.srv
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialMapEntry.msg
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py: /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialMapEntryArray.msg
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating Python code from SRV fiducial_msgs/InitializeMap"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/gensrv_py.py /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/srv/InitializeMap.srv -Ifiducial_msgs:/home/sitl1/sitl_ws/src/fiducials/fiducial_msgs/msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -p fiducial_msgs -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_Fiducial.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntry.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntryArray.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Generating Python msg __init__.py for fiducial_msgs"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg --initpy

/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /opt/ros/noetic/lib/genpy/genmsg_py.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_Fiducial.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntry.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntryArray.py
/home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Generating Python srv __init__.py for fiducial_msgs"
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genpy/cmake/../../../lib/genpy/genmsg_py.py -o /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv --initpy

fiducial_msgs_generate_messages_py: CMakeFiles/fiducial_msgs_generate_messages_py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_Fiducial.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialArray.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransform.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialTransformArray.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntry.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/_FiducialMapEntryArray.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/_InitializeMap.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/msg/__init__.py
fiducial_msgs_generate_messages_py: /home/sitl1/sitl_ws/devel/.private/fiducial_msgs/lib/python3/dist-packages/fiducial_msgs/srv/__init__.py
fiducial_msgs_generate_messages_py: CMakeFiles/fiducial_msgs_generate_messages_py.dir/build.make

.PHONY : fiducial_msgs_generate_messages_py

# Rule to build all files generated by this target.
CMakeFiles/fiducial_msgs_generate_messages_py.dir/build: fiducial_msgs_generate_messages_py

.PHONY : CMakeFiles/fiducial_msgs_generate_messages_py.dir/build

CMakeFiles/fiducial_msgs_generate_messages_py.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/fiducial_msgs_generate_messages_py.dir/cmake_clean.cmake
.PHONY : CMakeFiles/fiducial_msgs_generate_messages_py.dir/clean

CMakeFiles/fiducial_msgs_generate_messages_py.dir/depend:
	cd /home/sitl1/sitl_ws/build/fiducial_msgs && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs /home/sitl1/sitl_ws/src/fiducials/fiducial_msgs /home/sitl1/sitl_ws/build/fiducial_msgs /home/sitl1/sitl_ws/build/fiducial_msgs /home/sitl1/sitl_ws/build/fiducial_msgs/CMakeFiles/fiducial_msgs_generate_messages_py.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/fiducial_msgs_generate_messages_py.dir/depend

