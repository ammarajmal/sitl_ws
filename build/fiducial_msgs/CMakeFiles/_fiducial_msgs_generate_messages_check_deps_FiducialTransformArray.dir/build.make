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
CMAKE_SOURCE_DIR = /home/ammar/sitl_ws/src/fiducials/fiducial_msgs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ammar/sitl_ws/build/fiducial_msgs

# Utility rule file for _fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.

# Include the progress variables for this target.
include CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/progress.make

CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray:
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/genmsg/cmake/../../../lib/genmsg/genmsg_check_deps.py fiducial_msgs /home/ammar/sitl_ws/src/fiducials/fiducial_msgs/msg/FiducialTransformArray.msg std_msgs/Header:geometry_msgs/Transform:geometry_msgs/Quaternion:fiducial_msgs/FiducialTransform:geometry_msgs/Vector3

_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray: CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray
_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray: CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/build.make

.PHONY : _fiducial_msgs_generate_messages_check_deps_FiducialTransformArray

# Rule to build all files generated by this target.
CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/build: _fiducial_msgs_generate_messages_check_deps_FiducialTransformArray

.PHONY : CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/build

CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/cmake_clean.cmake
.PHONY : CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/clean

CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/depend:
	cd /home/ammar/sitl_ws/build/fiducial_msgs && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ammar/sitl_ws/src/fiducials/fiducial_msgs /home/ammar/sitl_ws/src/fiducials/fiducial_msgs /home/ammar/sitl_ws/build/fiducial_msgs /home/ammar/sitl_ws/build/fiducial_msgs /home/ammar/sitl_ws/build/fiducial_msgs/CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/_fiducial_msgs_generate_messages_check_deps_FiducialTransformArray.dir/depend

