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
CMAKE_SOURCE_DIR = /home/sitl2/sitl_ws/src/fiducials/fiducial_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sitl2/sitl_ws/build/fiducial_slam

# Utility rule file for _run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.

# Include the progress variables for this target.
include CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/progress.make

CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml:
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/sitl2/sitl_ws/build/fiducial_slam/test_results/fiducial_slam/rostest-test_create_map_aruco.xml "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sitl2/sitl_ws/src/fiducials/fiducial_slam --package=fiducial_slam --results-filename test_create_map_aruco.xml --results-base-dir \"/home/sitl2/sitl_ws/build/fiducial_slam/test_results\" /home/sitl2/sitl_ws/src/fiducials/fiducial_slam/test/create_map_aruco.xml "

_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml: CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml
_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml: CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/build.make

.PHONY : _run_tests_fiducial_slam_rostest_test_create_map_aruco.xml

# Rule to build all files generated by this target.
CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/build: _run_tests_fiducial_slam_rostest_test_create_map_aruco.xml

.PHONY : CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/build

CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/cmake_clean.cmake
.PHONY : CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/clean

CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/depend:
	cd /home/sitl2/sitl_ws/build/fiducial_slam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sitl2/sitl_ws/src/fiducials/fiducial_slam /home/sitl2/sitl_ws/src/fiducials/fiducial_slam /home/sitl2/sitl_ws/build/fiducial_slam /home/sitl2/sitl_ws/build/fiducial_slam /home/sitl2/sitl_ws/build/fiducial_slam/CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/_run_tests_fiducial_slam_rostest_test_create_map_aruco.xml.dir/depend

