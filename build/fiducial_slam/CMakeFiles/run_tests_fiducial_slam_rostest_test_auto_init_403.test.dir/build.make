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
CMAKE_SOURCE_DIR = /home/sitl1/sitl_ws/src/fiducials/fiducial_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sitl1/sitl_ws/build/fiducial_slam

# Utility rule file for run_tests_fiducial_slam_rostest_test_auto_init_403.test.

# Include the progress variables for this target.
include CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/progress.make

CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test:
	catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/catkin/cmake/test/run_tests.py /home/sitl1/sitl_ws/build/fiducial_slam/test_results/fiducial_slam/rostest-test_auto_init_403.xml "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/sitl1/sitl_ws/src/fiducials/fiducial_slam --package=fiducial_slam --results-filename test_auto_init_403.xml --results-base-dir \"/home/sitl1/sitl_ws/build/fiducial_slam/test_results\" /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/test/auto_init_403.test "

run_tests_fiducial_slam_rostest_test_auto_init_403.test: CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test
run_tests_fiducial_slam_rostest_test_auto_init_403.test: CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/build.make

.PHONY : run_tests_fiducial_slam_rostest_test_auto_init_403.test

# Rule to build all files generated by this target.
CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/build: run_tests_fiducial_slam_rostest_test_auto_init_403.test

.PHONY : CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/build

CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/clean

CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/depend:
	cd /home/sitl1/sitl_ws/build/fiducial_slam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sitl1/sitl_ws/src/fiducials/fiducial_slam /home/sitl1/sitl_ws/src/fiducials/fiducial_slam /home/sitl1/sitl_ws/build/fiducial_slam /home/sitl1/sitl_ws/build/fiducial_slam /home/sitl1/sitl_ws/build/fiducial_slam/CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_tests_fiducial_slam_rostest_test_auto_init_403.test.dir/depend

