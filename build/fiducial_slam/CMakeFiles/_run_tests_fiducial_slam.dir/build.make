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
CMAKE_SOURCE_DIR = /home/tesol/sitl_ws/src/fiducials/fiducial_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tesol/sitl_ws/build/fiducial_slam

# Utility rule file for _run_tests_fiducial_slam.

# Include the progress variables for this target.
include CMakeFiles/_run_tests_fiducial_slam.dir/progress.make

_run_tests_fiducial_slam: CMakeFiles/_run_tests_fiducial_slam.dir/build.make

.PHONY : _run_tests_fiducial_slam

# Rule to build all files generated by this target.
CMakeFiles/_run_tests_fiducial_slam.dir/build: _run_tests_fiducial_slam

.PHONY : CMakeFiles/_run_tests_fiducial_slam.dir/build

CMakeFiles/_run_tests_fiducial_slam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/_run_tests_fiducial_slam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/_run_tests_fiducial_slam.dir/clean

CMakeFiles/_run_tests_fiducial_slam.dir/depend:
	cd /home/tesol/sitl_ws/build/fiducial_slam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tesol/sitl_ws/src/fiducials/fiducial_slam /home/tesol/sitl_ws/src/fiducials/fiducial_slam /home/tesol/sitl_ws/build/fiducial_slam /home/tesol/sitl_ws/build/fiducial_slam /home/tesol/sitl_ws/build/fiducial_slam/CMakeFiles/_run_tests_fiducial_slam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/_run_tests_fiducial_slam.dir/depend
