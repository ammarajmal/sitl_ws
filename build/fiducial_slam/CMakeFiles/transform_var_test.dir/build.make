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

# Include any dependencies generated for this target.
include CMakeFiles/transform_var_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/transform_var_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/transform_var_test.dir/flags.make

CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.o: CMakeFiles/transform_var_test.dir/flags.make
CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.o: /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/test/transform_var_test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sitl1/sitl_ws/build/fiducial_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.o -c /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/test/transform_var_test.cpp

CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/test/transform_var_test.cpp > CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.i

CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/test/transform_var_test.cpp -o CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.s

CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.o: CMakeFiles/transform_var_test.dir/flags.make
CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.o: /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/src/transform_with_variance.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sitl1/sitl_ws/build/fiducial_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.o -c /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/src/transform_with_variance.cpp

CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/src/transform_with_variance.cpp > CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.i

CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sitl1/sitl_ws/src/fiducials/fiducial_slam/src/transform_with_variance.cpp -o CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.s

# Object files for target transform_var_test
transform_var_test_OBJECTS = \
"CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.o" \
"CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.o"

# External object files for target transform_var_test
transform_var_test_EXTERNAL_OBJECTS =

/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: CMakeFiles/transform_var_test.dir/test/transform_var_test.cpp.o
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: CMakeFiles/transform_var_test.dir/src/transform_with_variance.cpp.o
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: CMakeFiles/transform_var_test.dir/build.make
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: gtest/lib/libgtest.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libtf2_ros.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libactionlib.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libtf2.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libimage_transport.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libmessage_filters.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libclass_loader.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libPocoFoundation.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libdl.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libroscpp.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libroslib.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/librospack.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libpython3.8.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libcv_bridge.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/librosconsole.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/librostime.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /opt/ros/noetic/lib/libcpp_common.so
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test: CMakeFiles/transform_var_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sitl1/sitl_ws/build/fiducial_slam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable /home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/transform_var_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/transform_var_test.dir/build: /home/sitl1/sitl_ws/devel/.private/fiducial_slam/lib/fiducial_slam/transform_var_test

.PHONY : CMakeFiles/transform_var_test.dir/build

CMakeFiles/transform_var_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/transform_var_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/transform_var_test.dir/clean

CMakeFiles/transform_var_test.dir/depend:
	cd /home/sitl1/sitl_ws/build/fiducial_slam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sitl1/sitl_ws/src/fiducials/fiducial_slam /home/sitl1/sitl_ws/src/fiducials/fiducial_slam /home/sitl1/sitl_ws/build/fiducial_slam /home/sitl1/sitl_ws/build/fiducial_slam /home/sitl1/sitl_ws/build/fiducial_slam/CMakeFiles/transform_var_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/transform_var_test.dir/depend

