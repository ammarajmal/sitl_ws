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
CMAKE_SOURCE_DIR = /home/tesol/sitl_ws/src/vision_msgs

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tesol/sitl_ws/build/vision_msgs

# Include any dependencies generated for this target.
include test/CMakeFiles/vision_msgs_test.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/vision_msgs_test.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/vision_msgs_test.dir/flags.make

test/CMakeFiles/vision_msgs_test.dir/main.cpp.o: test/CMakeFiles/vision_msgs_test.dir/flags.make
test/CMakeFiles/vision_msgs_test.dir/main.cpp.o: /home/tesol/sitl_ws/src/vision_msgs/test/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/tesol/sitl_ws/build/vision_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/vision_msgs_test.dir/main.cpp.o"
	cd /home/tesol/sitl_ws/build/vision_msgs/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vision_msgs_test.dir/main.cpp.o -c /home/tesol/sitl_ws/src/vision_msgs/test/main.cpp

test/CMakeFiles/vision_msgs_test.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vision_msgs_test.dir/main.cpp.i"
	cd /home/tesol/sitl_ws/build/vision_msgs/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/tesol/sitl_ws/src/vision_msgs/test/main.cpp > CMakeFiles/vision_msgs_test.dir/main.cpp.i

test/CMakeFiles/vision_msgs_test.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vision_msgs_test.dir/main.cpp.s"
	cd /home/tesol/sitl_ws/build/vision_msgs/test && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/tesol/sitl_ws/src/vision_msgs/test/main.cpp -o CMakeFiles/vision_msgs_test.dir/main.cpp.s

# Object files for target vision_msgs_test
vision_msgs_test_OBJECTS = \
"CMakeFiles/vision_msgs_test.dir/main.cpp.o"

# External object files for target vision_msgs_test
vision_msgs_test_EXTERNAL_OBJECTS =

/home/tesol/sitl_ws/devel/.private/vision_msgs/lib/vision_msgs/vision_msgs_test: test/CMakeFiles/vision_msgs_test.dir/main.cpp.o
/home/tesol/sitl_ws/devel/.private/vision_msgs/lib/vision_msgs/vision_msgs_test: test/CMakeFiles/vision_msgs_test.dir/build.make
/home/tesol/sitl_ws/devel/.private/vision_msgs/lib/vision_msgs/vision_msgs_test: gtest/lib/libgtest.so
/home/tesol/sitl_ws/devel/.private/vision_msgs/lib/vision_msgs/vision_msgs_test: test/CMakeFiles/vision_msgs_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/tesol/sitl_ws/build/vision_msgs/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/tesol/sitl_ws/devel/.private/vision_msgs/lib/vision_msgs/vision_msgs_test"
	cd /home/tesol/sitl_ws/build/vision_msgs/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vision_msgs_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/vision_msgs_test.dir/build: /home/tesol/sitl_ws/devel/.private/vision_msgs/lib/vision_msgs/vision_msgs_test

.PHONY : test/CMakeFiles/vision_msgs_test.dir/build

test/CMakeFiles/vision_msgs_test.dir/clean:
	cd /home/tesol/sitl_ws/build/vision_msgs/test && $(CMAKE_COMMAND) -P CMakeFiles/vision_msgs_test.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/vision_msgs_test.dir/clean

test/CMakeFiles/vision_msgs_test.dir/depend:
	cd /home/tesol/sitl_ws/build/vision_msgs && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tesol/sitl_ws/src/vision_msgs /home/tesol/sitl_ws/src/vision_msgs/test /home/tesol/sitl_ws/build/vision_msgs /home/tesol/sitl_ws/build/vision_msgs/test /home/tesol/sitl_ws/build/vision_msgs/test/CMakeFiles/vision_msgs_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/vision_msgs_test.dir/depend

