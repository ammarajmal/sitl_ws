# CMake generated Testfile for 
# Source directory: /home/ammar/sitl_ws/src/camera_info_manager_py
# Build directory: /home/ammar/sitl_ws/build/camera_info_manager_py
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_camera_info_manager_py_rostest_tests_load_cpp_camera_info.test "/home/ammar/sitl_ws/build/camera_info_manager_py/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ammar/sitl_ws/build/camera_info_manager_py/test_results/camera_info_manager_py/rostest-tests_load_cpp_camera_info.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/ammar/sitl_ws/src/camera_info_manager_py --package=camera_info_manager_py --results-filename tests_load_cpp_camera_info.xml --results-base-dir \"/home/ammar/sitl_ws/build/camera_info_manager_py/test_results\" /home/ammar/sitl_ws/src/camera_info_manager_py/tests/load_cpp_camera_info.test ")
set_tests_properties(_ctest_camera_info_manager_py_rostest_tests_load_cpp_camera_info.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/ammar/sitl_ws/src/camera_info_manager_py/CMakeLists.txt;24;add_rostest;/home/ammar/sitl_ws/src/camera_info_manager_py/CMakeLists.txt;0;")
add_test(_ctest_camera_info_manager_py_rostest_tests_unit_test.test "/home/ammar/sitl_ws/build/camera_info_manager_py/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ammar/sitl_ws/build/camera_info_manager_py/test_results/camera_info_manager_py/rostest-tests_unit_test.xml" "--return-code" "/usr/bin/python3 /opt/ros/noetic/share/rostest/cmake/../../../bin/rostest --pkgdir=/home/ammar/sitl_ws/src/camera_info_manager_py --package=camera_info_manager_py --results-filename tests_unit_test.xml --results-base-dir \"/home/ammar/sitl_ws/build/camera_info_manager_py/test_results\" /home/ammar/sitl_ws/src/camera_info_manager_py/tests/unit_test.test ")
set_tests_properties(_ctest_camera_info_manager_py_rostest_tests_unit_test.test PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/rostest/cmake/rostest-extras.cmake;52;catkin_run_tests_target;/home/ammar/sitl_ws/src/camera_info_manager_py/CMakeLists.txt;25;add_rostest;/home/ammar/sitl_ws/src/camera_info_manager_py/CMakeLists.txt;0;")
subdirs("gtest")
