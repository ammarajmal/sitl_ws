execute_process(COMMAND "/home/sitl1/sitl_ws/build/aruco_detect/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sitl1/sitl_ws/build/aruco_detect/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
