execute_process(COMMAND "/home/hussainv10/ros_wkspace_asgn4/build/assignment4/urdf_parser_py/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/hussainv10/ros_wkspace_asgn4/build/assignment4/urdf_parser_py/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
