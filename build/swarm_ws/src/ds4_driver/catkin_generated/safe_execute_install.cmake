execute_process(COMMAND "/home/casper/swarm_ws/build/swarm_ws/src/ds4_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/casper/swarm_ws/build/swarm_ws/src/ds4_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
