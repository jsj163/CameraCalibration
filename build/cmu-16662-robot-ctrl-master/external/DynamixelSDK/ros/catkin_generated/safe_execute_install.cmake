execute_process(COMMAND "/home/parallels/Autonomy_ws/hw4/build/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/parallels/Autonomy_ws/hw4/build/cmu-16662-robot-ctrl-master/external/DynamixelSDK/ros/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
