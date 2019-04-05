# Install script for directory: /home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/yunfei/Projects/robotAutonomy/CameraCalibration/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/msg" TYPE FILE FILES
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/AX.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/EX.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MX.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MX2.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MX2Ext.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/MXExt.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/PRO.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/PROExt.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/RX.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XH.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XL.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XL320.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XM.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/XMExt.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelState.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelStateList.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelInfo.msg"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/msg/DynamixelLoadInfo.msg"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/srv" TYPE FILE FILES
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/JointCommand.srv"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/WheelCommand.srv"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/GetDynamixelInfo.srv"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/srv/DynamixelCommand.srv"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/cmake" TYPE FILE FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/build/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgs-msg-paths.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/devel/include/dynamixel_workbench_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/devel/share/roseus/ros/dynamixel_workbench_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/devel/share/common-lisp/ros/dynamixel_workbench_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/devel/share/gennodejs/ros/dynamixel_workbench_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/home/yunfei/bin/python" -m compileall "/home/yunfei/Projects/robotAutonomy/CameraCalibration/devel/lib/python2.7/dist-packages/dynamixel_workbench_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/devel/lib/python2.7/dist-packages/dynamixel_workbench_msgs")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/build/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgs.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/cmake" TYPE FILE FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/build/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgs-msg-extras.cmake")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs/cmake" TYPE FILE FILES
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/build/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgsConfig.cmake"
    "/home/yunfei/Projects/robotAutonomy/CameraCalibration/build/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/catkin_generated/installspace/dynamixel_workbench_msgsConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/dynamixel_workbench_msgs" TYPE FILE FILES "/home/yunfei/Projects/robotAutonomy/CameraCalibration/src/cmu-16662-robot-ctrl-master/external/dynamixel-workbench-msgs/dynamixel_workbench_msgs/package.xml")
endif()

