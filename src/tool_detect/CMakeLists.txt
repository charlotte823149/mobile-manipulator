cmake_minimum_required(VERSION 3.0.2)
project(tool_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_filters
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS 
  rospy 
  std_msgs
  message_filters
  sensor_msgs
  cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
 src/htool_detect.py
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)