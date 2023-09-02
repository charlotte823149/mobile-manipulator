cmake_minimum_required(VERSION 3.0.2)
project(human_and_pose)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
 CATKIN_DEPENDS rospy std_msgs

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
 src/human_and_pose_node.py
 src/human_and_hand_new.py 
 DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS  
 src/human_tracking.py
 DESTINATION 
 ${CATKIN_PACKAGE_BIN_DESTINATION}
)
