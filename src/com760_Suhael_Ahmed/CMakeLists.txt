cmake_minimum_required(VERSION 3.0.2)
project(com760_Suhael_Ahmed)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xacro
  robot_state_publisher
)

catkin_package()

catkin_install_python(
  PROGRAMS
    src/scripts/FollowWall.py
    src/scripts/GoToPoint.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

