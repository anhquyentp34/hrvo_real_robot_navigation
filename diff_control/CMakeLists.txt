cmake_minimum_required(VERSION 3.0.2)
project(diff_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy geometry_msgs std_msgs
)

catkin_install_python(PROGRAMS
  scripts/cmd_vel_gate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
