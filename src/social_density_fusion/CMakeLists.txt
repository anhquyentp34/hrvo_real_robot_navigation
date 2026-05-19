cmake_minimum_required(VERSION 3.0.2)
project(social_density_fusion)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_msgs
  visualization_msgs
  social_msgs
  tf2_ros
  message_generation
)

add_message_files(
  FILES
  DensityInfo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs tf2_ros
)

catkin_install_python(PROGRAMS
  scripts/social_density_fusion_node.py
  scripts/lidar_map_polygon_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
