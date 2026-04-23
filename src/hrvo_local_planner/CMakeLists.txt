cmake_minimum_required(VERSION 3.0.2)
project(hrvo_local_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pluginlib
  nav_core
  costmap_2d
  geometry_msgs
  nav_msgs
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hrvo_local_planner
  CATKIN_DEPENDS
    roscpp
    pluginlib
    nav_core
    costmap_2d
    geometry_msgs
    nav_msgs
    gazebo_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    angles
)

include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
)

add_library(hrvo_local_planner
  src/hrvo_local_planner.cpp
  src/Agent.cpp
  src/Goal.cpp
  src/KdTree.cpp
  src/Simulator.cpp
  src/Vector2.cpp
)

target_link_libraries(hrvo_local_planner
  ${catkin_LIBRARIES}
)

add_dependencies(hrvo_local_planner
  ${catkin_EXPORTED_TARGETS}
)

install(TARGETS hrvo_local_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
