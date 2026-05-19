cmake_minimum_required(VERSION 3.0.2)
project(hrvo_local_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  pedsim_msgs
  people_msgs
  pluginlib
  nav_core
  costmap_2d
  geometry_msgs
  nav_msgs
  tf
  gazebo_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  angles
  std_msgs
  visualization_msgs
)

add_message_files(
  FILES
  HRVOAgent.msg
  HRVOInput.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES hrvo_local_planner
  CATKIN_DEPENDS
    message_runtime
    roscpp
    rospy
    pedsim_msgs
    people_msgs
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
    std_msgs
    visualization_msgs
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
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_executable(diff_drive_twist_adapter src/diff_drive_twist_adapter.cpp)
target_link_libraries(diff_drive_twist_adapter ${catkin_LIBRARIES})
add_dependencies(diff_drive_twist_adapter ${catkin_EXPORTED_TARGETS})

install(TARGETS hrvo_local_planner diff_drive_twist_adapter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES hrvo_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY cfg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg
)

catkin_install_python(PROGRAMS
  scripts/hrvo_pedestrian_adapter_node.py
  scripts/hrvo_input_fusion_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
