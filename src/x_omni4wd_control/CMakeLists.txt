cmake_minimum_required(VERSION 3.0.2)
project(x_omni4wd_control)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
  controller_manager_msgs
  controller_manager
  joint_state_controller
  velocity_controllers
  x_omni4wd_description
  gazebo_ros
  gazebo_ros_control
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES x_omni4wd_control
  CATKIN_DEPENDS
    roscpp
    geometry_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    tf2_geometry_msgs
    controller_manager_msgs
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ executable
add_executable(xdrive_kinematics_node
  src/xdrive_kinematics_node.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(xdrive_kinematics_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_dependencies(xdrive_kinematics_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(TARGETS xdrive_kinematics_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

# Install Python scripts (share; legacy)
install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE)

catkin_install_python(PROGRAMS
  scripts/cmd_vel_to_omni_wheels.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

