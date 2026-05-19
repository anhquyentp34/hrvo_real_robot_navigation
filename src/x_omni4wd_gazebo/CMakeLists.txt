cmake_minimum_required(VERSION 3.0.2)
project(x_omni4wd_gazebo)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  sensor_msgs
  gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf sensor_msgs gazebo_ros
    INCLUDE_DIRS include
    LIBRARIES x_omni_force_based_move
)

###########
## Build ##
###########
add_library(x_omni_force_based_move src/x_omni_force_based_move.cpp)
target_link_libraries(x_omni_force_based_move ${Boost_LIBRARIES} ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  x_omni_force_based_move
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/x_omni4wd_controller.py
  scripts/x_omni4wd_kinematics.py
  scripts/x_omni4wd_odom_pub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/gazebo_icr2026_env.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install config files
install(FILES
  config/x_omni4wd_control.yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

