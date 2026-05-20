cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sfm_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(gazebo REQUIRED)

# lightsfm is a header-only dependency in this workspace.
# Resolve it relative to catkin source space instead of hard-coded user paths.
set(LIGHTSFM_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/lightsfm/include")
if(NOT EXISTS "${LIGHTSFM_INCLUDE_DIR}/lightsfm/sfm.hpp")
  message(FATAL_ERROR
    "lightsfm headers not found at ${LIGHTSFM_INCLUDE_DIR}. "
    "Please clone lightsfm into <catkin_ws>/src/lightsfm.")
endif()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${LIGHTSFM_INCLUDE_DIR}
)


link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES PedestrianSFMPlugin
  CATKIN_DEPENDS gazebo_ros roscpp
)


add_library(PedestrianSFMPlugin src/PedestrianSFMPlugin.cpp)
target_link_libraries(PedestrianSFMPlugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES}) #${Boost_LIBRARIES

install(TARGETS
  PedestrianSFMPlugin
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

