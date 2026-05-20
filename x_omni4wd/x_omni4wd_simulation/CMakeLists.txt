cmake_minimum_required(VERSION 3.0.2)
project(x_omni4wd_simulation)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS scripts/cleanup_gazebo_sim.py scripts/unpause_gazebo.py scripts/gazebo_staged_bringup.py scripts/gazebo_health_monitor.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
