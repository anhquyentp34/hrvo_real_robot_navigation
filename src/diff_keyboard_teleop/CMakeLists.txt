cmake_minimum_required(VERSION 3.0.2)
project(diff_keyboard_teleop)

find_package(catkin REQUIRED)
catkin_package()

catkin_install_python(
  PROGRAMS
    scripts/diff_keyboard_teleop.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
