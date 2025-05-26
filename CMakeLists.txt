cmake_minimum_required(VERSION 3.0.2)
project(ultrades_ros)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/simple_plant.py
  scripts/control_panel.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
