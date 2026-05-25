cmake_minimum_required(VERSION 3.0.2)
project(mechdog_hardware_interface)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
)

catkin_install_python(PROGRAMS
  scripts/mechdog_hardware_interface.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
