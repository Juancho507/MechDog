cmake_minimum_required(VERSION 3.0.2)
project(mechdog_description)

find_package(catkin REQUIRED COMPONENTS
  urdf
  xacro
  robot_state_publisher
)

catkin_package()

catkin_install_python(PROGRAMS
  # No Python programs in this package
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)