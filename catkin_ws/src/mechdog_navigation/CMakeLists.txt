cmake_minimum_required(VERSION 3.0.2)
project(mechdog_navigation)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin and required packages (NO gazebo or simulation packages)
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  tf2_ros
  move_base_msgs
  actionlib
  actionlib_msgs
  visualization_msgs
)

## System dependencies
find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS
    rospy
    roscpp
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    tf
    tf2_ros
    move_base_msgs
    actionlib
    actionlib_msgs
    visualization_msgs
)

###########
## Build ##
###########

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Install Python scripts (navigation nodes)
catkin_install_python(PROGRAMS
  scripts/global_planner_node.py
  scripts/local_planner_node.py
  scripts/safe_learning_node.py
  scripts/occupancy_grid_node.py
  scripts/navigation_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

## Install Python module (for shared utilities)
catkin_python_setup()

#############
## Testing ##
#############

# Unit tests can be added here
# catkin_add_nosetests(test)
