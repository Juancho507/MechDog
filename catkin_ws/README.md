# MechDog Catkin workspace (ROS 1 Noetic)

This workspace lives inside the MechDog repository for version control.

## Build

```bash
cd catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## Packages

| Package | Role |
|---------|------|
| `mechdog_description` | URDF/xacro, RViz, joint limits placeholder |

## Quick check (model + TF)

```bash
roslaunch mechdog_description display.launch
```

Optional Gazebo material tags in URDF:

```bash
roslaunch mechdog_description display.launch simulation:=true
```
