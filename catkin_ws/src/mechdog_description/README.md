# mechdog_description

URDF/xacro for MechDog: `base_footprint` → `base_link` → `lidar_link` and fixed leg placeholders (aligned with `docs/01_ARCHITECTURE_ROS_RULES.md`).

## Launch

```bash
# After sourcing the workspace
roslaunch mechdog_description display.launch
```

With Gazebo-specific `<gazebo>` tags in the URDF (`simulation:=true`):

```bash
roslaunch mechdog_description display.launch simulation:=true
```

## Generate URDF (check)

```bash
rosrun xacro xacro $(rospack find mechdog_description)/urdf/mechdog.urdf.xacro simulation:=false
```
