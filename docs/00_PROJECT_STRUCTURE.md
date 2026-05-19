# MechDog — Proposed ROS Project Structure

## CATKIN WORKSPACE LAYOUT

```
~/catkin_ws/
└── src/
    ├── mechdog_description/          # URDF, meshes, robot definition
    │   ├── urdf/
    │   │   ├── mechdog.urdf.xacro    # Main robot description (SIM + REAL)
    │   │   ├── mechdog_gazebo.xacro  # Gazebo-specific plugins
    │   │   └── materials.xacro       # Visual properties
    │   ├── meshes/
    │   │   ├── base_link.stl
    │   │   ├── leg_*.stl
    │   │   └── lidar.stl
    │   ├── config/
    │   │   └── joint_limits.yaml     # Articulación limits
    │   └── launch/
    │       └── display.launch        # RViz visualization only
    │
    ├── mechdog_sim/                  # SIMULATION LAYER (isolated)
    │   ├── launch/
    │   │   ├── gazebo_pathway.launch           # Main sim entry point
    │   │   ├── pybullet_sim.launch             # Alternative (legacy compat)
    │   │   └── stress_test_sudden_obstacle.launch
    │   ├── worlds/
    │   │   ├── pathway_obstacles_static.world  # Fixed obstacles
    │   │   └── pathway_obstacles_dynamic.world # Moving obstacles
    │   ├── config/
    │   │   └── sim_noise_params.yaml           # Noise injection config
    │   ├── scripts/
    │   │   ├── noise_injection_node.py         # Add realistic noise
    │   │   ├── obstacle_spawner.py             # Procedural obstacle gen
    │   │   └── ground_truth_logger.py          # Log perfect odom/pose
    │   └── CMakeLists.txt / package.xml
    │
    ├── mechdog_hw/                   # HARDWARE LAYER (isolated)
    │   ├── launch/
    │   │   └── hardware_drivers.launch         # All HW drivers
    │   ├── config/
    │   │   ├── hw_calibration.yaml             # Sensor offsets, PID tuning
    │   │   └── motor_config.yaml               # CAN IDs, limits
    │   ├── src/
    │   │   ├── lidar_driver_node.cpp           # Publishes /mechdog/sensor/scan
    │   │   ├── motor_controller_node.cpp       # Subscribes /mechdog/cmd_vel_safe
    │   │   ├── imu_driver_node.cpp             # Publishes /mechdog/imu
    │   │   └── state_estimator_node.cpp        # EKF fusion → /mechdog/odom
    │   ├── include/mechdog_hw/
    │   └── CMakeLists.txt / package.xml
    │
    ├── mechdog_navigation/           # CORE ALGORITHMS (portable)
    │   ├── launch/
    │   │   ├── navigation_stack.launch         # Main CORE entry point (move_base + safe_controller)
    │   │   ├── move_base.launch                # move_base wrapper (opcional, puede estar en navigation_stack)
    │   │   └── rviz_navigation.launch          # Visualization
    │   ├── config/
    │   │   ├── costmap_common_params.yaml      # Footprint, inflation, observation sources
    │   │   ├── global_costmap_params.yaml      # Static map, update freq, plugins
    │   │   ├── local_costmap_params.yaml       # Rolling window, size, resolution
    │   │   ├── base_local_planner_params.yaml  # DWA: velocidades, NO-HOLONÓMICO (vy=0)
    │   │   ├── global_planner_params.yaml      # NavfnROS o GlobalPlanner config
    │   │   ├── move_base_params.yaml           # Controller freq, recovery behaviors
    │   │   ├── safe_controller.yaml            # Seguridad física adicional
    │   │   └── environments/
    │   │       ├── sim_clean.yaml              # No noise (debug)
    │   │       ├── sim_noisy.yaml              # Realistic sim
    │   │       └── real_hw.yaml                # Hardware overrides (brake_decel, thresholds)
    │   ├── src/
    │   │   └── safe_controller_node.py         # Capa adicional: collision polygon + emergency brake
    │   ├── scripts/
    │   │   └── send_goal.py                    # Utility: publicar MoveBaseGoal vía actionlib
    │   ├── rviz/
    │   │   └── navigation.rviz                 # Configuración rviz (costmaps, paths, TF)
    │   └── CMakeLists.txt / package.xml
    │
    ├── mechdog_metrics/              # METRICS & ANALYSIS
    │   ├── launch/
    │   │   └── record_run.launch               # Rosbag recording
    │   ├── scripts/
    │   │   ├── compare_maps.py                 # Sim vs Real map comparison
    │   │   ├── trajectory_deviation.py         # RMSE calculation
    │   │   ├── emergency_frequency.py          # Safety metrics
    │   │   └── compare_runs.py                 # Multi-run statistics
    │   ├── config/
    │   │   └── metrics_config.yaml
    │   └── analysis/
    │       ├── notebooks/
    │       │   └── sim_vs_real_analysis.ipynb
    │       └── templates/
    │           └── report_template.md
    │
    └── mechdog_msgs/                 # CUSTOM MESSAGES (optional)
        ├── msg/
        │   ├── EmergencyStop.msg               # Bool + reason string
        │   └── CollisionPolygon.msg            # For visualization
        ├── srv/
        │   └── ReplanPath.srv                  # Trigger replanning
        └── CMakeLists.txt / package.xml
```

---

## PACKAGE DEPENDENCIES

### mechdog_description
```xml
<depend>urdf</depend>
<depend>xacro</depend>
<depend>robot_state_publisher</depend>
```

### mechdog_sim
```xml
<depend>gazebo_ros</depend>
<depend>gazebo_ros_control</depend>
<depend>mechdog_description</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>tf</depend>
<depend>rospy</depend>
```

### mechdog_hw
```xml
<depend>roscpp</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>hardware_interface</depend>  <!-- Si usa ros_control -->
<depend>controller_manager</depend>
<depend>mechdog_description</depend>
```

### mechdog_navigation (CORE)
```xml
<!-- CRÍTICO: SOLO dependencias estándar de ROS + Navigation Stack -->
<depend>rospy</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>actionlib</depend>
<depend>tf</depend>
<depend>tf2_ros</depend>

<!-- ROS Navigation Stack (battle-tested, portable) -->
<depend>move_base</depend>
<depend>move_base_msgs</depend>
<depend>costmap_2d</depend>
<depend>base_local_planner</depend>  <!-- DWAPlannerROS plugin -->
<depend>navfn</depend>                <!-- Global planner (Dijkstra) -->
<!-- Alternativa: <depend>global_planner</depend> para A* -->

<!-- NO DEBE depender de mechdog_sim ni mechdog_hw -->
```

### mechdog_metrics
```xml
<depend>rospy</depend>
<depend>rosbag</depend>
<depend>nav_msgs</depend>
<depend>cv_bridge</depend>
<depend>matplotlib</depend>  <!-- Para gráficas -->
```

---

## BUILD SYSTEM (CMakeLists.txt Snippets)

### mechdog_navigation/CMakeLists.txt (Python nodes)

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(mechdog_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  actionlib
  tf
  tf2_ros
  move_base
  move_base_msgs
  costmap_2d
  base_local_planner
  navfn
)

catkin_package()

# Install Python scripts (solo safe_controller, planning lo maneja move_base)
catkin_install_python(PROGRAMS
  src/safe_controller_node.py
  scripts/send_goal.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install launch files
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
```

### mechdog_hw/CMakeLists.txt (C++ nodes)

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(mechdog_hw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mechdog_hw
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# LIDAR driver
add_executable(lidar_driver_node src/lidar_driver_node.cpp)
target_link_libraries(lidar_driver_node ${catkin_LIBRARIES})

# Motor controller
add_executable(motor_controller_node src/motor_controller_node.cpp)
target_link_libraries(motor_controller_node ${catkin_LIBRARIES})

# State estimator (EKF)
add_executable(state_estimator_node src/state_estimator_node.cpp)
target_link_libraries(state_estimator_node ${catkin_LIBRARIES})

install(TARGETS lidar_driver_node motor_controller_node state_estimator_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

---

## INITIALIZATION WORKFLOW

### Workspace Setup (First Time)

```bash
# 1. Create catkin workspace
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make  # Inicializa workspace

# 2. Clone/create packages
cd src
# (Aquí irían los git clone de cada paquete, o crear desde cero)

# 3. Install dependencies
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
catkin_make

# 5. Source workspace
source devel/setup.bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

---

## EXECUTION PATTERNS

### Pattern 1: Full Simulation (Training)

```bash
# Terminal 1: Core ROS
roscore

# Terminal 2: Simulation layer
roslaunch mechdog_sim gazebo_pathway.launch \
    world:=pathway_obstacles_dynamic \
    seed:=42 \
    add_noise:=true \
    gui:=true

# Terminal 3: Navigation stack (CORE)
roslaunch mechdog_navigation navigation_stack.launch \
    environment:=sim_noisy \
    goal_x:=10.0 \
    goal_y:=0.0

# Terminal 4: Metrics recording
roslaunch mechdog_metrics record_run.launch \
    output_dir:=~/rosbags/sim_training/run_$(date +%Y%m%d_%H%M%S)

# Terminal 5: Visualization (optional)
roslaunch mechdog_navigation rviz_navigation.launch
```

### Pattern 2: Hardware Real (Deployment)

```bash
# Terminal 1: Core ROS
roscore

# Terminal 2: Hardware drivers
roslaunch mechdog_hw hardware_drivers.launch \
    calibration_file:=$(rospack find mechdog_hw)/config/hw_calibration.yaml

# Terminal 3: Navigation stack (SAME AS SIM!)
roslaunch mechdog_navigation navigation_stack.launch \
    environment:=real_hw \
    goal_x:=10.0 \
    goal_y:=0.0

# Terminal 4: Metrics recording
roslaunch mechdog_metrics record_run.launch \
    output_dir:=~/rosbags/real_hw/run_$(date +%Y%m%d_%H%M%S)
```

**Key insight**: Terminal 3 es IDÉNTICO, solo cambia `environment:=`.

---

## VERSION CONTROL (.gitignore)

```gitignore
# Build artifacts
build/
devel/
install/
.catkin_workspace
*.pyc
__pycache__/

# IDE
.vscode/
.idea/
*.swp

# Logs
*.log
*.bag

# Generated maps
maps/*.pgm
maps/*.yaml

# Rosbags (large files, store externally)
rosbags/

# Metrics outputs
reports/*.png
reports/*.json
```

---

## FILE ORGANIZATION RULES

### Rule 1: No Cross-Package Code Imports

❌ **PROHIBIDO**:
```python
# En mechdog_navigation/src/global_planner_node.py
from mechdog_sim.scripts.obstacle_spawner import get_obstacles  # ¡NO!
```

✅ **CORRECTO**:
```python
# Comunicación solo via topics
import rospy
from nav_msgs.msg import OccupancyGrid

def map_callback(msg):
    obstacles = extract_obstacles_from_map(msg)  # Lógica local
```

### Rule 2: Config Files in Package, Not in ~

❌ **PROHIBIDO**:
```bash
rosparam load ~/my_configs/safe_controller.yaml  # Path absoluto
```

✅ **CORRECTO**:
```xml
<rosparam command="load" file="$(find mechdog_navigation)/config/safe_controller.yaml"/>
```

### Rule 3: Launch Files Cascade (No Duplication)

```xml
<!-- mechdog_navigation/launch/navigation_stack.launch -->
<launch>
  <arg name="environment" default="sim_clean"/>
  
  <!-- Base config -->
  <rosparam command="load" file="$(find mechdog_navigation)/config/global_planner.yaml"/>
  <rosparam command="load" file="$(find mechdog_navigation)/config/local_planner_dwa.yaml"/>
  <rosparam command="load" file="$(find mechdog_navigation)/config/safe_controller.yaml"/>
  
  <!-- Environment overrides -->
  <rosparam command="load" 
            file="$(find mechdog_navigation)/config/environments/$(arg environment).yaml"/>
  
  <!-- Nodes -->
  <node pkg="mechdog_navigation" type="global_planner_node.py" name="global_planner_node"/>
  <node pkg="mechdog_navigation" type="local_planner_dwa_node.py" name="local_planner_dwa_node"/>
  <node pkg="mechdog_navigation" type="safe_controller_node.py" name="safe_controller_node"/>
  <node pkg="mechdog_navigation" type="occupancy_grid_builder_node.py" name="occupancy_grid_builder_node"/>
</launch>
```

**No crear** `navigation_stack_sim.launch` y `navigation_stack_real.launch` duplicados. Usar argumentos.

---

## MIGRATION FROM CURRENT CODEBASE

### Mapping Old → New Structure

| Current File | New Location | Changes Required |
|--------------|--------------|------------------|
| `simulation/maze.py` | `mechdog_sim/worlds/pathway.world` + Gazebo world file | Convert maze generation to Gazebo world format or procedural spawning |
| `simulation/robot.py` | `mechdog_description/urdf/mechdog.urdf.xacro` | Rewrite kinematics + sensors as URDF/xacro |
| `exploration/algorithms.py` | **REEMPLAZADO** por ROS Navigation Stack | Usar `move_base` con plugins `navfn` (global) + `base_local_planner/DWAPlannerROS` (local) |
| `exploration/algorithms.py` → BFS/A* | `mechdog_navigation/config/global_planner_params.yaml` | Configurar NavfnROS (Dijkstra) o GlobalPlanner (A*) via YAML |
| `exploration/algorithms.py` → DWA | `mechdog_navigation/config/base_local_planner_params.yaml` | Configurar DWAPlannerROS: **max_vel_y: 0.0** (no-holonómico) |
| `exploration/occupancy_grid.py` | `costmap_2d` (paquete ROS estándar) | Usar costmaps de move_base, integra sensores automáticamente |
| `exploration/solver.py` | `mechdog_navigation/launch/navigation_stack.launch` | Launch file que orquesta move_base + safe_controller |
| `exploration/visualizer.py` | `mechdog_navigation/launch/rviz_navigation.launch` | Usar RViz con plugins de move_base (costmaps, paths, etc.) |
| `main.py` | *Deprecated* | Replaced by `roslaunch` |
| `visualize.py` | *Deprecated* | Replaced by `roslaunch` |
| `requirements.txt` | `package.xml` dependencies | Convert to rosdep format (agregar move_base, navfn, etc.) |
| `Dockerfile` | Updated with ROS Noetic | Add `ros:noetic-robot` base image, install navigation stack |

### Migration Steps (High-Level)

1. **Create ROS workspace structure** (see above)
2. **Convert simulation layer**:
   - Port `maze.py` to Gazebo world files
   - Create URDF from `robot.py` kinematics
3. **Convert CORE algorithms**:
   - Wrap `algorithms.py` classes as ROS nodes
   - Replace direct method calls with topic pub/sub
4. **Add noise injection layer** (new functionality)
5. **Create hardware abstraction** (placeholder for real hw)
6. **Build metrics package** (new functionality)
7. **Test in simulation** with new ROS architecture
8. **Validate portability** (run same commands on namespaced sim)
9. **Document and freeze** CORE interfaces

---

## DOCKER INTEGRATION (Optional, for Deployment)

### Dockerfile (ROS Noetic + MechDog)

```dockerfile
FROM ros:noetic-robot

# Install dependencies
RUN apt-get update && apt-get install -y \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /catkin_ws/src
COPY . /catkin_ws/src/

# Build
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Entrypoint
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc
CMD ["roslaunch", "mechdog_sim", "gazebo_pathway.launch"]
```

### docker-compose.yml (ROS Multi-Container)

```yaml
version: '3.8'

services:
  roscore:
    image: ros:noetic-robot
    command: roscore
    networks:
      - rosnet

  simulation:
    build: .
    command: roslaunch mechdog_sim gazebo_pathway.launch add_noise:=true
    environment:
      - ROS_MASTER_URI=http://roscore:11311
    networks:
      - rosnet
    depends_on:
      - roscore

  navigation:
    build: .
    command: roslaunch mechdog_navigation navigation_stack.launch environment:=sim_noisy
    environment:
      - ROS_MASTER_URI=http://roscore:11311
    networks:
      - rosnet
    depends_on:
      - roscore
      - simulation

networks:
  rosnet:
```

---

## SUMMARY

**Key principles of this structure**:
1. **Separation of Concerns**: sim/hw/core/metrics in isolated packages
2. **Portability**: mechdog_navigation has ZERO hw/sim dependencies
3. **Configuration over Code**: All tuning via YAML files
4. **Scalability**: Easy to add new planners/sensors as separate nodes
5. **ROS Standard Compliance**: Follows REP-103, REP-105 (coordinate frames)
6. **Sim-to-Real Ready**: Single codebase for both environments

**Migration complexity**: Medium-High (requires ROS knowledge), but guarantees long-term maintainability and hardware portability.
