# MechDog Navigation Package (CORE)

## Overview

`mechdog_navigation` is the **100% portable** navigation stack for the MechDog quadruped robot. This package is **completely hardware-agnostic** and works identically on both simulated and real hardware, enabling true **Sim-to-Real** transfer without code modification.

**Key Features:**
- Zero simulation dependencies (no Gazebo, no PyBullet)
- Identical behavior on simulated and physical robots
- Modular architecture with pluggable components
- Advanced safety system with dynamic collision avoidance
- Real-time occupancy grid mapping
- Multiple planning algorithms (BFS, A*)
- Dynamic Window Approach (DWA) local planning
- State machine-based navigation manager

## Architecture

```
mechdog_navigation/
├── Global Planner (discrete path planning: BFS, A*)
├── Local Planner (continuous trajectory: DWA)
├── Safe Learning (active safety with emergency braking)
├── Occupancy Grid (real-time mapping from sensors)
└── Navigation Manager (state machine coordinator)
```

### Component Interaction

```
Sensor Data (/scan, /odom)
    ↓
Occupancy Grid Mapper → /mechdog/map
    ↓
Global Planner → /mechdog/global_plan
    ↓
Local Planner (DWA) → /cmd_vel_raw
    ↓
Safe Learning (Safety Filter) → /cmd_vel (to robot)
    ↑
Navigation Manager (coordinates all components)
```

## Package Structure

```
mechdog_navigation/
├── CMakeLists.txt
├── package.xml                   # NO simulation dependencies
├── setup.py
├── README.md
├── config/
│   ├── navigation.yaml           # General navigation config
│   ├── global_planner.yaml       # BFS/A* parameters
│   ├── local_planner.yaml        # DWA parameters
│   ├── safe_learning.yaml        # Safety system config
│   ├── occupancy_grid.yaml       # Mapping parameters
│   └── environments/
│       ├── simulation.yaml       # Sim-specific overrides
│       └── real_hardware.yaml    # Hardware-specific overrides
├── launch/
│   ├── navigation.launch         # Complete stack
│   ├── mapping.launch            # Mapping only
│   ├── planning.launch           # Planners only
│   └── safe_learning.launch      # Safety only
├── scripts/
│   ├── global_planner_node.py
│   ├── local_planner_node.py
│   ├── safe_learning_node.py
│   ├── occupancy_grid_node.py
│   └── navigation_manager_node.py
└── src/
    └── mechdog_navigation/       # Python module
        └── __init__.py
```

## Dependencies

### ROS Packages (All Portable)
- `rospy` - ROS Python client
- `sensor_msgs` - Sensor message types
- `nav_msgs` - Navigation message types
- `geometry_msgs` - Geometry primitives
- `tf` - Transform library
- `actionlib` - Action server/client

### Python Dependencies
- `numpy` - Numerical computations
- `scipy` - Scientific computing (optional)

**NO Dependencies On:**
- ❌ Gazebo
- ❌ PyBullet
- ❌ mechdog_sim
- ❌ Any simulation-specific packages

## Quick Start

### 1. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Launch Complete Navigation Stack

**In Simulation:**
```bash
# First, launch simulation (in separate terminal)
roslaunch mechdog_sim simulation.launch

# Then launch navigation
roslaunch mechdog_navigation navigation.launch environment:=simulation
```

**On Real Hardware:**
```bash
# First, launch hardware drivers (in separate terminal)
roslaunch mechdog_hardware robot.launch

# Then launch navigation (EXACT SAME COMMAND, different environment)
roslaunch mechdog_navigation navigation.launch environment:=real_hardware
```

### 3. Send Navigation Goal

```bash
# Publish goal via command line
rostopic pub /mechdog/goal geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: 5.0
    y: 3.0
    z: 0.0
  orientation:
    w: 1.0"
```

Or use RViz:
```bash
rviz
# Add -> By topic -> /mechdog/global_plan -> Path
# Add -> By topic -> /mechdog/local_plan -> Path
# Add -> By topic -> /mechdog/map -> Map
# Use "2D Nav Goal" tool to set goal interactively
```

## ROS Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mechdog/scan` | `sensor_msgs/LaserScan` | LIDAR scan data |
| `/mechdog/odom` | `nav_msgs/Odometry` | Odometry (position, velocity) |
| `/mechdog/goal` | `geometry_msgs/PoseStamped` | Navigation goal |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/mechdog/map` | `nav_msgs/OccupancyGrid` | Real-time occupancy grid |
| `/mechdog/global_plan` | `nav_msgs/Path` | Global path (discrete) |
| `/mechdog/local_plan` | `nav_msgs/Path` | Local trajectory (continuous) |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (filtered by safety) |
| `/mechdog/emergency_stop` | `std_msgs/Bool` | Emergency stop status |
| `/mechdog/safety_status` | `std_msgs/String` | Safety system status |
| `/mechdog/navigation_status` | `std_msgs/String` | Navigation state machine status |

## Component Details

### 1. Global Planner

**Algorithms:**
- **BFS** (Breadth-First Search): Complete search, finds shortest path
- **A\*** (A-star): Heuristic search (Manhattan/Euclidean), faster

**Configuration:** `config/global_planner.yaml`

**Key Parameters:**
- `algorithm`: "bfs" or "astar"
- `cell_size`: Grid resolution (meters)
- `heuristic_weight`: A* heuristic weight (1.0 = optimal)
- `inflation_radius`: Safety margin around obstacles

**Example:**
```bash
roslaunch mechdog_navigation planning.launch enable_local:=false
```

### 2. Local Planner (DWA)

**Algorithm:** Dynamic Window Approach (DWA)

**Features:**
- Continuous trajectory generation
- Velocity sampling in dynamic window
- Multi-objective scoring (path tracking, goal distance, obstacle avoidance)
- Real-time collision checking

**Configuration:** `config/local_planner.yaml`

**Key Parameters:**
- `max_vel_x`: Maximum linear velocity (m/s)
- `max_vel_theta`: Maximum angular velocity (rad/s)
- `sim_time`: Trajectory simulation time (seconds)
- `scoring/path_distance_bias`: Weight for path tracking
- `scoring/goal_distance_bias`: Weight for goal approach
- `scoring/occdist_scale`: Weight for obstacle avoidance

### 3. Safe Learning (Safety Layer)

**Features:**
- **Dynamic Safety Polygon:** Expands with velocity
- **Braking Distance Calculation:** Physics-based stopping distance
- **Emergency Stop:** Immediate halt on critical proximity
- **Velocity Scaling:** Gradual slowdown near obstacles
- **Dead-End Detection:** Triggers recovery behaviors
- **Spin Recovery:** Rotates to find free space

**Configuration:** `config/safe_learning.yaml`

**Key Parameters:**
- `critical_distance`: Emergency stop threshold (meters)
- `warning_distance`: Begin slowing down (meters)
- `max_deceleration`: Maximum safe braking (m/s²)
- `safety_factor`: Braking distance multiplier

**Safety Guarantee:**
```
Required braking distance = v² / (2a) + v*t_reaction
Safety margin = braking_distance * safety_factor
```

### 4. Occupancy Grid Mapper

**Algorithm:** Bayesian log-odds update

**Features:**
- Real-time mapping from LIDAR
- Bresenham ray tracing
- Probabilistic occupancy update
- Configurable inflation
- Dynamic map updates

**Configuration:** `config/occupancy_grid.yaml`

**Key Parameters:**
- `map/width`, `map/height`: Map dimensions (meters)
- `map/resolution`: Cell size (meters)
- `update/hit_probability`: P(occupied | ray hit)
- `update/miss_probability`: P(free | ray pass through)

### 5. Navigation Manager

**State Machine:**
- `IDLE`: Waiting for goal
- `PLANNING`: Computing global path
- `MOVING`: Executing path
- `RECOVERY`: Stuck, trying recovery
- `ERROR`: Fatal error
- `GOAL_REACHED`: Success

**Configuration:** `config/navigation.yaml`

## Environment-Specific Configuration

The navigation stack supports environment-specific overrides without changing core logic.

### Simulation Environment
File: `config/environments/simulation.yaml`

**Characteristics:**
- Higher velocity limits (safe in sim)
- More aggressive planning
- Reduced safety margins
- Debug visualization enabled

### Real Hardware Environment
File: `config/environments/real_hardware.yaml`

**Characteristics:**
- Conservative velocity limits
- Larger safety margins
- Inertia compensation
- Battery monitoring
- Temperature tracking

**Usage:**
```bash
# Simulation
roslaunch mechdog_navigation navigation.launch environment:=simulation

# Real hardware (SAME COMMAND, different env)
roslaunch mechdog_navigation navigation.launch environment:=real_hardware
```

## Sim-to-Real Strategy

This package achieves Sim-to-Real transfer through:

1. **Topic Abstraction:** All sensor/actuator access via ROS topics
2. **Zero Simulation Dependencies:** No Gazebo/PyBullet imports
3. **Environment Overrides:** Separate configs for sim vs real
4. **Identical Code Paths:** Same nodes run in both environments
5. **Hardware-Agnostic Interfaces:** Standard ROS message types

**Result:** Deploy to real robot without changing a single line of code, only switch launch argument.

## Troubleshooting

### Robot Not Moving

**Check:**
```bash
# Is navigation running?
rosnode list | grep mechdog

# Are topics active?
rostopic hz /mechdog/scan
rostopic hz /mechdog/odom
rostopic hz /cmd_vel

# Is safety blocking?
rostopic echo /mechdog/safety_status
rostopic echo /mechdog/emergency_stop
```

### No Global Path

**Check:**
```bash
# Is map being published?
rostopic echo /mechdog/map --noarr

# Is goal received?
rostopic echo /mechdog/goal

# Check planner logs
rosnode info /global_planner
```

### Emergency Stops

**Check:**
```bash
# Safety status
rostopic echo /mechdog/safety_status

# Minimum obstacle distance (should see LIDAR readings)
rostopic echo /mechdog/scan | grep ranges

# Reduce velocity or increase safety margins in config
```

### Path Not Executing

**Check:**
```bash
# Is local planner running?
rosnode list | grep local_planner

# Is global path published?
rostopic echo /mechdog/global_plan

# Check local planner is receiving odometry
rostopic echo /mechdog/odom
```

## Performance Tuning

### For Speed
- Increase `max_vel_x` in `local_planner.yaml`
- Reduce `sim_time` (shorter lookahead)
- Reduce `vx_samples`, `vth_samples` (faster computation)

### For Safety
- Increase `critical_distance` in `safe_learning.yaml`
- Increase `safety_factor` (larger braking margin)
- Increase `inflation_radius` in `global_planner.yaml`

### For Accuracy
- Decrease `map/resolution` (finer grid)
- Increase `sim_time` (longer lookahead)
- Increase scoring weights for path tracking

## Advanced Usage

### Custom Recovery Behaviors

Edit `config/navigation.yaml`:
```yaml
recovery:
  enabled: true
  behaviors:
    - name: "clear_costmap"
      type: "clear_costmap"
    - name: "rotate_recovery"
      type: "rotate"
      duration: 3.0
    - name: "backup"
      type: "backup"
      distance: 0.5
```

### Dynamic Reconfiguration

Parameters can be updated at runtime:
```bash
# Increase max velocity
rosparam set /local_planner/velocity_limits/max_vel_x 1.5

# Reduce safety margin (use with caution!)
rosparam set /safe_learning/braking/critical_distance 0.10
```

### Logging and Metrics

Enable detailed logging:
```yaml
navigation:
  logging:
    level: "debug"
    log_path_plans: true
    log_recovery_events: true
```

Metrics are automatically tracked:
- Path length
- Execution time
- Recovery count
- Distance traveled

## Integration with Other Packages

```
mechdog_navigation (CORE - portable)
    ↑ provides navigation
    |
    ├─→ mechdog_sim (simulation environment)
    |
    └─→ mechdog_hardware (real robot drivers)
```

**Clean Separation:**
- `mechdog_navigation` has NO dependencies on sim or hardware
- `mechdog_sim` depends on mechdog_navigation (provides sensor topics)
- `mechdog_hardware` depends on mechdog_navigation (provides sensor topics)

## Authors

MechDog Team

## License

MIT License

## References

- ROS Navigation Stack: http://wiki.ros.org/navigation
- Dynamic Window Approach: Fox et al., 1997
- A* Pathfinding: Hart et al., 1968
- Occupancy Grid Mapping: Moravec & Elfes, 1985
