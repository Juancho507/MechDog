# MechDog Simulation Package

## Overview

`mechdog_sim` provides the complete simulation layer for the MechDog quadruped robot project. This package implements a Gazebo-based simulation environment with noise injection and sensor modeling to ensure **Sim-to-Real parity** - meaning the simulation accurately represents real-world conditions.

## Features

- **Gazebo Integration**: Full 3D physics simulation with configurable worlds
- **Noise Injection**: Artificial sensor noise to mimic real-world imperfections
  - LIDAR noise (gaussian, outliers, dropouts)
  - Odometry drift and slippage
  - Actuator delays and response lag
- **Metrics Collection**: Automated collection of performance metrics for validation
- **Sensor Simulation**: Accurate sensor modeling matching hardware specifications
- **ROS 1 Noetic Compatible**: Full compatibility with ROS 1 Noetic

## Package Structure

```
mechdog_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── simulation.yaml         # General simulation parameters
│   ├── noise_injection.yaml    # Noise models for Sim-to-Real
│   └── sensor_params.yaml      # Sensor specifications
├── launch/
│   ├── simulation.launch       # Main launch file (all-in-one)
│   └── gazebo_world.launch     # Gazebo world and robot spawn
├── scripts/
│   ├── noise_injector_node.py      # Injects noise into sensor data
│   ├── sensor_simulator_node.py     # Manages sensor simulation
│   └── metrics_collector_node.py    # Collects performance metrics
└── worlds/
    ├── open_path.world         # Path with obstacles
    └── open_path_world.world   # Simple empty world
```

## Dependencies

### ROS Packages
- `gazebo_ros`
- `gazebo_plugins`
- `robot_state_publisher`
- `joint_state_publisher`
- `tf`
- `mechdog_description`

### Python Dependencies
- `rospy`
- `numpy`

## Quick Start

### 1. Build the Package

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Launch Complete Simulation

```bash
# Launch with all features (GUI, noise injection, metrics)
roslaunch mechdog_sim simulation.launch

# Launch headless (no GUI)
roslaunch mechdog_sim simulation.launch gui:=false

# Launch without noise injection
roslaunch mechdog_sim simulation.launch noise_injection:=false

# Launch with specific world
roslaunch mechdog_sim simulation.launch world:=open_path
```

### 3. Launch Only Gazebo World

```bash
# Basic world
roslaunch mechdog_sim gazebo_world.launch

# Specific world with custom spawn position
roslaunch mechdog_sim gazebo_world.launch world_name:=open_path x:=2.0 y:=1.0
```

## Configuration

### Simulation Parameters

Edit `config/simulation.yaml` to modify:
- World dimensions (path width, length, wall height)
- Obstacle generation (count, shapes, sizes)
- Robot spawn position
- Physics engine settings

### Noise Injection

Edit `config/noise_injection.yaml` to adjust:
- LIDAR noise levels (gaussian stddev, outlier probability)
- Odometry drift parameters
- Ground friction variations
- Actuator delays

### Sensor Parameters

Edit `config/sensor_params.yaml` to configure:
- LIDAR specifications (range, resolution, update rate)
- Odometry covariance matrices
- Topic names and frame IDs

## ROS Topics

### Published by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/gazebo/lidar_clean` | `sensor_msgs/LaserScan` | Clean LIDAR data from Gazebo |
| `/gazebo/odom_clean` | `nav_msgs/Odometry` | Clean odometry from Gazebo |
| `/mechdog/scan` | `sensor_msgs/LaserScan` | Noisy LIDAR data (with noise injection) |
| `/mechdog/odom` | `nav_msgs/Odometry` | Noisy odometry (with drift) |

### Subscribed by Simulation

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands for robot |

## Metrics Collection

Metrics are automatically collected and saved to CSV files when `metrics_collection:=true`.

**Output Directory**: `~/catkin_ws/metrics_output/`

**Collected Metrics**:
- Scan count and quality
- Odometry accuracy
- Emergency stop events
- Total distance traveled
- Path deviations (if global path is available)
- Maximum velocity reached

**Files Generated**:
- `metrics_summary_YYYYMMDD_HHMMSS.csv` - Summary statistics
- `scan_quality_YYYYMMDD_HHMMSS.csv` - Time-series scan quality
- `path_deviations_YYYYMMDD_HHMMSS.csv` - Path tracking accuracy

## Sim-to-Real Strategy

This package implements several strategies to minimize the Sim-to-Real gap:

1. **Sensor Noise Injection**: Gaussian noise, outliers, and dropouts added to sensor readings
2. **Odometry Drift**: Cumulative drift simulating wheel slippage and friction
3. **Actuator Delays**: Command delays and first-order response lag
4. **Ground Friction Variations**: Dynamic friction coefficient changes
5. **Identical Topic Interfaces**: Same topics/messages as real hardware

## Troubleshooting

### Robot Falls Through Ground
- Increase spawn height: `roslaunch mechdog_sim simulation.launch spawn_z:=0.5`
- Check robot mass in URDF

### LIDAR Not Publishing
- Verify plugin is loaded: `rostopic list | grep scan`
- Check Gazebo console for errors: `gz log`

### Noisy Data Not Published
- Ensure noise injection is enabled: `noise_injection:=true`
- Check noise_injector node is running: `rosnode list`

### Poor Performance
- Reduce real-time factor in `simulation.yaml`
- Disable GUI: `gui:=false`
- Reduce LIDAR sample count in `sensor_params.yaml`

## Development

### Adding New Worlds

1. Create new `.world` file in `worlds/` directory
2. Define ground plane, lighting, and obstacles
3. Launch with: `roslaunch mechdog_sim simulation.launch world:=your_world_name`

### Modifying Noise Models

Edit `config/noise_injection.yaml` and adjust parameters. Changes take effect on next launch.

### Custom Metrics

Modify `scripts/metrics_collector_node.py` to add custom metric collection logic.

## Integration with Other Packages

This package is designed to integrate with:
- `mechdog_description` - Robot URDF and visualization
- `mechdog_planning` - Global and local path planning
- `mechdog_control` - Safe learning and control layer
- `mechdog_mapping` - Occupancy grid and SLAM

## Authors

- MechDog Team

## License

MIT License

## References

- ROS 1 Noetic Documentation: http://wiki.ros.org/noetic
- Gazebo Documentation: http://gazebosim.org/
- MechDog Project Repository: [URL]
