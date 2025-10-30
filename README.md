# Multi-Robot SLAM and Navigation System

## Overview

Complete autonomous exploration system for TurtleBot3 robots using 2D LiDAR SLAM with submap stitching, point cloud mapping (no grid conversion), frontier-based exploration, and RRT* path planning.

## System Architecture

```
Gazebo Simulation → Laser Scans/Odometry/IMU → Local Submap Generator → Global Map → Navigation Node → Robot Control
```

## Quick Start

### Build Workspace
```bash
cd /home/piyush/thesis_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Full System
```bash
# Launch with autonomous exploration
ros2 launch multi_robot_mapping full_system.launch.py

# Launch without navigation (manual control)
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# Use different world
ros2 launch multi_robot_mapping full_system.launch.py world:=park
```

## Packages

### 1. map_generation
- **Purpose**: 2D LiDAR SLAM with submap-based mapping
- **Node**: `local_submap_generator`
- **Topics**: Subscribes to `/{robot}/scan`, `/{robot}/odom`, `/{robot}/imu`; Publishes `/{robot}/global_map`

### 2. navigation
- **Purpose**: Autonomous exploration with frontier detection and RRT* planning
- **Node**: `navigation_node`
- **Topics**: Subscribes to `/{robot}/global_map`, `/{robot}/odom`; Publishes `/{robot}/cmd_vel`

### 3. multi_robot_mapping
- **Purpose**: Simulation environment and launch files
- **Worlds**: `maze.sdf`, `park.sdf`

## Key Features

- ✓ Point cloud-based mapping (no grid conversion)
- ✓ GPU-accelerated processing (CUDA)
- ✓ EKF sensor fusion (odometry + IMU)
- ✓ Frontier-based exploration
- ✓ RRT* path planning with collision checking
- ✓ Real-time visualization in RViz

## Parameters

**Mapping** (`scans_per_submap`=250, `voxel_size`=0.05, `enable_loop_closure`=false)

**Navigation** (`robot_radius`=0.22, `control_frequency`=10.0, `max_exploration_time`=3600.0)

## License

Apache 2.0
