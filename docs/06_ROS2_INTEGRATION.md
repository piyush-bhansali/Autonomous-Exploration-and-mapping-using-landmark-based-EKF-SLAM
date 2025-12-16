# ROS2 System Integration
## Nodes, Topics, Launch Files, and Configuration

---

## System Nodes

### Mapping Node: `local_submap_generator`

**Package:** `map_generation`
**Executable:** `local_submap_generator`

**Subscribed Topics:**
- `/{robot}/scan` (LaserScan, 10 Hz)
- `/{robot}/odom` (Odometry, 10 Hz)
- `/{robot}/imu` (Imu, 200 Hz)

**Published Topics:**
- `/{robot}/global_map` (PointCloud2, 0.1 Hz)
- `/tf` (odom → base_footprint, 200 Hz)

**Parameters:**
```yaml
robot_name: 'tb3_1'
scans_per_submap: 80
voxel_size: 0.05
enable_loop_closure: true
enable_scan_to_map_icp: true
```

### Navigation Node: `simple_navigation`

**Package:** `navigation`
**Executable:** `simple_navigation`

**Subscribed Topics:**
- `/{robot}/global_map` (PointCloud2)
- `/{robot}/odom` (Odometry, 10 Hz)
- `/{robot}/scan` (LaserScan, 10 Hz)

**Published Topics:**
- `/{robot}/cmd_vel` (Twist, 10 Hz)
- `/{robot}/planned_path` (Path)
- `/{robot}/frontier_markers` (MarkerArray)

**Parameters:**
```yaml
robot_name: 'tb3_1'
robot_radius: 0.22
enable_reactive_avoidance: true
scan_danger_distance: 0.5
min_frontier_distance: 2.0
```

---

## Launch System

### Full System Launch

**File:** `full_system.launch.py`
**Package:** `multi_robot_mapping`

```bash
ros2 launch multi_robot_mapping full_system.launch.py \
  world:=maze \
  enable_navigation:=true \
  use_rviz:=true
```

**What it launches:**
1. Gazebo simulation (maze or park world)
2. Robot spawner (TurtleBot3 with sensors)
3. ROS2-Gazebo bridge (sensor data topics)
4. local_submap_generator node
5. simple_navigation node (if enabled)
6. RViz visualization (if enabled)

**Configuration:**
- `NUM_ROBOTS = 1` (line 22) - Change to 2 for multi-robot
- Spawn poses: `ROBOT_SPAWN_POSES` dict (lines 25-28)
- Timer delays for sequential startup

---

## Topic Communication

### Data Flow

```
Gazebo Sensors
    ↓ (gz_ros2_bridge)
/{robot}/scan, /{robot}/odom, /{robot}/imu
    ↓
local_submap_generator
    ↓
/{robot}/global_map
    ↓
simple_navigation
    ↓
/{robot}/cmd_vel
    ↓ (gz_ros2_bridge)
Gazebo Robot Actuators
```

### QoS Profiles

| Topic | QoS | Reason |
|-------|-----|--------|
| `/scan` | RELIABLE | Ensure all scans received |
| `/odom` | RELIABLE | Critical for localization |
| `/imu` | BEST_EFFORT | High frequency, tolerate loss |
| `/cmd_vel` | RELIABLE | Ensure commands delivered |

---

## Configuration Files

### Bridge Configuration

**File:** `config/tb3_bridge.yaml`

Maps Gazebo topics to ROS2:
```yaml
- ros_topic_name: "/tb3_1/scan"
  gz_topic_name: "/world/maze_world/model/tb3_1/link/base_scan/sensor/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
```

### RViz Configuration

**File:** `config/rviz_config.rviz`

Displays:
- PointCloud2 (global map)
- LaserScan (current scan)
- Path (planned trajectory)
- MarkerArray (frontiers)
- TF frames

---

## Build & Run

```bash
# Build
cd ~/thesis_ws
colcon build --symlink-install
source install/setup.bash

# Run full system
ros2 launch multi_robot_mapping full_system.launch.py

# Run individual nodes
ros2 run map_generation local_submap_generator --ros-args -p robot_name:=tb3_1
ros2 run navigation simple_navigation --ros-args -p robot_name:=tb3_1

# Monitor
ros2 topic list
ros2 topic hz /{robot}/global_map
ros2 node info /local_submap_generator
```

---

## Troubleshooting

**Issue:** Nodes not starting
```bash
# Check if Gazebo is running
gz sim -l

# Check bridge topics
ros2 topic list | grep tb3

# View node logs
ros2 run rqt_console rqt_console
```

**Issue:** No map published
```bash
# Check if scans received
ros2 topic hz /tb3_1/scan

# Check EKF initialization
ros2 topic echo /tb3_1/odom --once
```

---

## Documentation Index

- **[01_SYSTEM_OVERVIEW.md](01_SYSTEM_OVERVIEW.md)** - Architecture, features, quick start
- **[02_SLAM_MAPPING.md](02_SLAM_MAPPING.md)** - Submap mapping, ICP, GPU acceleration
- **[03_EKF_SENSOR_FUSION.md](03_EKF_SENSOR_FUSION.md)** - Multi-sensor localization
- **[04_LOOP_CLOSURE_OPTIMIZATION.md](04_LOOP_CLOSURE_OPTIMIZATION.md)** - GTSAM backend
- **[05_NAVIGATION_MODULE.md](05_NAVIGATION_MODULE.md)** - Exploration, RRT*, control
- **[06_ROS2_INTEGRATION.md](06_ROS2_INTEGRATION.md)** - This document
