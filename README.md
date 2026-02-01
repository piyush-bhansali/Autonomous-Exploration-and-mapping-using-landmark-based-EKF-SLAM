# Autonomous Exploration System with SLAM

**Complete autonomous exploration system for TurtleBot3 using GPU-accelerated SLAM, sensor fusion, and intelligent navigation.**

---

## 🚀 Quick Start

```bash
# Build the system
cd ~/thesis_ws
colcon build --symlink-install
source install/setup.bash

# Launch full autonomous exploration (maze world)
ros2 launch autonomous_exploration full_system.launch.py

# Launch with park world
ros2 launch autonomous_exploration full_system.launch.py world:=park

# Mapping only (no navigation)
ros2 launch autonomous_exploration full_system.launch.py enable_navigation:=false
```

---

## 📚 Documentation

**Comprehensive technical documentation is available in `/docs`:**

| Document | Description |
|----------|-------------|
| **[01_SYSTEM_OVERVIEW.md](docs/01_SYSTEM_OVERVIEW.md)** | System architecture, features, performance metrics |
| **[02_SLAM_MAPPING.md](docs/02_SLAM_MAPPING.md)** | GPU-accelerated point cloud mapping, ICP, submaps |
| **[03_EKF_SENSOR_FUSION.md](docs/03_EKF_SENSOR_FUSION.md)** | Extended Kalman Filter, multi-sensor localization |
| **[04_LOOP_CLOSURE_OPTIMIZATION.md](docs/04_LOOP_CLOSURE_OPTIMIZATION.md)** | Scan Context features, GTSAM pose graph optimization |
| **[05_NAVIGATION_MODULE.md](docs/05_NAVIGATION_MODULE.md)** | Frontier detection, RRT* planning, Pure Pursuit control |
| **[06_ROS2_INTEGRATION.md](docs/06_ROS2_INTEGRATION.md)** | Nodes, topics, launch files, configuration |

---

## ✨ Key Features

### SLAM & Mapping
- ✅ **GPU-Accelerated**: Open3D tensor API with CUDA for 4-10x speedup
- ✅ **Submap-Based Mapping**: Memory-efficient incremental map building
- ✅ **Sensor Fusion**: EKF combines IMU (200Hz), Odometry (10Hz), ICP corrections
- ✅ **Real-time ICP**: Scan-to-map drift correction every scan (<50ms)
- ✅ **Loop Closure**: Scan Context + RANSAC + ICP verification
- ✅ **Pose Graph Optimization**: GTSAM backend for global consistency
- ✅ **Point Cloud Maps**: Native 3D representation, no grid conversion

### Navigation & Exploration
- ✅ **Frontier-Based Exploration**: Automatic detection of unexplored boundaries
- ✅ **DBSCAN Clustering**: Intelligent grouping and scoring of frontiers
- ✅ **RRT* Path Planning**: Asymptotically optimal with rewiring
- ✅ **Adaptive Pure Pursuit**: Velocity-dependent lookahead, angular smoothing
- ✅ **Reactive Avoidance**: LiDAR-based collision prevention (<100ms response)
- ✅ **Stuck Detection**: Automatic replanning on navigation failure
- ✅ **Dynamic Replanning**: Continuous goal re-evaluation

---

## 🏗️ System Architecture

```
Gazebo Simulation
    ↓
Sensor Data (LiDAR 10Hz, IMU 200Hz, Odom 10Hz)
    ↓
┌──────────────────────────────────────┐
│  SLAM Module (map_generation)        │
│  - EKF Sensor Fusion                 │
│  - Scan-to-Map ICP (GPU)             │
│  - Submap Stitching                  │
│  - Loop Closure Detection            │
│  - GTSAM Optimization                │
└────────────┬─────────────────────────┘
             │ Point Cloud Map
             ↓
┌──────────────────────────────────────┐
│  Navigation Module (navigation)      │
│  - Frontier Detection (DBSCAN)       │
│  - RRT* Path Planning                │
│  - Pure Pursuit Control              │
│  - Reactive Obstacle Avoidance       │
└────────────┬─────────────────────────┘
             │ Velocity Commands
             ↓
         Robot Actuators
```

---

## 📦 Packages

### 1. `map_generation` - SLAM & Mapping

**Node:** `local_submap_generator`

**Subscribed Topics:**
- `/{robot}/scan` (LaserScan): 2D LiDAR data
- `/{robot}/odom` (Odometry): Wheel encoder poses
- `/{robot}/imu` (Imu): Angular velocity

**Published Topics:**
- `/{robot}/global_map` (PointCloud2): Accumulated map
- `/tf`: odom → base_footprint transform

**Key Files:**
- `local_submap_generator.py` - Main SLAM node
- `ekf_lib.py` - Extended Kalman Filter
- `submap_stitcher.py` - Map integration
- `feature_extractor.py` - Scan Context features
- `loop_closure_detector.py` - Place recognition
- `gtsam_optimizer.py` - Pose graph optimization

### 2. `navigation` - Autonomous Exploration

**Node:** `simple_navigation`

**Subscribed Topics:**
- `/{robot}/global_map` (PointCloud2): Map from SLAM
- `/{robot}/odom` (Odometry): Robot pose
- `/{robot}/scan` (LaserScan): Obstacle detection

**Published Topics:**
- `/{robot}/cmd_vel` (Twist): Velocity commands
- `/{robot}/planned_path` (Path): Current trajectory
- `/{robot}/frontier_markers` (MarkerArray): Exploration goals

**Key Files:**
- `simple_navigation.py` - Main navigation FSM
- `simple_frontiers.py` - Frontier detection
- `rrt_star.py` - Path planning
- `pure_pursuit_controller.py` - Path tracking controller

### 3. `autonomous_exploration` - System Integration

**Launch File:** `full_system.launch.py`

**Features:**
- Gazebo simulation (maze/park worlds)
- Robot spawning with sensors
- ROS2-Gazebo bridge configuration
- Node orchestration with timed startup
- RViz visualization

---

## ⚙️ Parameters

### Mapping Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `scans_per_submap` | 80 | Scans before creating new submap (~8s) |
| `voxel_size` | 0.05 | Downsampling resolution (5cm) |
| `enable_loop_closure` | true | Enable GTSAM optimization |

### Navigation Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_radius` | 0.22 | Safety clearance (meters) |
| `scan_danger_distance` | 0.5 | Obstacle warning threshold |
| `min_frontier_distance` | 2.0 | Minimum frontier selection distance |
| `replan_score_threshold` | 0.5 | Trigger replan if 50% better frontier found |

---

## 📊 Performance

### Mapping Performance

| Metric | Value |
|--------|-------|
| Map Update Rate | 10 Hz |
| Submap Generation | 0.1 Hz (every 8s) |
| ICP Alignment Time | <50 ms (GPU) |
| Pose Estimation | 200 Hz (EKF) |
| Map Accuracy | <5 cm (with loop closure) |

### Navigation Performance

| Metric | Value |
|--------|-------|
| Control Loop | 10 Hz |
| Frontier Detection | 1 Hz |
| RRT* Planning Time | 0.5-2s |
| Obstacle Response | <100 ms |
| Exploration Efficiency | 85-90% coverage |

---

## 💻 System Requirements

**Minimum:**
- Ubuntu 22.04 or 24.04
- ROS2 Jazzy
- NVIDIA GPU with CUDA 11.0+ (GTX 1060 or better)
- 8 GB RAM
- 4-core CPU

**Recommended:**
- Ubuntu 24.04
- ROS2 Jazzy
- NVIDIA RTX 3060 or better
- 16 GB RAM
- 8-core CPU

---

## 🛠️ Installation

### 1. Install Dependencies

```bash
# ROS2 Jazzy (if not installed)
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install required packages
sudo apt install -y \
  ros-jazzy-gazebo-ros-pkgs \
  ros-jazzy-ros-gz \
  python3-pip \
  python3-colcon-common-extensions

# Python dependencies
pip3 install numpy scipy scikit-learn open3d gtsam
```

### 2. Build Workspace

```bash
cd ~/thesis_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Verify GPU

```bash
# Check CUDA availability
nvidia-smi

# Test Open3D CUDA
python3 src/map_generation/verify_gpu.py
```

---

## 🎯 Usage Examples

### Basic Autonomous Exploration

```bash
ros2 launch autonomous_exploration full_system.launch.py
```

### Mapping Without Navigation

```bash
ros2 launch autonomous_exploration full_system.launch.py enable_navigation:=false

# Control robot manually
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/tb3_1/cmd_vel
```

### Monitor System

```bash
# View topics
ros2 topic list

# Check map update rate
ros2 topic hz /tb3_1/global_map

# View robot pose
ros2 topic echo /tb3_1/odom --once

# Check node status
ros2 node info /local_submap_generator
```

---

## 📝 Code Statistics

- **Total Lines:** ~4500+ lines of Python
- **SLAM Module:** ~2200 lines (9 files)
- **Navigation Module:** ~1400 lines (4 files)
- **Integration:** ~900 lines (launch files, config)

---

## 🔧 Troubleshooting

### GPU Not Detected

```bash
# Check CUDA installation
nvcc --version

# Reinstall Open3D with CUDA
pip3 install --upgrade open3d
```

### Map Drift

```bash
# Enable loop closure for long-term consistency
# In full_system.launch.py, set:
enable_loop_closure: true
```

### Navigation Stuck

```bash
# The system has automatic stuck detection and recovery
# Check logs for replanning events:
ros2 run rqt_console rqt_console
```

---

## 📄 License

Apache 2.0

---

## 👤 Author

**Piyush Bhansali**
Thesis Project - Multi-Robot SLAM and Autonomous Exploration
ROS2 Jazzy | Gazebo Harmonic | Open3D | GTSAM

---

## 🙏 Acknowledgments

- ROS2 and Gazebo communities
- Open3D for GPU-accelerated point cloud processing
- GTSAM for pose graph optimization
- TurtleBot3 platform

---

**For detailed technical information, see the documentation in `/docs`.**
