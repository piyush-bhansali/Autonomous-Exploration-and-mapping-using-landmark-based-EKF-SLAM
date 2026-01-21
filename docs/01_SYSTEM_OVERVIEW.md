# Multi-Robot SLAM and Autonomous Exploration System
## System Overview

**Author:** Piyush Bhansali
**Framework:** ROS2 Jazzy
**Robot Platform:** TurtleBot3 Waffle Pi
**Simulation:** Gazebo Harmonic

---

## Table of Contents

1. [Introduction](#introduction)
2. [System Architecture](#system-architecture)
3. [Key Features](#key-features)
4. [Technology Stack](#technology-stack)
5. [System Components](#system-components)
6. [Data Flow](#data-flow)
7. [Performance Metrics](#performance-metrics)

---

## Introduction

This thesis presents a complete autonomous exploration system for mobile robots that combines:
- **Real-time SLAM** with GPU-accelerated point cloud processing
- **Sensor fusion** using Extended Kalman Filter (EKF)
- **Frontier-based exploration** with intelligent goal selection
- **RRT* path planning** with collision avoidance
- **Adaptive control** using Pure Pursuit controller

The system enables robots to autonomously explore unknown environments, build accurate maps, and navigate safely without human intervention.

---

## System Architecture

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     GAZEBO SIMULATION                            │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐       │
│  │  LiDAR   │  │   IMU    │  │ Odometry │  │  Camera  │       │
│  │ 360° 10Hz│  │  200 Hz  │  │  10 Hz   │  │(optional)│       │
│  └────┬─────┘  └────┬─────┘  └────┬─────┘  └──────────┘       │
└───────┼─────────────┼─────────────┼──────────────────────────────┘
        │             │             │
        │   ┌─────────┴─────────┐   │
        │   │   ROS2 BRIDGE     │   │
        │   │  (gz_ros2_bridge) │   │
        │   └─────────┬─────────┘   │
        │             │             │
        v             v             v
┌─────────────────────────────────────────────────────────────────┐
│                    MAPPING MODULE                                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │        Local Submap Generator (local_submap_generator)   │   │
│  │  ┌─────────┐  ┌─────────┐  ┌──────────────────────────┐ │   │
│  │  │   EKF   │  │Scan-to- │  │    Submap Stitcher       │ │   │
│  │  │ Fusion  │→ │Map ICP  │→ │  - Feature Extraction    │ │   │
│  │  │200 Hz   │  │GPU-Accel│  │  - Loop Closure (GTSAM)  │ │   │
│  │  └─────────┘  └─────────┘  └──────────────────────────┘ │   │
│  └──────────────────────────────────┬───────────────────────┘   │
│                                     │                            │
│                         Global Map (PointCloud2)                 │
│                                     │                            │
└─────────────────────────────────────┼────────────────────────────┘
                                      │
                                      v
┌─────────────────────────────────────────────────────────────────┐
│                  NAVIGATION MODULE                               │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │       Navigation Node (simple_navigation)                │   │
│  │  ┌──────────┐  ┌──────────┐  ┌──────────────────────┐   │   │
│  │  │ Frontier │  │   RRT*   │  │  Pure Pursuit        │   │   │
│  │  │ Detector │→ │  Planner │→ │  Controller          │   │   │
│  │  │  + DBSCAN│  │ Smoothing│  │  + Reactive Avoid    │   │   │
│  │  └──────────┘  └──────────┘  └──────────┬───────────┘   │   │
│  └──────────────────────────────────────────┼───────────────┘   │
└─────────────────────────────────────────────┼───────────────────┘
                                              │
                                              v
                                      Velocity Commands
                                        (Twist msgs)
```

### Module Hierarchy

```
thesis_ws/
├── src/
│   ├── map_generation/           # SLAM & Mapping
│   │   ├── local_submap_generator.py  (Main SLAM node)
│   │   ├── ekf_lib.py                 (EKF sensor fusion)
│   │   ├── submap_stitcher.py         (Map integration)
│   │   ├── feature_extractor.py       (Scan Context features)
│   │   ├── loop_closure_detector.py   (Place recognition)
│   │   ├── gtsam_optimizer.py         (Pose graph optimization)
│   │   └── mapping_utils.py           (ICP, transforms)
│   │
│   ├── navigation/                # Autonomous Exploration
│   │   ├── simple_navigation.py       (Main navigation FSM)
│   │   ├── simple_frontiers.py        (Frontier detection)
│   │   ├── rrt_star.py                (Path planning)
│   │   └── pure_pursuit_controller.py (Path tracking)
│   │
│   └── multi_robot_mapping/       # Launch & Simulation
│       ├── launch/
│       │   └── full_system.launch.py  (Complete system)
│       ├── worlds/
│       │   ├── maze.sdf               (Complex maze world)
│       │   └── park.sdf               (Open environment)
│       └── config/
│           └── tb3_bridge.yaml        (Gazebo-ROS2 bridge)
```

---

## Key Features

### SLAM & Mapping
- ✅ **GPU-Accelerated Processing**: All point cloud operations use Open3D tensor API with CUDA
- ✅ **Sensor Fusion**: EKF combines IMU (200Hz), Odometry (10Hz), and ICP corrections
- ✅ **Submap-Based Mapping**: Memory-efficient incremental map building (80 scans/submap)
- ✅ **Real-time ICP**: Scan-to-map alignment for drift correction every scan
- ✅ **Loop Closure Detection**: Scan Context + RANSAC + ICP verification
- ✅ **Pose Graph Optimization**: GTSAM-based backend for global consistency
- ✅ **Point Cloud Maps**: No occupancy grid conversion - native 3D representation

### Navigation & Exploration
- ✅ **Frontier-Based Exploration**: Automatic detection of unexplored boundaries
- ✅ **DBSCAN Clustering**: Intelligent grouping of frontier points
- ✅ **RRT* Path Planning**: Optimal paths with rewiring and smoothing
- ✅ **Adaptive Pure Pursuit**: Velocity-dependent lookahead with angular smoothing
- ✅ **Reactive Obstacle Avoidance**: LiDAR-based collision prevention
- ✅ **Stuck Detection & Recovery**: Automatic replanning on navigation failure
- ✅ **Dynamic Replanning**: Continuous frontier re-evaluation during execution

### Robustness Features
- ✅ **Thread-Safe Map Updates**: Locks prevent race conditions
- ✅ **Path Deviation Monitoring**: Automatic replanning if robot deviates >1.5m
- ✅ **Explored Frontier Tracking**: Prevents revisiting same locations
- ✅ **Emergency Stop**: Immediate halt if obstacle detected <0.3m
- ✅ **Numerical Stability**: Joseph form EKF covariance updates
- ✅ **Angle Normalization**: Prevents gimbal lock in orientation tracking

---

## Technology Stack

### Core Technologies

| Component | Technology | Version | Purpose |
|-----------|-----------|---------|---------|
| **Middleware** | ROS2 | Jazzy (Iron) | Robot communication framework |
| **Simulator** | Gazebo | Harmonic | Physics-based environment simulation |
| **Point Cloud** | Open3D | 0.18+ | GPU-accelerated 3D processing |
| **Optimization** | GTSAM | 4.2+ | Pose graph backend |
| **Clustering** | scikit-learn | Latest | DBSCAN frontier clustering |
| **Numerical** | NumPy | Latest | Matrix operations |
| **Spatial** | SciPy | Latest | KD-trees, transformations |

### Hardware Requirements

**Minimum:**
- CPU: Intel i5 / AMD Ryzen 5 (4 cores)
- RAM: 8 GB
- GPU: NVIDIA GTX 1060 (6GB VRAM) with CUDA 11.0+
- Storage: 10 GB free space

**Recommended:**
- CPU: Intel i7 / AMD Ryzen 7 (8 cores)
- RAM: 16 GB
- GPU: NVIDIA RTX 3060 (12GB VRAM) with CUDA 12.0+
- Storage: 20 GB SSD

---

## System Components

### 1. Mapping Module (`map_generation`)

**Node:** `local_submap_generator`
**Input Topics:**
- `/{robot_name}/scan` (LaserScan): 2D LiDAR data at 10 Hz
- `/{robot_name}/odom` (Odometry): Wheel encoder poses
- `/{robot_name}/imu` (Imu): Angular velocity measurements

**Output Topics:**
- `/{robot_name}/global_map` (PointCloud2): Accumulated point cloud map
- `/{robot_name}/submap_marker` (MarkerArray): Visualization of submaps

**Key Parameters:**
- `scans_per_submap: 80` - Number of scans before creating new submap
- `voxel_size: 0.05` - Downsampling resolution (5cm)
- `enable_loop_closure: true` - Enable GTSAM optimization
- `enable_scan_to_map_icp: true` - Enable real-time drift correction

### 2. Navigation Module (`navigation`)

**Node:** `simple_navigation`
**Input Topics:**
- `/{robot_name}/global_map` (PointCloud2): Map from SLAM
- `/{robot_name}/odom` (Odometry): Robot pose
- `/{robot_name}/scan` (LaserScan): For reactive avoidance

**Output Topics:**
- `/{robot_name}/cmd_vel` (Twist): Velocity commands
- `/{robot_name}/planned_path` (Path): Current RRT* path
- `/{robot_name}/frontier_markers` (MarkerArray): Detected frontiers

**Key Parameters:**
- `robot_radius: 0.22` - Safety clearance (meters)
- `scan_danger_distance: 0.5` - Obstacle warning threshold
- `min_frontier_distance: 2.0` - Minimum frontier selection distance

### 3. Simulation Module (`multi_robot_mapping`)

**Launch File:** `full_system.launch.py`
**Configurable Options:**
- `world`: maze | park (default: maze)
- `enable_navigation`: true | false (default: true)
- `use_rviz`: true | false (default: true)

**Supported Robots:** 1-2 TurtleBot3 units (configurable via NUM_ROBOTS)

---

## Data Flow

### 1. Sensor Data Acquisition (10 Hz)
```
Gazebo Simulator
    ↓ (gz_ros2_bridge)
LiDAR Scan → local_submap_generator
```

### 2. Pose Estimation (200 Hz prediction, 10 Hz update)
```
IMU (200 Hz) ─────┐
                  ├→ EKF Predict → EKF State
Odometry (10 Hz) ─┘                    ↓
                              EKF Update (fused pose)
```

### 3. Map Building (Every 80 scans ≈ 8 seconds)
```
Fused Pose + LiDAR Scan
    ↓
Transform to World Frame
    ↓
Scan-to-Map ICP (drift correction)
    ↓
Accumulate in Current Submap
    ↓
(if 80 scans reached)
    ↓
Create New Submap → Extract Features → Loop Closure Check
    ↓
Merge into Global Map (GPU-accelerated)
```

### 4. Navigation Loop (10 Hz)
```
Global Map + Robot Pose
    ↓
Detect Frontiers (DBSCAN clustering)
    ↓
Select Best Frontier (distance + heading + size)
    ↓
RRT* Planning (collision-free path)
    ↓
Pure Pursuit Control (adaptive velocity)
    ↓
Reactive Avoidance (if obstacle <0.5m)
    ↓
Publish cmd_vel
```

---

## Performance Metrics

### Mapping Performance

| Metric | Value | Notes |
|--------|-------|-------|
| **Map Update Rate** | 10 Hz | Every LiDAR scan processed |
| **Submap Generation** | 0.1 Hz | Every 8 seconds (80 scans) |
| **ICP Alignment Time** | <50 ms | GPU-accelerated |
| **Loop Closure Detection** | 1 Hz | Continuous background checks |
| **Pose Estimation** | 200 Hz | EKF prediction rate |
| **Map Accuracy** | <5 cm | With loop closure enabled |

### Navigation Performance

| Metric | Value | Notes |
|--------|-------|-------|
| **Control Loop** | 10 Hz | Velocity command rate |
| **Frontier Detection** | 1 Hz | Continuous re-evaluation |
| **RRT* Planning Time** | 0.5-2 s | Depends on environment complexity |
| **Path Smoothness** | 0.3 m spacing | Densified waypoints |
| **Obstacle Response** | <100 ms | Reactive avoidance latency |
| **Goal Reach Tolerance** | 0.3 m | Success threshold |

### Resource Usage

| Resource | Idle | Mapping | Full Exploration |
|----------|------|---------|------------------|
| **CPU** | 10% | 30-40% | 50-60% |
| **GPU** | 5% | 40-50% | 60-70% |
| **RAM** | 1 GB | 2-3 GB | 4-5 GB |
| **VRAM** | 500 MB | 1-2 GB | 2-3 GB |

---

## Quick Start

### Build the System
```bash
cd ~/thesis_ws
colcon build --symlink-install
source install/setup.bash
```

### Launch Full System
```bash
# Complete autonomous exploration (Maze world)
ros2 launch multi_robot_mapping full_system.launch.py

# Open environment
ros2 launch multi_robot_mapping full_system.launch.py world:=park

# Mapping only (no navigation)
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false
```

### Monitor Performance
```bash
# View ROS2 nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor map updates
ros2 topic hz /{robot_name}/global_map

# View robot pose
ros2 topic echo /{robot_name}/odom --once
```

---

## Documentation Structure

This documentation is organized into the following modules:

1. **System Overview** (this document)
2. **SLAM & Mapping Module** - Point cloud mapping, submaps, ICP
3. **EKF Sensor Fusion** - Multi-sensor localization
4. **Loop Closure & Optimization** - GTSAM backend, Scan Context features
5. **Navigation Module** - Frontier detection, RRT*, Pure Pursuit
6. **ROS2 Integration** - Nodes, topics, launch files, parameters

Each document provides in-depth technical details, algorithms, and implementation specifics for that subsystem.

---

## References

- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/
- **Open3D Documentation**: http://www.open3d.org/docs/
- **GTSAM Documentation**: https://gtsam.org/
- **Gazebo Harmonic**: https://gazebosim.org/docs/harmonic/

---

**Next:** Read [02_SLAM_MAPPING.md](02_SLAM_MAPPING.md) for detailed mapping system documentation.
