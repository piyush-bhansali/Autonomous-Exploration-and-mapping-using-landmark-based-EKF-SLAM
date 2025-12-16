# TurtleBot3 Waffle Pi & Gazebo Simulation Platform
## Robot Platform and Simulation Environment

**Robot Model:** TurtleBot3 Waffle Pi
**Simulator:** Gazebo Harmonic
**Physics Engine:** Gazebo Physics (ODE-based)
**Files:** `src/multi_robot_mapping/models/turtlebot3_waffle_pi/`

---

## Table of Contents

1. [TurtleBot3 Platform Overview](#turtlebot3-platform-overview)
2. [Robot Specifications](#robot-specifications)
3. [Sensor Suite](#sensor-suite)
4. [Gazebo Simulation](#gazebo-simulation)
5. [Maze World Environment](#maze-world-environment)
6. [Model Files](#model-files)

---

## TurtleBot3 Platform Overview

### What is TurtleBot3?

**TurtleBot3** is an open-source educational and research mobile robot platform developed by ROBOTIS. The **Waffle Pi** variant is the advanced model featuring:
- Differential drive mobility
- 360° LiDAR sensor
- Raspberry Pi 4 computer
- Modular design for sensor expansion

### Why TurtleBot3 Waffle Pi?

| Feature | Benefit for This Thesis |
|---------|------------------------|
| **Differential Drive** | Simple, predictable kinematics for SLAM |
| **2D LiDAR** | High-quality 360° scans for mapping |
| **Compact Size** | Navigates tight spaces in maze environments |
| **ROS2 Native** | Seamless integration with ROS2 Jazzy |
| **Open Source** | Complete access to models and drivers |
| **Well-Documented** | Extensive community support |

### Applications

- SLAM and autonomous navigation research
- Multi-robot coordination experiments
- Education and prototyping
- Algorithm development and testing

---

## Robot Specifications

### Physical Dimensions

```
┌─────────────────────────────────┐
│         TurtleBot3 Waffle Pi    │
│                                 │
│  ┌────── 281 mm ──────┐         │
│  │                     │         │
│  │   ╔═══════════╗     │ 306 mm │
│  │   ║  LiDAR    ║     │         │
│  │   ║   ┌─┐     ║     │         │
│  │   ║   │R│     ║     │         │
│  │   ║   │P│     ║     │         │
│  │   ║   │i│     ║     │         │
│  │   ╚═══════════╝     │         │
│  │   (O)       (O)     │ Height: │
│  └─────────────────────┘ 141 mm  │
│   Wheels    Wheels              │
└─────────────────────────────────┘
```

| Specification | Value | Notes |
|---------------|-------|-------|
| **Length** | 281 mm | Front to back |
| **Width** | 306 mm | Side to side (including wheels) |
| **Height** | 141 mm | Ground to top of LiDAR |
| **Wheelbase** | 287 mm | Distance between wheels |
| **Wheel Diameter** | 66 mm | Rubber tires |
| **Wheel Width** | 18 mm | Contact patch |
| **Ground Clearance** | ~10 mm | Base to ground |

### Mass Properties

| Component | Mass (kg) | Notes |
|-----------|-----------|-------|
| **Base Plate** | 1.373 | Main chassis with electronics |
| **LiDAR Sensor** | 0.114 | HLS-LFCD-LDS unit |
| **Left Wheel** | 0.028 | Motorized wheel assembly |
| **Right Wheel** | 0.028 | Motorized wheel assembly |
| **Caster (Front Left)** | 0.001 | Passive support |
| **Caster (Front Right)** | 0.001 | Passive support |
| **Camera (Optional)** | 0.035 | Raspberry Pi Camera |
| **Total** | **~1.8 kg** | Full assembly |

**Center of Mass:** `[-0.064, 0, 0.048]` meters (slightly rear-biased due to battery)

### Inertia Properties

**Base Link Inertia Tensor** (kg·m²):
```
Ixx = 0.0421    Ixy = 0.0       Ixz = 0.0
Iyx = 0.0       Iyy = 0.0421    Iyz = 0.0
Izx = 0.0       Izy = 0.0       Izz = 0.0753
```

---

## Sensor Suite

### 1. LiDAR Sensor (HLS-LFCD-LDS)

**Model:** Hitachi-LG Data Storage HLS-LFCD-LDS
**Type:** 2D Laser Distance Sensor
**Technology:** Triangulation-based laser ranging

#### Specifications

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Update Rate** | 10 Hz | Scan frequency |
| **Angular Range** | 360° (0° to 360°) | Full horizontal sweep |
| **Angular Resolution** | 1.0° | 360 samples per scan |
| **Range (Min)** | 0.12 m (120 mm) | Minimum detection distance |
| **Range (Max)** | 5.0 m | Maximum detection distance |
| **Range Resolution** | 0.015 m (15 mm) | Distance measurement precision |
| **Measurement Accuracy** | ±1 cm | Gaussian noise (σ = 0.01 m) |

#### Noise Model (Gazebo)

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- 1 cm standard deviation -->
</noise>
```

**Implications:**
- Realistic sensor uncertainty for SLAM testing
- Requires probabilistic mapping approaches
- ICP alignment needed for drift correction

#### Position on Robot

```
Pose: [-0.064, 0, 0.121] meters (x, y, z)
Orientation: [0, 0, 0] radians (roll, pitch, yaw)
```

**Frame:** `base_scan` (LiDAR frame) → `base_link` (robot center)

#### Output Data

**ROS2 Topic:** `/{robot_name}/scan`
**Message Type:** `sensor_msgs/msg/LaserScan`
**Data Structure:**
```
header:
  stamp: current_time
  frame_id: "{robot_name}/base_scan"
angle_min: 0.0        # Start angle (radians)
angle_max: 6.28       # End angle (2π radians)
angle_increment: 0.0175  # ~1 degree
time_increment: 0.0
scan_time: 0.1        # 10 Hz
range_min: 0.12
range_max: 5.0
ranges: [r1, r2, ..., r360]  # 360 distance measurements
intensities: []       # Not used
```

### 2. IMU Sensor (6-Axis)

**Type:** Inertial Measurement Unit
**Configuration:** 3-axis gyroscope + 3-axis accelerometer

#### Specifications

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Update Rate** | 200 Hz | High-frequency measurements |
| **Gyroscope Range** | ±250 °/s | Angular velocity measurement |
| **Accelerometer Range** | ±2g | Linear acceleration |
| **Gyroscope Noise** | σ = 0.0002 rad/s | Gaussian noise per axis |
| **Accelerometer Noise** | σ = 0.017 m/s² | Gaussian noise per axis |

#### Noise Models (Gazebo)

**Angular Velocity (Gyroscope):**
```xml
<angular_velocity>
  <x><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></x>
  <y><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></y>
  <z><noise type="gaussian"><mean>0.0</mean><stddev>2e-4</stddev></noise></z>
</angular_velocity>
```

**Linear Acceleration:**
```xml
<linear_acceleration>
  <x><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></x>
  <y><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></y>
  <z><noise type="gaussian"><mean>0.0</mean><stddev>1.7e-2</stddev></noise></z>
</linear_acceleration>
```

#### Usage in EKF

**Used Components:**
- **Angular Velocity (Z-axis):** For orientation prediction (yaw rate)
- **Linear Acceleration:** Currently not used (relying on odometry + ICP)

**Integration:**
- 200 Hz prediction step in EKF
- Corrected by odometry updates (10 Hz)
- Low drift due to frequent corrections

#### Output Data

**ROS2 Topic:** `/{robot_name}/imu`
**Message Type:** `sensor_msgs/msg/Imu`

### 3. Wheel Odometry

**Type:** Differential Drive Encoder-based Odometry
**Source:** Gazebo Differential Drive Plugin

#### Specifications

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Update Rate** | ~10-50 Hz | Variable (plugin-dependent) |
| **Position Noise (x, y)** | σ = 7.1 cm | Wheel slip and encoder errors |
| **Orientation Noise (θ)** | σ = 8.1° | Differential error accumulation |
| **Encoder Error** | 1.5% | Simulated wheel encoder noise |

#### Noise Model (Gazebo)

```xml
<pose_covariance_diagonal>
  0.005 0.005 0.001 0.001 0.001 0.02
</pose_covariance_diagonal>
<!--
  [x, y, z, roll, pitch, yaw]
  x, y: 0.005 m² → σ = 0.071 m = 7.1 cm
  yaw:  0.02 rad² → σ = 0.141 rad = 8.1°
-->
```

**Why these values?**
- Realistic for differential drive on smooth surfaces
- Accounts for wheel slip, encoder quantization
- Matched to real TurtleBot3 empirical data

#### Output Data

**ROS2 Topic:** `/{robot_name}/odom`
**Message Type:** `nav_msgs/msg/Odometry`
**Frame:** `{robot_name}/odom` → `{robot_name}/base_footprint`

### 4. Camera (Optional)

**Model:** Raspberry Pi Camera Module
**Resolution:** 640×480 @ 30 FPS
**FOV:** 62.2° × 48.8°

**Note:** Not currently used in this thesis (LiDAR-only SLAM).

---

## Gazebo Simulation

### Gazebo Harmonic

**Version:** Gazebo Harmonic (latest stable as of 2024)
**Physics Engine:** ODE (Open Dynamics Engine)
**Rendering:** Ogre2 (GPU-accelerated)

### Physics Configuration

```xml
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>      <!-- 1ms timestep -->
  <real_time_factor>1.0</real_time_factor>  <!-- Real-time simulation -->
</physics>
```

**Simulation Parameters:**
- **Timestep:** 1 ms (1000 Hz physics updates)
- **Real-time Factor:** 1.0 (matches wall-clock time)
- **Gravity:** 9.81 m/s² (Earth standard)

### Plugins Used

| Plugin | Purpose | Configuration |
|--------|---------|---------------|
| **Physics** | Simulate dynamics, collisions | ODE solver |
| **Scene Broadcaster** | Publish visual states | RViz visualization |
| **User Commands** | Interactive model spawning | GUI control |
| **Sensors** | Simulate LiDAR, IMU, Camera | Sensor data generation |
| **Differential Drive** | Motor control | Wheel velocities from cmd_vel |
| **IMU System** | 6-axis IMU data | 200 Hz output |
| **GPU LiDAR** | GPU-accelerated ray casting | 10 Hz, 360 samples |

### Differential Drive Plugin

**Configuration:**
```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>wheel_left_joint</left_joint>
  <right_joint>wheel_right_joint</right_joint>
  <wheel_separation>0.287</wheel_separation>
  <wheel_radius>0.033</wheel_radius>
  <max_linear_acceleration>1.0</max_linear_acceleration>
  <max_angular_acceleration>1.5</max_angular_acceleration>
</plugin>
```

**Features:**
- Converts `Twist` commands to wheel velocities
- Publishes odometry with realistic noise
- Simulates wheel slip and friction

### GPU LiDAR Plugin

**Advantages of GPU Acceleration:**
- Ray casting on GPU → 10-50x faster than CPU
- Real-time 360° scans at 10 Hz
- Accurate collision detection with complex geometry

---

## Maze World Environment

### World Overview

**File:** `src/multi_robot_mapping/worlds/maze.sdf`
**Name:** `maze_world`
**Purpose:** Complex indoor environment for SLAM and navigation testing

### Dimensions

| Feature | Specification |
|---------|--------------|
| **Total Size** | 30m × 30m |
| **Ground Plane** | Flat, 0.8 gray color |
| **Wall Height** | ~2.0 m |
| **Wall Thickness** | ~0.2 m |
| **Corridor Width** | 1.5 - 3.0 m |

### Layout Features

```
┌──────────────────────────────────┐
│  ┌────┐     ┌──────┐     ┌────┐ │
│  │    │     │      │     │    │ │
│  │    └─────┘      └─────┘    │ │
│  │                             │ │
│  │  ┌─────────────────────┐   │ │
│  │  │                     │   │ │
│  └──┘  START (SW corner)  │   │ │
│         Robot Spawn       │   │ │
│         (-12, -12)        │   │ │
│                           │   │ │
│     ┌─────────────────────┘   │ │
│     │                         │ │
│     │   ┌──────────┐  ┌───────┘ │
│     │   │          │  │         │
│     └───┘          └──┘         │
└──────────────────────────────────┘
```

**Complexity:**
- Multiple loops for loop closure testing
- Dead-ends for frontier detection challenges
- Narrow corridors (robot width: 0.306m, corridor: 1.5-3m)
- Open areas for RRT* planning

### Lighting

```xml
<light name="sun" type="directional">
  <pose>0.0 0.0 10.0 0 0 0</pose>
  <cast_shadows>true</cast_shadows>
  <diffuse>1 1 1 1</diffuse>
  <specular>0.2 0.2 0.2 1</specular>
  <direction>-0.5 0.5 -1</direction>
</light>
```

**Properties:**
- Directional (sun-like) lighting from above
- Shadows enabled for realistic sensor data
- Neutral white light (no color bias)

### Materials

**Walls:**
- Collision-enabled boxes
- Gray diffuse material (0.5, 0.5, 0.5)
- Static (non-movable)

**Ground:**
- Friction coefficient: 1.0
- Color: Light gray (0.8, 0.8, 0.8)

### Multi-Robot Spawn Positions

```python
ROBOT_SPAWN_POSES = {
    1: {'x': -12.0, 'y': -12.0, 'z': 0.01, 'yaw': 0.0},      # Southwest
    2: {'x': 12.0, 'y': 12.0, 'z': 0.01, 'yaw': 3.14159},    # Northeast
}
```

**Design Rationale:**
- Opposite corners for maximum separation
- Facing different directions to avoid initial collisions
- Multiple exploration paths available

---

## Model Files

### Directory Structure

```
multi_robot_mapping/
├── models/
│   └── turtlebot3_waffle_pi/
│       ├── model.config          # Gazebo model metadata
│       └── model.sdf             # Robot definition (SDF format)
├── urdf/
│   └── turtlebot3_waffle_pi.urdf # ROS2 robot description
├── meshes/
│   ├── bases/
│   │   └── waffle_pi_base.stl    # Chassis mesh
│   ├── sensors/
│   │   └── lds.stl               # LiDAR mesh
│   └── wheels/
│       ├── left_tire.stl
│       └── right_tire.stl
└── worlds/
    ├── maze.sdf                  # Maze environment
    └── park.sdf                  # Open environment
```

### model.config

```xml
<?xml version="1.0"?>
<model>
  <name>turtlebot3_waffle_pi</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <author>
    <name>ROBOTIS</name>
    <email>support@robotis.com</email>
  </author>
  <description>
    TurtleBot3 Waffle Pi mobile robot platform
  </description>
</model>
```

### model.sdf (Summary)

**Total Lines:** ~500 lines of XML
**Sections:**
1. **Links:** base_footprint, base_link, wheels, casters, sensors
2. **Joints:** Fixed, revolute (wheels), friction constraints
3. **Sensors:** LiDAR (gpu_lidar), IMU (imu), Camera (camera)
4. **Plugins:** DiffDrive, Sensors, IMU, Pose publisher
5. **Visual:** STL meshes with materials
6. **Collision:** Simplified geometries (boxes, cylinders)

### URDF File

**Purpose:** ROS2 robot state publisher, TF tree
**Content:** Transforms from SDF for ROS2 compatibility

---

## Integration with ROS2

### Launch Sequence

```bash
# 1. Start Gazebo with maze world
gz sim maze.sdf

# 2. Spawn TurtleBot3 model
gz service -s /world/maze_world/create --reqtype gz.msgs.EntityFactory

# 3. Bridge Gazebo ↔ ROS2
ros2 run ros_gz_bridge parameter_bridge /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
```

**Automated by:** `full_system.launch.py`

### Topic Mapping

| Gazebo Topic | ROS2 Topic | Type |
|--------------|------------|------|
| `/model/tb3_1/scan` | `/tb3_1/scan` | LaserScan |
| `/model/tb3_1/odometry` | `/tb3_1/odom` | Odometry |
| `/model/tb3_1/imu` | `/tb3_1/imu` | Imu |
| `/tb3_1/cmd_vel` | `/model/tb3_1/cmd_vel` | Twist |

---

## Performance Characteristics

### Mobility

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Max Linear Velocity** | 0.26 m/s | Hardware limit |
| **Max Angular Velocity** | 1.82 rad/s | ~104 °/s |
| **Acceleration (Linear)** | 1.0 m/s² | Configured limit |
| **Acceleration (Angular)** | 1.5 rad/s² | Configured limit |
| **Turning Radius** | 0 (in-place rotation) | Differential drive |

### Obstacle Avoidance

**Minimum Safe Distance:**
```
Robot radius: 0.153 m (half of 0.306m width)
Safety margin: 0.067 m
Total: 0.22 m (robot_radius parameter)
```

### Battery Life (Simulated)

Not modeled in simulation (infinite runtime).
**Real hardware:** ~2 hours continuous operation.

---

## Comparison: Real vs. Simulated

| Aspect | Real TurtleBot3 | Gazebo Simulation |
|--------|----------------|-------------------|
| **Sensor Noise** | Real-world variability | Gaussian models (matched empirically) |
| **Wheel Slip** | Unpredictable surfaces | Modeled friction (μ = 100,000) |
| **Timing** | Variable CPU load | Deterministic 1ms timesteps |
| **Physics** | Complex interactions | Simplified ODE approximations |
| **Advantages (Real)** | True sensor characteristics, real-world challenges |
| **Advantages (Sim)** | Repeatability, safety, rapid iteration, no hardware wear |

**For this thesis:** Simulation provides controlled, reproducible environment ideal for algorithm development and validation.

---

## References

### Official Documentation

- **TurtleBot3 Manual:** https://emanual.robotis.com/docs/en/platform/turtlebot3/
- **Gazebo Documentation:** https://gazebosim.org/docs/harmonic/
- **ROS2 Jazzy:** https://docs.ros.org/en/jazzy/

### Model Files

- **ROBOTIS GitHub:** https://github.com/ROBOTIS-GIT/turtlebot3
- **Gazebo Models:** https://app.gazebosim.org/ROBOTIS/fuel/models/TurtleBot3%20Waffle%20Pi

### Hardware Specifications

- **LiDAR Datasheet:** HLS-LFCD-LDS technical specifications
- **Raspberry Pi 4:** https://www.raspberrypi.com/products/raspberry-pi-4-model-b/

---

**Next:** See [06_ROS2_INTEGRATION.md](06_ROS2_INTEGRATION.md) for complete system integration details.
