# Complete Naming System Documentation
## Multi-Robot SLAM System - Gazebo to ROS Integration

This document provides a comprehensive overview of all topic names, frame names, and namespace conventions used throughout the system.

---

## 1. Robot Namespace Convention

**Robot Name**: `tb3_1` (or `tb3_2`, `tb3_3`, etc. for multiple robots)
- Set in launch file as `robot_name` parameter
- Used as namespace prefix for all robot-specific topics and frames

---

## 2. Gazebo (gz-sim) Internal Configuration

### 2.1 Model Definition (SDF)
**File**: `src/multi_robot_mapping/models/turtlebot3_waffle_pi/model.sdf`

**Model Name** (in SDF): `turtlebot3_waffle_pi`
- Gets renamed to `tb3_1` when spawned via `-name` argument

**Link Names** (non-namespaced in SDF):
- `base_footprint`
- `base_link`
- `wheel_left_link`
- `wheel_right_link`
- `imu_link`
- `base_scan`
- `camera_link`

### 2.2 Sensor Frame IDs (After Substitution)

**LiDAR Sensor**:
```xml
<gz_frame_id>tb3_1/base_scan</gz_frame_id>
```
- Substituted by launch file from `base_scan` → `tb3_1/base_scan`

**IMU Sensor**:
- Topic: `/world/maze_world/model/tb3_1/imu`
- Frame ID: (inherited from link, becomes `tb3_1/imu_link` via URDF)

### 2.3 DiffDrive Plugin Configuration

**Topics** (Gazebo internal, relative to model):
```xml
<topic>cmd_vel</topic>
<odom_topic>odometry</odom_topic>
```

**Actual Gazebo Topics** (with model prefix):
- **Command**: `/model/tb3_1/cmd_vel`
- **Odometry**: `/model/tb3_1/odometry`

**Frame IDs** (After substitution):
```xml
<frame_id>odom</frame_id>
<child_frame_id>tb3_1/base_footprint</child_frame_id>
```
- Substituted by launch file from `base_footprint` → `tb3_1/base_footprint`

**TF Publishing**: DISABLED
```xml
<!-- <tf_topic>/tf</tf_topic> -->
```
- DiffDrive TF is disabled to avoid conflicts with EKF

### 2.4 Joint State Publisher Plugin

**Topic** (Gazebo internal):
```xml
<!-- <topic>joint_states</topic> -->
```
- Commented out, uses default: `/world/maze_world/model/tb3_1/joint_state`

---

## 3. ROS Bridge Mapping (Gazebo ↔ ROS)

**Bridge Config**: `src/multi_robot_mapping/config/tb3_bridge.yaml`

### 3.1 Sensor Data (Gazebo → ROS)

| ROS Topic | Gazebo Topic | Message Type | Direction |
|-----------|--------------|--------------|-----------|
| `/tb3_1/joint_states` | `/world/maze_world/model/tb3_1/joint_state` | `sensor_msgs/msg/JointState` | GZ→ROS |
| `/tb3_1/odom` | `/model/tb3_1/odometry` | `nav_msgs/msg/Odometry` | GZ→ROS |
| `/tb3_1/imu` | `/world/maze_world/model/tb3_1/imu` | `sensor_msgs/msg/Imu` | GZ→ROS |
| `/tb3_1/scan` | `/world/maze_world/model/tb3_1/link/base_scan/sensor/hls_lfcd_lds/scan` | `sensor_msgs/msg/LaserScan` | GZ→ROS |

### 3.2 Control Commands (ROS → Gazebo)

| ROS Topic | Gazebo Topic | Message Type | Direction |
|-----------|--------------|--------------|-----------|
| `/tb3_1/cmd_vel` | `/model/tb3_1/cmd_vel` | `geometry_msgs/msg/Twist` | ROS→GZ |

### 3.3 TF Bridge (DISABLED)

```yaml
# TF bridge DISABLED - Causes conflicts with EKF-based TF publishing
# - ros_topic_name: "/tf"
#   gz_topic_name: "/model/tb3_1/tf"
```

**Why disabled**:
- DiffDrive would publish `odom → base_footprint` (unnamespaced)
- This creates orphaned frames not connected to robot model
- EKF publishes properly namespaced TF instead: `odom → tb3_1/base_footprint`

---

## 4. ROS TF Frame Tree

### 4.1 Frame Hierarchy

```
odom (world frame)
 └─ tb3_1/base_footprint
     └─ tb3_1/base_link
         ├─ tb3_1/wheel_left_link
         ├─ tb3_1/wheel_right_link
         ├─ tb3_1/imu_link
         ├─ tb3_1/base_scan (LiDAR)
         └─ tb3_1/camera_link
             └─ tb3_1/camera_rgb_frame
                 └─ tb3_1/camera_rgb_optical_frame
```

### 4.2 TF Publishers

**robot_state_publisher**:
- Publishes: `tb3_1/base_footprint` → all child links
- Source: URDF with `${namespace}` = `tb3_1/`
- Frequency: Based on `/tb3_1/joint_states` updates

**EKF (local_submap_generator node)**:
- Publishes: `odom` → `tb3_1/base_footprint`
- Source: EKF state estimate (fused IMU + Odom + ICP)
- Frequency: ~50 Hz (matches odometry)
- File: `src/map_generation/map_generation/local_submap_generator.py:187-196`

**Gazebo DiffDrive**:
- TF Publishing: **DISABLED** (via commented `<tf_topic>`)
- Only publishes odometry data via topic

---

## 5. URDF Configuration

**File**: `src/multi_robot_mapping/urdf/turtlebot3_waffle_pi.urdf`

**Namespace Substitution**:
```xml
<link name="${namespace}base_footprint"/>
<link name="${namespace}base_link"/>
<link name="${namespace}wheel_left_link"/>
<!-- etc. -->
```

**Applied in Launch File**:
```python
urdf_content = urdf_content.replace('${namespace}', f'{robot_name}/')
```
- Result: All link names get `tb3_1/` prefix

---

## 6. Launch File Substitutions

**File**: `src/multi_robot_mapping/launch/full_system.launch.py`

### 6.1 SDF Substitutions (Lines 147-161)

```python
# 1. Mesh path substitution
robot_sdf_content.replace('package://...', 'file://...')

# 2. Robot name substitution (unused in current SDF)
robot_sdf_content.replace('{robot_name}', robot_name)

# 3. LiDAR frame ID namespace
robot_sdf_content.replace(
    '<gz_frame_id>base_scan</gz_frame_id>',
    f'<gz_frame_id>{robot_name}/base_scan</gz_frame_id>'
)

# 4. DiffDrive child_frame_id namespace
robot_sdf_content.replace(
    '<child_frame_id>base_footprint</child_frame_id>',
    f'<child_frame_id>{robot_name}/base_footprint</child_frame_id>'
)
```

### 6.2 Bridge Config Substitutions (Lines 184-185)

```python
config_content.replace('{robot_name}', robot_name)  # tb3_1
config_content.replace('{world_name}', 'maze_world')
```

### 6.3 Robot Spawn Arguments (Lines 176-177)

```python
'-name', robot_name  # Renames model from 'turtlebot3_waffle_pi' to 'tb3_1'
```

---

## 7. EKF and Mapping Node Topics

### 7.1 Subscribed Topics

**local_submap_generator node**:
- `/tb3_1/odom` - Raw odometry from Gazebo DiffDrive
- `/tb3_1/imu` - IMU data from Gazebo
- `/tb3_1/scan` - LiDAR scans from Gazebo

### 7.2 Published Topics

**local_submap_generator node**:
- `/tb3_1/ekf_pose` - EKF estimated pose (PoseStamped)
- `/tb3_1/current_submap` - Current submap point cloud
- `/tb3_1/global_map` - Accumulated global map
- `/tf` - Transform: `odom` → `tb3_1/base_footprint`

---

## 8. Navigation Node Topics

### 8.1 Subscribed Topics

**simple_navigation node**:
- `/tb3_1/global_map` - Global map for planning
- `/tb3_1/ekf_pose` - Robot pose for navigation

### 8.2 Published Topics

**simple_navigation node**:
- `/tb3_1/cmd_vel` - Velocity commands to robot

---

## 9. Critical Naming Dependencies

### 9.1 Frame Name Consistency

**Must Match**:
1. DiffDrive `child_frame_id` = `tb3_1/base_footprint`
2. EKF TF `child_frame_id` = `tb3_1/base_footprint`
3. URDF base link = `tb3_1/base_footprint`

**If mismatched**: TF tree breaks, localization fails

### 9.2 Topic Name Consistency

**DiffDrive odom topic**: `/model/tb3_1/odometry` (Gazebo)
**Bridge maps to**: `/tb3_1/odom` (ROS)
**EKF subscribes to**: `/tb3_1/odom` (ROS)

**If mismatched**: EKF doesn't receive odometry, can't initialize

### 9.3 Sensor Frame IDs

**LiDAR**:
- Gazebo: `<gz_frame_id>tb3_1/base_scan</gz_frame_id>`
- ROS scan msg: `frame_id = "tb3_1/base_scan"`
- TF tree: `tb3_1/base_link` → `tb3_1/base_scan`

**If mismatched**: Scan data can't be transformed to robot frame

---

## 10. Multi-Robot Scaling

### 10.1 Current Single Robot

Robot: `tb3_1`
- All topics: `/tb3_1/*`
- All frames: `tb3_1/*`
- World frame: `odom` (shared)

### 10.2 Adding Second Robot

Robot: `tb3_2`
- All topics: `/tb3_2/*`
- All frames: `tb3_2/*`
- World frame: `odom` (shared)

**TF Tree with Multiple Robots**:
```
odom (shared world)
 ├─ tb3_1/base_footprint (robot 1)
 │   └─ tb3_1/base_link → ...
 └─ tb3_2/base_footprint (robot 2)
     └─ tb3_2/base_link → ...
```

---

## 11. Common Issues and Solutions

### Issue 1: "TF frame not found"
**Cause**: Frame name mismatch between publisher and subscriber
**Check**:
- DiffDrive `child_frame_id` in SDF
- EKF TF broadcast `child_frame_id`
- URDF link names
**Solution**: Ensure all use `{robot_name}/base_footprint`

### Issue 2: "No odometry received"
**Cause**: Topic name mismatch
**Check**:
- Gazebo DiffDrive `<odom_topic>`
- Bridge mapping `gz_topic_name`
- EKF subscription topic
**Solution**: Ensure `/model/{robot_name}/odometry` → `/tb3_{robot_name}/odom`

### Issue 3: "Scan data can't be transformed"
**Cause**: LiDAR frame ID not in TF tree
**Check**:
- SDF `<gz_frame_id>` after substitution
- Scan message `frame_id`
- URDF has `{robot_name}/base_scan` link
**Solution**: Ensure launch file substitutes `base_scan` → `{robot_name}/base_scan`

### Issue 4: "Multiple TF publishers conflict"
**Cause**: Both DiffDrive and EKF publishing `odom` → `base_footprint`
**Check**:
- DiffDrive `<tf_topic>` should be commented out
- Bridge TF mapping should be disabled
**Solution**: Only EKF publishes `odom` → `{robot_name}/base_footprint` TF

---

## 12. Debugging Commands

### Check Active Topics
```bash
ros2 topic list | grep tb3_1
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
# Or
ros2 run tf2_ros tf2_echo odom tb3_1/base_footprint
```

### Check Gazebo Topics
```bash
gz topic -l | grep tb3_1
```

### Check Frame IDs in Messages
```bash
ros2 topic echo /tb3_1/scan --once | grep frame_id
ros2 topic echo /tb3_1/odom --once | grep frame_id
```

### Verify Bridge is Running
```bash
ros2 node list | grep bridge
ros2 node info /tb3_1_bridge
```

---

## 13. Reference: Full Topic List

### Gazebo Internal Topics
```
/model/tb3_1/cmd_vel                                   [gz.msgs.Twist]
/model/tb3_1/odometry                                  [gz.msgs.Odometry]
/world/maze_world/model/tb3_1/joint_state             [gz.msgs.Model]
/world/maze_world/model/tb3_1/imu                     [gz.msgs.IMU]
/world/maze_world/model/tb3_1/link/base_scan/sensor/hls_lfcd_lds/scan  [gz.msgs.LaserScan]
```

### ROS Topics
```
/tb3_1/cmd_vel          [geometry_msgs/msg/Twist]      - Navigation commands
/tb3_1/odom             [nav_msgs/msg/Odometry]        - Raw DiffDrive odometry
/tb3_1/joint_states     [sensor_msgs/msg/JointState]   - Wheel joint positions
/tb3_1/imu              [sensor_msgs/msg/Imu]          - IMU measurements
/tb3_1/scan             [sensor_msgs/msg/LaserScan]    - LiDAR scans
/tb3_1/ekf_pose         [geometry_msgs/msg/PoseStamped] - EKF fused estimate
/tb3_1/current_submap   [sensor_msgs/msg/PointCloud2]  - Current submap
/tb3_1/global_map       [sensor_msgs/msg/PointCloud2]  - Global accumulated map
/tf                     [tf2_msgs/msg/TFMessage]       - Transform tree
```

---

## 14. Summary: Namespace Flow

```
SDF Model (no namespace)
    ↓ (launch file substitution)
Gazebo Model (named tb3_1)
    ↓ (gz-sim topics with /model/tb3_1/ prefix)
Gazebo Topics (/model/tb3_1/*)
    ↓ (ros_gz_bridge)
ROS Topics (/tb3_1/*)
    ↓ (used by ROS nodes)
ROS Nodes (subscribe with /tb3_1/ prefix)
```

```
SDF Frames (no namespace)
    ↓ (launch file substitution)
URDF Frames (tb3_1/* prefix)
    ↓ (robot_state_publisher)
TF Tree (tb3_1/base_footprint, etc.)
```

---

**Document Version**: 1.0
**Last Updated**: 2025-11-26
**System**: Multi-Robot SLAM with Gazebo Harmonic + ROS 2 Jazzy
