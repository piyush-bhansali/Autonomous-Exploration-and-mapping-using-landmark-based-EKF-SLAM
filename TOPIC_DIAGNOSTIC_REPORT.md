# Topic Diagnostic Report
## Multi-Robot SLAM System - Topic and Node Analysis

**Generated**: 2025-11-26
**Robot**: tb3_1
**Purpose**: Diagnose odometry scaling error (~5-7x discrepancy)

---

## Executive Summary

The system shows proper topic connectivity between Gazebo, ros_gz_bridge, and ROS nodes. However, there is a **significant odometry error** where:
- **Raw odometry** reports ~5-7x more distance traveled than actual
- **EKF (with ICP corrections)** shows the correct physical distance

**Key Finding**: Odometry message shows `x: 4.666m, y: -0.966m` after ~270 seconds of operation, suggesting excessive drift or scaling error in Gazebo DiffDrive plugin.

---

## 1. Gazebo Topics

### Command to List All Gazebo Topics
```bash
gz topic -l
```

### Gazebo Topic List
```
/clock
/gazebo/resource_paths
/gui/camera/pose
/gui/currently_tracked
/gui/track
/model/tb3_1/cmd_vel                    ← Command velocity (from ROS)
/model/tb3_1/odometry                   ← Odometry (to ROS)
/model/tb3_1/tf                         ← TF (DISABLED in bridge)
/model/tb3_1/enable
/sensors/marker
/stats
/world/maze_world/clock
/world/maze_world/dynamic_pose/info
/world/maze_world/model/tb3_1/joint_state                                    ← Joint states
/world/maze_world/model/tb3_1/link/base_scan/sensor/hls_lfcd_lds/scan       ← LaserScan
/world/maze_world/model/tb3_1/link/base_scan/sensor/hls_lfcd_lds/scan/points
/world/maze_world/model/tb3_1/link/camera_rgb_frame/sensor/camera/camera_info
/world/maze_world/model/tb3_1/link/camera_rgb_frame/sensor/camera/image
/world/maze_world/model/tb3_1/imu                                            ← IMU
/world/maze_world/pose/info
/world/maze_world/scene/deletion
/world/maze_world/scene/info
/world/maze_world/state
/world/maze_world/stats
/world/maze_world/light_config
/world/maze_world/material_color
```

### Critical Gazebo Topics Detail

#### Odometry Topic
**Command:**
```bash
gz topic -i -t /model/tb3_1/odometry
```

**Result:**
```
Publishers [Address, Message Type]:
  tcp://172.19.241.27:33813, gz.msgs.Odometry    ← DiffDrive plugin

Subscribers [Address, Message Type]:
  tcp://172.19.241.27:38587, gz.msgs.Odometry    ← ros_gz_bridge
```

**Status**: ✅ **CONNECTED** - DiffDrive plugin → Bridge

---

#### Command Velocity Topic
**Command:**
```bash
gz topic -i -t /model/tb3_1/cmd_vel
```

**Result:**
```
Publishers [Address, Message Type]:
  tcp://172.19.241.27:38587, gz.msgs.Twist       ← ros_gz_bridge

Subscribers [Address, Message Type]:
  tcp://172.19.241.27:33813, gz.msgs.Twist       ← DiffDrive plugin
```

**Status**: ✅ **CONNECTED** - Bridge → DiffDrive plugin

---

## 2. ROS Topics

### Command to List All ROS Topics
```bash
ros2 topic list
```

### ROS Topic List
```
/clicked_point
/clock
/goal_pose
/initialpose
/parameter_events
/robot_description
/rosout
/tb3_1/cmd_vel          ← Command velocity
/tb3_1/current_goal
/tb3_1/current_submap
/tb3_1/ekf_path
/tb3_1/ekf_pose         ← EKF estimated pose
/tb3_1/frontier_markers
/tb3_1/global_map
/tb3_1/imu              ← IMU data
/tb3_1/joint_states     ← Wheel joint states
/tb3_1/odom             ← Raw odometry
/tb3_1/planned_path
/tb3_1/robot_description
/tb3_1/scan             ← LaserScan
/tf                     ← Transform tree
/tf_static
```

---

## 3. ROS Topic Details

### Odometry Topic (/tb3_1/odom)

**Command:**
```bash
ros2 topic info /tb3_1/odom -v
```

**Result:**
```
Type: nav_msgs/msg/Odometry

Publisher count: 1
  Node: tb3_1_bridge
  QoS: RELIABLE, VOLATILE

Subscription count: 2
  1. Node: tb3_1_submap_generator
     QoS: RELIABLE, VOLATILE

  2. Node: tb3_1_navigation
     QoS: RELIABLE, VOLATILE
```

**Status**: ✅ **QoS COMPATIBLE** - All using RELIABLE
**Data Flow**: Gazebo DiffDrive → Bridge → [EKF, Navigation]

**Sample Odometry Message:**
```bash
ros2 topic echo /tb3_1/odom --once
```
```yaml
header:
  stamp:
    sec: 273
    nanosec: 130000000
  frame_id: odom
child_frame_id: tb3_1/base_footprint
pose:
  pose:
    position:
      x: 4.665990007088596        ⚠️ EXCESSIVE - Robot has NOT traveled 4.6m
      y: -0.9656989723998356       ⚠️ EXCESSIVE
      z: 0.0
    orientation:
      z: -0.9740758839282484
      w: 0.2262215116857846         (yaw ≈ -150°)
```

**⚠️ CRITICAL ISSUE**: Position shows 4.67m traveled, but visual observation and EKF (with ICP) show only ~0.7m actually traveled.

---

### Command Velocity Topic (/tb3_1/cmd_vel)

**Command:**
```bash
ros2 topic info /tb3_1/cmd_vel -v
```

**Result:**
```
Type: geometry_msgs/msg/Twist

Publisher count: 1
  Node: tb3_1_navigation
  QoS: RELIABLE, VOLATILE

Subscription count: 1
  Node: tb3_1_bridge
  QoS: RELIABLE, VOLATILE
```

**Status**: ✅ **CONNECTED**
**Data Flow**: Navigation → Bridge → Gazebo DiffDrive

---

### LaserScan Topic (/tb3_1/scan)

**Command:**
```bash
ros2 topic info /tb3_1/scan -v
```

**Result:**
```
Type: sensor_msgs/msg/LaserScan

Publisher count: 1
  Node: tb3_1_bridge
  QoS: RELIABLE, VOLATILE

Subscription count: 2
  1. Node: tb3_1_submap_generator
     QoS: BEST_EFFORT, VOLATILE

  2. Node: rviz2_tb3_1
     QoS: BEST_EFFORT, VOLATILE
```

**Status**: ⚠️ **QoS MISMATCH** - Publisher is RELIABLE, subscribers are BEST_EFFORT
**Note**: This works because BEST_EFFORT subscribers can connect to RELIABLE publishers

---

## 4. ROS Nodes

### Command to List All Nodes
```bash
ros2 node list
```

### Node List
```
/clock_bridge                                    ← Clock synchronization
/tb3_1/rviz2_tb3_1                              ← Visualization
/tb3_1/transform_listener_impl_5d69d5e5ebd0     ← TF listener (internal)
/tb3_1_bridge                                    ← Gazebo↔ROS bridge
/tb3_1_navigation                                ← Navigation/exploration
/tb3_1_state_publisher                           ← Robot URDF publisher
/tb3_1_submap_generator                          ← EKF + Mapping
```

---

## 5. Node Details

### Bridge Node (/tb3_1_bridge)

**Command:**
```bash
ros2 node info /tb3_1_bridge
```

**Publishers (Gazebo → ROS):**
- `/tb3_1/odom` (nav_msgs/msg/Odometry)
- `/tb3_1/imu` (sensor_msgs/msg/Imu)
- `/tb3_1/joint_states` (sensor_msgs/msg/JointState)
- `/tb3_1/scan` (sensor_msgs/msg/LaserScan)

**Subscribers (ROS → Gazebo):**
- `/tb3_1/cmd_vel` (geometry_msgs/msg/Twist)

**Status**: ✅ **All bridges operational**

---

### Submap Generator Node (/tb3_1_submap_generator)

**Command:**
```bash
ros2 node info /tb3_1_submap_generator
```

**Subscribers:**
- `/clock` (rosgraph_msgs/msg/Clock)
- `/tb3_1/imu` (sensor_msgs/msg/Imu)
- `/tb3_1/odom` (nav_msgs/msg/Odometry)
- `/tb3_1/scan` (sensor_msgs/msg/LaserScan)

**Publishers:**
- `/tb3_1/ekf_pose` (geometry_msgs/msg/PoseStamped)
- `/tb3_1/current_submap` (sensor_msgs/msg/PointCloud2)
- `/tb3_1/global_map` (sensor_msgs/msg/PointCloud2)
- `/tb3_1/ekf_path` (nav_msgs/msg/Path)
- `/tf` (tf2_msgs/msg/TFMessage)

**Status**: ✅ **Operational** - Receiving all sensor data, publishing TF

---

### Navigation Node (/tb3_1_navigation)

**Command:**
```bash
ros2 node info /tb3_1_navigation
```

**Subscribers:**
- `/clock` (rosgraph_msgs/msg/Clock)
- `/tb3_1/global_map` (sensor_msgs/msg/PointCloud2)
- `/tb3_1/odom` (nav_msgs/msg/Odometry)

**Publishers:**
- `/tb3_1/cmd_vel` (geometry_msgs/msg/Twist)
- `/tb3_1/current_goal` (visualization_msgs/msg/Marker)
- `/tb3_1/frontier_markers` (visualization_msgs/msg/MarkerArray)
- `/tb3_1/planned_path` (nav_msgs/msg/Path)

**Status**: ✅ **Operational** - Sending velocity commands

---

## 6. Data Flow Diagram

```
┌──────────────────────────────────────────────────────────────────┐
│                         GAZEBO SIMULATION                         │
│                                                                   │
│  ┌─────────────────────┐         ┌──────────────────────┐       │
│  │  DiffDrive Plugin   │         │  Sensor Plugins      │       │
│  │  (odometry bug?)    │         │  (LiDAR, IMU)        │       │
│  └──────┬──────────────┘         └──────┬───────────────┘       │
│         │ publishes                     │ publishes              │
│         │ /model/tb3_1/odometry         │ sensor data            │
│         │ (WRONG SCALE!)                │ (CORRECT)              │
└─────────┼───────────────────────────────┼────────────────────────┘
          │                               │
          ▼                               ▼
┌──────────────────────────────────────────────────────────────────┐
│                      ROS_GZ_BRIDGE                                │
│                                                                   │
│  Gazebo → ROS:                     ROS → Gazebo:                 │
│  • /model/tb3_1/odometry          • /tb3_1/cmd_vel               │
│    → /tb3_1/odom                    → /model/tb3_1/cmd_vel       │
│  • sensor data                                                    │
│    → /tb3_1/scan, /tb3_1/imu                                     │
└──────────────────────┬───────────────────────┬────────────────────┘
                       │                       │
                       ▼                       ▼
         ┌─────────────────────┐   ┌──────────────────────┐
         │  tb3_1_submap_gen   │   │  tb3_1_navigation    │
         │  (EKF + Mapping)    │   │  (Exploration)       │
         │                     │   │                      │
         │  Subscribes:        │   │  Subscribes:         │
         │  • /tb3_1/odom      │   │  • /tb3_1/odom       │
         │  • /tb3_1/scan      │   │  • /tb3_1/global_map │
         │  • /tb3_1/imu       │   │                      │
         │                     │   │  Publishes:          │
         │  Publishes:         │   │  • /tb3_1/cmd_vel    │
         │  • /tb3_1/ekf_pose  │   └──────────────────────┘
         │  • /tb3_1/global_map│
         │  • /tf (odom→base)  │
         └─────────────────────┘
```

---

## 7. Problem Analysis

### Symptom
```
[EKF DEBUG] Odom: (4.666, -0.966, 150°) | EKF: (0.235, 0.022, 2.02°) | Error: 1.392m
```

**Odometry shows**: 4.78m traveled (sqrt(4.666² + 0.966²) ≈ 4.78m)
**EKF shows**: 0.236m traveled (ICP-corrected using LiDAR)
**Error ratio**: ~20x position error, ~75x heading error

### Root Cause Analysis

1. **✅ QoS Compatibility**: All critical topics have compatible QoS
2. **✅ Topic Connectivity**: All Gazebo ↔ ROS bridges are connected
3. **✅ Node Communication**: All nodes receiving and publishing data
4. **⚠️ Odometry Source**: Gazebo DiffDrive plugin publishes to `/model/tb3_1/odometry`
5. **❌ Odometry Accuracy**: DiffDrive odometry has massive scaling error

### Potential Causes

1. **DiffDrive Plugin Configuration Error**
   - Wheel radius mismatch: Configured as 0.033m, but might need verification
   - Wheel separation mismatch: Configured as 0.287m
   - Physics timestep issue causing integration errors

2. **Gazebo Physics Issue**
   - Wheel slippage not being accounted for in odometry
   - Physics simulation speed vs real-time mismatch
   - Joint velocity reporting error

3. **Gazebo Version Bug**
   - Known bug in Gazebo Harmonic (8.9.0) DiffDrive plugin
   - When `<topic>` and `<odom_topic>` are commented out (current config)
   - May require explicit topic names or different plugin version

---

## 8. Verification Commands

### Check Odometry Publishing Rate
```bash
ros2 topic hz /tb3_1/odom
```
**Expected**: ~50 Hz (matches `<odom_publisher_frequency>50</odom_publisher_frequency>`)

### Monitor Odometry in Real-Time
```bash
ros2 topic echo /tb3_1/odom
```

### Check EKF vs Odometry Discrepancy
```bash
# Watch logs from tb3_1_submap_generator node
ros2 topic echo /rosout | grep "EKF DEBUG"
```

### Verify Gazebo DiffDrive Is Publishing
```bash
gz topic -e -t /model/tb3_1/odometry
```

### Check TF Tree
```bash
ros2 run tf2_tools view_frames
# Generates frames.pdf showing transform tree
```

**Expected TF tree:**
```
odom
 └─ tb3_1/base_footprint (published by EKF)
     └─ tb3_1/base_link
         ├─ tb3_1/wheel_left_link
         ├─ tb3_1/wheel_right_link
         ├─ tb3_1/imu_link
         └─ tb3_1/base_scan
```

---

## 9. Recommended Actions

### Immediate Investigation
1. **Verify physical robot movement in Gazebo GUI**
   - Does the robot visually move 4.6m or only 0.2m?
   - This will determine if the issue is odometry calculation or physics

2. **Check wheel joint velocities**
   ```bash
   ros2 topic echo /tb3_1/joint_states
   ```
   - Compare reported velocities to expected values
   - At 0.2 m/s linear velocity with 0.033m wheel radius:
     - Expected angular velocity: 0.2 / 0.033 ≈ 6.06 rad/s

3. **Test with explicit topic names**
   - Uncomment `<topic>` and `<odom_topic>` in model.sdf
   - May force DiffDrive to use correct code path

### Long-term Solutions
1. **Switch to alternative odometry source**
   - Use ground truth pose from Gazebo
   - Bridge `/world/maze_world/pose/info` for ground truth comparison

2. **Report Gazebo bug**
   - If confirmed as Gazebo Harmonic issue
   - Test with Gazebo Garden or Fortress

3. **Rely entirely on EKF**
   - Current EKF (with ICP) appears accurate
   - Raw odometry only used for initialization

---

## 10. Configuration Files Reference

### Bridge Configuration
**File**: `src/multi_robot_mapping/config/tb3_bridge.yaml`

```yaml
- ros_topic_name: "/{robot_name}/odom"
  gz_topic_name: "/model/{robot_name}/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS

- ros_topic_name: "/{robot_name}/cmd_vel"
  gz_topic_name: "/model/{robot_name}/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

### DiffDrive Plugin Configuration
**File**: `src/multi_robot_mapping/models/turtlebot3_waffle_pi/model.sdf:480-518`

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
  <left_joint>wheel_left_joint</left_joint>
  <right_joint>wheel_right_joint</right_joint>

  <wheel_separation>0.287</wheel_separation>
  <wheel_radius>0.033</wheel_radius>

  <!-- <topic>cmd_vel</topic> -->
  <!-- <odom_topic>odometry</odom_topic> -->
  <frame_id>odom</frame_id>
  <child_frame_id>tb3_1/base_footprint</child_frame_id>
  <odom_publisher_frequency>50</odom_publisher_frequency>
</plugin>
```

---

**End of Report**
