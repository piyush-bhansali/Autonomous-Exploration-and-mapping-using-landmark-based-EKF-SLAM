# Multi-Robot Frame Setup Documentation

## Overview

This document explains the coordinate frame structure for the multi-robot mapping system using **per-robot odometry frames** with ICP-based drift correction.

---

## Frame Hierarchy

### **Per-Robot TF Tree**

Each robot has its own independent TF tree (simplified structure):

```
tb3_1/odom → tb3_1/base_link → tb3_1/base_scan
tb3_2/odom → tb3_2/base_link → tb3_2/base_scan
...
```

**Note:** We do NOT use a separate `map` frame because:
- Drift correction happens **within the map data** via ICP (not via TF transforms)
- Navigation uses `odom` frame directly
- Simpler = fewer failure points

### **Frame Definitions**

| Frame | Description | Published By | Update Rate |
|-------|-------------|--------------|-------------|
| `{robot}/odom` | Odometry frame (EKF-corrected: wheel encoders + IMU) | Gazebo bridge | ~100 Hz |
| `{robot}/base_link` | Robot body center | `robot_state_publisher` | ~100 Hz |
| `{robot}/base_scan` | LiDAR sensor position | `robot_state_publisher` | Static |

---

## Drift Correction Strategy

### **Three-Layer Drift Correction (No TF Transform Needed)**

#### **1. EKF Sensor Fusion** (Real-time, Milliseconds)
**Location:** `local_submap_generator.py:54-181`

```python
self.ekf = EKF()  # Fuses odometry + IMU
```

**Corrects:**
- High-frequency sensor noise
- Wheel slip
- Gyroscope drift

**Result:** Smooth, locally-accurate odometry

---

#### **2. Scan-to-Map ICP** (Per Scan, ~0.1 seconds)
**Location:** `local_submap_generator.py:208-291`

```python
def scan_to_map_icp(self, scan_points_world, accumulated_points_world, ...):
    # Aligns current scan to accumulated map
    # Corrects short-term drift (< 2m)
```

**Corrects:**
- Accumulated odometry drift within current submap
- Scan alignment errors

**Result:** Each scan is precisely aligned to the growing map

---

#### **3. Submap-to-Global ICP** (Per Submap, ~6 seconds)
**Location:** `submap_stitcher.py:96-133`

```python
def register_icp_2d(self, source, target, initial_guess):
    # Aligns new submap to global map
    # Corrects medium-term drift (2-10m)
```

**Corrects:**
- Long-term odometry drift between submaps
- Global map alignment

**Result:** Globally consistent map

---

### **Transform Responsibilities**

### **`odom → base_link` Transform**

- **Broadcaster**: Gazebo bridge (via `/tb3_X/odom` topic)
- **Purpose**: Robot pose from wheel odometry + IMU (already EKF-filtered by Gazebo)
- **Update Rate**: ~100 Hz

### **`base_link → base_scan` Transform**

- **Broadcaster**: `robot_state_publisher` (from URDF)
- **Purpose**: Static offset from robot center to LiDAR
- **Offset**: `[-0.064, 0.0, 0.121]` meters

---

## Point Cloud Frame Convention

### **Published Frame: `{robot_name}/odom`**

All point clouds are published in the **odom frame**:

```python
# local_submap_generator.py:133-138
pc2_msg = self.numpy_to_pointcloud2(
    global_points,
    f'{self.robot_name}/odom',  # Odom frame
    self.get_clock().now().to_msg()
)
```

**Why odom frame (not map)?**
- ✅ Navigation uses odom frame directly
- ✅ Drift correction embedded in map data (via ICP)
- ✅ Simpler - no TF broadcasting overhead
- ✅ Points are already drift-corrected via three-layer ICP

**Processing Pipeline:**
1. Scan points transformed to `odom` frame using EKF-corrected poses
2. Scan-to-map ICP corrects short-term drift
3. Submap ICP stitching corrects long-term drift
4. **Result:** Published points in `odom` frame are already drift-corrected

---

## RViz Configuration

### **Per-Robot Visualization**

Each robot has its own RViz configuration file:

| Robot | Config File | Fixed Frame |
|-------|-------------|-------------|
| tb3_1 | `rviz/tb3_1_visualization.rviz` | `tb3_1/odom` |
| tb3_2 | `rviz/tb3_2_visualization.rviz` | `tb3_2/odom` |

### **Launching RViz**

**Option 1: Launch with full system**
```bash
ros2 launch multi_robot_mapping full_system.launch.py
# Automatically launches RViz for tb3_1
```

**Option 2: Launch RViz for specific robot**
```bash
ros2 launch multi_robot_mapping visualize_robot.launch.py robot_name:=tb3_1
ros2 launch multi_robot_mapping visualize_robot.launch.py robot_name:=tb3_2
```

**Option 3: Direct RViz launch**
```bash
ros2 run rviz2 rviz2 -d ~/thesis_ws/src/multi_robot_mapping/rviz/tb3_1_visualization.rviz
```

---

## Multi-Robot Mapping Workflow

### **Phase 1: Independent Mapping (Current)**

1. Each robot builds its own map in its own `{robot}/map` frame
2. Maps are independent (no shared reference frame)
3. Each robot publishes:
   - `/tb3_X/global_map` (PointCloud2 in `tb3_X/map` frame)
   - TF: `tb3_X/map → tb3_X/odom`

### **Phase 2: Multi-Robot Loop Closure (Future)**

When robots meet and detect they're observing the same location:

1. **Detect inter-robot loop closure**
   - Robot 1 scans area A → saves features
   - Robot 2 scans area A → matches features with Robot 1's database

2. **Compute relative transform**
   - `T_tb3_1_to_tb3_2` = transform from tb3_1/map to tb3_2/map

3. **Options for map alignment**:
   - **Option A**: Merge maps into single shared `world` frame
   - **Option B**: Keep separate maps, publish `tb3_1/map ↔ tb3_2/map` transform

---

## Debugging TF Issues

### **Check TF Tree**

```bash
# View complete TF tree (generates PDF)
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo tb3_1/map tb3_1/base_link

# List all frames
ros2 run tf2_ros tf2_monitor
```

### **Common Issues**

| Issue | Cause | Solution |
|-------|-------|----------|
| "Frame [map] does not exist" | RViz using wrong fixed frame | Change Fixed Frame to `tb3_1/map` in RViz |
| Point cloud not visible | Frame mismatch | Check PointCloud2 topic `frame_id` matches RViz Fixed Frame |
| TF timeout errors | `map → odom` not being published | Verify `local_submap_generator` node is running |
| Map drifts with robot | Publishing in wrong frame | Should publish in `map` frame, not `odom` |

---

## Future Enhancements

### **World Frame Integration**

To visualize all robots in a single RViz window:

1. Add common `world` frame at simulation origin
2. Publish `world → tb3_X/map` for each robot
3. Set RViz Fixed Frame to `world`
4. All robots visible in single view

**Implementation**: See `WORLD_FRAME_MIGRATION.md` (to be created)

### **Loop Closure Updates**

When loop closure is implemented, the `map → odom` transform will be updated:

```python
# Pseudocode for loop closure correction
def apply_loop_closure_correction(self, drift_correction):
    # Update map → odom transform to eliminate accumulated drift
    self.map_to_odom_transform = drift_correction @ self.map_to_odom_transform

    # Rebuild global map with corrected poses
    self.stitcher._rebuild_global_map()
```

---

## Summary

| Aspect | Current Implementation |
|--------|----------------------|
| **Frame Strategy** | Per-robot odom frames (`tb3_1/odom`, `tb3_2/odom`) |
| **Point Cloud Frame** | `{robot_name}/odom` (ICP-corrected) |
| **TF Broadcasting** | None needed (drift corrected in map data) |
| **Drift Correction** | Three layers: EKF + Scan ICP + Submap ICP |
| **RViz Config** | Separate config per robot |
| **Multi-Robot** | Independent maps, merge later via loop closure |

This setup supports:
- ✅ Independent mapping per robot
- ✅ Scalable to N robots
- ✅ **Simpler architecture** (no map→odom transform complexity)
- ✅ Drift correction embedded in map data
- ✅ Compatible with existing navigation code

## Why No `map` Frame?

**Traditional ROS2 SLAM uses `map → odom` transform to:**
1. Broadcast drift corrections via TF
2. Allow navigation to use stable `map` frame for goals

**We don't need it because:**
1. ✅ **Drift is corrected in the map itself** via ICP (not via TF)
2. ✅ **Navigation uses `odom` directly** (see simple_navigation.py:77)
3. ✅ **Simpler = fewer bugs** (no 10 Hz TF broadcast overhead)
4. ✅ **Loop closure updates map data**, not TF transform

**When you might need `map` frame in the future:**
- If integrating with Nav2 (requires standard `map → odom → base_link` tree)
- If implementing dynamic drift correction that updates in real-time
- If robots need to share a common global reference frame
