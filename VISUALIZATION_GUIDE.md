# Global Map Visualization Guide

This guide explains how to visualize **only the stitched global map** without scans or submaps.

---

## Option 1: Live RViz Visualization (Real-time)

### Method A: Launch System Without Default RViz, Then Launch Clean Visualization

**Terminal 1: Launch main system**
```bash
cd ~/thesis_ws
source install/setup.bash

# For exploration mode:
ros2 launch multi_robot_mapping full_system.launch.py use_rviz:=false

# OR for test mode (straight line):
ros2 launch multi_robot_mapping test_mapping.launch.py use_rviz:=false pattern:=long_corridor
```

**Terminal 2: Launch clean global map visualization**
```bash
cd ~/thesis_ws
source install/setup.bash
ros2 launch multi_robot_mapping visualize_global_map.launch.py
```

**What you'll see:**
- ✅ **Green points**: Global stitched map (continuously updated)
- ✅ **Robot model**: TurtleBot3 (semi-transparent)
- ✅ **Grid**: Reference grid at ground level
- ❌ **No scans**: Laser scan rays hidden
- ❌ **No submaps**: Current submap being built is hidden

---

## Option 2: Offline Visualization (Saved Maps)

Use this to visualize saved `.pcd` map files without running ROS2.

### Usage

```bash
cd ~/thesis_ws
python3 visualize_map.py <path_to_map.pcd>
```

### Examples

```bash
# Visualize map from exploration mode
python3 visualize_map.py ./submaps/tb3_1/global_map.pcd

# Visualize map from test mode
python3 visualize_map.py ./test_results/submaps/tb3_1/global_map.pcd
```

### Controls

| Action | Control |
|--------|---------|
| Rotate view | Mouse drag |
| Zoom | Mouse wheel |
| Pan | Shift + mouse drag |
| Reset view | R key |
| Quit | Q or ESC |

### What you'll see

The script displays:
- 📊 **Statistics**: Point count, bounds, density
- 🎨 **Green point cloud**: The global map
- 📐 **Coordinate frame**: RGB axes at origin (X=red, Y=green, Z=blue)
- 🖼️ **3D viewer**: Interactive Open3D visualization window

---

## Configuration Files

### 1. `global_map_only.rviz`

**Location:** `src/multi_robot_mapping/rviz/global_map_only.rviz`

**Contents:**
- **Global Map** display (green, 3mm points)
- **Robot Model** (70% transparent)
- **Grid** (1m cells, 50×50 grid)
- **Top-down view** (90° pitch for bird's-eye view)

**Topic subscribed:** `/tb3_1/global_map`

### 2. `visualize_global_map.launch.py`

**Location:** `src/multi_robot_mapping/launch/visualize_global_map.launch.py`

**Purpose:** Launch RViz2 with clean global map visualization

### 3. `visualize_map.py`

**Location:** `~/thesis_ws/visualize_map.py`

**Purpose:** Standalone Python script for offline map visualization

---

## Comparison: Before vs After

### Default Visualization (`mapping_visualization.rviz`)
Shows:
- ✅ Current submap being built (yellow)
- ✅ Global stitched map (green)
- ✅ Robot model
- ⚠️ **Cluttered** - hard to see map structure

### Clean Visualization (`global_map_only.rviz`)
Shows:
- ❌ Current submap (hidden)
- ✅ Global stitched map (green) - **ONLY THIS!**
- ✅ Robot model (transparent)
- ✅ **Clean** - map structure clearly visible

---

## Tips for Visualization

### 1. Adjust Point Size in RViz

If points are too small/large:
1. Click on "Global Map" in Displays panel
2. Expand the display properties
3. Adjust "Size (m)" value:
   - Default: `0.03` (3cm)
   - Smaller for more detail: `0.02` (2cm)
   - Larger for better visibility: `0.05` (5cm)

### 2. Change Point Color

Current color: Green (`0; 255; 0`)

To change:
1. Click "Global Map" → "Color Transformer"
2. Change from "FlatColor" to:
   - **"Intensity"**: Color by point intensity
   - **"AxisColor"**: Color by Z-height
   - **"RGB8"**: Use RGB colors (if available)

### 3. Change View Angle

**Top-down view** (current default):
- Pitch: 90° (looking straight down)
- Yaw: 0°

**Isometric view** (3D perspective):
- Pitch: 45°
- Yaw: 45°

**Side view**:
- Pitch: 0°
- Yaw: 0° or 90°

### 4. Save Custom Views

1. Set your desired view angle
2. In "Views" panel → Right-click "Current View"
3. Click "Save view as..."
4. Name it and it will appear under "Saved" views

---

## Monitoring Map Quality

### What to Look For

**✅ Good map indicators:**
- Clean, sharp edges on walls
- No "ghosting" (double walls)
- Consistent point density
- Straight walls appear straight
- Corners are sharp and well-defined

**❌ Poor map indicators:**
- Fuzzy or thick walls (ghosting)
- Gaps or missing sections
- Wavy walls that should be straight
- Scattered noise points far from surfaces
- Increasing point density over time (duplicates not removed)

### Terminal Output to Monitor

Watch for these messages:

```
Before downsampling: 6200 points
Submap 1 stitched: 5800 total points (after voxel downsample)
```

**Good sign:** "After downsample" < "Before downsample" (removing duplicates)

```
ICP: fitness=0.45, success=true
```

**Good sign:** fitness > 0.2 and success=true (good alignment)

---

## Troubleshooting

### Issue: RViz shows "No messages received"

**Solution:**
```bash
# Check if global map topic is being published
ros2 topic list | grep global_map

# Check message rate
ros2 topic hz /tb3_1/global_map

# Expected: ~0.5 Hz (published every 2 seconds)
```

### Issue: Map looks cluttered/fuzzy

**Possible causes:**
1. Voxel downsampling not working → Check terminal for "after voxel downsample" messages
2. ICP failing → Check for "ICP failed" warnings
3. Too many scans per submap → Reduce `scans_per_submap` parameter

### Issue: Offline visualization fails

**Solution:**
```bash
# Install Open3D if missing
pip3 install open3d

# Check if map file exists
ls -lh ./submaps/tb3_1/global_map.pcd

# Verify file is not empty
du -h ./submaps/tb3_1/global_map.pcd
```

---

## File Locations Reference

| File Type | Location | Description |
|-----------|----------|-------------|
| Global map (exploration) | `./submaps/tb3_1/global_map.pcd` | Map from full_system.launch.py |
| Global map (test) | `./test_results/submaps/tb3_1/global_map.pcd` | Map from test_mapping.launch.py |
| Individual submaps | `./submaps/tb3_1/submap_*.pcd` | Individual submap files |
| Submap metadata | `./submaps/tb3_1/submap_*_metadata.npz` | Pose, timestamp, transform |
| Trajectory (test) | `./test_results/tb3_1/long_corridor_trajectory.txt` | Robot path during test |

---

## Quick Reference Commands

```bash
# 1. Launch system + clean visualization
ros2 launch multi_robot_mapping test_mapping.launch.py use_rviz:=false pattern:=long_corridor
ros2 launch multi_robot_mapping visualize_global_map.launch.py

# 2. Visualize saved map offline
python3 visualize_map.py ./test_results/submaps/tb3_1/global_map.pcd

# 3. Monitor global map topic
ros2 topic echo /tb3_1/global_map --once

# 4. Check map file size
ls -lh ./submaps/tb3_1/global_map.pcd

# 5. Count points in map file
python3 -c "import open3d as o3d; pcd=o3d.io.read_point_cloud('./submaps/tb3_1/global_map.pcd'); print(f'Points: {len(pcd.points)}')"
```

---

## Summary

**Three ways to visualize:**

1. **🎥 Real-time RViz (clean)**: `visualize_global_map.launch.py`
2. **📊 Offline viewer**: `visualize_map.py <file.pcd>`
3. **🔧 Default RViz (full)**: `mapping_visualization.rviz` (shows everything)

**Use clean visualization when:**
- You want to see map structure clearly
- Checking for ghosting or duplicate points
- Monitoring ICP alignment quality
- Recording videos or taking screenshots

**Use default visualization when:**
- Debugging scan accumulation
- Checking submap creation
- Verifying sensor data

Enjoy visualizing your maps! 🗺️
