# TF2 Fix for LiDAR Offset - Donut Artifact Resolution

## Root Cause Identified

The **donut-shaped artifact** during in-place rotation was caused by the LiDAR being physically offset from the robot's rotation center.

### Physical Configuration:

From `model.sdf` line 456:
```xml
<joint name="lidar_joint" type="fixed">
  <parent>base_link</parent>
  <child>base_scan</child>
  <pose>-0.064 0 0.121 0 0 0</pose>  ← LiDAR offset!
</joint>
```

**LiDAR offset from rotation center (base_link):**
- **X: -0.064m (6.4cm backward)**
- **Y: 0m**
- **Z: 0.121m (12.1cm up)**

### The Problem:

During in-place rotation, the robot rotates around `base_link` at position (0, 0), but the LiDAR **orbits in a circle** around this center!

```
        ↑ Forward (Y+)
        |
   [base_link]  ← Rotation center at (0, 0)
        |
        | -6.4cm (X-)
        |
      [LiDAR]    ← Orbits in circle with radius = 6.4cm
```

**During 360° rotation:**
```
Time 0 (0°):    LiDAR at (0.000, -0.064) → sees north wall
Time 1 (90°):   LiDAR at (0.064,  0.000) → sees north wall (from different position!)
Time 2 (180°):  LiDAR at (0.000,  0.064) → sees north wall (from different position!)
Time 3 (270°):  LiDAR at (-0.064, 0.000) → sees north wall (from different position!)
```

**Result:** The same north wall appears at multiple positions in the map, forming a **thick donut** with diameter ≈ 12.8cm.

## Previous (Wrong) Transformation:

```python
# OLD: Transform from robot center (base_link)
R = quaternion_to_rotation_matrix(pose['qx'], pose['qy'], pose['qz'], pose['qw'])
t = np.array([pose['x'], pose['y'], pose['z']])  # Position of base_link

points_robot = np.column_stack((x_robot, y_robot, z_robot))
points_world = (R @ points_robot.T).T + t
```

**Problem:** This assumes scan originates from `base_link`, but it actually originates from `base_scan` which is offset by 6.4cm!

## TF2 Solution:

TF2 (Transform Framework 2) maintains a transform tree and handles all coordinate transformations correctly:

```
map → odom → base_link → base_scan
                  ↑          ↑
            rotation    LiDAR offset
             center    (-6.4cm, 0, 12.1cm)
```

### New Implementation:

```python
def scan_to_world_points_tf2(self, scan_msg):
    # Convert scan to Cartesian in LiDAR frame (base_scan)
    x_scan = valid_ranges * np.cos(valid_angles)
    y_scan = valid_ranges * np.sin(valid_angles)
    z_scan = np.zeros_like(x_scan)

    # Get transform from base_scan to map frame using TF2
    # This automatically accounts for the LiDAR offset!
    transform = self.tf_buffer.lookup_transform(
        'map',                           # target frame
        f'{self.robot_name}/base_scan',  # source frame (LiDAR)
        scan_msg.header.stamp,           # timestamp
        timeout=Duration(seconds=0.1)
    )

    # Extract rotation and translation from transform
    R = quaternion_to_rotation_matrix(...)
    t = np.array([trans.x, trans.y, trans.z])

    # Transform points from LiDAR frame to world frame
    points_world = (R @ points_scan.T).T + t
    return points_world
```

**Why this works:**

TF2 computes the full transform chain:
1. `base_scan → base_link`: Accounts for -6.4cm offset
2. `base_link → odom`: Accounts for robot rotation and translation
3. `odom → map`: Converts to global map frame

**Result:** Scan points are transformed as if originating from the LiDAR's actual position, accounting for its circular motion during rotation.

## Changes Made:

### 1. Added TF2 imports (lines 20-23):
```python
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from rclpy.duration import Duration
```

### 2. Initialize TF2 buffer and listener (lines 64-67):
```python
# Initialize TF2 for proper coordinate transformations
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)
self.get_logger().info('✓ TF2 listener initialized')
```

### 3. Created new TF2-based transformation method (lines 279-332):
```python
def scan_to_world_points_tf2(self, scan_msg):
    """Transform laser scan using TF2 (accounts for LiDAR offset)"""
    # ... implementation ...
```

### 4. Updated scan_to_submap_points() to use TF2 (lines 365-387):
```python
def scan_to_submap_points(self, scan_msg, current_pose, submap_start_pose):
    """Use TF2 to properly transform scan points"""
    return self.scan_to_world_points_tf2(scan_msg)
```

## Expected Behavior After Fix:

### rotate_in_place pattern (360° rotation):

**Before (with donut):**
```
  ***********
 *           *
*             *  ← Thick donut (walls appear in circle)
*             *
 *           *
  ***********
```

**After (correct):**
```
      |          ← North wall (straight line)
      |
      o          ← Robot at center
      |
      |          ← South wall (straight line)
```

### Log output:
```
Creating submap 0: 30 scans, 0.00m, 60.1°
  Start pose: (0.000, 0.000, θ=0.0°)
  End pose:   (0.000, 0.000, θ=60.1°)
  Submap 0: Added as first submap (world frame)

Creating submap 1: 30 scans, 0.00m, 60.2°
  Start pose: (0.000, 0.000, θ=60.1°)
  End pose:   (0.000, 0.000, θ=120.3°)
  Submap 1: Merged to global map (world frame)
  Before downsampling: 2500 points
  Submap 1 stitched: 1245 total points (after voxel downsample)
```

**Key observations:**
- Position stays at (0.000, 0.000) ✓
- Only rotation changes ✓
- Walls appear at same position regardless of robot orientation ✓

## Technical Details:

### Transform Chain:

**map → odom:**
- Usually identity (unless using global localization)
- For our case: identity transform

**odom → base_link:**
- Translation: (x, y, 0) from odometry
- Rotation: θ from odometry (z-axis rotation)

**base_link → base_scan:**
- Translation: (-0.064, 0, 0.121) [FIXED offset]
- Rotation: identity (LiDAR mounted rigidly)

**Combined transform (base_scan → map):**
```
T_map_scan = T_map_odom × T_odom_base × T_base_scan

Where:
- T_odom_base changes with robot motion (rotation)
- T_base_scan is CONSTANT (fixed LiDAR offset)
```

### Why the Donut Had Diameter ≈ 12.8cm:

The LiDAR offset is 6.4cm from the rotation center. During 360° rotation:
- Minimum distance from center: 0cm (when aligned with center)
- Maximum distance from center: 6.4cm × 2 = 12.8cm (diameter)

This created a circular "smear" of the same walls.

## Verification Steps:

1. **Build and source:**
```bash
cd ~/thesis_ws
colcon build --packages-select map_generation --symlink-install
source install/setup.bash
```

2. **Run rotation test:**
```bash
ros2 launch multi_robot_mapping test_mapping.launch.py pattern:=rotate_in_place
```

3. **Check for straight walls:**
- In RViz, the global map should show TWO STRAIGHT WALLS
- No circular/donut artifacts
- Walls should be thin (0.3m thickness)

4. **Verify TF2 is working:**
```bash
# In another terminal
ros2 run tf2_ros tf2_echo map tb3_1/base_scan
```

Should show the transform updating as robot rotates.

## Dependencies:

The TF2 fix requires these ROS2 packages:
- `tf2_ros` - TF2 Python bindings
- `tf2_geometry_msgs` - TF2 geometry message support

These are standard ROS2 packages and should already be installed.

## Files Modified:

1. **src/map_generation/map_generation/local_submap_generator.py**
   - Added TF2 imports (lines 20-23)
   - Initialize TF2 buffer/listener (lines 64-67)
   - New method: `scan_to_world_points_tf2()` (lines 279-332)
   - Updated: `scan_to_submap_points()` to use TF2 (lines 365-387)

## Conclusion:

The TF2 fix properly accounts for the **6.4cm LiDAR offset** from the robot's rotation center. This eliminates the donut artifact by ensuring that scan points are transformed from their true origin (the LiDAR sensor position) rather than assuming they originate from the robot's center.

**Key principle:** Use the robot's transform tree (TF2) instead of manual transformations to automatically handle all frame offsets and relationships.
