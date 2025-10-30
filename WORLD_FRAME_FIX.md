# World Frame Fix - Circular Artifact Resolution

## Problem Identified

During in-place rotation testing, the map showed a **circular artifact** - the same physical walls appeared at multiple rotational positions, forming a circle around the robot instead of staying at their fixed world positions.

## Root Cause

The original implementation stored scan points in **submap-local frame**:

### What Was Happening (WRONG):

```python
# scan_to_submap_points() old logic:
1. Transform scan to WORLD frame using current robot pose ✓
2. Transform from WORLD frame to SUBMAP-LOCAL frame ✗

# During rotation at (0, 0):
Time 0 (0°):   North wall → World (0, 5) → Submap-local (0, 5)
Time 1 (60°):  North wall → World (0, 5) → Submap-local (4.3, 2.5)  ← WRONG!
Time 2 (120°): North wall → World (0, 5) → Submap-local (4.3, -2.5) ← WRONG!
...
```

**Problem**: The submap coordinate system was fixed at the initial robot orientation (0°). As the robot rotated, the same wall got stored at different angles relative to this fixed frame, creating the circular artifact.

## Solution

Store scan points directly in **WORLD FRAME** without converting to submap-local frame.

### What Should Happen (CORRECT):

```python
# scan_to_submap_points() new logic:
1. Transform scan to WORLD frame using current robot pose ✓
2. Return world frame points directly ✓

# During rotation at (0, 0):
Time 0 (0°):   North wall → World (0, 5) ← CORRECT
Time 1 (60°):  North wall → World (0, 5) ← SAME position!
Time 2 (120°): North wall → World (0, 5) ← SAME position!
...
```

**Result**: Same physical object always has same world coordinates, regardless of robot pose.

## Changes Made

### 1. local_submap_generator.py

#### Modified `scan_to_submap_points()` (line 316-337):

**Before**:
```python
def scan_to_submap_points(self, scan_msg, current_pose, submap_start_pose):
    # Transform to world frame
    points_world = self.scan_to_world_points(scan_msg, current_pose)

    # Transform to submap-local frame (CAUSED BUG)
    R_submap = quaternion_to_rotation_matrix(...)
    t_submap = np.array([...])
    points_submap = (R_submap.T @ (points_world - t_submap).T).T
    return points_submap
```

**After**:
```python
def scan_to_submap_points(self, scan_msg, current_pose, submap_start_pose):
    """
    Convert laser scan to points in WORLD frame

    IMPORTANT: We store points in world frame to avoid rotation artifacts.
    During in-place rotation, the same physical wall should have the same
    world coordinates, regardless of robot orientation.
    """
    # Simply transform to world frame and return (FIX)
    return self.scan_to_world_points(scan_msg, current_pose)
```

#### Modified `publish_current_submap()` (line 136-145):

**Before**:
```python
# Transform submap-local points to world frame for visualization
R_submap = quaternion_to_rotation_matrix(...)
t_submap = np.array([...])
points_world = (R_submap @ points_submap.T).T + t_submap
```

**After**:
```python
# Points are already in world frame, no transformation needed
points_world = np.vstack(self.current_submap_points)
```

### 2. submap_stitcher.py

#### Modified `add_and_stitch_submap()` (line 214-258):

**Before**:
```python
# Compute odometry-based transform
odom_transform = self.estimate_2d_transform(prev_pose, current_pose)
initial_transform = prev_transform @ odom_transform

# Run ICP registration
final_transform = self.register_icp_2d(source=pcd, target=global_map, ...)

# Transform and add to global map
pcd_final = o3d.geometry.PointCloud(pcd)
pcd_final.transform(final_transform)  # WRONG - already in world frame!
self.global_map += pcd_final
```

**After**:
```python
# Points already in world frame - just merge directly!
identity_transform = np.eye(4)  # No transform needed

# Store submap data
submap_data = {..., 'global_transform': identity_transform}

# Add directly to global map (no transformation)
self.global_map += pcd  # FIX
```

## Why This Fix Works

### For All Motion Types:

#### 1. **In-Place Rotation** (what was broken):
```
Robot at (0, 0):
- Facing 0°:   Sees wall → (0, 5) in world
- Facing 90°:  Sees wall → (0, 5) in world  ← Same!
- Facing 180°: Sees wall → (0, 5) in world  ← Same!
```
✅ Wall stays at fixed position

#### 2. **Straight-Line Motion** (still works):
```
Robot moving forward:
- At (0, 0): Sees wall 5m ahead → (5, 0) in world
- At (1, 0): Sees wall 4m ahead → (5, 0) in world  ← Same!
- At (2, 0): Sees wall 3m ahead → (5, 0) in world  ← Same!
```
✅ Wall stays at fixed position

#### 3. **Mixed Motion** (still works):
```
Robot moving and turning:
- At (0, 0) facing 0°:    Sees wall → (5, 0) in world
- At (1, 1) facing 45°:   Sees wall → (5, 0) in world  ← Same!
- At (2, 2) facing 90°:   Sees wall → (5, 0) in world  ← Same!
```
✅ Wall stays at fixed position

### Key Principle:

**Physical objects have fixed positions in the world.**

Storing points in world frame ensures:
1. Same object = same coordinates (no matter robot pose)
2. Voxel downsampling correctly removes duplicates
3. No circular artifacts during rotation
4. Simpler logic (no back-and-forth transformations)

## Trade-offs

### What We Gained:
✅ **Fixed rotation artifacts** - walls don't form circles
✅ **Simpler logic** - no coordinate frame conversions
✅ **Correct voxel downsampling** - merges true duplicates
✅ **Works for all motion types** - rotation, translation, mixed

### What We Lost:
❌ **No ICP refinement** - relies purely on odometry accuracy
❌ **No loop closure correction** - can't adjust past submaps
❌ **Drift accumulation** - odometry errors accumulate over time

## Future Improvements

To get the best of both worlds, consider:

### Option 1: ICP in World Frame
- Store points in world frame (current approach)
- Run ICP between new submap and global map **in world frame**
- Apply small correction transform if drift detected
- Requires careful drift detection logic

### Option 2: Pose Graph Optimization
- Store points in world frame
- Build pose graph of robot trajectory
- Detect loop closures using feature matching
- Optimize entire pose graph when loop detected
- Re-transform all submaps with optimized poses

### Option 3: Hybrid Approach
- Store points in world frame for current submaps
- Periodically run global optimization
- Adjust all stored points when significant drift detected

## Testing Results

After applying this fix, the `rotate_in_place` pattern should show:
- ✅ Two straight walls (not circular artifacts)
- ✅ Walls at fixed positions in world frame
- ✅ Clean 360° coverage from 6 submaps
- ✅ Proper voxel downsampling (no ghost walls)

### Expected Behavior:

**rotate_in_place (360° rotation):**
```
Creating submap 0: 30 scans, 0.02m, 60.3°
  Submap 0: Added as first submap (world frame)

Creating submap 1: 30 scans, 0.03m, 60.1°
  Submap 1: Merged to global map (world frame)
  Before downsampling: 2500 points
  Submap 1 stitched: 1245 total points (after voxel downsample)

...
```

Notice:
- No ICP registration (not needed - already in world frame)
- Direct merge to global map
- Voxel downsampling removes duplicate scans of same walls

## Files Modified

1. **src/map_generation/map_generation/local_submap_generator.py**
   - Line 316-337: `scan_to_submap_points()` - now returns world frame
   - Line 136-145: `publish_current_submap()` - no transformation
   - Line 152-159: Fallback visualization - no transformation

2. **src/map_generation/map_generation/submap_stitcher.py**
   - Line 181-212: First submap - comments updated
   - Line 214-258: Subsequent submaps - removed ICP, direct merge

## Verification Commands

```bash
# Rebuild
cd ~/thesis_ws
colcon build --packages-select map_generation --symlink-install
source install/setup.bash

# Test rotation pattern
ros2 launch multi_robot_mapping test_mapping.launch.py pattern:=rotate_in_place

# Expected: Two straight walls, no circular artifacts
```

## Conclusion

This fix resolves the fundamental issue with rotation artifacts by respecting the principle that **physical objects don't move when the robot rotates**. Points are now stored at their true world positions, eliminating the circular artifact bug.
