# In-Place Rotation Test Pattern

## Overview
Added a new test pattern `ROTATE_IN_PLACE` to evaluate how the mapping system handles pure rotational motion without translation.

## What It Does
- Robot performs a complete 360° rotation at its center
- Uses only angular velocity (angular.z = 0.5 rad/s)
- Linear velocity is zero (twist.linear.x = 0.0)
- Takes approximately 12.6 seconds to complete (2π radians / 0.5 rad/s)
- Creates **6 submaps** during rotation (one every 60° / 3 seconds)

## Purpose
This pattern helps visualize:
1. **Rotational mapping quality**: How well the system captures 360° scan coverage
2. **Odometry drift during rotation**: Whether in-place rotation causes positional drift
3. **Heading accuracy**: Whether the robot returns to the same orientation after 360°
4. **Wall reconstruction**: How rotating scans create the map when stationary

## Expected Results
✅ **Ideal behavior:**
- Positional drift < 0.1m (robot shouldn't translate)
- Heading error < 5° (should return to same orientation)
- Clean 360° wall reconstruction in map
- Multiple submaps created as robot accumulates scans at same location

⚠️ **Potential issues to observe:**
- Drift > 0.1m indicates differential drive model issues
- Heading error > 5° indicates odometry error during rotation
- Thick or blurry walls indicate scan alignment issues
- Duplicate points indicate voxel downsampling issues

## Usage

### Quick Start (with clean global map visualization)
```bash
cd ~/thesis_ws
source install/setup.bash
./run_with_clean_viz.sh rotate_in_place
```

### Manual Launch
```bash
# Terminal 1: Launch system (no RViz)
ros2 launch multi_robot_mapping test_mapping.launch.py \
    pattern:=rotate_in_place \
    use_rviz:=false

# Terminal 2: Launch clean visualization (after 8 seconds)
ros2 launch multi_robot_mapping visualize_global_map.launch.py
```

### Standard Launch (with default RViz showing both yellow and green)
```bash
ros2 launch multi_robot_mapping test_mapping.launch.py pattern:=rotate_in_place
```

## What to Look For

### 1. Terminal Output
The test controller will log:
```
============================================================
EXECUTING ROTATE IN PLACE PATTERN
This tests IN-PLACE ROTATIONAL MAPPING
Expected: Zero drift, clean 360° scan coverage
============================================================

--- Rotating 360° clockwise ---
Rotated 360.0° (target: 360.0°)

============================================================
ROTATE IN PLACE PATTERN COMPLETED
Final position: (x, y)
Start position: (x, y)
Positional drift: 0.XXXm
Heading error: X.X°
============================================================

✅ Excellent! Drift within tolerance: 0.XXXm
✅ Good heading accuracy: X.X° error
```

### 2. RViz Visualization
**In clean visualization (green only):**
- Should see circular/radial pattern of walls from rotation point
- Walls should appear as thin lines (0.3m thickness)
- No ghosting or duplicate structures
- Points accumulate around robot's rotation center

**In default visualization (yellow + green):**
- Yellow: Current submap being built
- Green: Stitched global map
- Watch submaps being created and stitched during rotation

### 3. Submap Generation
Expected behavior with new rotation-aware thresholds:
- Creates submap when: **30 scans** AND (**0.6m distance** OR **60° rotation**)
- For in-place rotation: distance ≈ 0m, but rotation triggers every 60°
- **360° rotation creates 6 submaps** (one every 60°)
- Each submap: ~30 scans (3 seconds) covering 60° sector of walls
- Voxel downsampling (0.05m) removes duplicate points between overlapping submaps
- Result: Complete 360° coverage with good overlap for ICP registration

## Implementation Details

### Code Location
`/home/piyush/thesis_ws/src/map_generation/map_generation/test_robot_controller.py`

### Key Changes
1. Added `ROTATE_IN_PLACE = 5` to `MovementPattern` enum (line 31)
2. Added `execute_rotate_in_place()` method (lines 315-367)
3. Added pattern to execution logic (line 395)
4. Updated launch file documentation (line 16)

### Rotation Mechanics
The `rotate()` method in test_robot_controller.py:
```python
def rotate(self, angle, speed=None):
    twist = Twist()
    twist.angular.z = speed if angle > 0 else -speed
    # twist.linear.x NOT set = defaults to 0.0
    # This creates in-place rotation
```

Differential drive plugin calculates wheel velocities:
```
v_left = (v - ω×L/2) / r
v_right = (v + ω×L/2) / r

When v=0 (no linear velocity):
v_left = -ω×L/2 / r  (negative = backward)
v_right = +ω×L/2 / r  (positive = forward)
```

Result: Left wheel rotates backward, right wheel rotates forward → zero turning radius (in-place rotation)

## Comparison with Other Patterns

| Pattern | Tests | Translation | Rotation | Loop Closure |
|---------|-------|-------------|----------|--------------|
| LONG_CORRIDOR | Linear drift | 10m | 0° | No |
| SQUARE | Loop closure | 16m | 360° | Yes |
| FIGURE_EIGHT | Complex trajectory | ~12m | 720° | Yes |
| SPIRAL | Continuous rotation | ~15m | 720° | No |
| **ROTATE_IN_PLACE** | **Rotational mapping** | **0m** | **360°** | **No** |

## Files Modified
1. `/home/piyush/thesis_ws/src/map_generation/map_generation/test_robot_controller.py`
   - Added ROTATE_IN_PLACE pattern
   - Added execute_rotate_in_place() method

2. `/home/piyush/thesis_ws/src/multi_robot_mapping/launch/test_mapping.launch.py`
   - Updated documentation with rotate_in_place usage
   - Updated pattern description

## Next Steps
After running this pattern, compare with other patterns to understand:
1. How rotational scans differ from translational scans
2. Whether drift is worse during rotation vs translation
3. How voxel downsampling handles overlapping rotational scans
4. Whether ICP registration works well with rotational submaps
