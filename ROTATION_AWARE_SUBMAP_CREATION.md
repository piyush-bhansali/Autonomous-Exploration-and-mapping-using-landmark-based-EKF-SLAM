# Rotation-Aware Submap Creation

## Problem Statement

**Original Issue**: During in-place rotation, the robot doesn't translate (x, y position constant), so the distance threshold (0.8m) was never met. This meant **no submaps were created** during pure rotation, causing all scans to accumulate in a single "current_submap" that never got stitched to the global map.

## Solution

Added **angular displacement (rotation)** as an additional trigger for submap creation.

### New Submap Creation Condition

**Before** (distance only):
```python
if scans >= 50 AND distance >= 0.8m:
    create_submap()
```

**After** (distance OR rotation):
```python
if scans >= 30 AND (distance >= 0.6m OR rotation >= 60°):
    create_submap()
```

## New Parameters

| Parameter | Old Value | New Value | Rationale |
|-----------|-----------|-----------|-----------|
| `scans_per_submap` | 50 scans | **30 scans** | Better for rotation (3s @ 10Hz = 86° rotation) |
| `min_distance_between_submaps` | 0.8m | **0.6m** | Maintains similar density at 0.2 m/s (3s travel) |
| `min_rotation_between_submaps` | N/A | **60°** (π/3 rad) | NEW: Triggers submap every 60° of rotation |

### Timing Analysis

**At 30 scans (3 seconds):**

| Motion Type | Robot Behavior | Distance | Rotation | Trigger? |
|-------------|----------------|----------|----------|----------|
| Translation (0.2 m/s) | Moving forward | 0.6m | ~0° | ✅ Distance |
| Rotation (0.5 rad/s) | Spinning in place | ~0m | 86° | ✅ Rotation |
| Mixed (figure-8) | Moving + turning | 0.4m | 45° | ❌ Wait more |

This ensures submaps are created appropriately for both pure translation and pure rotation.

## Impact on Different Movement Patterns

### 1. LONG_CORRIDOR (10m straight)
**Before**: 50 scans + 0.8m → ~12-13 submaps
**After**: 30 scans + 0.6m → ~16-17 submaps
- **More frequent submaps** = better ICP convergence
- **Smaller submaps** = less drift accumulation per submap

### 2. SQUARE (4×4m loop)
**Before**: 50 scans + 0.8m → ~20 submaps (16m perimeter)
**After**: 30 scans + 0.6m → ~26-27 submaps
- More submaps at corners (rotation contributes)
- Better loop closure detection (more opportunities)

### 3. ROTATE_IN_PLACE (360° spin)
**Before**: **0 submaps** (distance never met!) ❌
**After**: **6 submaps** (every 60°) ✅
- Submap 0: 0° → 60° (3s)
- Submap 1: 60° → 120° (3s)
- Submap 2: 120° → 180° (3s)
- Submap 3: 180° → 240° (3s)
- Submap 4: 240° → 300° (3s)
- Submap 5: 300° → 360° (3s)

**Total time**: 12.6 seconds for 360° rotation

## Implementation Details

### Code Changes

**File**: `src/map_generation/map_generation/local_submap_generator.py`

#### 1. Added New Parameter (lines 39, 49)
```python
self.declare_parameter('min_rotation_between_submaps', 60.0)  # degrees
self.min_rotation_between_submaps = np.radians(
    self.get_parameter('min_rotation_between_submaps').value
)
```

#### 2. Updated should_create_submap() (lines 377-409)
```python
def should_create_submap(self):
    # Check scan count requirement
    if self.scans_in_current_submap < self.scans_per_submap:
        return False

    # Calculate LINEAR distance traveled
    dx = self.current_pose['x'] - self.submap_start_pose['x']
    dy = self.current_pose['y'] - self.submap_start_pose['y']
    distance = np.sqrt(dx**2 + dy**2)

    # Calculate ANGULAR displacement (rotation)
    # Use atan2 to handle angle wraparound correctly
    dtheta = abs(np.arctan2(
        np.sin(self.current_pose['theta'] - self.submap_start_pose['theta']),
        np.cos(self.current_pose['theta'] - self.submap_start_pose['theta'])
    ))

    # Create submap if sufficient scans AND (moved OR rotated)
    if distance >= self.min_distance_between_submaps or dtheta >= self.min_rotation_between_submaps:
        self.get_logger().info(
            f'Creating submap {self.submap_id}: '
            f'{self.scans_in_current_submap} scans, '
            f'{distance:.2f}m, {np.degrees(dtheta):.1f}°'
        )
        return True

    return False
```

**Key points:**
- `np.arctan2(sin(Δθ), cos(Δθ))` handles angle wraparound (e.g., 350° → 10° = 20° difference, not 340°)
- Uses `abs()` to get magnitude of rotation (direction doesn't matter)
- OR condition allows either distance OR rotation to trigger submap

#### 3. Updated Launch File (line 212-214)
```python
'scans_per_submap': 30,  # 30 scans @ 10Hz = 3 seconds
'min_distance_between_submaps': 0.6,  # 0.6m distance OR 60° rotation
'min_rotation_between_submaps': 60.0,  # 60 degrees (π/3 radians)
```

## Expected Log Output

### ROTATE_IN_PLACE Pattern

```bash
============================================================
EXECUTING ROTATE IN PLACE PATTERN
This tests IN-PLACE ROTATIONAL MAPPING
Expected: Zero drift, clean 360° scan coverage
============================================================

Local Submap Generator initialized for tb3_1
Scans per submap: 30, Min distance: 0.6m, Min rotation: 60°

--- Rotating 360° clockwise ---

Creating submap 0: 30 scans, 0.02m, 60.3°
  Submap 0: Added as first submap

Creating submap 1: 30 scans, 0.03m, 60.1°
  ICP: fitness=0.876, success=true
  ✓ ICP successful (fitness 0.876)
  Submap 1 stitched: 1245 total points (after voxel downsample)

Creating submap 2: 30 scans, 0.02m, 59.8°
  ICP: fitness=0.891, success=true
  ✓ ICP successful (fitness 0.891)
  Submap 2 stitched: 1856 total points (after voxel downsample)

Creating submap 3: 30 scans, 0.04m, 60.2°
  ICP: fitness=0.883, success=true
  ✓ ICP successful (fitness 0.883)
  Submap 3 stitched: 2401 total points (after voxel downsample)

Creating submap 4: 30 scans, 0.03m, 60.0°
  ICP: fitness=0.879, success=true
  ✓ ICP successful (fitness 0.879)
  Submap 4 stitched: 2956 total points (after voxel downsample)

Creating submap 5: 30 scans, 0.02m, 59.9°
  ICP: fitness=0.885, success=true
  ✓ ICP successful (fitness 0.885)
  Submap 5 stitched: 3512 total points (after voxel downsample)

Rotated 360.0° (target: 360.0°)

============================================================
ROTATE IN PLACE PATTERN COMPLETED
Final position: (-12.023, -11.987)
Start position: (-12.000, -12.000)
Positional drift: 0.038m
Heading error: 2.3°
============================================================

✅ Excellent! Drift within tolerance: 0.038m
✅ Good heading accuracy: 2.3° error
```

**Notice**:
- Distance stays ~0m throughout (in-place rotation)
- Rotation triggers every ~60°
- ICP fitness remains high (>0.85) = good alignment
- Total drift <0.1m = excellent in-place rotation

## Angle Wraparound Handling

The code uses `atan2(sin(Δθ), cos(Δθ))` to correctly handle angle wraparound:

```python
# Example: Robot rotates from 350° to 10°
theta_start = np.radians(350)  # 6.109 rad
theta_current = np.radians(10)  # 0.175 rad

# Naive difference (WRONG)
dtheta_wrong = theta_current - theta_start  # -5.934 rad = -340°

# Correct difference using atan2
dtheta_correct = np.arctan2(
    np.sin(theta_current - theta_start),
    np.cos(theta_current - theta_start)
)  # 0.349 rad = 20° ✓
```

This ensures the algorithm detects 20° of rotation, not 340° backward rotation.

## Testing the Changes

### Test 1: ROTATE_IN_PLACE Pattern
```bash
cd ~/thesis_ws
source install/setup.bash
./run_with_clean_viz.sh rotate_in_place
```

**Expected**: 6 submaps created, clean 360° wall reconstruction

### Test 2: LONG_CORRIDOR Pattern (verify no regression)
```bash
./run_with_clean_viz.sh long_corridor
```

**Expected**: ~17 submaps created, straight corridor map

### Test 3: SQUARE Pattern (verify rotation helps at corners)
```bash
./run_with_clean_viz.sh square
```

**Expected**: ~27 submaps created, extra submaps at 90° corners

## Files Modified

1. **src/map_generation/map_generation/local_submap_generator.py**
   - Added `min_rotation_between_submaps` parameter
   - Modified `should_create_submap()` to check rotation
   - Updated logging to show both distance and rotation

2. **src/multi_robot_mapping/launch/test_mapping.launch.py**
   - Changed `scans_per_submap`: 50 → 30
   - Changed `min_distance_between_submaps`: 0.8m → 0.6m
   - Added `min_rotation_between_submaps`: 60°

3. **src/map_generation/map_generation/test_robot_controller.py**
   - Added `ROTATE_IN_PLACE` pattern (MovementPattern.ROTATE_IN_PLACE = 5)
   - Added `execute_rotate_in_place()` method

4. **ROTATE_IN_PLACE_PATTERN.md** (new)
   - Documentation for rotate_in_place pattern

5. **run_with_clean_viz.sh**
   - Added pattern argument support

## Benefits

1. **Handles pure rotation**: Robot can now create submaps even when stationary
2. **Better corner handling**: Mixed motion (translation + rotation) gets submaps at corners
3. **More frequent submaps**: 30 scans vs 50 = less drift accumulation per submap
4. **Improved ICP convergence**: Smaller submaps with more overlap = better alignment
5. **Flexible**: Works for all motion types (translation, rotation, or mixed)

## Potential Issues to Monitor

1. **More submaps = more computation**: ICP registration happens more frequently
2. **Small submaps**: 30 scans might be too few for very sparse environments
3. **False triggers**: Small angular drift during straight-line motion might trigger rotation threshold
   - Mitigation: 60° threshold is high enough to avoid this (would need significant drift)

## Next Steps

After testing the rotate_in_place pattern, consider:
1. Tune rotation threshold (try 45° for 8 submaps, or 90° for 4 submaps)
2. Add adaptive thresholds based on velocity magnitude
3. Implement rotation-aware ICP initialization (use rotation for initial guess)
