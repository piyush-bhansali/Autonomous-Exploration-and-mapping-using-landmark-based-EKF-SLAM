# Submap-Based Mapping System - Complete Overview

## What Was Implemented

You asked for a submap-based mapping system with:
1. ✅ **Fixed scan count per submap (250 scans)**
2. ✅ **Distance threshold to avoid duplicates (1.5m minimum)**
3. ✅ **Feature extraction for loop closure**
4. ✅ **Store submaps individually**
5. ✅ **ICP stitching to global map**
6. ✅ **Modular feature extraction interface**

---

## Architecture Overview

```
┌────────────────────────────────────────────────────────────────┐
│                    LOCAL SUBMAP GENERATOR                      │
│  - Accumulates 250 scans                                       │
│  - Checks distance >= 1.5m                                     │
│  - Triggers submap creation                                    │
└───────────────────────────┬────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│                      SUBMAP STITCHER                           │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │
│  │  1. Process  │─▸│  2. Extract  │─▸│  3. Register │        │
│  │  Point Cloud │  │  Features    │  │  with ICP    │        │
│  └──────────────┘  └──────────────┘  └──────────────┘        │
│         │                 │                  │                 │
│         ▼                 ▼                  ▼                 │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐        │
│  │ Downsampled  │  │ FPFH/SHOT/   │  │ Add to Global│        │
│  │ + Cleaned    │  │ Geometric    │  │ Map          │        │
│  └──────────────┘  └──────────────┘  └──────────────┘        │
└───────────────────────────┬────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│                    SUBMAP STORAGE                              │
│  For each submap:                                              │
│  - point_cloud (PCD file)                                      │
│  - features (descriptors + keypoints)                          │
│  - poses (start, end, center)                                  │
│  - global_transform                                            │
│  - metadata (scan_count, timestamp, bounds)                    │
└───────────────────────────┬────────────────────────────────────┘
                            │
                            ▼
┌────────────────────────────────────────────────────────────────┐
│                 LOOP CLOSURE DETECTOR (Skeleton)               │
│  1. Spatial indexing (KD-tree of submap centers)              │
│  2. Feature matching (descriptor similarity)                   │
│  3. Geometric verification (RANSAC + ICP) [TODO]              │
└────────────────────────────────────────────────────────────────┘
```

---

## Files Created/Modified

### NEW FILES:

1. **`feature_extractor.py`** (NEW - 600 lines)
   - Modular interface for different feature types
   - Supports: FPFH, SHOT (placeholder), Geometric
   - GPU-accelerated keypoint detection
   - ISS keypoint detector + uniform sampling fallback

2. **`loop_closure_detector.py`** (NEW - 250 lines)
   - Skeleton implementation for loop closure
   - Spatial indexing with KD-tree
   - Feature matching interface
   - Geometric verification (to be implemented)

3. **`submap_stitcher.py`** (COMPLETELY REWRITTEN - 450 lines)
   - Stores individual submaps for loop closure
   - Extracts features from each submap
   - ICP stitching to global map
   - Saves submaps individually

### MODIFIED FILES:

4. **`local_submap_generator.py`** (MODIFIED)
   - New scan-count based trigger (250 scans)
   - Distance threshold check (1.5m minimum)
   - Passes scan_count to stitcher
   - Saves individual submaps

5. **`ekf_lib.py`** (FIXED EARLIER)
   - Fixed `predict_imu()` with dead reckoning
   - Now predicts position AND orientation at 200 Hz

---

## How It Works Now

### Step-by-Step Process:

```
1. Robot moves and scans
   ↓
2. local_submap_generator accumulates scans
   - Counter: scans_in_current_submap++
   ↓
3. Check trigger conditions EVERY scan:
   - scans_in_current_submap >= 250? ✓
   - distance_traveled >= 1.5m? ✓
   ↓
4. Create submap:
   a) Process point cloud (downsample, outlier removal)
   b) Extract features (FPFH/SHOT/Geometric)
   c) ICP registration with global map
   d) Add to global map
   e) Store submap data structure
   f) Add to spatial index
   ↓
5. Check loop closure (if enabled):
   - Find nearby submaps spatially
   - Match features
   - Verify geometry (TODO)
   - Apply correction (TODO)
   ↓
6. Save:
   - global_map.pcd
   - submaps/submap_0000.pcd, submap_0001.pcd, ...
   - submaps/submap_0000_metadata.npz, ...
   ↓
7. Reset counters and repeat
```

---

## New Parameters

### In `local_submap_generator.py`:

```python
scans_per_submap: 250              # Fixed number of scans per submap
min_distance_between_submaps: 1.5  # Minimum distance (meters)
feature_method: 'fpfh'             # Feature type: 'fpfh', 'shot', 'geometric'
enable_loop_closure: False         # Enable/disable loop closure detection
```

### Launch File Usage:

```bash
ros2 launch multi_robot_mapping test_mapping.launch.py \
    pattern:=square \
    scans_per_submap:=250 \
    min_distance:=1.5 \
    feature_method:='fpfh' \
    enable_loop_closure:=false
```

---

## Submap Data Structure

Each submap is stored with complete information:

```python
submap = {
    'id': 0,                                # Submap identifier
    'point_cloud': o3d.geometry.PointCloud, # Processed point cloud
    'features': {                            # Feature extraction result
        'method': 'fpfh',
        'keypoints': o3d.geometry.PointCloud,  # ~500 keypoints
        'keypoint_indices': np.array,          # Indices in original cloud
        'descriptors': np.ndarray,             # N × 33 for FPFH
        'metadata': {
            'num_keypoints': 543,
            'descriptor_dim': 33,
            'computation_time': 0.45
        }
    },
    'pose_start': {x, y, z, qx, qy, qz, qw, timestamp},
    'pose_end': {x, y, z, qx, qy, qz, qw, timestamp},
    'pose_center': {x, y, z, qx, qy, qz, qw},  # Center of submap
    'global_transform': np.ndarray(4, 4),       # Transform to world
    'scan_count': 250,                          # Number of scans
    'bounds': {                                 # Spatial bounds
        'min_x': -1.5, 'max_x': 1.5,
        'min_y': -1.2, 'max_y': 1.8,
        'min_z': 0.0, 'max_z': 0.5
    },
    'timestamp_created': 1706123456.789
}
```

---

## Feature Extraction

### Supported Methods:

#### 1. FPFH (Fast Point Feature Histograms) [DEFAULT]
- **Speed:** Fast (~0.5s per submap)
- **Descriptor:** 33-dimensional
- **Good for:** Local geometric structure
- **Use case:** General-purpose loop closure

#### 2. SHOT (Signature of Histograms) [PLACEHOLDER]
- **Speed:** Slower (~1.5s per submap)
- **Descriptor:** 352-dimensional
- **Good for:** More discriminative matching
- **Status:** TODO - Need to implement

#### 3. Geometric Features [FAST]
- **Speed:** Very fast (~0.1s per submap)
- **Descriptor:** 4-dimensional (linearity, planarity, scattering, curvature)
- **Good for:** Coarse matching, quick filtering
- **Use case:** Initial candidate selection

### Keypoint Detection:

#### ISS (Intrinsic Shape Signatures) [DEFAULT]
- Detects salient points (corners, edges)
- Typically extracts ~500 keypoints from 10,000 points
- Better than uniform sampling

#### Uniform Sampling [FALLBACK]
- If ISS fails, use 10% random sampling
- Faster but less discriminative

---

## Loop Closure Detection (Skeleton)

### Current Implementation Status:

✅ **Spatial indexing** - KD-tree for fast candidate search
✅ **Feature matching** - Descriptor comparison
⚠️  **Geometric verification** - TODO (RANSAC + ICP)
⚠️  **Pose graph optimization** - TODO (g2o/GTSAM integration)

### How to Complete Loop Closure:

```python
# In loop_closure_detector.py, implement:

def _verify_geometric(self, submap1, submap2, match_result):
    """
    1. Extract matched keypoint 3D positions
    2. RANSAC to estimate initial transform
    3. ICP to refine transform
    4. Check fitness threshold
    """
    # Get matched keypoint positions
    keypoints1 = np.asarray(match_result['keypoints1'].points)
    keypoints2 = np.asarray(match_result['keypoints2'].points)
    matches = match_result['matches']

    source_pts = keypoints1[matches[:, 0]]
    target_pts = keypoints2[matches[:, 1]]

    # RANSAC
    transform, inliers = estimate_transform_ransac(
        source_pts, target_pts,
        max_iterations=1000,
        threshold=0.1
    )

    if np.sum(inliers) < 15:  # Need at least 15 inliers
        return None

    # ICP refinement
    refined_transform, fitness = refine_transform_icp(
        submap1['point_cloud'],
        submap2['point_cloud'],
        transform,
        max_correspondence_dist=0.5
    )

    if fitness > self.icp_fitness_threshold:
        return {
            'current_id': submap1['id'],
            'matched_id': submap2['id'],
            'transform': refined_transform,
            'fitness': fitness,
            'num_inliers': np.sum(inliers)
        }

    return None
```

---

## Testing

### Quick Test:

```bash
cd ~/thesis_ws
source install/setup.bash

# Run with new scan-count mode
ros2 launch multi_robot_mapping test_mapping.launch.py \
    pattern:=long_corridor
```

### Expected Behavior:

```
[local_submap_generator]: Local Submap Generator Started (NEW SCAN-COUNT MODE)
[local_submap_generator]:   Scans per submap: 250
[local_submap_generator]:   Min distance between submaps: 1.5m
[local_submap_generator]:   Feature method: fpfh
[local_submap_generator]:   Loop closure: False

...

[local_submap_generator]: DEBUG: Current submap: 247/250 scans
[local_submap_generator]: DEBUG: Current submap: 250/250 scans
[local_submap_generator]: ✓ Submap trigger: 250 scans, 1.87m traveled

============================================================
Processing Submap 0
============================================================
  Submap 0: 25000 → 8543 points (GPU-processed)
  Extracting features for submap 0...
  ISS keypoints: 543/8543 (6.4%)
  ✓ Features: 543 keypoints, 33-D descriptors (0.45s)
  ✓ Submap 0: Added as first submap (base)
============================================================
```

### Output Files:

```
submaps/tb3_1/
├── global_map.pcd                  # Full global map
├── submaps/
│   ├── submap_0000.pcd            # Individual submaps
│   ├── submap_0000_metadata.npz
│   ├── submap_0001.pcd
│   ├── submap_0001_metadata.npz
│   └── ...
```

---

## Improvements Suggested

### 1. **Adaptive Scan Count**
Instead of fixed 250, adjust based on environment:
```python
if in_corridor:
    scans_per_submap = 150  # Less overlap needed
elif in_open_space:
    scans_per_submap = 300  # More overlap for matching
```

### 2. **Overlap Percentage Check**
Instead of fixed distance (1.5m), use overlap:
```python
def compute_scan_overlap(submap1, submap2):
    """
    Compute percentage of overlapping field of view
    Target: 30-50% overlap for good matching
    """
    pass
```

### 3. **Feature Type Selection**
Different features for different scenarios:
```python
if mostly_planar:  # Corridors, hallways
    use 'geometric'  # Fast, sufficient
elif rich_geometry:  # Furniture, cluttered
    use 'fpfh'  # Better discrimination
elif outdoor:
    use 'shot'  # Most robust
```

### 4. **Multi-Resolution Features**
Extract features at multiple scales:
```python
features_coarse = extract_features(pcd, voxel_size=0.1)  # Quick filtering
features_fine = extract_features(pcd, voxel_size=0.05)   # Precise matching
```

### 5. **Pose Graph Optimization**
After loop closure detection:
```python
# Use g2o or GTSAM to optimize all submap poses
# This distributes error across entire trajectory
pose_graph = PoseGraph()
for submap in submaps:
    pose_graph.add_node(submap['id'], submap['pose_center'])
for loop_closure in loop_closures:
    pose_graph.add_edge(loop_closure['current_id'],
                       loop_closure['matched_id'],
                       loop_closure['transform'])
optimized_poses = pose_graph.optimize()
```

---

## Feature Method Comparison

| Method | Speed | Descriptor Dim | Discriminative | Robust to Noise | Best For |
|--------|-------|----------------|----------------|-----------------|----------|
| **FPFH** | Fast (0.5s) | 33 | Medium | Good | General purpose |
| **SHOT** | Slow (1.5s) | 352 | High | Excellent | Complex scenes |
| **Geometric** | Very Fast (0.1s) | 4 | Low | Fair | Corridors, simple |

---

## Next Steps

### To Complete the System:

1. **Implement Geometric Verification** (loop_closure_detector.py)
   - RANSAC for robust transform estimation
   - ICP refinement
   - Fitness threshold check

2. **Add Pose Graph Optimization**
   - Install g2o or GTSAM
   - Build pose graph from submap poses
   - Add loop closure constraints
   - Optimize and redistribute error

3. **Test Loop Closure**
   - Create test with robot returning to start
   - Verify loop closure detection works
   - Check global map consistency improvement

4. **Multi-Robot Extension**
   - Share submaps between robots
   - Match submaps from different robots
   - Merge maps with relative pose estimation

5. **Add Exploration Algorithm**
   - Frontier detection
   - Information gain calculation
   - Path planning to frontiers

---

## Summary

**What You Have Now:**

✅ Submap creation with fixed 250 scans
✅ Distance threshold (1.5m) to avoid duplicates
✅ Feature extraction (FPFH/Geometric)
✅ Individual submap storage
✅ ICP stitching to global map
✅ Spatial indexing for loop closure
✅ Modular, extensible architecture

**What's Next:**

⚠️ Complete geometric verification (RANSAC + ICP)
⚠️ Implement pose graph optimization
⚠️ Test loop closure on real trajectories
⚠️ Add multi-robot support
⚠️ Implement exploration

**Build Status:** ✅ All files compile successfully!

---

**Questions? Need clarification on any part?**
