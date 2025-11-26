# Mapping Module: Functions and Tunable Parameters

**Date**: 2025-01-20
**Module**: `map_generation`
**Location**: `/home/piyush/thesis_ws/src/map_generation/map_generation/`

---

## Table of Contents

1. [Module Overview](#module-overview)
2. [Core Classes](#core-classes)
3. [Standalone Functions](#standalone-functions)
4. [Tunable Parameters Summary](#tunable-parameters-summary)
5. [Parameter Tuning Guidelines](#parameter-tuning-guidelines)

---

## Module Overview

The mapping module consists of 7 Python files implementing SLAM functionality:

| File | Purpose | Lines of Code |
|------|---------|---------------|
| `local_submap_generator.py` | Main ROS2 node, coordinates mapping pipeline | ~334 |
| `submap_stitcher.py` | ICP-based submap alignment and stitching | ~378 |
| `ekf_lib.py` | Extended Kalman Filter for pose estimation | ~264 |
| `mapping_utils.py` | Coordinate transforms, ICP utilities | ~237 |
| `feature_extractor.py` | FPFH and ISS keypoint extraction | ~200 |
| `loop_closure_detector.py` | Loop detection and pose graph optimization | ~150 |
| `utils.py` | ROS2 message conversions, quaternion math | ~75 |

---

## Core Classes

### 1. `LocalSubmapGenerator` (local_submap_generator.py)

**Purpose**: Main ROS2 node that orchestrates the entire mapping pipeline.

#### Constructor Parameters

```python
def __init__(self):
    # ROS2 Parameters (can be set via launch file or command line)
    self.declare_parameter('robot_name', 'tb3_1')
    self.declare_parameter('scans_per_submap', 50)
    self.declare_parameter('save_directory', './submaps')
    self.declare_parameter('voxel_size', 0.05)
    self.declare_parameter('feature_method', 'hybrid')
    self.declare_parameter('enable_loop_closure', False)
    self.declare_parameter('enable_scan_to_map_icp', True)
```

#### Methods (13)

| Method | Description |
|--------|-------------|
| `__init__()` | Initialize node, subscribers, publishers |
| `imu_callback()` | Process IMU data for EKF prediction |
| `odom_callback()` | Process odometry for EKF update |
| `scan_callback()` | Process LiDAR scans, apply ICP correction |
| `should_create_submap()` | Check if submap creation criteria met |
| `create_submap()` | Create and stitch new submap 
| `shutdown()` | Clean shutdown, save final map |

#### Key Workflow

```
1. odom_callback → EKF update → current_pose
2. imu_callback → EKF predict_imu → refine pose
3. scan_callback → scan_to_map_icp → ICP correction → update EKF
4. scan_callback → accumulate points → should_create_submap?
5. create_submap → SubmapStitcher.add_and_stitch_submap
6. publish_current_submap → ROS2 PointCloud2 message
```

---

### 2. `EKF` (ekf_lib.py)

**Purpose**: Extended Kalman Filter for fusing IMU and odometry.

#### Constructor Parameters

```python
def __init__(self):
    # State: [x, y, theta]
    self.state = np.zeros(3)
    self.P = np.eye(3) * 0.1  # Initial covariance

    # Process noise (motion model uncertainty)
    self.Q = np.diag([0.02, 0.02, 0.1])  # [x, y, theta]

    # Measurement noise (odometry uncertainty)
    self.R = np.diag([0.01, 0.01, 0.05])  # [x, y, theta]

    # IMU-only prediction noise (dead reckoning)
    self.Q_imu = np.diag([0.01, 0.01, 0.02])  # [x, y, theta]
```

#### Methods (12)

| Method | Description |
|--------|-------------|
| `initialize(x, y, theta, vx, vy)` | Initialize state with first odometry |
| `predict(omega, dt)` | Predict step using IMU angular velocity |
| `predict_imu(omega, dt=None)` | IMU-only prediction with auto dt |
| `update(x_meas, y_meas, theta_meas, vx_odom)` | Kalman update from odometry |
| `get_state()` | Return current state as dict |
| `get_covariance()` | Return covariance matrix |
| `get_uncertainty()` | Return standard deviations |
| `get_statistics()` | Return filter statistics |
| `reset()` | Reset to uninitialized state |
| `set_process_noise(σ_x, σ_y, σ_θ)` | Tune Q matrix |
| `set_measurement_noise(σ_x, σ_y, σ_θ)` | Tune R matrix |
| `is_converged(pos_threshold, orient_threshold)` | Check convergence |

#### Key Equations

**Prediction**:
```
x_pred = x + vx·cos(θ)·dt
y_pred = y + vx·sin(θ)·dt
θ_pred = θ + ω·dt

P = F·P·F^T + Q
```

**Update**:
```
K = P·H^T·(H·P·H^T + R)^-1
x_new = x + K·(z - H·x)
P_new = (I - K·H)·P
```

---

### 3. `SubmapStitcher` (submap_stitcher.py)

**Purpose**: Align and stitch submaps using GPU-accelerated ICP.

#### Constructor Parameters

```python
def __init__(self,
             voxel_size: float = 0.05,
             icp_max_correspondence_dist: float = 0.25,
             icp_fitness_threshold: float = 0.45,
             feature_extraction_method: str = 'hybrid',
             enable_loop_closure: bool = False):
```

| Parameter | Default | Description | Tuning Range |
|-----------|---------|-------------|--------------|
| `voxel_size` | 0.05m | Point cloud downsampling resolution | 0.02-0.1m |
| `icp_max_correspondence_dist` | 0.25m | Max distance for point matching in ICP | 0.1-0.5m |
| `icp_fitness_threshold` | 0.45 | Min fitness score to accept ICP result | 0.3-0.7 |
| `feature_extraction_method` | 'hybrid' | Method for loop closure features | 'fpfh', 'iss', 'hybrid' |
| `enable_loop_closure` | False | Enable loop closure detection | True/False |

#### Methods (9)

| Method | Description |
|--------|-------------|
| `process_submap(points, submap_id)` | Downsample and preprocess submap |
| `extract_features(pcd, submap_id)` | Extract features for loop closure |
| `compute_submap_bounds(pcd_tensor)` | Calculate bounding box |
| `register_icp_2d(source, target, initial_guess)` | GPU ICP alignment |
| `add_and_stitch_submap(points, submap_id, ...)` | Main stitching pipeline |
| `save_global_map(filepath)` | Save map to PCD file |
| `get_global_map_points()` | Get map as numpy array |
| `_optimize_pose_graph_with_loop_closure(...)` | Distribute loop closure error |
| `_rebuild_global_map()` | Rebuild map after optimization |

#### ICP Pipeline

```
1. process_submap → voxel_down_sample(voxel_size)
2. register_icp_2d:
   - Point-to-Point ICP
   - Max correspondence distance: icp_max_correspondence_dist
   - Max iterations: 100
   - Convergence: relative_fitness < 1e-6
3. Check fitness > icp_fitness_threshold
4. Reject if translation > 2m or rotation > 30°
5. Transform and merge into global map
```

---

### 4. `FeatureExtractor` (feature_extractor.py)

**Purpose**: Extract geometric features for loop closure detection.

#### Constructor Parameters

```python
def __init__(self,
             method: str = 'hybrid',
             voxel_size: float = 0.05):
```

| Parameter | Default | Description | Options |
|-----------|---------|-------------|---------|
| `method` | 'hybrid' | Feature extraction method | 'fpfh', 'iss', 'hybrid' |
| `voxel_size` | 0.05m | Downsampling resolution | 0.02-0.1m |

#### Methods (5)

| Method | Description |
|--------|-------------|
| `extract(pcd)` | Main feature extraction entry point |
| `_extract_fpfh(pcd, voxel_size)` | Fast Point Feature Histograms |
| `_extract_iss_keypoints(pcd)` | Intrinsic Shape Signatures |
| `_extract_hybrid(pcd, voxel_size)` | ISS keypoints + FPFH descriptors |

#### Feature Methods Comparison

| Method | Speed | Robustness | Best Use Case |
|--------|-------|------------|---------------|
| **FPFH** | Fast | Medium | Dense point clouds, fast loop closure |
| **ISS** | Slow | High | Sparse, distinctive environments |
| **Hybrid** | Medium | High | General purpose (recommended) |

---

### 5. `LoopClosureDetector` (loop_closure_detector.py)

**Purpose**: Detect when robot revisits previous locations.

#### Constructor Parameters

```python
def __init__(self,
             spatial_search_radius: float = 5.0,
             scan_context_threshold: float = 0.7,
             min_feature_matches: int = 15,
             ransac_threshold: float = 0.1,
             icp_fitness_threshold: float = 0.3):
```

| Parameter | Default | Description | Tuning Range |
|-----------|---------|-------------|--------------|
| `spatial_search_radius` | 5.0m | Search radius for candidate loops | 3.0-10.0m |
| `scan_context_threshold` | 0.7 | Min similarity for scan context | 0.6-0.9 |
| `min_feature_matches` | 15 | Min feature correspondences | 10-30 |
| `ransac_threshold` | 0.1m | RANSAC inlier threshold | 0.05-0.2m |
| `icp_fitness_threshold` | 0.3 | Min ICP fitness for loop | 0.2-0.5 |

#### Methods (4)

| Method | Description |
|--------|-------------|
| `add_submap_to_index(submap_id, position)` | Add to spatial index |
| `detect_loop_closure(current_submap, submaps, ...)` | Multi-stage loop detection |
| `_find_spatial_candidates(position, submaps, ...)` | KD-tree spatial search |
| `_verify_loop_with_icp(current_pcd, candidate_pcd)` | ICP verification |

#### Loop Detection Pipeline

```
1. Spatial search (KD-tree) → candidates within radius
2. Scan context matching → filter by similarity
3. Feature matching (FPFH) → geometric verification
4. RANSAC → outlier rejection
5. ICP refinement → precise alignment
6. If fitness > threshold → Loop detected!
```

---

## Standalone Functions

### mapping_utils.py

#### `scan_to_world_points_with_lidar_offset(scan_msg, pose)`

**Purpose**: Convert LiDAR scan to world frame with sensor offset correction.

**Parameters**:
- `scan_msg`: ROS LaserScan message
- `pose`: Robot pose dict with keys: `['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']`

**Algorithm**:
```python
1. Filter valid ranges (range_min < r < range_max, finite)
2. Convert polar (angle, range) → Cartesian (x, y) in LiDAR frame
3. Apply LiDAR offset: [-0.064, 0.0, 0.121] meters
4. Transform to world frame using quaternion rotation + translation
```

**Returns**: Nx3 numpy array of world points

---

#### `scan_to_map_icp(scan_points, accumulated_points, device, voxel_size, logger)`

**Purpose**: GPU-accelerated ICP for scan-to-map drift correction.

**Parameters**:
- `scan_points_world`: Current scan in world frame (Nx3)
- `accumulated_points_world`: Previous scans (Mx3)
- `device`: Open3D device (CUDA or CPU)
- `voxel_size`: Downsampling resolution
- `logger`: ROS2 logger (optional)

**Algorithm**:
```python
1. Check minimum points (50 for both source and target)
2. Convert to GPU tensors
3. Voxel downsample (2× voxel_size for speed)
4. Point-to-Point ICP:
   - Max correspondence: 0.1m
   - Max iterations: 20
5. Constrain to 2D (zero out Z components)
6. Accept if fitness > 0.25
```

**Returns**: `(corrected_points, pose_correction_dict)` or `(points, None)`

**Pose correction dict**:
```python
{
    'dx': float,      # X translation correction (m)
    'dy': float,      # Y translation correction (m)
    'dtheta': float,  # Rotation correction (rad)
    'fitness': float  # ICP fitness score (0-1)
}
```

---

#### `match_scan_context(sc1, sc2, num_sectors=60)`

**Purpose**: Rotation-invariant place recognition using scan context.

**Algorithm**:
```python
1. Reshape scan contexts to 2D grid (rings × sectors)
2. Try all circular shifts (0 to num_sectors)
3. Compute cosine similarity for each shift
4. Return best similarity and corresponding shift
```

**Returns**: `(best_similarity, best_shift)`

---

#### `match_geometric_features(descriptors1, descriptors2, max_distance=0.75)`

**Purpose**: Match FPFH feature descriptors using nearest neighbor search.

**Algorithm**:
```python
1. Build KD-tree for descriptors2
2. For each descriptor in set1, find nearest in set2
3. Filter matches by distance threshold
```

**Returns**: Nx3 array `[[idx1, idx2, distance], ...]`

---

#### `estimate_transform_from_points(source, target)`

**Purpose**: Compute 2D rigid transformation from point correspondences.

**Algorithm** (SVD-based):
```python
1. Center both point clouds
2. Compute cross-covariance H = source_centered^T @ target_centered
3. SVD: H = U @ Σ @ V^T
4. Rotation: R = V @ U^T
5. Translation: t = target_center - R @ source_center
```

**Returns**: 4×4 homogeneous transformation matrix

---

#### `estimate_transform_from_poses(pose_from, pose_to)`

**Purpose**: Compute relative transformation between two poses.

**Returns**: 4×4 transformation matrix `T_rel = T_to @ inv(T_from)`

---

### utils.py

#### `numpy_to_pointcloud2(points, frame_id, stamp)`

**Purpose**: Convert numpy array to ROS2 PointCloud2 message.

**Algorithm**:
```python
1. Define fields: [x, y, z] as FLOAT32
2. Flatten points to 1D byte array
3. Build PointCloud2 message with proper metadata
```

---

#### `quaternion_to_yaw(qx, qy, qz, qw)`

**Purpose**: Extract yaw angle from quaternion (2D orientation).

**Formula**:
```python
yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))
```

---

#### `yaw_to_quaternion(yaw)`

**Purpose**: Convert yaw angle to quaternion.

**Formula**:
```python
qx = 0
qy = 0
qz = sin(yaw/2)
qw = cos(yaw/2)
```

---

#### `quaternion_to_rotation_matrix(qx, qy, qz, qw)`

**Purpose**: Convert quaternion to 3×3 rotation matrix.

**Formula**:
```python
R = [[1-2(qy²+qz²),  2(qx*qy-qw*qz),  2(qx*qz+qw*qy)],
     [2(qx*qy+qw*qz), 1-2(qx²+qz²),    2(qy*qz-qw*qx)],
     [2(qx*qz-qw*qy), 2(qy*qz+qw*qx),  1-2(qx²+qy²)]]
```

---

## Tunable Parameters Summary

### 🎯 High-Impact Parameters (Tune These First)

| Parameter | Location | Default | Impact | Tuning Range |
|-----------|----------|---------|--------|--------------|
| **scans_per_submap** | LocalSubmapGenerator | 50 | Submap size/overlap | 30-100 |
| **voxel_size** | SubmapStitcher | 0.05m | Map resolution | 0.02-0.1m |
| **icp_fitness_threshold** | SubmapStitcher | 0.45 | ICP acceptance | 0.3-0.7 |
| **Q (process noise)** | EKF | [0.02, 0.02, 0.1] | Model trust | 0.01-0.05 |
| **R (measurement noise)** | EKF | [0.01, 0.01, 0.05] | Odom trust | 0.005-0.02 |
| **enable_scan_to_map_icp** | LocalSubmapGenerator | True | Short-term drift correction | True/False |

---

### 🔧 Medium-Impact Parameters

| Parameter | Location | Default | Impact | Tuning Range |
|-----------|----------|---------|--------|--------------|
| **icp_max_correspondence_dist** | SubmapStitcher | 0.25m | ICP convergence | 0.1-0.5m |
| **Q_imu (IMU noise)** | EKF | [0.01, 0.01, 0.02] | Dead reckoning trust | 0.005-0.05 |
| **enable_loop_closure** | SubmapStitcher | False | Loop closure | True/False |
| **feature_method** | FeatureExtractor | 'hybrid' | Loop detection | fpfh/iss/hybrid |
| **spatial_search_radius** | LoopClosureDetector | 5.0m | Loop candidate search | 3-10m |

---

### ⚙️ Low-Impact Parameters (Fine-Tuning)

| Parameter | Location | Default | Impact | Tuning Range |
|-----------|----------|---------|--------|--------------|
| **scan_context_threshold** | LoopClosureDetector | 0.7 | Place recognition | 0.6-0.9 |
| **min_feature_matches** | LoopClosureDetector | 15 | Feature matching | 10-30 |
| **ransac_threshold** | LoopClosureDetector | 0.1m | Outlier rejection | 0.05-0.2m |
| **save_directory** | LocalSubmapGenerator | './submaps' | Map save location | Any path |
| **robot_name** | LocalSubmapGenerator | 'tb3_1' | ROS2 namespace | Any string |

---

## Parameter Tuning Guidelines

### 🎯 Scenario-Based Tuning

#### **Small Indoor Rooms (< 5m×5m)**
```python
scans_per_submap = 30          # Smaller submaps for tight spaces
voxel_size = 0.03              # Higher resolution
icp_fitness_threshold = 0.5    # Stricter acceptance
Q = [0.015, 0.015, 0.08]       # Trust motion model more
```

#### **Large Open Spaces (> 10m×10m)**
```python
scans_per_submap = 75          # Larger submaps for coverage
voxel_size = 0.08              # Lower resolution for speed
icp_fitness_threshold = 0.4    # More lenient acceptance
Q = [0.03, 0.03, 0.15]         # Less trust in motion model
```

#### **Narrow Corridors**
```python
scans_per_submap = 50          # Standard
voxel_size = 0.04              # Medium-high resolution
icp_fitness_threshold = 0.35   # More lenient (poor constraints)
Q = [0.025, 0.025, 0.12]       # Higher process noise
```

#### **Cluttered Environments (Furniture, Obstacles)**
```python
scans_per_submap = 40          # More frequent submaps
voxel_size = 0.04              # Higher resolution
icp_fitness_threshold = 0.5    # Stricter (good features)
enable_loop_closure = True     # Helpful for revisiting
```

---

### 🔍 Diagnosis: Parameter Adjustment

#### **Problem: Maps have donut/spiral shape**
**Cause**: Drift accumulation, poor ICP correction

**Solution**:
```python
enable_scan_to_map_icp = True     # Ensure enabled
icp_fitness_threshold = 0.35      # Lower threshold (accept more)
Q = [0.02, 0.02, 0.1]             # Increase process noise
R = [0.01, 0.01, 0.05]            # Increase measurement noise
```

---

#### **Problem: ICP fails frequently (low fitness)**
**Cause**: Poor correspondences, feature-poor environment

**Solution**:
```python
icp_max_correspondence_dist = 0.3  # Increase search radius
icp_fitness_threshold = 0.35       # Lower acceptance threshold
voxel_size = 0.06                  # Reduce resolution (faster)
```

---

#### **Problem: Map is too sparse/low resolution**
**Cause**: Aggressive downsampling

**Solution**:
```python
voxel_size = 0.03                  # Decrease (higher resolution)
scans_per_submap = 40              # More frequent submaps
```

---

#### **Problem: Map is too dense/slow processing**
**Cause**: Too many points

**Solution**:
```python
voxel_size = 0.08                  # Increase (lower resolution)
scans_per_submap = 75              # Less frequent submaps
```

---

#### **Problem: EKF diverges (uncertainty grows unbounded)**
**Cause**: Process noise too high, no measurement updates

**Solution**:
```python
Q = [0.015, 0.015, 0.08]           # Decrease process noise
R = [0.008, 0.008, 0.04]           # Trust odometry more
enable_scan_to_map_icp = True      # Enable ICP corrections
```

---

#### **Problem: Loop closure not detecting loops**
**Cause**: Threshold too strict, poor features

**Solution**:
```python
enable_loop_closure = True         # Ensure enabled
spatial_search_radius = 7.0        # Larger search radius
scan_context_threshold = 0.65      # Lower threshold
min_feature_matches = 10           # Fewer required matches
icp_fitness_threshold = 0.25       # More lenient ICP
```

---

### 📊 Performance vs Quality Trade-offs

| Priority | Configuration | Use Case |
|----------|---------------|----------|
| **Max Speed** | voxel_size=0.1, scans_per_submap=100, loop_closure=False | Fast exploration |
| **Max Quality** | voxel_size=0.02, scans_per_submap=30, loop_closure=True | Detailed mapping |
| **Balanced** | voxel_size=0.05, scans_per_submap=50, loop_closure=False | General purpose |

---

### 🧪 Experimental Parameters

**For advanced users only**:

```python
# EKF: Very aggressive process noise (trust measurements more)
Q = [0.05, 0.05, 0.2]

# ICP: Very strict acceptance (only perfect matches)
icp_fitness_threshold = 0.7

# Loop closure: Very loose constraints (detect everything)
scan_context_threshold = 0.5
icp_fitness_threshold_loop = 0.2
```

---

## Summary Statistics

**Total Classes**: 5
**Total Standalone Functions**: 8
**Total Methods**: 50+
**Tunable Parameters**: 20+
**ROS2 Parameters**: 7

**Core Pipeline**:
```
LiDAR → scan_to_world_points → scan_to_map_icp → EKF update →
accumulate → create_submap → SubmapStitcher.add_and_stitch →
register_icp_2d → global_map → publish
```

**Key Files to Modify for Tuning**:
1. `local_submap_generator.py` - Main parameters (lines 35-42)
2. `ekf_lib.py` - Noise covariances (lines 24-50)
3. `submap_stitcher.py` - ICP thresholds (lines 15-20)

---

**End of Document**
