# Mapping Module: Functions, Data Types, and Parameters

**Date**: 2025-01-18 (Updated)
**Module**: `map_generation`
**Location**: `/home/piyush/thesis_ws/src/map_generation/map_generation/`

---

## Table of Contents

1. [Module Overview](#module-overview)
2. [Data Type Flow & Memory Architecture](#data-type-flow--memory-architecture)
3. [Core Classes with Data Types](#core-classes-with-data-types)
4. [Standalone Functions with Data Types](#standalone-functions-with-data-types)
5. [GPU vs CPU Processing Summary](#gpu-vs-cpu-processing-summary)
6. [Tunable Parameters Summary](#tunable-parameters-summary)
7. [Parameter Tuning Guidelines](#parameter-tuning-guidelines)

---

## Module Overview

The mapping module consists of 8 Python files implementing GPU-accelerated SLAM functionality:

| File | Purpose | Lines | GPU/CPU |
|------|---------|-------|---------|
| `local_submap_generator.py` | Main ROS2 node, coordinates mapping pipeline | ~580 | Both |
| `submap_stitcher.py` | ICP-based submap alignment and stitching | ~383 | GPU |
| `mapping_utils.py` | Coordinate transforms, scan-to-map ICP | ~374 | GPU+CPU |
| `ekf_lib.py` | Extended Kalman Filter for pose estimation | ~203 | CPU |
| `feature_extractor.py` | FPFH and geometric feature extraction | ~258 | GPU→CPU |
| `utils.py` | ROS2 message conversions, quaternion math | ~73 | CPU |

---

## Data Type Flow & Memory Architecture

### **Complete Data Pipeline:**

```
┌─────────────────────────────────────────────────────────────────────┐
│ SENSOR INPUT                                                         │
├─────────────────────────────────────────────────────────────────────┤
│ LaserScan (ROS msg) → CPU RAM                                       │
└─────────────────────────────────────────────────────────────────────┘
                        ↓ transform_scan_to_relative_frame()
┌─────────────────────────────────────────────────────────────────────┐
│ SCAN PROCESSING (CPU)                                                │
├─────────────────────────────────────────────────────────────────────┤
│ np.ndarray (Nx3, float32) → CPU RAM                                 │
│ Operations: Polar→Cartesian, filtering, transforms                  │
└─────────────────────────────────────────────────────────────────────┘
                        ↓ Accumulate scans
┌─────────────────────────────────────────────────────────────────────┐
│ SUBMAP ACCUMULATION (CPU)                                            │
├─────────────────────────────────────────────────────────────────────┤
│ List[np.ndarray] → Stack → np.ndarray (Mx3, float32) → CPU RAM     │
└─────────────────────────────────────────────────────────────────────┘
                        ↓ process_submap()
┌─────────────────────────────────────────────────────────────────────┐
│ GPU TRANSFER & PROCESSING                                            │
├─────────────────────────────────────────────────────────────────────┤
│ np.ndarray → o3c.Tensor (GPU VRAM) → o3d.t.geometry.PointCloud     │
│                                                                       │
│ GPU Operations:                                                      │
│ • Voxel downsampling                                                │
│ • ICP registration (Point-to-Point)                                 │
│ • Feature extraction (NNS, covariance)                              │
│ • Point cloud transformations                                        │
│ • Point concatenation                                               │
└─────────────────────────────────────────────────────────────────────┘
                        ↓ get_global_map_points()
┌─────────────────────────────────────────────────────────────────────┐
│ GPU→CPU TRANSFER                                                     │
├─────────────────────────────────────────────────────────────────────┤
│ o3c.Tensor.cpu().numpy() → np.ndarray (Px3, float32) → CPU RAM     │
│ Cost: ~0.5-1ms for 2000 points                                      │
└─────────────────────────────────────────────────────────────────────┘
                        ↓ numpy_to_pointcloud2()
┌─────────────────────────────────────────────────────────────────────┐
│ ROS MESSAGE PUBLISHING                                               │
├─────────────────────────────────────────────────────────────────────┤
│ PointCloud2 (ROS msg) → Published to /{robot_name}/global_map      │
└─────────────────────────────────────────────────────────────────────┘
```

### **Data Type Legend:**

| Type | Storage | Description | Example Use |
|------|---------|-------------|-------------|
| `np.ndarray` | CPU RAM | NumPy array (float32/float64) | Point coordinates, transforms |
| `o3c.Tensor` | GPU VRAM or CPU | Open3D core tensor (GPU-capable) | Internal GPU computations |
| `o3d.t.geometry.PointCloud` | GPU VRAM or CPU | Open3D tensor point cloud | ICP, voxelization, features |
| `dict` | CPU RAM | Python dictionary | Pose data, metadata |
| `LaserScan` | CPU RAM | ROS2 sensor message | LiDAR input |
| `PointCloud2` | CPU RAM | ROS2 point cloud message | Inter-process communication |
| `o3c.Device` | N/A | Device descriptor ("CUDA:0" or "CPU:0") | Specifies compute target |

### **Memory Transfer Costs:**

| Operation | Direction | Time (2000 pts) | Overhead |
|-----------|-----------|-----------------|----------|
| numpy → tensor | CPU → GPU | ~0.5ms | Low |
| tensor → numpy | GPU → CPU | ~0.5ms | Low |
| Total round-trip | CPU→GPU→CPU | ~1ms | Acceptable |

### **When GPU is Used:**

```
✅ GPU-Accelerated:
   • ICP registration (scan-to-map, submap-to-global)
   • Voxel downsampling
   • Point cloud transformations
   • Nearest neighbor search (feature extraction)
   • Loop closure ICP refinement

❌ CPU-Only:
   • EKF prediction/update
   • LaserScan parsing
   • Quaternion math
   • ROS message conversions
```

---

## Core Classes with Data Types

### 1. `LocalSubmapGenerator` (local_submap_generator.py)

**Purpose**: Main ROS2 node orchestrating the mapping pipeline.

#### Constructor Parameters

```python
def __init__(self):
    # ROS2 Parameters (configurable via launch file)
    self.declare_parameter('robot_name', 'tb3_1')                  # str
    self.declare_parameter('scans_per_submap', 80)                 # int
    self.declare_parameter('save_directory', './submaps')          # str
    self.declare_parameter('voxel_size', 0.05)                     # float
    self.declare_parameter('feature_method', 'hybrid')             # str
```

#### Key Methods with Data Types

| Method | Input Types | Output Types | Processing |
|--------|-------------|--------------|------------|
| `imu_callback()` | `Imu` (ROS msg) | None | CPU |
| `odom_callback()` | `Odometry` (ROS msg) | None | CPU |
| `cmd_vel_callback()` | `Twist` (ROS msg) | None | CPU |
| `joint_states_callback()` | `JointState` (ROS msg) | None | CPU |
| `scan_callback()` | `LaserScan` (ROS msg) | None | CPU+GPU |
| `should_create_submap()` | None | `bool` | CPU |
| `create_submap()` | None | None | GPU |
| `shutdown()` | None | None | GPU→Disk |
| `_publish_ekf_pose()` | None | None | CPU |
| `_publish_global_map_callback()` | None | None | CPU |

#### Key Workflow

```
1. odom_callback(Odometry) → EKF.update() → current_pose (dict)
2. imu_callback(Imu) → EKF.predict_imu() → refine pose
3. scan_callback(LaserScan):
   a. transform_scan_to_relative_frame() → points_local (np.ndarray Nx3)
   b. scan_to_map_icp() → points_corrected (np.ndarray), pose_correction (dict)
   c. Append to current_submap_points (List[np.ndarray])
4. create_submap():
   a. Stack points → np.ndarray (Mx3)
   b. SubmapStitcher.integrate_submap_to_global_map()
   c. GPU: process_submap() → ICP → merge into global map
5. publish_global_map() → PointCloud2 (ROS msg)
```

---

### 2. `EKF` (ekf_lib.py)

**Purpose**: Extended Kalman Filter for sensor fusion (CPU-based).

#### Constructor Parameters

```python
def __init__(self):
    # State: np.ndarray (3,) = [x, y, theta]
    self.state = np.zeros(3)

    # Covariance: np.ndarray (3x3)
    self.P = np.eye(3) * 0.1

    # Velocities: float
    self.vx = 0.0
    self.vy = 0.0

    # Measurement noise matrices: np.ndarray (3x3)
    self.R_odom = np.diag([0.005, 0.005, 0.02])          # Odometry noise
    self.R_icp = np.diag([0.0001, 0.0001, 0.0001])       # ICP noise (very accurate)

    # Process noise: np.ndarray (3x3)
    self.Q_imu = np.diag([0.0001, 0.0001, 0.0001])       # IMU prediction noise

    # Time step: float
    self.dt = 0.005  # 200 Hz
```

#### Methods with Data Types

| Method | Input Types | Output Types | Processing |
|--------|-------------|--------------|------------|
| `initialize()` | `float × 5` (x, y, θ, vx, vy) | None | CPU |
| `predict_imu()` | `float` (ω) | None | CPU |
| `update()` | `float × 3, Optional[float], str` | None | CPU |
| `get_state()` | None | `dict` {'x', 'y', 'theta', 'vx', 'vy'} | CPU |
| `get_covariance()` | None | `np.ndarray (3x3)` | CPU |
| `get_uncertainty()` | None | `dict` {'sigma_x', 'sigma_y', 'sigma_theta'} | CPU |
| `get_statistics()` | None | `dict` | CPU |
| `reset()` | None | None | CPU |
| `set_measurement_noise()` | `float × 3, str` | None | CPU |
| `is_converged()` | `float × 2` | `bool` | CPU |

#### Key Equations

**Prediction (IMU):**
```python
x_pred = x + vx·cos(θ)·dt
y_pred = y + vx·sin(θ)·dt
θ_pred = θ + ω·dt
P = F @ P @ F.T + Q_imu
```

```python
K = P @ H.T @ (H @ P @ H.T + R)^(-1)  # Kalman gain
state = state + K @ (z - H @ state)     # State update
P = (I - K @ H) @ P @ (I - K @ H).T + K @ R @ K.T  # Covariance update
```

---

### 3. `SubmapStitcher` (submap_stitcher.py)

**Purpose**: GPU-accelerated submap alignment and global map management.

#### Constructor Parameters

```python
def __init__(self,
             voxel_size: float = 0.05,                          # m
             icp_max_correspondence_dist: float = 0.1,          # m
             icp_fitness_threshold: float = 0.45,               # ratio
             feature_extraction_method: str = 'hybrid',         # 'fpfh'|'iss'|'hybrid'
```

| Parameter | Default | Type | Range | Impact |
|-----------|---------|------|-------|--------|
| `voxel_size` | 0.05m | `float` | 0.02-0.1m | Map resolution |
| `icp_max_correspondence_dist` | 0.1m | `float` | 0.05-0.3m | ICP convergence |
| `icp_fitness_threshold` | 0.45 | `float` | 0.3-0.7 | ICP acceptance |
| `feature_extraction_method` | 'hybrid' | `str` | 'fpfh', 'iss', 'hybrid' | Loop closure |

#### Methods with Data Types

| Method | Input Types | Output Types | Device |
|--------|-------------|--------------|--------|
| `process_submap()` | `np.ndarray (Nx3, float32), int` | `o3d.t.geometry.PointCloud` | GPU |
| `extract_features()` | `o3d.t.geometry.PointCloud, int` | `Optional[Dict]` | GPU→CPU |
| `align_submap_with_icp()` | `o3d.t.geometry.PointCloud × 2, Optional[np.ndarray (4x4)]` | `Tuple[bool, np.ndarray (4x4), float]` | GPU |
| `integrate_submap_to_global_map()` | `np.ndarray (Nx3), int, dict × 3, np.ndarray (4x4)` | `Tuple[bool, Optional[dict]]` | GPU |
| `save_global_map()` | `str` (filepath) | None | GPU→Disk |
| `get_global_map_points()` | None | `Optional[np.ndarray (Px3, float32)]` | GPU→CPU (cached) |
| `_rebuild_global_map()` | None | None | GPU |

#### Data Type Flow in `integrate_submap_to_global_map()`:

```python
Input:  points (np.ndarray Nx3, float32, CPU RAM)
   ↓ process_submap()
Step 1: o3c.Tensor (Nx3, float32, GPU VRAM)
   ↓ voxel_down_sample()
Step 2: o3d.t.geometry.PointCloud (Mx3, GPU VRAM)  # M < N
   ↓ align_submap_with_icp()
Step 3: ICP registration (GPU)
   ↓ reg_result.transformation.cpu().numpy()
Step 4: np.ndarray (4x4, float64, CPU RAM)  # Transform matrix
   ↓ transform() + concatenate()
Step 5: Update global_map_tensor (GPU VRAM)
   ↓ voxel_down_sample()
Output: global_map_tensor (o3d.t.geometry.PointCloud, GPU VRAM)
```

#### ICP Pipeline

```
1. Input: source (o3d.t.geometry.PointCloud, GPU)
           target (o3d.t.geometry.PointCloud, GPU)
2. o3d.t.pipelines.registration.icp():
   - TransformationEstimationPointToPoint()
   - Max correspondence: icp_max_correspondence_dist (0.1m)
   - Max iterations: 50
   - Convergence: relative_fitness < 1e-6
3. Constrain to 2D: Zero out Z translation/rotation
4. Reject if: fitness < icp_fitness_threshold (0.45)
5. Reject if: translation > 1.0m or rotation > 20°
6. Output: transform (np.ndarray 4x4), fitness (float)
```

---

### 4. `FeatureExtractor` (feature_extractor.py)


#### Constructor Parameters

```python
def __init__(self,
             method: str = 'hybrid'):  # 'fpfh' | 'iss' | 'hybrid' | 'scan_context' | 'geometric'
```

#### Methods with Data Types

| Method | Input Types | Output Types | Device |
|--------|-------------|--------------|--------|
| `extract()` | `o3d.t.geometry.PointCloud` | `Dict` | GPU→CPU |
| `_extract_scan_context()` | `o3d.t.geometry.PointCloud` | `Tuple[np.ndarray (1, 1200), Dict]` | GPU→CPU |
| `_extract_keypoints_uniform()` | `o3d.t.geometry.PointCloud, float` | `np.ndarray (K,) int` | GPU→CPU |
| `_extract_geometric()` | `o3d.t.geometry.PointCloud, np.ndarray` | `Tuple[np.ndarray (K, 4), Dict]` | GPU→CPU |

#### Feature Methods Comparison

| Method | Output | Speed | Device | Best Use Case |
|--------|--------|-------|--------|---------------|
| **scan_context** | np.ndarray (1, 1200) | Fast | GPU→CPU | Global localization |
| **geometric** | np.ndarray (K, 4) | Medium | GPU (NNS) → CPU | Local features |
| **hybrid** | Both above | Medium | GPU→CPU | General purpose (recommended) |


```python
# Parameters
max_range: 10.0m
num_rings: 20
num_sectors: 60
# Output: np.ndarray (1, 1200) = (1, num_rings × num_sectors)
# Encodes: Occupancy in polar bins (rotation-invariant)
```

#### Geometric Descriptor (Per Keypoint)

```python
# For each keypoint (uniformly sampled):
# 1. GPU NNS: Find 30 nearest neighbors
# 2. CPU: Compute covariance matrix
# 3. CPU: Extract eigenvalues λ1 ≥ λ2 ≥ λ3
# 4. CPU: Compute features:
#    - linearity = (λ1 - λ2) / λ1
#    - planarity = (λ2 - λ3) / λ1
#    - scattering = λ3 / λ1
#    - avg_dist = mean neighbor distance
# Output: np.ndarray (4,) per keypoint
```

---



#### Constructor Parameters

```python
def __init__(self,
             spatial_search_radius: float = 5.0,                # m
             scan_context_threshold: float = 0.7,               # similarity
             min_feature_matches: int = 15,                     # count
             ransac_threshold: float = 0.1,                     # m
             icp_fitness_threshold: float = 0.3):               # ratio
```

#### Methods with Data Types

| Method | Input Types | Output Types | Device |
|--------|-------------|--------------|--------|
| `add_submap_to_index()` | `int, np.ndarray (2,)` | None | CPU |
| `_stage1_coarse_matching()` | `Dict, List[Dict], float` | `List[Dict]` | CPU |
| `_stage2_fine_verification()` | `Dict, List[Dict]` | `Optional[Dict]` | GPU+CPU |
| `_ransac_2d()` | `np.ndarray (N, 3) × 2, int` | `Tuple[np.ndarray (4x4), np.ndarray (N,) bool]` | CPU |
| `_estimate_2d_transform()` | `np.ndarray (M, 3) × 2` | `np.ndarray (4x4)` | CPU |
| `_refine_with_icp()` | `o3d.t.geometry.PointCloud × 2, np.ndarray (4x4)` | `Tuple[np.ndarray (4x4), float]` | GPU |
| `get_statistics()` | None | `dict` | CPU |

#### Loop Detection Pipeline

```
Stage 1: Coarse Matching (CPU)
├─ Spatial filtering (KD-tree): candidates within 5m radius
└─ Output: Top 5 candidates

Stage 2: Fine Verification (GPU+CPU)
├─ Geometric feature matching (CPU KD-tree)
├─ RANSAC (CPU): Estimate transform, filter outliers
├─ ICP refinement (GPU): Precise alignment
└─ Accept if: fitness > 0.3, inliers ≥ 15

Output: Loop closure dict with transform (np.ndarray 4x4)
```

---



#### Constructor Parameters

```python
def __init__(self,
             translation_noise_sigma: float = 0.1,              # m
             rotation_noise_sigma: float = 0.05,                # rad
             max_translation: float = 5.0,                      # m
             max_rotation: float = np.pi):                      # rad
```

#### Methods with Data Types

| Method | Input Types | Output Types | Device |
|--------|-------------|--------------|--------|
| `optimize_pose_graph()` | `List[Dict], Dict` | `Optional[List[np.ndarray (4x4)]]` | CPU |
| `get_statistics()` | None | `Dict` | CPU |

#### Optimization Process

```python
   ↓
   ↓
Step 2: Levenberg-Marquardt optimization
   - Max iterations: 100
   - Convergence: relative_error < 1e-5
   ↓
Step 3: Extract optimized poses
   ↓
Output: List[np.ndarray (4x4)] - optimized transforms
```

---

## Standalone Functions with Data Types

### mapping_utils.py (9 functions)

#### `scan_to_map_icp()`

**Signature:**
```python
def scan_to_map_icp(
    scan_points_world: np.ndarray,        # (Nx3, float32) - Current scan
    accumulated_points_world: np.ndarray, # (Mx3, float32) - Previous scans
    device: o3c.Device,                   # CUDA:0 or CPU:0
    voxel_size: float,                    # Downsampling resolution
    logger=None                           # ROS2 logger (optional)
) -> Tuple[np.ndarray, Optional[dict]]:  # (corrected_points, pose_correction)
```

**Data Flow:**
```
Input:  np.ndarray (CPU RAM)
   ↓ o3c.Tensor(..., device=device)
GPU:    o3c.Tensor (GPU VRAM)
   ↓ o3d.t.geometry.PointCloud
GPU:    Point-to-Point ICP (GPU)
   ↓ reg_result.transformation.cpu().numpy()
Output: np.ndarray (CPU RAM), dict
```

**Device**: GPU if CUDA available, else CPU
**Time**: ~15-25ms (GPU), ~60-120ms (CPU) for 2000 points

---

#### `match_scan_context_cosine()`

**Signature:**
```python
def match_scan_context_cosine(
    num_rings: int = 20,      # Number of rings
    num_sectors: int = 60     # Number of sectors
) -> Tuple[float, int]:      # (best_similarity, best_shift)
```

**Device**: CPU
**Time**: ~2-5ms
**Algorithm**: Rotation-invariant cosine similarity with circular shifts

---

#### `match_scan_context_with_voting()`

**Signature:**
```python
def match_scan_context_with_voting(
    num_rings: int = 20,
    num_sectors: int = 60,
    threshold: float = 0.5          # Minimum agreement ratio
) -> Tuple[float, int, Optional[np.ndarray]]:  # (agreement, shift, mask)
```

**Device**: CPU
**Time**: ~3-8ms
**Algorithm**: Binary occupancy voting (both occupied or both empty)

---

#### `is_distinctive_submap()`

**Signature:**
```python
def is_distinctive_submap(
    geometric_descriptors: np.ndarray,  # (K, 4) - Geometric features
    min_distinctiveness: float = 0.25,  # Feature variance threshold
    min_keypoints: int = 15             # Minimum number of keypoints
) -> Tuple[bool, Dict]:                # (is_distinctive, metrics)
```

**Device**: CPU
**Time**: ~1-2ms

---

#### `match_geometric_features()`

**Signature:**
```python
def match_geometric_features(
    descriptors1: np.ndarray,  # (K1, 4) - Features from submap 1
    descriptors2: np.ndarray,  # (K2, 4) - Features from submap 2
    max_distance: float = 0.75 # Maximum descriptor distance
) -> np.ndarray:               # (M, 3) - [[idx1, idx2, distance], ...]
```

**Device**: CPU
**Time**: ~2-5ms
**Algorithm**: KD-tree nearest neighbor search in descriptor space

---

#### `publish_global_map()`

**Signature:**
```python
def publish_global_map(
    global_points: Optional[np.ndarray],  # (Px3, float32) - Map points
    publisher,                            # ROS2 publisher
    clock,                                # ROS2 clock
    frame_id: str                         # Frame ID (e.g., 'tb3_1/odom')
) -> None:
```

**Device**: CPU
**Time**: ~5-10ms for 2000 points
**Purpose**: Convert numpy → PointCloud2 → publish

---

#### `compute_relative_pose()`

**Signature:**
```python
def compute_relative_pose(
    current_pose: dict,   # {'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'}
    reference_pose: dict  # Same structure
) -> dict:               # Relative pose in same format
```

**Device**: CPU
**Time**: <1ms
**Purpose**: Compute T_ref_to_current = inv(T_ref) @ T_current

---

#### `transform_scan_to_relative_frame()`

**Signature:**
```python
def transform_scan_to_relative_frame(
    scan_msg,              # LaserScan (ROS msg)
    relative_pose: dict    # {'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'}
) -> np.ndarray:          # (Nx3, float32) - Points in relative frame
```

**Device**: CPU
**Time**: ~3-8ms
**Algorithm**:
```
1. Parse LaserScan: polar → Cartesian
2. Filter valid ranges
3. Apply LiDAR offset: [-0.064, 0.0, 0.121] m
4. Transform to relative frame using quaternion rotation
```

---

### utils.py (3 functions)

#### `numpy_to_pointcloud2()`

**Signature:**
```python
def numpy_to_pointcloud2(
    points: np.ndarray,  # (Nx3, float32) - Point coordinates
    frame_id: str,       # ROS frame ID
    stamp                # ROS timestamp
) -> PointCloud2:       # ROS2 PointCloud2 message
```

**Device**: CPU
**Time**: ~3-5ms for 2000 points
**Algorithm**: Flatten numpy array → convert to bytes → populate PointCloud2 fields

---

#### `quaternion_to_yaw()`

**Signature:**
```python
def quaternion_to_yaw(
    qx: float, qy: float, qz: float, qw: float
) -> float:  # Yaw angle in radians
```

**Device**: CPU
**Time**: <0.1ms
**Formula**: `yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy² + qz²))`

---

#### `yaw_to_quaternion()`

**Signature:**
```python
def yaw_to_quaternion(
    yaw: float  # Yaw angle in radians
) -> tuple:    # (qx, qy, qz, qw)
```

**Device**: CPU
**Time**: <0.1ms
**Formula**: `qx=0, qy=0, qz=sin(yaw/2), qw=cos(yaw/2)`

---

#### `quaternion_to_rotation_matrix()`

**Signature:**
```python
def quaternion_to_rotation_matrix(
    qx: float, qy: float, qz: float, qw: float
) -> np.ndarray:  # (3x3) rotation matrix
```

**Device**: CPU
**Time**: <0.1ms
**Formula**: Standard quaternion-to-rotation-matrix conversion

---

## GPU vs CPU Processing Summary

### **GPU-Accelerated Operations (CUDA)**

| Operation | Function/Method | Input Type | Output Type | Speedup |
|-----------|----------------|------------|-------------|---------|
| **ICP Registration** | `scan_to_map_icp()` | np.ndarray → o3c.Tensor | np.ndarray | 3-5x |
| **Voxel Downsampling** | `process_submap()` | o3d.t.geometry.PointCloud | o3d.t.geometry.PointCloud | 4-6x |
| **Point Cloud Transform** | `pcd.transform()` | o3d.t.geometry.PointCloud | o3d.t.geometry.PointCloud | 3-4x |
| **Nearest Neighbor Search** | `NearestNeighborSearch` | o3c.Tensor | o3c.Tensor | 5-10x |
| **Point Concatenation** | `o3c.concatenate()` | o3c.Tensor × 2 | o3c.Tensor | 2-3x |

### **CPU-Only Operations**

| Operation | Function/Method | Reason |
|-----------|----------------|--------|
| **EKF Prediction/Update** | `EKF.predict_imu()`, `EKF.update()` | Small matrix operations, no library support |
| **LaserScan Parsing** | `transform_scan_to_relative_frame()` | Sequential processing, low computational cost |
| **Quaternion Math** | `quaternion_to_yaw()`, etc. | Scalar operations |
| **ROS Conversions** | `numpy_to_pointcloud2()` | Memory formatting, not compute-intensive |
| **RANSAC** | `_ransac_2d()` | Random sampling, requires CPU control flow |

### **Memory Transfer Points (CPU ↔ GPU)**

| Transfer | Location | Cost | Frequency |
|----------|----------|------|-----------|
| numpy → tensor | `process_submap()` | ~0.5ms | Per submap (~80 scans) |
| tensor → numpy | `get_global_map_points()` | ~0.5ms | Cached, 1Hz publish |
| numpy → tensor | `scan_to_map_icp()` | ~0.5ms | Per scan (~10Hz) |
| tensor → numpy | ICP result | ~0.1ms | Per ICP (~10Hz) |

**Total GPU overhead per scan**: ~1-2ms (negligible compared to 15ms ICP speedup)

---

## Tunable Parameters Summary

### 🎯 High-Impact Parameters (Tune These First)

| Parameter | Location | Default | Type | Impact | Tuning Range |
|-----------|----------|---------|------|--------|--------------|
| **scans_per_submap** | LocalSubmapGenerator | 80 | int | Submap size/overlap | 30-100 |
| **voxel_size** | SubmapStitcher | 0.05m | float | Map resolution | 0.02-0.1m |
| **icp_fitness_threshold** | SubmapStitcher | 0.45 | float | ICP acceptance | 0.3-0.7 |
| **R_odom** | EKF | [0.005, 0.005, 0.02] | np.ndarray (3,) | Odometry trust | 0.001-0.01 |
| **R_icp** | EKF | [0.0001, 0.0001, 0.0001] | np.ndarray (3,) | ICP trust | 0.00005-0.0005 |
| **Q_imu** | EKF | [0.0001, 0.0001, 0.0001] | np.ndarray (3,) | Process noise | 0.00005-0.0005 |

---

### 🔧 Medium-Impact Parameters

| Parameter | Location | Default | Type | Impact | Tuning Range |
|-----------|----------|---------|------|--------|--------------|
| **icp_max_correspondence_dist** | SubmapStitcher | 0.1m | float | ICP convergence radius | 0.05-0.3m |
| **feature_method** | FeatureExtractor | 'hybrid' | str | Feature type | 'fpfh', 'iss', 'hybrid', 'scan_context', 'geometric' |

---

### ⚙️ Low-Impact Parameters (Fine-Tuning)

| Parameter | Location | Default | Type | Impact | Tuning Range |
|-----------|----------|---------|------|--------|--------------|
| **save_directory** | LocalSubmapGenerator | './submaps' | str | Map save location | Any path |
| **robot_name** | LocalSubmapGenerator | 'tb3_1' | str | ROS2 namespace | Any string |

---

## Parameter Tuning Guidelines

### 🎯 Scenario-Based Tuning

#### **Small Indoor Rooms (< 5m×5m)**
```python
scans_per_submap = 30          # Smaller submaps for tight spaces
voxel_size = 0.03              # Higher resolution
icp_fitness_threshold = 0.5    # Stricter acceptance
R_odom = [0.003, 0.003, 0.015] # Trust odometry more
Q_imu = [0.00008, 0.00008, 0.00008]  # Lower process noise
```

#### **Large Open Spaces (> 10m×10m)**
```python
scans_per_submap = 100         # Larger submaps for coverage
voxel_size = 0.08              # Lower resolution for speed
icp_fitness_threshold = 0.4    # More lenient acceptance
R_odom = [0.008, 0.008, 0.03]  # Less trust in odometry
Q_imu = [0.00015, 0.00015, 0.00015]  # Higher process noise
```

#### **Narrow Corridors**
```python
scans_per_submap = 60          # Standard
voxel_size = 0.04              # Medium-high resolution
icp_fitness_threshold = 0.35   # More lenient (poor constraints)
R_odom = [0.006, 0.006, 0.025] # Balanced trust
```

#### **Cluttered Environments (Furniture, Obstacles)**
```python
scans_per_submap = 40          # More frequent submaps
voxel_size = 0.04              # Higher resolution
icp_fitness_threshold = 0.5    # Stricter (good features)
feature_method = 'hybrid'      # Use all features
```

---

### 🔍 Diagnosis: Parameter Adjustment

#### **Problem: Maps have donut/spiral shape**
**Cause**: Drift accumulation, poor ICP correction

**Solution**:
```python
icp_fitness_threshold = 0.35      # Lower threshold (accept more ICP corrections)
R_odom = [0.008, 0.008, 0.03]     # Increase odometry noise (trust less)
R_icp = [0.00008, 0.00008, 0.00008]  # Decrease ICP noise (trust more)
Q_imu = [0.00012, 0.00012, 0.00012]  # Increase process noise
```

---

#### **Problem: ICP fails frequently (low fitness)**
**Cause**: Poor correspondences, feature-poor environment

**Solution**:
```python
icp_max_correspondence_dist = 0.15  # Increase search radius
icp_fitness_threshold = 0.35        # Lower acceptance threshold
voxel_size = 0.06                   # Reduce resolution (faster, more robust)
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
scans_per_submap = 100             # Less frequent submaps
```

---

#### **Problem: EKF diverges (uncertainty grows unbounded)**
**Cause**: Process noise too high, insufficient measurement updates

**Solution**:
```python
Q_imu = [0.00008, 0.00008, 0.00008]  # Decrease process noise
R_odom = [0.003, 0.003, 0.015]       # Trust odometry more
R_icp = [0.00008, 0.00008, 0.00008]  # Trust ICP corrections more
```

---

#### **Problem: Loop closure not detecting loops**
**Cause**: Thresholds too strict, poor features

**Solution**:
```python
spatial_search_radius = 7.0        # Larger search radius
scan_context_threshold = 0.65      # Lower threshold (more lenient)
min_feature_matches = 10           # Fewer required matches
```

---

### 📊 Performance vs Quality Trade-offs

| Priority | Configuration | Use Case | GPU Load | CPU Load |
|----------|---------------|----------|----------|----------|

---

## Summary Statistics

**Total Classes**: 6
**Total Standalone Functions**: 12
**Total Methods**: 65+
**Tunable Parameters**: 25+
**ROS2 Parameters**: 7
**GPU-Accelerated Operations**: 6
**CPU-Only Operations**: 18

**Core Pipeline with Data Types**:
```
LaserScan (ROS msg)
  → transform_scan_to_relative_frame() → np.ndarray (Nx3, CPU)
  → scan_to_map_icp() → np.ndarray (Nx3, CPU) + dict
  → EKF.update() → dict (CPU)
  → accumulate → List[np.ndarray] (CPU)
  → create_submap() → np.vstack() → np.ndarray (Mx3, CPU)
  → SubmapStitcher.integrate_submap_to_global_map():
      • process_submap() → o3c.Tensor (GPU) → o3d.t.geometry.PointCloud (GPU)
      • align_submap_with_icp() → ICP (GPU) → np.ndarray (4x4, CPU)
      • transform() → o3d.t.geometry.PointCloud (GPU)
      • concatenate() → global_map_tensor (GPU)
  → get_global_map_points() → np.ndarray (Px3, CPU)
  → numpy_to_pointcloud2() → PointCloud2 (ROS msg, CPU)
  → publish()
```

**Key Files to Modify for Tuning**:
1. `local_submap_generator.py` - Main parameters (lines 39-53)
2. `ekf_lib.py` - Noise covariances (lines 17-39)
3. `submap_stitcher.py` - ICP thresholds (lines 17-26)

---

**End of Document**
