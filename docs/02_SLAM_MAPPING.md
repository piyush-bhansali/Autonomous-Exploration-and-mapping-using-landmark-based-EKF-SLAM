# SLAM & Mapping Module
## GPU-Accelerated Point Cloud Mapping

**Module:** `map_generation`
**Main Node:** `local_submap_generator`
**Code:** `src/map_generation/map_generation/local_submap_generator.py`

---

## Table of Contents

1. [Overview](#overview)
2. [Architecture](#architecture)
3. [Submap-Based Mapping](#submap-based-mapping)
4. [Scan-to-Map ICP](#scan-to-map-icp)
5. [GPU Acceleration](#gpu-acceleration)
6. [Coordinate Frames](#coordinate-frames)
7. [Implementation Details](#implementation-details)
8. [Parameters & Tuning](#parameters--tuning)

---

## Overview

The SLAM mapping module builds accurate 3D point cloud maps in real-time using:
- **Submap decomposition** for memory efficiency
- **GPU-accelerated ICP** for drift correction
- **Tensor point clouds** (Open3D) for CUDA operations
- **Voxel downsampling** to manage point cloud density

### Key Capabilities

| Feature | Implementation | Performance |
|---------|---------------|-------------|
| **Map Representation** | Point cloud (no grid) | Native 3D, lossless |
| **Processing** | Open3D Tensor API | GPU-accelerated |
| **Drift Correction** | Scan-to-map ICP | <50ms per scan |
| **Submap Size** | 80 scans (~8 seconds) | Memory-efficient |
| **Downsampling** | Voxel grid (5cm) | 10-20x reduction |
| **Update Rate** | 10 Hz | Real-time |

---

## Architecture

```
┌────────────────────────────────────────────────────────────────┐
│              Local Submap Generator Node                       │
│                                                                 │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │  Scan Input  │  │  EKF Fusion  │  │  Scan-to-Map ICP   │   │
│  │  (LiDAR)     │→ │  (Pose Est.) │→ │  (Drift Correct)   │   │
│  │   10 Hz      │  │   200 Hz     │  │   GPU-Accelerated  │   │
│  └──────────────┘  └──────────────┘  └─────────┬──────────┘   │
│                                                 │               │
│  ┌──────────────────────────────────────────────┘               │
│  │                                                              │
│  v                                                              │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │           Submap Accumulation (Local Frame)              │  │
│  │  - Transform scan to submap-local coordinates            │  │
│  │  - Accumulate points in current submap                   │  │
│  │  - Track scan count (target: 80 scans)                   │  │
│  └───────────────────────┬──────────────────────────────────┘  │
│                          │                                     │
│                          │ (Every 80 scans)                    │
│                          v                                     │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                 Submap Stitcher                          │  │
│  │  ┌────────────┐  ┌──────────────┐  ┌────────────────┐   │  │
│  │  │  Voxel     │  │   Feature    │  │  Merge into    │   │  │
│  │  │ Downsample │→ │  Extraction  │→ │  Global Map    │   │  │
│  │  │  (GPU)     │  │ (Scan Contxt)│  │   (GPU)        │   │  │
│  │  └────────────┘  └──────────────┘  └────────────────┘   │  │
│  └──────────────────────────────────────────────────────────┘  │
│                                                                 │
│  Output: Global Map (PointCloud2)                              │
└─────────────────────────────────────────────────────────────────┘
```

---

## Submap-Based Mapping

### Why Submaps?

**Problem:** Accumulating all scans directly creates memory and computational bottlenecks.

**Solution:** Divide mapping into local submaps:
1. **Accumulate** scans in small local frames (80 scans)
2. **Process** each submap independently (downsample, extract features)
3. **Merge** submaps into global map with pose corrections

### Benefits

| Aspect | Advantage |
|--------|-----------|
| **Memory** | O(N) → O(n × k) where n << N |
| **Computation** | Smaller ICP search spaces |
| **Loop Closure** | Submap-level associations |
| **Recovery** | Errors isolated to submaps |

### Submap Lifecycle

```python
# Configuration (local_submap_generator.py:40)
scans_per_submap = 80  # Parameter: ~8 seconds at 10 Hz

# State tracking
self.current_submap_scans = []  # Accumulator
self.scan_count = 0             # Counter

# Process flow
for each scan:
    1. Transform scan to submap-local frame
    2. Append to current_submap_scans
    3. scan_count += 1

    if scan_count == scans_per_submap:
        # Submap complete
        4. Create PointCloud from accumulated scans
        5. Voxel downsample (5cm)
        6. Extract features (Scan Context)
        7. Send to Submap Stitcher
        8. Reset current_submap_scans = []
        9. scan_count = 0
```

### Coordinate Frame Strategy

Each submap has its own **local coordinate frame**:
- **Origin:** Robot pose when submap was created
- **Benefit:** Relative poses within submap are small → numerically stable
- **Transform:** Submap-to-world transform stored for global integration

**Implementation:**
```python
# local_submap_generator.py:397-420
def _handle_new_submap():
    # Store world pose when submap starts
    submap_start_pose = {
        'x': self.ekf_state[0],
        'y': self.ekf_state[1],
        'theta': self.ekf_state[2]
    }

    # All scans in this submap use relative poses from this origin
    for scan in submap:
        relative_pose = compute_relative_pose(
            current_pose=scan_pose,
            reference_pose=submap_start_pose
        )
        # Transform points using relative pose
        transformed_points = transform_scan_to_relative_frame(...)
```

---

## Scan-to-Map ICP

### Purpose

**Real-time drift correction** by aligning each incoming scan with the accumulated submap.

### Algorithm

**Input:**
- Current scan (360 points, 2D)
- Accumulated submap points (5000-10000 points, 3D)
- Initial pose estimate from EKF

**Process:**
```
1. Transform scan to world frame using EKF pose
2. Convert to Open3D tensor PointCloud (GPU)
3. Create target from accumulated points (GPU)
4. Run ICP alignment:
   - Method: Point-to-Point
   - Max iterations: 30
   - Convergence: 1e-6
   - Hardware: CUDA GPU
5. Extract pose correction (dx, dy, dθ)
6. Validate correction magnitude
7. Update EKF with corrected pose
```

**Implementation:** `local_submap_generator.py:337-391`

### ICP Parameters

```python
# mapping_utils.py:40-52
reg_result = o3d.t.pipelines.registration.icp(
    source=source_pcd,           # Scan (tensor PointCloud)
    target=target_pcd,           # Submap (tensor PointCloud)
    max_correspondence_distance=0.5,  # 50cm max matching distance
    init_source_to_target=np.eye(4),  # Initial guess (identity)
    estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(
        relative_fitness=1e-6,
        relative_rmse=1e-6,
        max_iteration=30
    )
)
```

### Validation & Correction

```python
# Extract transformation
T = reg_result.transformation.cpu().numpy()

# Compute correction
dx = T[0, 3]  # X translation
dy = T[1, 3]  # Y translation
dtheta = np.arctan2(T[1, 0], T[0, 0])  # Rotation

# Validation thresholds (local_submap_generator.py:370-385)
if abs(dx) > 0.2 or abs(dy) > 0.2 or abs(dtheta) > 0.26:  # 15°
    logger.warn('ICP correction too large - rejecting')
    return  # Skip this correction

# Apply correction to EKF
corrected_pose = {
    'x': raw_odom_x + dx,
    'y': raw_odom_y + dy,
    'theta': raw_odom_theta + dtheta
}
ekf.update(corrected_pose, measurement_noise=R_icp)
```

### Performance Metrics

| Metric | CPU | GPU (CUDA) |
|--------|-----|------------|
| **ICP Time** | 150-200 ms | **30-50 ms** |
| **Throughput** | 5-7 Hz | **20+ Hz** |
| **Memory** | 500 MB | 200 MB (VRAM) |
| **Accuracy** | <2 cm | <2 cm |

---

## GPU Acceleration

### Open3D Tensor API

All point cloud operations use **Open3D tensor-based PointClouds** for GPU execution.

**Device Selection:**
```python
# local_submap_generator.py:72-77
if o3c.cuda.is_available():
    self.device = o3c.Device("CUDA:0")
    logger.info("GPU acceleration enabled")
else:
    self.device = o3c.Device("CPU:0")
    logger.warn("GPU not available, using CPU")
```

### Tensor PointCloud Creation

```python
import open3d as o3d
import open3d.core as o3c

# Create points as numpy array
points_np = np.array([[x1, y1, z1], [x2, y2, z2], ...])

# Convert to GPU tensor
points_tensor = o3c.Tensor(points_np, dtype=o3c.float32, device=self.device)

# Create tensor PointCloud
pcd = o3d.t.geometry.PointCloud(self.device)
pcd.point.positions = points_tensor
```

### GPU-Accelerated Operations

| Operation | API | Speedup |
|-----------|-----|---------|
| **ICP Alignment** | `o3d.t.pipelines.registration.icp()` | 4-6x |
| **Voxel Downsampling** | `pcd.voxel_down_sample()` | 8-10x |
| **Normal Estimation** | `pcd.estimate_normals()` | 5-7x |
| **KD-Tree Search** | Tensor NNS | 3-5x |

### Memory Management

**VRAM Usage:**
- Scan (360 points): ~5 KB
- Submap (5000 points): ~60 KB
- Global map (100k points): ~1.2 MB
- ICP workspace: ~50 MB
- **Total:** ~200-500 MB VRAM

**Optimization:**
```python
# Voxel downsampling reduces memory footprint
original_size = len(pcd.point.positions)  # e.g., 10,000 points
pcd_down = pcd.voxel_down_sample(voxel_size=0.05)  # 5cm voxels
reduced_size = len(pcd_down.point.positions)  # e.g., 800 points

reduction = original_size / reduced_size  # ~12.5x smaller
```

---

## Coordinate Frames

### Frame Hierarchy

```
odom (world frame)
  └─ base_footprint (robot center)
      └─ base_scan (LiDAR sensor)
```

**TF Transforms Published:**
- `odom → base_footprint` (by local_submap_generator, via TF broadcaster)

### Transform Chain

```python
# local_submap_generator.py:184-196
def _publish_tf():
    t = TransformStamped()
    t.header.stamp = current_time
    t.header.frame_id = f'{self.robot_name}/odom'
    t.child_frame_id = f'{self.robot_name}/base_footprint'

    # EKF state [x, y, theta]
    t.transform.translation.x = self.ekf_state[0]
    t.transform.translation.y = self.ekf_state[1]
    t.transform.translation.z = 0.0

    # Convert yaw to quaternion
    qx, qy, qz, qw = yaw_to_quaternion(self.ekf_state[2])
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw

    self.tf_broadcaster.sendTransform(t)
```

### Scan Transformation Pipeline

```
1. LaserScan (polar) → Cartesian points in base_scan frame
2. base_scan → base_footprint (fixed offset: x=-0.064m)
3. base_footprint → odom (EKF pose, time-varying)

Result: Points in world (odom) frame
```

**Code:** `mapping_utils.py:461-540` (`transform_scan_to_relative_frame`)

---

## Implementation Details

### Main Callbacks

#### 1. Scan Callback (10 Hz)
```python
# local_submap_generator.py:234-244
def scan_callback(msg: LaserScan):
    if not ekf_initialized:
        return  # Wait for odom

    # Store for ICP processing
    self.latest_scan = msg
```

#### 2. Odometry Callback (10 Hz)
```python
# local_submap_generator.py:246-317
def odom_callback(msg: Odometry):
    # Extract pose
    x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
    theta = quaternion_to_yaw(...)

    # Initialize EKF on first message
    if not ekf_initialized:
        ekf.initialize(x, y, theta, vx=0.0, vy=0.0)
        return

    # Update EKF with odometry measurement
    ekf.update(
        measurement={'x': x, 'y': y, 'theta': theta},
        measurement_noise=R_odom  # [0.005, 0.005, 0.02]
    )

    # Process scan if available
    if latest_scan is not None:
        process_scan()
```

#### 3. IMU Callback (200 Hz)
```python
# local_submap_generator.py:224-232
def imu_callback(msg: Imu):
    if not ekf_initialized:
        return

    # Extract angular velocity
    omega = msg.angular_velocity.z

    # EKF prediction step
    ekf.predict_imu(omega=omega, dt=0.005)  # 5ms @ 200Hz
```

### Scan Processing Flow

```python
# local_submap_generator.py:319-500
def _process_scan():
    # 1. Get EKF pose estimate
    ekf_state = ekf.get_state()  # [x, y, theta]

    # 2. Convert scan to world frame
    scan_points_world = scan_to_cartesian(latest_scan)
    transform_to_world(scan_points_world, ekf_state)

    # 3. Scan-to-map ICP (always runs when submap has sufficient points)
    if len(accumulated_points) > 100:
        corrected_pose = scan_to_map_icp(
            scan_points_world,
            accumulated_points,
            device=self.device
        )
        if corrected_pose:
            ekf.update(corrected_pose, R_icp)  # High confidence

    # 4. Compute relative pose for submap-local storage
    relative_pose = compute_relative_pose(
        current_pose=ekf.get_state(),
        reference_pose=submap_start_pose
    )

    # 5. Transform scan to submap-local frame
    scan_local = transform_scan_to_relative_frame(
        scan_points=latest_scan,
        relative_pose=relative_pose,
        sensor_offset={'x': -0.064, 'y': 0.0}
    )

    # 6. Accumulate in current submap
    current_submap_scans.append(scan_local)
    accumulated_points = np.vstack([accumulated_points, scan_local])
    scan_count += 1

    # 7. Check if submap complete
    if scan_count >= scans_per_submap:
        _handle_new_submap()
```

---

## Parameters & Tuning

### Core Parameters

```python
# ROS2 parameters (local_submap_generator.py:39-46)
self.declare_parameter('robot_name', 'tb3_1')
self.declare_parameter('scans_per_submap', 80)
self.declare_parameter('save_directory', './submaps')
self.declare_parameter('voxel_size', 0.05)
self.declare_parameter('feature_method', 'hybrid')
self.declare_parameter('enable_loop_closure', True)
```

### Tuning Guide

| Parameter | Default | Effect | Tuning |
|-----------|---------|--------|--------|
| `scans_per_submap` | 80 | Submap size | ↑ = larger submaps, less overhead<br>↓ = smaller submaps, more features |
| `voxel_size` | 0.05 | Downsampling | ↑ = faster, less detail<br>↓ = slower, more detail |
| `enable_loop_closure` | true | Global optimization | true = consistent, slower<br>false = fast, drifts over time |

### ICP Tuning

```python
# mapping_utils.py:44-46
max_correspondence_distance = 0.5  # Match points within 50cm
max_iteration = 30                 # ICP iterations
convergence_criteria = 1e-6        # Relative fitness/RMSE threshold
```

**Guidelines:**
- **max_correspondence_distance:** Set to 2-3× expected noise
- **max_iteration:** Balance speed vs. accuracy (20-50 typical)
- **convergence_criteria:** Smaller = more accurate, but slower

### Performance vs. Accuracy Trade-offs

| Configuration | Speed | Accuracy | Use Case |
|---------------|-------|----------|----------|
| **High Speed** | 15 Hz map updates | ±10 cm | Real-time exploration |
| voxel=0.1, icp=off, scans=50 |
| **Balanced** | 10 Hz map updates | ±5 cm | General use |
| voxel=0.05, icp=on, scans=80 |
| **High Accuracy** | 5 Hz map updates | ±2 cm | Precision mapping |
| voxel=0.02, icp=on, scans=120 |

---

## Topics & Messages

### Subscribed Topics

| Topic | Type | QoS | Rate | Purpose |
|-------|------|-----|------|---------|
| `/{robot}/scan` | LaserScan | RELIABLE | 10 Hz | LiDAR measurements |
| `/{robot}/odom` | Odometry | RELIABLE | 10 Hz | Wheel encoder poses |
| `/{robot}/imu` | Imu | BEST_EFFORT | 200 Hz | Angular velocity |

### Published Topics

| Topic | Type | QoS | Rate | Content |
|-------|------|-----|------|---------|
| `/{robot}/global_map` | PointCloud2 | RELIABLE | 0.1 Hz | Full accumulated map |
| `/tf` | TFMessage | - | 200 Hz | odom→base_footprint |

### Message Definitions

**PointCloud2 Structure:**
```python
header:
  stamp: current_time
  frame_id: '{robot_name}/odom'
height: 1  # Unorganized point cloud
width: N   # Number of points
fields:
  - name: 'x', offset: 0, datatype: FLOAT32
  - name: 'y', offset: 4, datatype: FLOAT32
  - name: 'z', offset: 8, datatype: FLOAT32
point_step: 12 bytes
data: [x1, y1, z1, x2, y2, z2, ...]
```

---

## Debugging & Troubleshooting

### Common Issues

**Issue 1: ICP Failures**
```
Symptom: Warnings "ICP correction too large - rejecting"
Cause: Poor initial guess or degenerate geometry
Solution:
- Check EKF convergence (σ_pos < 0.05)
- Verify scan quality (no inf/nan ranges)
- Increase max_correspondence_distance to 0.7
```

**Issue 2: GPU Not Detected**
```
Symptom: "GPU not available, using CPU"
Cause: CUDA/cuDNN not installed or Open3D built without CUDA
Solution:
- Check: nvidia-smi (GPU visible?)
- Check: o3c.cuda.is_available() in Python
- Reinstall: pip install open3d --upgrade (use pre-built CUDA wheels)
```

**Issue 3: Map Drift**
```
Symptom: Map distorts over time
Cause: EKF tuning issues or loop closure disabled
Solution:
- Reduce: Q (process noise) in EKF
- Enable: loop closure detection (enable_loop_closure = true)
- Tune: R_icp (ICP measurement noise) for better scan-to-map corrections
```

### Performance Monitoring

```bash
# Check map update rate
ros2 topic hz /{robot_name}/global_map

# Monitor ICP performance (check node logs)
ros2 run rqt_console rqt_console

# View point cloud size
ros2 topic echo /{robot_name}/global_map --once | grep width

# GPU utilization
nvidia-smi -l 1  # Update every second
```

---

**Next:** Read [03_EKF_SENSOR_FUSION.md](03_EKF_SENSOR_FUSION.md) for EKF implementation details.
