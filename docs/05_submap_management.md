# Submap Management and Global Map Construction

## Table of Contents
1. [Introduction](#1-introduction)
2. [Submap Creation](#2-submap-creation)
3. [Coordinate Transformations](#3-coordinate-transformations)
4. [Submap Stitching](#4-submap-stitching)
5. [Global Map Maintenance](#5-global-map-maintenance)

---

## 1. Introduction

**Submaps** are local point cloud segments collected over a fixed number of scans. They provide:
- **Bounded complexity:** Fixed-size ICP targets
- **Temporal coherence:** Points collected under consistent localization
- **Hierarchical structure:** Building blocks for global map

### 1.1 Architecture

```
Scan 1-50  → Submap 0 ────┐
Scan 51-100 → Submap 1 ────┼──> Global Map
Scan 101-150 → Submap 2 ───┘
```

---

## 2. Submap Creation

### 2.1 Trigger Criteria

A new submap is created when:

$$
n_{\text{scans}} \geq N_{\text{threshold}}
$$

**Typical value:** $N_{\text{threshold}} = 50$ scans

**Rationale:**
- Sufficient points for reliable ICP ($\sim$18k points)
- Short enough to limit drift within submap
- Balances update frequency vs. computational cost

### 2.2 Local Frame Construction

Each submap has a **local coordinate frame** anchored at the submap start pose.

**Submap Start Pose** (in map frame):

$$
\mathbf{T}_{\text{submap}}^{\text{map}} = \begin{bmatrix}
\mathbf{R}(\theta_0) & \mathbf{t}_0 \\
0 & 1
\end{bmatrix}
$$

Where $(x_0, y_0, \theta_0)$ is the robot pose when the submap was initialized.

### 2.3 Point Accumulation

As scans arrive, points are transformed to the **submap-local frame**:

**Step 1: Current pose relative to submap start**

$$
\mathbf{T}_{\text{current}}^{\text{submap}} = (\mathbf{T}_{\text{submap}}^{\text{map}})^{-1} \cdot \mathbf{T}_{\text{current}}^{\text{map}}
$$

**Step 2: Transform scan points**

For each scan point $p_i^{\text{robot}}$ in robot frame:

$$
p_i^{\text{submap}} = \mathbf{T}_{\text{current}}^{\text{submap}} \cdot \mathbf{T}_{\text{robot}}^{\text{current}} \cdot p_i^{\text{robot}}
$$

**Step 3: Accumulate**

$$
\mathcal{S}_{\text{submap}} = \mathcal{S}_{\text{submap}} \cup \{p_i^{\text{submap}}\}
$$

**Result:** All points expressed in common submap-local frame.

---

## 3. Coordinate Transformations

### 3.1 Transformation Chain

```
scan_point^robot → scan_point^submap → scan_point^map
```

**Full transformation:**

$$
p^{\text{map}} = \mathbf{T}_{\text{submap}}^{\text{map}} \cdot \mathbf{T}_{\text{current}}^{\text{submap}} \cdot p^{\text{robot}}
$$

### 3.2 Inverse Transformation

To convert a map-frame correction back to local frame:

$$
\Delta p^{\text{submap}} = (\mathbf{R}_{\text{submap}}^{\text{map}})^{-1} \cdot \Delta p^{\text{map}}
$$

**Rotation-only** (orientation unchanged):

$$
\mathbf{R}^{-1} = \mathbf{R}^T \quad \text{(orthogonality)}
$$

### 3.3 Transform Composition

Given $\mathbf{T}_1$ and $\mathbf{T}_2$:

$$
\mathbf{T}_{\text{composed}} = \mathbf{T}_2 \cdot \mathbf{T}_1 = \begin{bmatrix}
\mathbf{R}_2 \mathbf{R}_1 & \mathbf{R}_2 \mathbf{t}_1 + \mathbf{t}_2 \\
0 & 1
\end{bmatrix}
$$

**Implementation:**
```python
def compose_transforms(T1, T2):
    """Compose two 4x4 transformation matrices."""
    return T2 @ T1
```

---

## 4. Submap Stitching

### 4.1 Global Map Integration

When a submap is complete:

**Step 1: Transform to Map Frame**

$$
\mathcal{S}_{\text{map}} = \{ \mathbf{T}_{\text{submap}}^{\text{map}} \cdot p \mid p \in \mathcal{S}_{\text{submap}} \}
$$

**Step 2: Align to Global Map (ICP)**

If global map exists:

$$
\mathbf{T}_{\text{correction}} = \text{ICP}(\mathcal{S}_{\text{map}}, \mathcal{M}_{\text{global}})
$$

**Step 3: Apply Correction**

$$
\mathcal{S}_{\text{corrected}} = \{ \mathbf{T}_{\text{correction}} \cdot p \mid p \in \mathcal{S}_{\text{map}} \}
$$

**Step 4: Merge into Global Map**

$$
\mathcal{M}_{\text{global}} \leftarrow \mathcal{M}_{\text{global}} \cup \mathcal{S}_{\text{corrected}}
$$

### 4.2 Voxel Grid Filtering

To prevent unbounded growth, downsample the global map:

$$
\mathcal{M}_{\text{global}}' = \text{voxel\_filter}(\mathcal{M}_{\text{global}}, v)
$$

**Voxel size:** $v = 0.05$ m

**Benefit:** Maintains constant point density regardless of overlaps.

### 4.3 Pose Correction Feedback

If ICP correction is significant, feed back to EKF:

$$
\mathbf{x}_{\text{EKF}}^+ = \mathbf{x}_{\text{EKF}} + \mathbf{K}_{\text{EKF}} \cdot \Delta \mathbf{x}_{\text{ICP}}
$$

Where $\Delta \mathbf{x}_{\text{ICP}} = [dx, dy, d\theta]^T$ from submap ICP.

---

## 5. Global Map Maintenance

### 5.1 Data Structure

**Open3D TensorPointCloud:**
- GPU-accelerated operations
- Efficient voxel downsampling
- Fast nearest-neighbor search

```python
global_map = o3d.t.geometry.PointCloud(device)
global_map.point.positions = o3c.Tensor(points, o3c.float32, device)
```

### 5.2 Incremental Update

```python
def integrate_submap(submap_points, T_submap_to_map, global_map, voxel_size):
    """
    Integrate new submap into global map.
    """
    # Transform submap to map frame
    submap_transformed = submap_points @ T_submap_to_map[:3, :3].T + T_submap_to_map[:3, 3]

    # Create point cloud
    submap_pcd = o3d.t.geometry.PointCloud(device)
    submap_pcd.point.positions = o3c.Tensor(submap_transformed, o3c.float32, device)

    # ICP alignment (if global map exists)
    if len(global_map.point.positions) > 0:
        result = o3d.t.pipelines.registration.icp(
            submap_pcd, global_map,
            max_correspondence_distance=voxel_size * 2,
            criteria=...
        )
        submap_pcd = submap_pcd.transform(result.transformation)

    # Merge
    global_map.point.positions = o3c.concatenate([
        global_map.point.positions,
        submap_pcd.point.positions
    ], axis=0)

    # Downsample
    global_map = global_map.voxel_down_sample(voxel_size)

    return global_map
```

### 5.3 Map Serialization

**Save to disk:**
```python
o3d.t.io.write_point_cloud("global_map.pcd", global_map)
```

**PCD format:**
- ASCII or binary
- Standard point cloud format
- Compatible with CloudCompare, PCL, etc.

---

## 6. Implementation

### 6.1 Submap State Machine

```python
class SubmapManager:
    def __init__(self, scans_per_submap=50):
        self.scans_per_submap = scans_per_submap
        self.current_submap_points = []
        self.scan_count = 0
        self.submap_start_pose = None

    def process_scan(self, scan_points, current_pose):
        """
        Add scan to current submap.
        """
        if self.submap_start_pose is None:
            self.submap_start_pose = current_pose

        # Transform to submap frame
        relative_pose = compute_relative_pose(current_pose, self.submap_start_pose)
        points_local = transform_points(scan_points, relative_pose)

        self.current_submap_points.append(points_local)
        self.scan_count += 1

        if self.scan_count >= self.scans_per_submap:
            self.finalize_submap()

    def finalize_submap(self):
        """
        Complete current submap and start new one.
        """
        # Stack all points
        submap = np.vstack(self.current_submap_points)

        # Integrate into global map
        self.integrate_to_global_map(submap, self.submap_start_pose)

        # Reset for next submap
        self.current_submap_points = []
        self.scan_count = 0
        self.submap_start_pose = None
```

### 6.2 Code Mapping

| Component | File | Class/Function |
|-----------|------|---------------|
| Submap management | `local_submap_generator.py` | `LocalSubmapGenerator` |
| Stitching | `submap_stitcher.py` | `SubmapStitcher` |
| Transform utils | `mapping_utils.py` | `compute_relative_pose()` |

---

## References

1. **Konolige, K., & Agrawal, M. (2008).** "FrameSLAM: From Bundle Adjustment to Real-Time Visual Mapping." *IEEE Transactions on Robotics*, 24(5), 1066-1077.

2. **Bosse, M., & Zlot, R. (2009).** "Keypoint Design and Evaluation for Place Recognition in 2D Lidar Maps." *Robotics and Autonomous Systems*, 57(12), 1211-1224.

3. **Magnusson, M., Lilienthal, A., & Duckett, T. (2007).** "Scan Registration for Autonomous Mining Vehicles Using 3D-NDT." *Journal of Field Robotics*, 24(10), 803-827.

---

**Next:** `06_uncertainty_quantification.md` — Information theory and confidence metrics
