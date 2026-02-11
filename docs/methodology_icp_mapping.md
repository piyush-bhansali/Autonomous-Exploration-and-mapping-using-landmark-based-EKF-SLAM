# ICP-Based SLAM Methodology

## 1. Overview

This document describes the ICP (Iterative Closest Point) based mapping methodology implemented for the comparative SLAM study. The ICP approach uses dense scan matching to build occupancy grid maps through incremental point cloud registration, providing high geometric detail without requiring feature extraction.

## 2. System Architecture

### 2.1 Pipeline Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    ICP-Based SLAM Pipeline                   │
│                                                               │
│  LiDAR Scan                                                  │
│      │                                                        │
│      ├──> Odometry Prediction ──> EKF Prediction            │
│      │                                  │                     │
│      │                                  ↓                     │
│      ├──> Scan-to-Submap ICP ──> Pose Correction           │
│      │         │                        │                     │
│      │         │                        ↓                     │
│      │         │              EKF Update (ICP measurement)   │
│      │         │                        │                     │
│      │         ↓                        ↓                     │
│      └──> Accumulate Points ──> Current Pose Estimate       │
│                  │                                            │
│                  ↓                                            │
│          Submap Complete?                                    │
│                  │                                            │
│                  ├── Yes ──> Create Submap                   │
│                  │              │                             │
│                  │              ↓                             │
│                  │        Submap Stitching                   │
│                  │              │                             │
│                  │              ↓                             │
│                  │         Global Map                        │
│                  │                                            │
│                  └── No ──> Continue Accumulation            │
│                                                               │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 Core Components

| Component | Purpose | Implementation |
|-----------|---------|----------------|
| **Scan-to-Submap ICP** | Dense point cloud alignment | `mapping_utils.scan_to_map_icp()` |
| **Odometry Integration** | Motion prediction | `local_submap_generator.odom_callback()` |
| **EKF Pose Estimation** | Probabilistic state estimation | `ekf_slam.py` |
| **Submap Accumulation** | Local map building | `local_submap_generator._process_scan_icp_mode()` |
| **Submap Stitching** | Global map construction | `submap_stitcher.py` |
| **Uncertainty Quantification** | Hessian-based covariance | `evaluation_utils.compute_ekf_confidence()` |

## 3. Scan-to-Submap ICP

### 3.1 Algorithm

The ICP algorithm performs iterative point cloud registration:

**Input:**
- Source point cloud: $\mathbf{P}_{source} = \{p_i\}_{i=1}^N$
- Target point cloud: $\mathbf{P}_{target} = \{q_j\}_{j=1}^M$
- Initial transformation estimate: $T_0 = [R_0 | t_0]$

**Algorithm:**
```
For iteration k = 1 to max_iterations:
  1. Find correspondences:
     For each point p_i in P_source:
       c_i = argmin_j ||T_k * p_i - q_j||

  2. Compute optimal transformation:
     T_{k+1} = argmin_T Σ_i ||T * p_i - q_{c_i}||²

  3. Check convergence:
     If ||T_{k+1} - T_k|| < ε:
       Return T_{k+1}
```

**Output:**
- Optimal transformation: $T^* = [R^* | t^*]$
- Alignment error: $e = \sum_i ||T^* \cdot p_i - q_{c_i}||^2$

### 3.2 Point-to-Plane Metric

For 2D laser scans in structured environments, point-to-line distance provides better convergence:

$$d_i = |(T \cdot p_i - q_{c_i}) \cdot \mathbf{n}_{c_i}|$$

where $\mathbf{n}_{c_i}$ is the normal vector at the closest point. This metric is more robust to parallel surfaces than point-to-point distance.

### 3.3 Implementation Details

**Voxel Downsampling:**
```python
voxel_size = 0.05  # 5cm grid
source_down = source.voxel_down_sample(voxel_size)
target_down = target.voxel_down_sample(voxel_size)
```

**Correspondence Search:**
- Uses KD-tree for efficient nearest neighbor search
- Maximum correspondence distance: 0.3m
- Rejects correspondences beyond threshold

**Transformation Estimation:**
- Uses SVD for closed-form solution
- Point-to-plane minimization for 2D laser scans
- Converges typically in 5-10 iterations

**Convergence Criteria:**
- Relative transformation change: $\Delta T < 10^{-6}$
- Or maximum iterations: 30

### 3.4 Outlier Rejection

**Statistical Outlier Removal:**
```python
# Reject points with distance > mean + 2*std
threshold = mean_distance + 2.0 * std_distance
inliers = distances < threshold
```

**Distance-based Filtering:**
- Maximum correspondence distance: 0.3m
- Prevents matching across gaps or through walls
- Improves robustness to dynamic obstacles

## 4. Odometry-Based Prediction

### 4.1 Motion Model

The robot uses differential drive kinematics:

$$
\begin{align}
x_{t} &= x_{t-1} + \Delta d \cos(\theta_{t-1} + \Delta\theta/2) \\
y_{t} &= y_{t-1} + \Delta d \sin(\theta_{t-1} + \Delta\theta/2) \\
\theta_{t} &= \theta_{t-1} + \Delta\theta
\end{align}
$$

where:
- $\Delta d$: Linear distance traveled
- $\Delta\theta$: Angular rotation
- Mid-point integration improves accuracy for curved paths

### 4.2 Odometry Computation

From wheel encoder readings:

$$
\begin{align}
\Delta d &= \frac{(\Delta s_L + \Delta s_R)}{2} \\
\Delta\theta &= \frac{(\Delta s_R - \Delta s_L)}{b}
\end{align}
$$

where:
- $\Delta s_L, \Delta s_R$: Left and right wheel displacements
- $b$: Wheelbase (distance between wheels)

### 4.3 Process Noise Model

Odometry uncertainty grows with motion:

$$
Q = \begin{bmatrix}
\sigma_d^2 \cdot |\Delta d| + \sigma_{min}^2 & 0 \\
0 & \sigma_\theta^2 \cdot |\Delta\theta| + \sigma_{min}^2
\end{bmatrix}
$$

**Parameters:**
- $\sigma_d = 0.01$: Distance-proportional coefficient (1% error)
- $\sigma_\theta = 0.005$: Rotation-proportional coefficient
- $\sigma_{min} = 0.0001$: Minimum noise floor when stationary

This motion-scaled noise model prevents singular covariance and accounts for cumulative drift.

## 5. EKF State Estimation

### 5.1 State Vector

For ICP-based mapping, the EKF maintains only robot pose (no landmarks):

$$
\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}
$$

Covariance matrix:

$$
\mathbf{P} = \begin{bmatrix}
\sigma_x^2 & \sigma_{xy} & \sigma_{x\theta} \\
\sigma_{xy} & \sigma_y^2 & \sigma_{y\theta} \\
\sigma_{x\theta} & \sigma_{y\theta} & \sigma_\theta^2
\end{bmatrix}
$$

### 5.2 Prediction Step

Using odometry measurements $\mathbf{u} = [\Delta d, \Delta\theta]^T$:

**State prediction:**
$$
\mathbf{x}_t^- = f(\mathbf{x}_{t-1}, \mathbf{u}_t)
$$

**Jacobian of motion model:**
$$
F = \begin{bmatrix}
1 & 0 & -\Delta d \sin(\theta + \Delta\theta/2) \\
0 & 1 & \Delta d \cos(\theta + \Delta\theta/2) \\
0 & 0 & 1
\end{bmatrix}
$$

**Control Jacobian:**
$$
G = \begin{bmatrix}
\cos(\theta + \Delta\theta/2) & -\frac{\Delta d}{2} \sin(\theta + \Delta\theta/2) \\
\sin(\theta + \Delta\theta/2) & \frac{\Delta d}{2} \cos(\theta + \Delta\theta/2) \\
0 & 1
\end{bmatrix}
$$

**Covariance prediction:**
$$
\mathbf{P}_t^- = F \mathbf{P}_{t-1} F^T + G Q G^T
$$

### 5.3 Update Step (ICP Measurement)

When ICP provides a pose correction $[\Delta x, \Delta y, \Delta\theta]^T$:

**Measurement model:**
$$
\mathbf{z} = H \mathbf{x} + \mathbf{v}
$$

where $H = I_{3\times3}$ (direct pose observation) and $\mathbf{v} \sim \mathcal{N}(0, R_{ICP})$.

**Kalman gain:**
$$
K = \mathbf{P}^- H^T (H \mathbf{P}^- H^T + R_{ICP})^{-1}
$$

**State update:**
$$
\mathbf{x} = \mathbf{x}^- + K(\mathbf{z} - H\mathbf{x}^-)
$$

**Covariance update (Joseph form):**
$$
\mathbf{P} = (I - KH)\mathbf{P}^-(I - KH)^T + KR_{ICP}K^T
$$

The Joseph form ensures numerical stability and positive definiteness.

## 6. ICP Uncertainty Quantification

### 6.1 Hessian-Based Covariance

Following Censi (2007), the ICP covariance is computed from the optimization Hessian:

$$
\Sigma_{ICP} = \sigma^2 H^{-1}
$$

where:
- $\sigma^2 = 0.01^2$ (1cm LiDAR noise variance)
- $H = \sum_i J_i^T J_i$ (Hessian from point-to-plane residuals)

### 6.2 Jacobian Computation

For each correspondence $i$:

$$
J_i = \begin{bmatrix}
\frac{\partial r_i}{\partial x} & \frac{\partial r_i}{\partial y} & \frac{\partial r_i}{\partial \theta}
\end{bmatrix}
$$

where $r_i$ is the point-to-line residual:

$$
r_i = |(T \cdot p_i - q_i) \cdot \mathbf{n}_i|
$$

For 2D laser scans:

$$
J_i = \begin{bmatrix}
n_x & n_y & -n_x(p_{i,x}\sin\theta + p_{i,y}\cos\theta) + n_y(p_{i,x}\cos\theta - p_{i,y}\sin\theta)
\end{bmatrix}
$$

where $\mathbf{n} = [n_x, n_y]^T$ is the surface normal.

### 6.3 Information Matrix Accumulation

$$
H = \sum_{i=1}^{N} J_i^T J_i
$$

This represents the Fisher Information Matrix—the more correspondences and the better their geometric distribution, the smaller the covariance (higher information).

### 6.4 Geometric Interpretation

The Hessian eigenvalues reveal constraint directions:

- **Large eigenvalue**: Strong constraint (e.g., perpendicular to wall)
- **Small eigenvalue**: Weak constraint (e.g., along corridor)

Eigenvalue ratio indicates degeneracy:
- Ratio ≈ 1: Well-constrained (corners, cluttered areas)
- Ratio ≫ 1: Poorly constrained (long corridors, open spaces)

## 7. Submap Management

### 7.1 Accumulation Strategy

Points are accumulated in the **local submap frame**:

```python
# Transform scan to submap local frame
relative_pose = compute_relative_pose(current_pose, submap_start_pose)
points_local = transform_scan(scan, relative_pose)

# Accumulate
current_submap_points.append(points_local)
```

**Submap creation trigger:**
- Scan count reaches threshold (50 scans)
- Ensures adequate point density for next submap's ICP matching

### 7.2 Scan-to-Submap Matching

Each new scan is matched against accumulated submap points:

```python
if len(current_submap_points) >= 5:
    accumulated_map = vstack(current_submap_points)
    corrected_scan, pose_correction = scan_to_map_icp(
        new_scan,
        accumulated_map,
        max_iterations=30
    )
```

**Benefits:**
- Reduces drift by matching against local map
- Provides pose corrections before accumulation
- Accumulates corrected scans for better map quality

### 7.3 Pose Correction Validation

ICP corrections are validated before application:

```python
if correction_distance < 0.5 and correction_angle < 45°:
    apply_pose_correction(dx, dy, dtheta)
else:
    reject_correction()  # Likely failed match
```

This prevents catastrophic failures from incorrect ICP convergence.

## 8. Global Map Stitching

### 8.1 Submap-to-Global Registration

When a submap is complete:

1. **Transform to global frame:**
   ```python
   T_local_to_global = get_transformation(submap_start_pose)
   points_global = T_local_to_global @ points_local
   ```

2. **ICP with existing global map:**
   ```python
   if global_map_exists:
       refined_transform, correction = icp(submap, global_map)
       apply_correction_to_robot_trajectory()
   ```

3. **Merge into global map:**
   ```python
   global_map = voxel_merge(global_map, submap, voxel_size=0.05)
   ```

### 8.2 Loop Closure

While not explicitly implemented, the submap-to-global ICP provides implicit loop closure:

- When revisiting areas, ICP aligns new submap with existing global map
- Pose correction propagates to robot trajectory
- Closes loops within ICP convergence basin (~1-2 meters)

### 8.3 Map Representation

**Voxel Grid:**
- Resolution: 5cm
- Memory-efficient for large environments
- Enables fast nearest-neighbor queries

**Point Cloud:**
- Stores geometric detail
- Suitable for visualization
- Can be converted to occupancy grid for navigation

## 9. Performance Characteristics

### 9.1 Computational Complexity

| Operation | Complexity | Notes |
|-----------|-----------|-------|
| KD-tree construction | $O(N \log N)$ | $N$ = point count |
| Nearest neighbor search | $O(\log N)$ per query | For each source point |
| SVD transformation | $O(1)$ | Fixed 2D/3D size |
| Per ICP iteration | $O(N \log M)$ | $N$ source, $M$ target |
| Voxel downsampling | $O(N)$ | Linear scan |

**Total per scan:** $O(N \log M)$ where typical $N, M \approx 500-1000$ after downsampling.

### 9.2 Memory Requirements

- **Per scan:** ~10KB (500 points × 20 bytes/point)
- **Submap (50 scans):** ~500KB
- **Global map:** Grows linearly with environment size
- **Example:** 1000m² office ≈ 50MB at 5cm resolution

### 9.3 Accuracy vs. Computational Cost

**Voxel size trade-off:**
- Smaller voxels (2cm): Higher accuracy, more computation
- Larger voxels (10cm): Lower accuracy, faster processing

**ICP iterations trade-off:**
- More iterations: Better convergence, slower
- Fewer iterations: Faster, may not fully converge

**Optimal parameters for indoor robots:**
- Voxel size: 5cm
- Max ICP iterations: 30
- Correspondence distance: 30cm

## 10. Advantages and Limitations

### 10.1 Advantages

✅ **No feature extraction overhead**
- Processes raw point clouds directly
- Works in feature-poor environments (long corridors, warehouses)

✅ **Dense geometric representation**
- Captures fine details
- High-resolution maps

✅ **Mature, well-studied algorithms**
- ICP convergence well-understood
- Many optimized implementations available

✅ **Uncertainty quantification**
- Hessian-based covariance from first principles
- Geometric interpretation through eigenanalysis

✅ **Implicit loop closure**
- Submap-to-global ICP provides local loop closure
- No explicit place recognition needed for small loops

### 10.2 Limitations

❌ **Computational cost scales with point density**
- Denser scans → more computation
- Trade-off between accuracy and speed

❌ **Memory grows with environment size**
- Must store dense point clouds
- Can be mitigated with voxel filtering

❌ **Local minima in ICP**
- Requires good initial estimate (odometry)
- Can fail with large displacement or rotation

❌ **Drift accumulation**
- Without explicit landmarks, gradual drift over long distances
- Relies on implicit loop closure through submap stitching

❌ **Limited long-range loop closure**
- ICP convergence basin typically 1-2 meters
- Larger loops require additional place recognition

## 11. Parameters and Configuration

### 11.1 ICP Parameters

```python
icp_params = {
    'max_iterations': 30,
    'convergence_threshold': 1e-6,
    'max_correspondence_distance': 0.3,  # meters
    'voxel_size': 0.05,  # meters
    'outlier_rejection_threshold': 2.0,  # std devs
}
```

### 11.2 EKF Parameters

```python
ekf_params = {
    'process_noise_distance': 0.01,  # 1% of distance
    'process_noise_rotation': 0.005,
    'min_noise_floor': 0.0001,
    'icp_rejection_distance': 0.5,  # meters
    'icp_rejection_angle': np.radians(45),  # radians
}
```

### 11.3 Submap Parameters

```python
submap_params = {
    'scans_per_submap': 50,
    'min_scans_for_icp': 5,
    'voxel_size_global': 0.05,  # meters
}
```

## 12. Experimental Validation

### 12.1 Evaluation Metrics

**Localization Accuracy:**
- Absolute Trajectory Error (ATE)
- Relative Pose Error (RPE)

**Map Quality:**
- Precision (false positives)
- Recall (coverage)
- Resolution

**Computational Performance:**
- Processing time per scan
- Memory usage
- CPU utilization

**Uncertainty Calibration:**
- Consistency (NEES test)
- Confidence interval coverage

### 12.2 Baseline Comparison

The ICP-based approach serves as a baseline for comparison with feature-based SLAM:

| Aspect | ICP-Based | Feature-Based |
|--------|-----------|---------------|
| Map representation | Dense point cloud | Sparse landmarks |
| Feature extraction | None | Required |
| Data association | Implicit (ICP) | Explicit (Mahalanobis) |
| Memory scaling | $O(environment\\_size)$ | $O(num\\_landmarks)$ |
| Computation | $O(N \log M)$ per scan | $O(L^2)$ per landmark |

## 13. Implementation Notes

### 13.1 ROS2 Integration

```python
class LocalSubmapGenerator(Node):
    def __init__(self):
        # Set mapping mode
        self.mapping_mode = 'icp'

    def scan_callback(self, msg):
        if self.mapping_mode == 'icp':
            self._process_scan_icp_mode(msg)
```

### 13.2 Coordinate Frames

- **odom frame**: Odometry-based pose (drifts over time)
- **map frame**: Global consistent frame (corrected by ICP)
- **base_link**: Robot body frame

Transform chain: `map → odom → base_link`

### 13.3 Key Files

| File | Purpose |
|------|---------|
| `local_submap_generator.py` | Main SLAM node |
| `mapping_utils.py` | ICP implementation |
| `submap_stitcher.py` | Global map management |
| `ekf_slam.py` | EKF state estimation |
| `transform_utils.py` | Coordinate transformations |
| `evaluation_utils.py` | Uncertainty quantification |

## 14. Conclusion

The ICP-based SLAM methodology provides a robust baseline for autonomous mapping using dense scan matching. Its strengths lie in handling feature-poor environments and providing high geometric detail without feature extraction overhead. The Hessian-based uncertainty quantification offers principled confidence estimates grounded in information theory.

The primary trade-offs are computational cost scaling with point cloud density and memory requirements for storing dense maps. These characteristics make ICP-based SLAM well-suited for moderate-sized indoor environments where computational resources are available and high map fidelity is desired.

The comparative evaluation against feature-based SLAM will quantify these trade-offs empirically across different environment types and exploration scenarios.

## References

Besl, P. J., & McKay, N. D. (1992). Method for registration of 3-D shapes. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239-256.

Censi, A. (2007). An accurate closed-form estimate of ICP's covariance. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 3167-3172.

Grisetti, G., Stachniss, C., & Burgard, W. (2007). Improved techniques for grid mapping with Rao-Blackwellized particle filters. *IEEE Transactions on Robotics*, 23(1), 34-46.

Gutmann, J. S., & Konolige, K. (2000). Incremental mapping of large cyclic environments. *Proceedings of IEEE International Symposium on Computational Intelligence in Robotics and Automation (CIRA)*, 318-325.

Hess, W., Kohler, D., Rapp, H., & Andor, D. (2016). Real-time loop closure in 2D LIDAR SLAM. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 1271-1278.

Lu, F., & Milios, E. (1997). Globally consistent range scan alignment for environment mapping. *Autonomous Robots*, 4(4), 333-349.

Rusinkiewicz, S., & Levoy, M. (2001). Efficient variants of the ICP algorithm. *Proceedings of International Conference on 3D Digital Imaging and Modeling (3DIM)*, 145-152.

Segal, A., Haehnel, D., & Thrun, S. (2009). Generalized-ICP. *Proceedings of Robotics: Science and Systems (RSS)*.

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press.
