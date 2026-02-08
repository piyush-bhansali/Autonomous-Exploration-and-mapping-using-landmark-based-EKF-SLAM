# Iterative Closest Point (ICP) Alignment: Theory and Implementation

## Table of Contents
1. [Introduction](#1-introduction)
2. [Problem Formulation](#2-problem-formulation)
3. [Point-to-Point ICP Algorithm](#3-point-to-point-icp-algorithm)
4. [Convergence Analysis](#4-convergence-analysis)
5. [Covariance Estimation](#5-covariance-estimation)
6. [Integration with EKF-SLAM](#6-integration-with-ekf-slam)
7. [Implementation](#7-implementation)

---

## 1. Introduction

The **Iterative Closest Point** (ICP) algorithm solves the problem of aligning two point clouds. In SLAM, we use ICP to:
1. **Scan-to-map alignment:** Register current LiDAR scan to accumulated submap
2. **Submap-to-global alignment:** Stitch submaps into global map
3. **Pose correction:** Compute correction to odometry-based pose estimate

### 1.1 Why ICP in Hybrid SLAM?

**Complementary to Landmark SLAM:**
- **Dense matching:** Uses all scan points (not just features)
- **Robust:** Works in feature-poor environments
- **Accurate:** Sub-centimeter precision in good conditions

**Limitations:**
- **Local minima:** Requires good initialization
- **Computational cost:** $O(n \log m)$ per iteration
- **No explicit loop closure:** Short-term accuracy only

---

## 2. Problem Formulation

### 2.1 Rigid Body Transformation

We seek a rigid transformation $\mathbf{T} \in SE(2)$ (2D special Euclidean group) that aligns source point cloud to target:

$$
\mathbf{T} = \begin{bmatrix}
\mathbf{R} & \mathbf{t} \\
0 & 1
\end{bmatrix} \in \mathbb{R}^{3 \times 3}
$$

Where:
- $\mathbf{R} \in SO(2)$: Rotation matrix

$$
\mathbf{R}(\theta) = \begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix}
$$

- $\mathbf{t} = [t_x, t_y]^T \in \mathbb{R}^2$: Translation vector

**Parametrization:** $(t_x, t_y, \theta) \in \mathbb{R}^2 \times SO(2)$

### 2.2 Point Cloud Alignment

**Given:**
- Source (scan): $\mathcal{P} = \{p_i\}_{i=1}^{N_P}$
- Target (map): $\mathcal{Q} = \{q_j\}_{j=1}^{N_Q}$

**Goal:** Find $\mathbf{T}^*$ that minimizes alignment error:

$$
\mathbf{T}^* = \arg\min_{\mathbf{T}} E(\mathbf{T})
$$

### 2.3 Error Metrics

**Point-to-Point:**

$$
E_{\text{point}}(\mathbf{T}) = \sum_{i=1}^{N_P} \| \mathbf{T} p_i - q_{\text{nn}(i)} \|^2
$$

Where $q_{\text{nn}(i)}$ is the nearest neighbor of $\mathbf{T} p_i$ in $\mathcal{Q}$.

**Point-to-Plane** (not used in 2D):
In 3D, this uses surface normals for better convergence.

---

## 3. Point-to-Point ICP Algorithm

### 3.1 Standard ICP

The classic ICP algorithm alternates between:
1. **Correspondence:** Find nearest neighbors
2. **Transformation:** Solve for optimal $\mathbf{T}$

**Algorithm:**

```
ICP(source P, target Q, T_init, max_iterations, tolerance):

    T ← T_init
    prev_error ← ∞

    for iteration = 1 to max_iterations:

        // 1. CORRESPONDENCE STEP
        correspondences ← []
        for each p_i in P:
            p_transformed ← T * p_i
            q_j ← nearest_neighbor(p_transformed, Q)
            correspondences.append((p_i, q_j))

        // 2. TRANSFORMATION STEP
        T_new ← solve_least_squares(correspondences)

        // 3. CONVERGENCE CHECK
        error ← compute_error(correspondences, T_new)
        if |error - prev_error| < tolerance:
            break

        T ← T_new
        prev_error ← error

    return T, error, correspondences
```

### 3.2 Correspondence Step

For each source point $p_i$, find the closest target point:

$$
j^* = \arg\min_{j \in \{1, \ldots, N_Q\}} \| \mathbf{T} p_i - q_j \|
$$

**Efficient Search:** Use KD-tree for $O(\log N_Q)$ nearest neighbor queries.

**Total Correspondence Cost:** $O(N_P \log N_Q)$

### 3.3 Transformation Step (2D Closed-Form Solution)

Given correspondences $\{(p_i, q_i)\}_{i=1}^N$, find optimal $(\mathbf{R}, \mathbf{t})$.

**Step 1: Compute Centroids**

$$
\bar{p} = \frac{1}{N} \sum_{i=1}^N p_i, \quad \bar{q} = \frac{1}{N} \sum_{i=1}^N q_i
$$

**Step 2: Center Point Clouds**

$$
\tilde{p}_i = p_i - \bar{p}, \quad \tilde{q}_i = q_i - \bar{q}
$$

**Step 3: Compute Cross-Covariance Matrix**

$$
\mathbf{H} = \sum_{i=1}^N \tilde{p}_i \tilde{q}_i^T = \begin{bmatrix}
h_{11} & h_{12} \\
h_{21} & h_{22}
\end{bmatrix}
$$

**Step 4: Singular Value Decomposition (SVD)**

$$
\mathbf{H} = \mathbf{U} \boldsymbol{\Sigma} \mathbf{V}^T
$$

**Step 5: Optimal Rotation**

$$
\mathbf{R}^* = \mathbf{V} \mathbf{U}^T
$$

**Handle Reflection:** If $\det(\mathbf{R}^*) < 0$, flip the last column of $\mathbf{V}$:

$$
\mathbf{R}^* = \mathbf{V} \begin{bmatrix}
1 & 0 \\
0 & \det(\mathbf{V}\mathbf{U}^T)
\end{bmatrix} \mathbf{U}^T
$$

**Step 6: Optimal Translation**

$$
\mathbf{t}^* = \bar{q} - \mathbf{R}^* \bar{p}
$$

**Extract Angle:**

$$
\theta^* = \text{atan2}(\mathbf{R}^*_{21}, \mathbf{R}^*_{11})
$$

### 3.4 Error Computation

The alignment error after transformation:

$$
E = \frac{1}{N} \sum_{i=1}^N \| \mathbf{R}^* p_i + \mathbf{t}^* - q_i \|^2
$$

**Root Mean Square Error (RMSE):**

$$
\text{RMSE} = \sqrt{E}
$$

---

## 4. Convergence Analysis

### 4.1 Convergence Properties

**Theorem (Besl & McKay, 1992):**
The ICP algorithm is **guaranteed to converge** to a local minimum of the error function.

**Proof Sketch:**
1. The correspondence step finds the best match given current $\mathbf{T}$
2. The transformation step finds the optimal $\mathbf{T}$ given correspondences
3. Each step reduces (or maintains) the error
4. Error is bounded below by 0
5. By monotonicity, the algorithm converges

**Key Insight:** Convergence to **local** minimum, not necessarily global.

### 4.2 Convergence Rate

**Empirical Observation:** ICP typically converges in 10-50 iterations.

**Factors Affecting Convergence:**
- **Initialization quality:** Better init → fewer iterations
- **Point cloud overlap:** Higher overlap → faster convergence
- **Noise level:** More noise → slower convergence
- **Outlier ratio:** More outliers → more iterations

### 4.3 Convergence Criteria

Stop when **any** of the following is met:

1. **Error threshold:** $|E_{k} - E_{k-1}| < \epsilon_{\text{error}}$
2. **Transform threshold:** $\|\mathbf{T}_k - \mathbf{T}_{k-1}\| < \epsilon_{\text{transform}}$
3. **Maximum iterations:** $k \geq k_{\max}$

**Typical Values:**
- $\epsilon_{\text{error}} = 10^{-6}$
- $\epsilon_{\text{transform}} = 10^{-4}$ (in pose space)
- $k_{\max} = 30$

### 4.4 Failure Modes

**1. Local Minima:**
- Occurs when initialization is poor
- Solution: Multi-scale ICP or better initialization

**2. Symmetric Ambiguity:**
- Identical structures at multiple locations
- Solution: Use landmarks for global consistency

**3. Low Overlap:**
- < 50% overlap → unreliable
- Solution: Reject if final error is too high

---

## 5. Covariance Estimation

ICP provides a point estimate, but we need **uncertainty** for EKF integration.

### 5.1 Residual-Based Covariance

The covariance of the transformation estimate can be approximated from the alignment residuals.

**Residual at Convergence:**

$$
r_i = \mathbf{R}^* p_i + \mathbf{t}^* - q_i
$$

**Residual Variance:**

$$
\sigma_r^2 = \frac{1}{N - 3} \sum_{i=1}^N \|r_i\|^2
$$

(Degrees of freedom = $N - 3$ for 3 parameters: $t_x, t_y, \theta$)

### 5.2 Fisher Information Matrix

The Fisher information matrix approximates the inverse covariance:

$$
\mathbf{I}(\mathbf{T}) = \mathbf{J}^T \mathbf{J}
$$

Where $\mathbf{J}$ is the Jacobian of residuals w.r.t. transformation parameters.

**Jacobian Entry:**

$$
\mathbf{J}_i = \frac{\partial r_i}{\partial [t_x, t_y, \theta]^T}
$$

For a 2D rotation and translation:

$$
\frac{\partial (\mathbf{R}(\theta) p + \mathbf{t})}{\partial \theta} = \begin{bmatrix}
-\sin\theta & -\cos\theta \\
\cos\theta & -\sin\theta
\end{bmatrix} \begin{bmatrix}
p_x \\
p_y
\end{bmatrix} = \begin{bmatrix}
-p_x \sin\theta - p_y \cos\theta \\
p_x \cos\theta - p_y \sin\theta
\end{bmatrix}
$$

**Full Jacobian:**

$$
\mathbf{J}_i = \begin{bmatrix}
1 & 0 & -p_x \sin\theta - p_y \cos\theta \\
0 & 1 & p_x \cos\theta - p_y \sin\theta
\end{bmatrix}_{2 \times 3}
$$

### 5.3 Covariance Estimate

$$
\mathbf{P}_{\text{ICP}} = \sigma_r^2 \cdot (\mathbf{J}^T \mathbf{J})^{-1} = \sigma_r^2 \cdot \mathbf{I}^{-1}
$$

This is a $3 \times 3$ covariance matrix for $[t_x, t_y, \theta]^T$.

**Interpretation:**
- High residual variance $\sigma_r^2$ → high uncertainty
- Many correspondences → small covariance (more information)
- Poor geometry → large covariance (less information)

### 5.4 Practical Considerations

**Minimum Covariance:**
Enforce a minimum uncertainty to account for model errors:

$$
\mathbf{P}_{\text{final}} = \mathbf{P}_{\text{ICP}} + \mathbf{P}_{\text{min}}
$$

Where:

$$
\mathbf{P}_{\text{min}} = \text{diag}([0.01^2, 0.01^2, (1°)^2])
$$

---

## 6. Integration with EKF-SLAM

### 6.1 ICP as a Measurement

ICP provides a **relative pose correction**:

$$
\Delta \mathbf{x} = [\\Delta x, \Delta y, \Delta \theta]^T
$$

This can be fed to the EKF as a **pose measurement**.

### 6.2 Measurement Model

The ICP correction is a direct observation of robot pose:

$$
\mathbf{z}_{\text{ICP}} = \mathbf{x}_{\text{EKF}} + \Delta \mathbf{x} + \mathbf{v}_{\text{ICP}}
$$

Where $\mathbf{v}_{\text{ICP}} \sim \mathcal{N}(\mathbf{0}, \mathbf{P}_{\text{ICP}})$

### 6.3 EKF Update with ICP

```python
# After ICP alignment
dx, dy, dtheta = icp_correction
P_icp = icp_covariance

# Corrected pose estimate
x_corrected = x_ekf + dx
y_corrected = y_ekf + dy
theta_corrected = theta_ekf + dtheta

# EKF measurement update
ekf.update(
    x=x_corrected,
    y=y_corrected,
    theta=theta_corrected,
    measurement_covariance=P_icp,
    measurement_type='icp'
)
```

### 6.4 Outlier Rejection

Before applying ICP correction, validate:

1. **RMSE check:** $\text{RMSE} < \tau_{\text{error}}$ (e.g., 0.1 m)
2. **Correction magnitude:** $\|\Delta \mathbf{x}\| < \tau_{\text{correction}}$ (e.g., 0.5 m, 45°)
3. **Overlap ratio:** $> 50\%$ of source points matched

If any check fails, **reject** the ICP result.

---

## 7. Implementation

### 7.1 Scan-to-Map ICP Pipeline

```python
def scan_to_map_icp(scan_points, map_points, device, voxel_size, logger):
    """
    Align scan to map using ICP.

    Args:
        scan_points: Nx3 numpy array (source)
        map_points: Mx3 numpy array (target)
        device: Open3D device (CPU or CUDA)
        voxel_size: Downsampling resolution
        logger: ROS logger

    Returns:
        aligned_points: Transformed scan points
        correction: {'dx', 'dy', 'dtheta', 'covariance', 'rmse'}
    """

    # 1. Convert to Open3D point clouds
    source = o3d.t.geometry.PointCloud(device)
    source.point.positions = o3c.Tensor(scan_points, o3c.float32, device)

    target = o3d.t.geometry.PointCloud(device)
    target.point.positions = o3c.Tensor(map_points, o3c.float32, device)

    # 2. Downsampling (for efficiency)
    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    # 3. ICP registration
    criteria = o3d.t.pipelines.registration.ICPConvergenceCriteria(
        max_iteration=30,
        relative_rmse=1e-6,
        relative_fitness=1e-6
    )

    result = o3d.t.pipelines.registration.icp(
        source_down,
        target_down,
        max_correspondence_distance=voxel_size * 2,
        init_source_to_target=np.eye(4),
        estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=criteria
    )

    # 4. Extract transformation
    T = result.transformation.cpu().numpy()
    R = T[0:2, 0:2]
    t = T[0:2, 3]

    dx, dy = t
    dtheta = np.arctan2(R[1, 0], R[0, 0])

    # 5. Compute covariance
    fitness = result.fitness  # Fraction of inliers
    rmse = result.inlier_rmse

    # Simple covariance model
    sigma_trans = rmse
    sigma_rot = rmse / voxel_size  # Heuristic

    covariance = np.diag([
        sigma_trans**2,
        sigma_trans**2,
        sigma_rot**2
    ])

    # 6. Apply transformation
    aligned = source.transform(T)
    aligned_points = aligned.point.positions.cpu().numpy()

    correction = {
        'dx': dx,
        'dy': dy,
        'dtheta': dtheta,
        'covariance': covariance,
        'rmse': rmse,
        'fitness': fitness
    }

    return aligned_points, correction
```

### 7.2 Validation and Rejection

```python
def is_icp_correction_valid(correction, max_distance=0.5, max_angle=np.radians(45)):
    """
    Validate ICP correction before applying to EKF.
    """
    # Check magnitude
    distance = np.sqrt(correction['dx']**2 + correction['dy']**2)
    angle = abs(correction['dtheta'])

    if distance > max_distance or angle > max_angle:
        return False, "Correction too large"

    # Check RMSE
    if correction['rmse'] > 0.1:
        return False, "RMSE too high"

    # Check fitness (overlap)
    if correction['fitness'] < 0.5:
        return False, "Insufficient overlap"

    return True, "Valid"
```

### 7.3 Code Mapping

| Component | File | Function |
|-----------|------|----------|
| Scan-to-map ICP | `mapping_utils.py` | `scan_to_map_icp()` |
| Submap ICP | `submap_stitcher.py` | `integrate_submap_to_global_map()` |
| EKF integration | `local_submap_generator.py` | `_apply_pose_correction()` |

---

## 8. Performance Optimization

### 8.1 Downsampling

**Voxel downsampling** reduces point count while preserving structure:

$$
\text{downsample}(\mathcal{P}, v) = \{p_i \mid p_i = \text{centroid}(\text{voxel}_i)\}
$$

**Typical voxel size:** 0.05 m (5 cm)

**Benefit:** $O(N \log M) \to O(N' \log M')$ where $N', M' \ll N, M$

### 8.2 GPU Acceleration

Open3D supports CUDA-accelerated ICP:
- 10-50× speedup over CPU
- Critical for real-time operation

**Check availability:**
```python
import open3d.core as o3c
if o3c.cuda.is_available():
    device = o3c.Device("CUDA:0")
else:
    device = o3c.Device("CPU:0")
```

### 8.3 Multi-Scale ICP

For large initial misalignment, use **coarse-to-fine** strategy:

1. **Coarse level:** Large voxel size (0.2 m), rough alignment
2. **Fine level:** Small voxel size (0.05 m), refinement

---

## References

1. **Besl, P. J., & McKay, N. D. (1992).** "A Method for Registration of 3-D Shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239-256.

2. **Chen, Y., & Medioni, G. (1992).** "Object Modelling by Registration of Multiple Range Images." *Image and Vision Computing*, 10(3), 145-155.

3. **Rusinkiewicz, S., & Levoy, M. (2001).** "Efficient Variants of the ICP Algorithm." *3DIM 2001*.

4. **Censi, A. (2008).** "An ICP Variant Using a Point-to-Line Metric." *ICRA 2008*.

5. **Pomerleau, F., Colas, F., Siegwart, R., & Magnenat, S. (2013).** "Comparing ICP Variants on Real-World Data Sets." *Autonomous Robots*, 34(3), 133-148.

---

**Next:** `05_submap_management.md` — Submap creation, coordinate transforms, and global map stitching
