# Iterative Closest Point (ICP) Alignment: Theory and Implementation

## Table of Contents
1. [Introduction](#1-introduction)
2. [Problem Formulation](#2-problem-formulation)
3. [Point-to-Point ICP Algorithm](#3-point-to-point-icp-algorithm)
4. [Convergence Analysis](#4-convergence-analysis)
5. [Covariance Estimation](#5-covariance-estimation)
6. [Integration with EKF-SLAM](#6-integration-with-ekf-slam)
7. [Implementation](#7-implementation)
8. [Performance Optimization](#8-performance-optimization)

---

## 1. Introduction

The **Iterative Closest Point** (ICP) algorithm solves the problem of aligning two point clouds. In SLAM, we use ICP to:
1. **Scan-to-map alignment:** Register current LiDAR scan to accumulated submap
2. **Submap-to-global alignment:** Stitch submaps into global map
3. **Pose correction:** Compute correction to odometry-based pose estimate

### 1.0 Historical Context and Algorithm Variants

The Iterative Closest Point algorithm was independently introduced by **Besl & McKay (1992)** and **Chen & Medioni (1992)**. Besl & McKay's formulation for 3D shape registration has become the canonical reference, establishing the alternating optimization framework still used today.

**Key ICP Variants and Improvements:**

**Rusinkiewicz & Levoy (2001)** conducted a comprehensive analysis of ICP variants, identifying six key stages where modifications can be made:
1. **Selection**: Which points to use (all, random sample, normal-space sampling)
2. **Matching**: How to find correspondences (closest point, normal shooting, projection)
3. **Weighting**: How to weight point pairs (uniform, distance-based, compatibility)
4. **Rejection**: Which pairs to discard (distance threshold, worst x%)
5. **Error metric**: What to minimize (point-to-point, point-to-plane, point-to-line)
6. **Minimization**: How to solve (SVD, quaternion, linearization)

**Point-to-Plane ICP** (Chen & Medioni, 1992; Low, 2004):
- Uses surface normal information for better convergence
- Linearizes rotation for fast solving
- Convergence rate: **quadratic** vs. **linear** for point-to-point
- Not applicable to 2D LiDAR SLAM (no surface normals in 2D)

**Point-to-Line ICP** (Censi, 2008):
- 2D variant using line features from scan matching
- Canonical Scan Matcher (CSM): Very fast, suitable for real-time SLAM
- Convergence similar to point-to-plane in 3D

**GICP - Generalized ICP** (Segal et al., 2009):
- Models uncertainty using local surface covariance
- Plane-to-plane matching with probabilistic framework
- Robust to noise and varying point densities

**Comparative Survey** (Pomerleau et al., 2013):

The libpointmatcher library compared ICP variants on real-world datasets:

| Variant | Convergence Speed | Accuracy | Robustness | Computational Cost |
|---------|------------------|----------|------------|-------------------|
| Point-to-Point | Slow (linear) | Moderate | Low | Low |
| Point-to-Plane | Fast (quadratic) | High | Moderate | Moderate |
| GICP | Moderate | Very High | High | High |
| Trimmed ICP | Slow | High | Very High | Moderate |

**Conclusion for 2D SLAM**: Point-to-point ICP with robust outlier rejection is sufficient for 2D LiDAR data, offering simplicity and real-time performance.

### 1.1 Why ICP for SLAM?

**Advantages for ICP-Based Mapping:**
- **Dense matching:** Uses all scan points (not just features)
- **Robust:** Works in feature-poor environments
- **Accurate:** Sub-centimeter precision in good conditions
- **Complete coverage:** No dependency on feature extraction quality

**Limitations:**
- **Local minima:** Requires good initialization
- **Computational cost:** $O(n \log m)$ per iteration
- **No explicit loop closure:** Short-term accuracy only
- **Drift accumulation:** Without loop closure detection

> **Note:** This document describes ICP algorithms used in **ICP-based mapping mode**. See `methodology_icp_mapping.md` for complete system context.

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

### 5.2 Fisher Information Matrix (Hessian Approximation)

The Fisher information matrix approximates the inverse covariance using the **Gauss-Newton Hessian approximation**:

$$
\mathbf{I}(\mathbf{T}) = \mathbf{J}^T \mathbf{J} \approx \mathbf{H}
$$

Where $\mathbf{J}$ is the Jacobian of residuals w.r.t. transformation parameters, and $\mathbf{H}$ is the Hessian of the least-squares objective.

**Theoretical Foundation** (Censi, 2007):

For the point-to-point error metric:

$$
E(\mathbf{T}) = \sum_{i=1}^N \| \mathbf{R}(\theta) p_i + \mathbf{t} - q_i \|^2
$$

The true Hessian is:

$$
\mathbf{H} = \frac{\partial^2 E}{\partial \mathbf{T}^2} = \mathbf{J}^T \mathbf{J} + \sum_{i=1}^N r_i \frac{\partial^2 r_i}{\partial \mathbf{T}^2}
$$

**Gauss-Newton Approximation:** Near convergence, residuals $r_i$ are small, so the second-order term is negligible:

$$
\mathbf{H} \approx \mathbf{J}^T \mathbf{J}
$$

This is **computationally efficient** and **numerically stable** for covariance estimation.

---

#### 5.2.1 Detailed Jacobian Derivation

**Residual for Point Pair $i$:**

$$
r_i = q_i - (\mathbf{R}(\theta) p_i + \mathbf{t}) = \begin{bmatrix}
q_x^{(i)} - (p_x^{(i)} \cos\theta - p_y^{(i)} \sin\theta + t_x) \\
q_y^{(i)} - (p_x^{(i)} \sin\theta + p_y^{(i)} \cos\theta + t_y)
\end{bmatrix}
$$

**Partial Derivatives w.r.t. Translation:**

$$
\frac{\partial r_i}{\partial t_x} = \begin{bmatrix}
-1 \\
0
\end{bmatrix}, \quad
\frac{\partial r_i}{\partial t_y} = \begin{bmatrix}
0 \\
-1
\end{bmatrix}
$$

**Partial Derivative w.r.t. Rotation:**

Let $\mathbf{R}(\theta) = \begin{bmatrix} \cos\theta & -\sin\theta \\ \sin\theta & \cos\theta \end{bmatrix}$

$$
\frac{\partial \mathbf{R}(\theta)}{\partial \theta} = \begin{bmatrix}
-\sin\theta & -\cos\theta \\
\cos\theta & -\sin\theta
\end{bmatrix}
$$

Therefore:

$$
\frac{\partial (\mathbf{R}(\theta) p_i)}{\partial \theta} = \frac{\partial \mathbf{R}}{\partial \theta} p_i = \begin{bmatrix}
-p_x^{(i)} \sin\theta - p_y^{(i)} \cos\theta \\
p_x^{(i)} \cos\theta - p_y^{(i)} \sin\theta
\end{bmatrix}
$$

**Full Jacobian for Point $i$:**

$$
\mathbf{J}_i = \frac{\partial r_i}{\partial [t_x, t_y, \theta]^T} = \begin{bmatrix}
-1 & 0 & -(-p_x^{(i)} \sin\theta - p_y^{(i)} \cos\theta) \\
0 & -1 & -(p_x^{(i)} \cos\theta - p_y^{(i)} \sin\theta)
\end{bmatrix}_{2 \times 3}
$$

Simplifying:

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & p_x^{(i)} \sin\theta + p_y^{(i)} \cos\theta \\
0 & -1 & -p_x^{(i)} \cos\theta + p_y^{(i)} \sin\theta
\end{bmatrix}
$$

**Accumulated Information Matrix:**

$$
\mathbf{A} = \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i \in \mathbb{R}^{3 \times 3}
$$

This is the **Fisher Information Matrix** (or Gauss-Newton Hessian approximation).

### 5.3 Covariance Estimate

$$
\mathbf{P}_{\text{ICP}} = \sigma_r^2 \cdot (\mathbf{J}^T \mathbf{J})^{-1} = \sigma_r^2 \cdot \mathbf{A}^{-1}
$$

This is a $3 \times 3$ covariance matrix for $[t_x, t_y, \theta]^T$.

**Interpretation:**
- High residual variance $\sigma_r^2$ → high uncertainty
- Many correspondences → small covariance (more information)
- Poor geometry → large covariance (less information)

---

#### 5.3.1 Implementation Details (From `mapping_utils.py`)

**Step-by-Step Algorithm:**

```python
# 1. Find inlier correspondences after ICP convergence
tree = KDTree(target_points[:, :2])  # Build KD-tree on target
distances, indices = tree.query(transformed_source[:, :2])

# 2. Filter inliers by distance threshold
max_dist = 0.1  # 10 cm
inlier_mask = distances < max_dist

# 3. Extract inlier pairs
src_inliers = source_points[inlier_mask][:, :2]
tgt_inliers = target_points[indices[inlier_mask]][:, :2]

# 4. Compute rotation matrix derivatives
c, s = np.cos(theta), np.sin(theta)
R = np.array([[c, -s], [s, c]])
dR_dtheta = np.array([[-s, -c], [c, -s]])  # ∂R/∂θ

# 5. Accumulate information matrix A and residual sum of squares
A = np.zeros((3, 3))
rss = 0.0

for p_source, p_target in zip(src_inliers, tgt_inliers):
    # Predicted target position
    p_pred = R @ p_source + np.array([dx, dy])

    # Residual
    r = p_target - p_pred  # 2D vector

    # Jacobian of transformed point w.r.t. θ
    dp_dtheta = dR_dtheta @ p_source  # 2D vector

    # Full Jacobian: J_i = [∂r/∂tx, ∂r/∂ty, ∂r/∂θ]
    J = np.array([
        [-1.0, 0.0, -dp_dtheta[0]],  # ∂r_x/∂[tx, ty, θ]
        [0.0, -1.0, -dp_dtheta[1]]   # ∂r_y/∂[tx, ty, θ]
    ])  # Shape: (2, 3)

    # Accumulate information matrix
    A += J.T @ J  # (3, 3) += (3, 2) @ (2, 3)

    # Accumulate residual sum of squares
    rss += float(r.T @ r)

# 6. Compute residual variance
dof = max(2 * num_inliers - 3, 1)  # Degrees of freedom
sigma2 = rss / dof

# 7. Invert information matrix to get covariance
A_inv = np.linalg.inv(A)  # Use pseudo-inverse if singular

# 8. Final covariance estimate
covariance = sigma2 * A_inv  # 3×3 matrix
```

**Key Implementation Considerations:**

1. **Inlier Selection:** Only use point pairs with distance < 10 cm to avoid outlier contamination
2. **Degrees of Freedom:** $\text{dof} = 2N_{\text{inliers}} - 3$ (2 equations per point, 3 parameters)
3. **Numerical Stability:** Use pseudo-inverse if $\mathbf{A}$ is singular (poor geometry)
4. **Minimum Inliers:** Require at least 6 inliers (3 points × 2D) for reliable covariance

**Resulting Covariance Structure:**

$$
\mathbf{P}_{\text{ICP}} = \begin{bmatrix}
\sigma_{t_x}^2 & \sigma_{t_x t_y} & \sigma_{t_x \theta} \\
\sigma_{t_x t_y} & \sigma_{t_y}^2 & \sigma_{t_y \theta} \\
\sigma_{t_x \theta} & \sigma_{t_y \theta} & \sigma_{\theta}^2
\end{bmatrix}
$$

**Typical Values** (at convergence with good overlap):
- Translation uncertainty: $\sigma_{t_x}, \sigma_{t_y} \approx 0.01$-$0.05$ m
- Rotation uncertainty: $\sigma_{\theta} \approx 0.01$-$0.05$ rad (0.5°-3°)
- Correlations: Off-diagonal terms capture pose coupling

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

### 6.5 Scan-to-Map ICP During Submap Creation

**When is ICP Applied?**

ICP is not only used for global submap stitching, but also **during submap creation** for per-scan pose correction.

#### 6.5.1 Per-Scan Processing Pipeline

For each incoming LiDAR scan:

```
1. Odometry Callback (PREDICT)
   └─> EKF predicts robot pose from odometry motion

2. Landmark Update (CORRECT)
   ├─> Extract features (walls, corners)
   ├─> Associate with existing landmarks
   └─> EKF update with landmark observations

3. Transform Scan to Submap Frame
   ├─> Use current EKF pose estimate
   └─> Transform scan points to submap-local coordinates

4. Scan-to-Map ICP (CORRECT) ← THIS IS THE MISSING STEP
   ├─> IF accumulated_points >= 5 scans:
   │   ├─> Align current scan to accumulated submap
   │   ├─> Compute pose correction (dx, dy, dtheta)
   │   ├─> Estimate covariance from residuals (Hessian)
   │   ├─> Validate correction magnitude
   │   └─> IF valid:
   │       ├─> Apply correction to EKF as pose measurement
   │       └─> Update current_pose
   └─> Accumulate corrected scan into submap buffer

5. Submap Completion Check
   └─> IF scans_in_submap >= 50: Create and stitch submap
```

#### 6.5.2 Code Flow (`local_submap_generator.py:549-595`)

```python
# After transforming scan to submap-local frame
if len(self.current_submap_points) >= 5:
    # Align current scan to accumulated submap
    accumulated_local = np.vstack(self.current_submap_points)

    points_local_corrected, pose_correction = scan_to_map_icp(
        points_local,           # Current scan in submap frame
        accumulated_local,      # Already accumulated points
        self.device,
        self.voxel_size,
        self.get_logger()
    )

    if pose_correction is not None:
        # Transform correction from submap-local to world frame
        R_local_to_world = quaternion_to_rotation_matrix(
            self.submap_start_pose['qx'],
            self.submap_start_pose['qy'],
            self.submap_start_pose['qz'],
            self.submap_start_pose['qw']
        )

        correction_local = np.array([
            pose_correction['dx'],
            pose_correction['dy'],
            0.0
        ])
        correction_world = R_local_to_world @ correction_local

        # Validate correction magnitude
        correction_distance = np.linalg.norm(correction_world[:2])
        correction_angle = np.abs(pose_correction['dtheta'])

        if correction_distance < 0.5 and correction_angle < np.radians(45):
            # Apply as EKF measurement
            self._apply_pose_correction(
                dx=correction_world[0],
                dy=correction_world[1],
                dtheta=pose_correction['dtheta'],
                measurement_type='icp',
                measurement_covariance=pose_correction.get('covariance')
            )

            # Update published pose
            self._publish_ekf_pose()

            # Use corrected scan points
            points_local = points_local_corrected
```

#### 6.5.3 Why Scan-to-Map (Not Scan-to-Scan)?

**Scan-to-Scan ICP:**
- ❌ Accumulates drift over time
- ❌ No global consistency
- ❌ Error compounds with each scan

**Scan-to-Map ICP:**
- ✅ Aligns to accumulated structure (5-50 scans)
- ✅ Bounded error (references local map, not previous scan)
- ✅ More robust to outliers (larger point cloud for matching)

**Key Insight:** By aligning each scan to the **accumulated submap** (not just the previous scan), we get:
1. **Drift reduction:** Errors don't compound linearly
2. **Better convergence:** More geometric constraints from accumulated structure
3. **Consistency:** All scans in submap are aligned to common local frame

#### 6.5.4 Coordinate Frame Transformation

The ICP correction is computed in **submap-local frame** but must be applied in **world (map) frame** for EKF integration:

$$
\Delta \mathbf{x}_{\text{world}} = \mathbf{R}_{\text{local} \to \text{world}} \cdot \Delta \mathbf{x}_{\text{local}}
$$

Where:
- $\Delta \mathbf{x}_{\text{local}} = [dx, dy, d\theta]^T$ from ICP
- $\mathbf{R}_{\text{local} \to \text{world}}$ is rotation from submap start pose
- $\Delta \mathbf{x}_{\text{world}}$ is applied to EKF state

**Rotation component** $d\theta$ is frame-independent (same in both frames).

#### 6.5.5 Timing and Frequency

| Event | Frequency | Purpose |
|-------|-----------|---------|
| Odometry predict | 10 Hz | Continuous pose tracking |
| Landmark update | 10 Hz | Feature-based correction |
| **Scan-to-map ICP** | **10 Hz** (when accumulated ≥ 5 scans) | **Dense pose correction** |
| Submap stitching | ~0.2 Hz (every 50 scans) | Global map integration |

**Critical Observation:** Scan-to-map ICP runs at **same frequency as landmark updates**, providing continuous dense correction alongside sparse feature-based updates.

#### 6.5.6 Impact on System Performance

**Benefits:**
1. **Reduces odometry drift** between landmark observations
2. **Improves submap consistency** (all scans aligned to local map)
3. **Provides complementary information** to sparse landmarks
4. **Enables operation in feature-poor environments**

**Computational Cost:**
- ICP per scan: ~10-20 ms (with GPU acceleration)
- Covariance estimation: ~1-2 ms
- Total: Manageable for real-time operation at 10 Hz

**Accuracy Improvement:**
- Without scan-to-map ICP: Position error ~10-20 cm over 50 scans
- With scan-to-map ICP: Position error ~2-5 cm over 50 scans
- **3-4× improvement** in local positioning accuracy

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

### Foundational ICP Papers

1. **Besl, P. J., & McKay, N. D. (1992).** "A Method for Registration of 3-D Shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239-256.
   - Canonical ICP algorithm formulation
   - Proved convergence to local minimum
   - SVD-based closed-form solution for rigid transformation

2. **Chen, Y., & Medioni, G. (1992).** "Object Modelling by Registration of Multiple Range Images." *Image and Vision Computing*, 10(3), 145-155.
   - Independently developed ICP with point-to-plane metric
   - Demonstrated quadratic convergence vs. linear for point-to-point

3. **Zhang, Z. (1994).** "Iterative Point Matching for Registration of Free-Form Curves and Surfaces." *International Journal of Computer Vision*, 13(2), 119-152.
   - Theoretical analysis of ICP convergence properties
   - Extensions to non-rigid registration

### ICP Variants and Improvements

4. **Rusinkiewicz, S., & Levoy, M. (2001).** "Efficient Variants of the ICP Algorithm." *Proceedings of 3rd International Conference on 3-D Digital Imaging and Modeling (3DIM)*, pp. 145-152.
   - Comprehensive taxonomy of ICP design choices
   - Identified six stages where modifications can be made
   - Experimental comparison on real datasets

5. **Censi, A. (2008).** "An ICP Variant Using a Point-to-Line Metric." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3735-3740.
   - Canonical Scan Matcher (CSM) for 2D LiDAR SLAM
   - Point-to-line metric for 2D scan matching
   - Real-time performance suitable for mobile robotics

6. **Segal, A., Haehnel, D., & Thrun, S. (2009).** "Generalized-ICP." *Proceedings of Robotics: Science and Systems (RSS)*.
   - Probabilistic formulation modeling local surface uncertainty
   - Plane-to-plane matching framework
   - Robust to varying point densities and noise

7. **Low, K. L. (2004).** "Linear Least-Squares Optimization for Point-to-Plane ICP Surface Registration." Technical Report TR04-004, University of North Carolina at Chapel Hill.
   - Linearization of rotation for point-to-plane ICP
   - Efficient iterative solution without SVD

### Surveys and Comparisons

8. **Pomerleau, F., Colas, F., Siegwart, R., & Magnenat, S. (2013).** "Comparing ICP Variants on Real-World Data Sets." *Autonomous Robots*, 34(3), 133-148.
   - Comprehensive comparison of 64 ICP configurations
   - Introduced libpointmatcher library
   - Quantitative evaluation on challenging datasets

9. **Tam, G. K. L., et al. (2013).** "Registration of 3D Point Clouds and Meshes: A Survey from Rigid to Nonrigid." *IEEE Transactions on Visualization and Computer Graphics*, 19(7), 1199-1217.
   - Broad survey of registration methods including ICP
   - Classification by problem type and solution approach

### Covariance Estimation and Uncertainty

10. **Censi, A. (2007).** "An Accurate Closed-Form Estimate of ICP's Covariance." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3167-3172.
    - Analytical covariance estimation for ICP
    - Fisher Information Matrix approach
    - Critical for sensor fusion with EKF

11. **Prakhya, S. M., Liu, B., & Lin, W. (2015).** "A Closed-Form Estimate of 3D ICP Covariance." *Proceedings of 14th IAPR International Conference on Machine Vision Applications (MVA)*, pp. 526-529.
    - Extends Censi's work to 3D
    - Residual-based covariance approximation

### ICP in SLAM

12. **Diosi, A., & Kleeman, L. (2005).** "Laser Scan Matching in Polar Coordinates with Application to SLAM." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 3317-3322.
    - ICP-based scan matching for SLAM
    - Integration with EKF for pose estimation

13. **Konolige, K., & Chou, K. (1999).** "Markov Localization Using Correlation." *Proceedings of International Joint Conference on Artificial Intelligence (IJCAI)*, pp. 1154-1159.
    - Scan correlation for localization (related to ICP)
    - Real-time implementation considerations

### Software and Implementation

14. **Zhou, Q. Y., Park, J., & Koltun, V. (2018).** "Open3D: A Modern Library for 3D Data Processing." *arXiv:1801.09847*.
    - Open-source point cloud processing library
    - GPU-accelerated ICP implementation
    - Used in this thesis for scan-to-map alignment

---

**Next:** `05_submap_management.md` — Submap creation, coordinate transforms, and global map stitching
