# ICP-Based Submap Stitching with Uncertainty Quantification

**Technical Report**
**Date:** February 25, 2026
**System:** Feature-Based EKF-SLAM with Submap Management

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [ICP Uncertainty Calculation](#2-icp-uncertainty-calculation)
3. [Submap Stitching Pipeline](#3-submap-stitching-pipeline)
4. [Wall Endpoint Correction](#4-wall-endpoint-correction)
5. [Implementation Details](#5-implementation-details)
6. [References](#6-references)

---

## 1. Introduction

This report documents the implementation of Iterative Closest Point (ICP) based submap stitching with uncertainty quantification using the Censi covariance estimation method. The system performs two key operations:

1. **Point Cloud Alignment**: Align incoming submaps to the global map using ICP
2. **Semantic Update**: Correct wall landmark endpoints based on ICP transformation

The implementation uses a hybrid approach:
- GPU-accelerated tensor operations for ICP computation (speed)
- Legacy point cloud format for correspondence extraction (covariance calculation)

---

## 2. ICP Uncertainty Calculation

### 2.1 Theoretical Foundation

The uncertainty of ICP registration is estimated using the **Censi method** [1], which relates the pose covariance to the geometry of point correspondences and measurement noise.

**Key Insight**: The covariance of the estimated transformation is inversely proportional to the Fisher Information Matrix (Hessian) of the least-squares objective.

### 2.2 Problem Formulation

Given:
- **Source point cloud** S = {p₁, p₂, ..., pₙ} (submap points)
- **Target point cloud** T (global map points)
- **ICP transformation** T(x) that minimizes:

```
E(x) = Σᵢ ||R(θ) · pᵢ + t - qᵢ||²
```

where:
- x = [tₓ, tᵧ, θ]ᵀ is the 2D pose (translation + rotation)
- R(θ) is the 2D rotation matrix
- qᵢ is the corresponding point in T for pᵢ

We seek the **covariance matrix** Σₓ of the estimated pose x.

### 2.3 Jacobian Derivation

For each correspondence (pᵢ, qᵢ), we compute the Jacobian of the transformed point with respect to the pose parameters.

**Transformation equation:**
```
p'ᵢ = R(θ) · pᵢ + t = [cos θ  -sin θ] [pₓ] + [tₓ]
                       [sin θ   cos θ] [pₕ]   [tᵧ]
```

**Partial derivatives:**

∂p'ᵢ/∂tₓ = [1, 0]ᵀ

∂p'ᵢ/∂tᵧ = [0, 1]ᵀ

∂p'ᵢ/∂θ = ∂(R(θ)·pᵢ)/∂θ

For the rotation derivative:
```
∂R(θ)/∂θ = [-sin θ  -cos θ]
           [ cos θ  -sin θ]
```

Therefore:
```
∂(R(θ)·pᵢ)/∂θ = [-sin θ  -cos θ] [pₓ] = [-sin(θ)·pₓ - cos(θ)·pₕ]
                 [ cos θ  -sin θ] [pₕ]   [ cos(θ)·pₓ - sin(θ)·pₕ]
```

**Jacobian matrix** Jᵢ ∈ ℝ²ˣ³:
```
Jᵢ = [1   0   -sin(θ)·pₓ - cos(θ)·pₕ]
     [0   1    cos(θ)·pₓ - sin(θ)·pₕ]
```

### 2.4 Hessian Computation

The **Gauss-Newton approximation** of the Hessian is:

```
H = Σᵢ JᵢᵀJᵢ
```

where the sum is over all N correspondences.

For each correspondence:
```
JᵢᵀJᵢ = [1                    0                      J₁₃  ]
        [0                    1                      J₂₃  ]
        [J₁₃                  J₂₃                    J₁₃² + J₂₃²]
```

where:
- J₁₃ = -sin(θ)·pₓ - cos(θ)·pₕ
- J₂₃ = cos(θ)·pₓ - sin(θ)·pₕ

### 2.5 Hessian Stabilization

To prevent numerical instability when H is near-singular:

**Eigenvalue clamping:**
```
H = VΛVᵀ  (eigendecomposition)
Λ' = max(Λ, ε·I)  where ε = 10⁻⁶
H_stable = VΛ'Vᵀ
```

### 2.6 Covariance Estimation

The pose covariance is computed using the **Censi formula**:

```
Σₓ = σ² · H⁻¹
```

where:
- σ² = variance of LiDAR measurement noise (0.01 m)²
- H⁻¹ = inverse of the stabilized Hessian

**Interpretation:**
- σ² scales the covariance based on sensor noise
- H⁻¹ captures geometric configuration of correspondences
- More correspondences → smaller uncertainty
- Better geometric spread → smaller uncertainty

**Resulting covariance matrix:**
```
Σₓ = [σₓ²      σₓᵧ     σₓθ  ]
     [σₓᵧ      σᵧ²     σᵧθ  ]
     [σₓθ      σᵧθ     σθ²  ]
```

### 2.7 Implementation Pseudocode

```python
# After ICP convergence
correspondences = find_nearest_neighbors(source_transformed, target, max_dist=0.03)

# Initialize Hessian
H = zeros(3, 3)
c, s = cos(theta), sin(theta)

for (src_idx, tgt_idx) in correspondences:
    p = source_points[src_idx]  # Original source point (before transformation)

    # Jacobian: [2x3] matrix
    J = [[1.0,  0.0,  -s*p.x - c*p.y],
         [0.0,  1.0,   c*p.x - s*p.y]]

    # Accumulate: H += J^T @ J
    H += J.T @ J

# Stabilize eigenvalues
eigvals, eigvecs = eigh(H)
eigvals = maximum(eigvals, 1e-6)
H_stable = eigvecs @ diag(eigvals) @ eigvecs.T

# Compute covariance
sigma_lidar = 0.01  # 1 cm LiDAR noise
Sigma = (sigma_lidar ** 2) * inv(H_stable)

return Sigma  # [3x3] covariance matrix
```

---

## 3. Submap Stitching Pipeline

### 3.1 Overall Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Feature-Based EKF-SLAM (every 50 scans)                   │
│  ↓                                                           │
│  Generate Submap Point Cloud (0.05m spacing)                │
│  ↓                                                           │
│  Points in Map Frame (via EKF pose)                         │
└─────────────────────────────────────────────────────────────┘
                          ↓
┌─────────────────────────────────────────────────────────────┐
│  Submap Stitching (SubmapStitcher)                         │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Step 1: Voxel Downsampling (0.05m)                  │ │
│  └───────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Step 2: ICP Alignment                                │ │
│  │    - Point-to-point ICP (GPU tensors)                │ │
│  │    - Max correspondence distance: 0.03m (3σ odom)    │ │
│  │    - Max iterations: 50                              │ │
│  │    - Convergence check: fitness > 0.3, RMSE < 0.05  │ │
│  └───────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Step 3: Uncertainty Quantification                   │ │
│  │    - Extract correspondences via KD-tree              │ │
│  │    - Compute Censi covariance                         │ │
│  └───────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Step 4: Apply Correction                             │ │
│  │    - Transform submap points                          │ │
│  │    - Correct wall endpoints (semantic update)         │ │
│  └───────────────────────────────────────────────────────┘ │
│  ┌───────────────────────────────────────────────────────┐ │
│  │  Step 5: Global Map Integration                       │ │
│  │    - Concatenate corrected points                     │ │
│  │    - Voxel downsample global map                      │ │
│  └───────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### 3.2 Submap Preprocessing

**Input**: Nx3 numpy array of points generated from FeatureMap landmarks

**Processing**:
1. Convert to Open3D tensor (GPU if available)
2. Voxel downsample at 0.05m resolution
3. Reduces point density while preserving structure

```python
def process_submap(points):
    points_tensor = Tensor(points.astype(float32), device=GPU)
    pcd_tensor = PointCloud(device=GPU)
    pcd_tensor.point.positions = points_tensor
    pcd_tensor = pcd_tensor.voxel_down_sample(voxel_size=0.05)
    return pcd_tensor
```

**Rejection criteria**:
- Fewer than 50 points → submap discarded

### 3.3 ICP Alignment Algorithm

**Input**:
- Source: Current submap point cloud (already in map frame via EKF)
- Target: Accumulated global map
- Initial guess: Identity (EKF pose already applied)

**Open3D Tensor ICP Parameters**:
```
max_correspondence_distance = 0.03 m  # 3σ of odometry drift
max_iteration = 50
relative_fitness_threshold = 1e-6
relative_rmse_threshold = 1e-6
estimation_method = TransformationEstimationPointToPoint()
```

**Algorithm Steps**:

1. **Initialization**:
   - Start from identity transformation (EKF already aligned points)
   - Build KD-tree for target point cloud

2. **Iteration** (repeat until convergence):
   ```
   For i = 1 to 50:
       a) Find correspondences:
          For each source point p:
              q = nearest_neighbor(target, transform(p, T_current))
              if distance(p, q) < 0.03:
                  add (p, q) to correspondence set

       b) Estimate transformation:
          T_new = argmin_T Σ ||T·p - q||²  (closed-form SVD solution)

       c) Update:
          T_current = T_new

       d) Check convergence:
          if |fitness_new - fitness_old| / fitness_old < 1e-6:
              break
   ```

3. **Convergence Metrics**:
   - **Fitness**: fraction of source points with valid correspondence
   - **RMSE**: root mean square error of inlier correspondences

**Quality Checks**:
```
if fitness < 0.3:      # Less than 30% overlap
    reject alignment
if inlier_rmse > 0.05: # More than 5cm error
    reject alignment
```

### 3.4 Transformation Extraction

From the 4x4 transformation matrix T:

```
T = [R₀₀  R₀₁  0  tₓ]
    [R₁₀  R₁₁  0  tᵧ]
    [ 0    0   1   0]
    [ 0    0   0   1]
```

Extract 2D pose:
```
dx = T[0, 3]
dy = T[1, 3]
dtheta = atan2(T[1, 0], T[0, 0])
```

### 3.5 Fallback Strategy

If ICP fails (low fitness or high RMSE):
- Use EKF pose without correction
- Points added to global map as-is
- Confidence tracker reflects degraded accuracy via EKF covariance

---

## 4. Wall Endpoint Correction

### 4.1 Motivation

After ICP alignment corrects the submap pose, wall landmark endpoints must be updated to reflect the refined position. This maintains consistency between:
- **Point cloud** (geometric representation)
- **Semantic features** (wall landmarks in FeatureMap)

### 4.2 Wall Representation

Walls are stored in **Hessian normal form** with tangential extents:

```
Wall = {
    rho: float,      # Perpendicular distance from origin
    alpha: float,    # Angle of normal vector
    t_min: float,    # Minimum tangential extent
    t_max: float     # Maximum tangential extent
}
```

**Geometry**:
```
normal = [cos(alpha), sin(alpha)]
tangent = [-sin(alpha), cos(alpha)]
line_point = rho * normal
start = line_point + t_min * tangent
end = line_point + t_max * tangent
```

### 4.3 Endpoint Correction Algorithm

**Input**:
- FeatureMap with walls in submap frame
- ICP correction: (R, t) where R ∈ SO(2), t ∈ ℝ²

**For each wall landmark**:

#### Step 1: Reconstruct Endpoints from Scalar Extents

```python
# Hessian parameters
alpha_src = wall['alpha']
rho_src = wall['rho']

# Basis vectors
normal_src = [cos(alpha_src), sin(alpha_src)]
tangent_src = [-sin(alpha_src), cos(alpha_src)]

# Line point
line_pt_src = rho_src * normal_src

# Endpoints
start_raw = line_pt_src + wall['t_min'] * tangent_src
end_raw = line_pt_src + wall['t_max'] * tangent_src
```

#### Step 2: Apply ICP Transformation

```python
start_corr = R @ start_raw + t
end_corr = R @ end_raw + t
```

#### Step 3: Recompute Hessian Parameters

**Corrected angle**:
```python
dtheta = atan2(R[1,0], R[0,0])
alpha_corr = normalize_angle(alpha_src + dtheta)
```

**Corrected basis vectors**:
```python
normal_corr = [cos(alpha_corr), sin(alpha_corr)]
tangent_corr = [-sin(alpha_corr), cos(alpha_corr)]
```

**Corrected rho** (perpendicular distance):
```python
centroid_corr = 0.5 * (start_corr + end_corr)
rho_corr = dot(centroid_corr, normal_corr)

# Enforce rho >= 0 (flip normal if needed)
if rho_corr < 0:
    rho_corr = -rho_corr
    alpha_corr = normalize_angle(alpha_corr + π)
    normal_corr = -normal_corr
    tangent_corr = -tangent_corr
```

#### Step 4: Project to Tangential Coordinates

```python
t_s = dot(start_corr, tangent_corr)
t_e = dot(end_corr, tangent_corr)
t_min_corr = min(t_s, t_e)
t_max_corr = max(t_s, t_e)
```

#### Step 5: Upsert into Global Wall Registry

**Case 1: New wall**
```python
global_walls[landmark_id] = {
    'rho': rho_corr,
    'alpha': alpha_corr,
    't_min': t_min_corr,
    't_max': t_max_corr
}
```

**Case 2: Existing wall (extend extents)**
```python
existing = global_walls[landmark_id]
alpha_e = existing['alpha']
tangent_e = [-sin(alpha_e), cos(alpha_e)]

# Project new endpoints onto existing wall's tangent
proj_s = dot(start_corr, tangent_e)
proj_e = dot(end_corr, tangent_e)

# Extend extents
existing['t_min'] = min(existing['t_min'], proj_s, proj_e)
existing['t_max'] = max(existing['t_max'], proj_s, proj_e)
```

**Rationale**: As robot re-observes walls from different viewpoints, the observed extents grow. By maintaining t_min/t_max, we track the longest observed section of each wall.

### 4.4 Coordinate Frame Consistency

```
┌─────────────────────────────────────────────────────────┐
│  Submap Frame (via EKF)                                 │
│    Wall: (rho_src, alpha_src, t_min_src, t_max_src)   │
│    Points: generated at 0.05m spacing                   │
└─────────────────────────────────────────────────────────┘
                          ↓ ICP Correction (R, t)
┌─────────────────────────────────────────────────────────┐
│  Global Map Frame                                       │
│    Wall: (rho_corr, alpha_corr, t_min_corr, t_max_corr)│
│    Points: transformed by ICP                           │
└─────────────────────────────────────────────────────────┘
```

Both point cloud and semantic features are corrected consistently, ensuring:
- Point cloud aligns with global map
- Wall endpoints match point cloud geometry
- Semantic map remains accurate

---

## 5. Implementation Details

### 5.1 Performance Optimization

**GPU Acceleration**:
- ICP computation on CUDA tensors (10-50ms per alignment)
- CPU fallback if CUDA unavailable

**Hybrid Approach**:
- Tensor API for ICP (fast)
- Legacy API for correspondence extraction (covariance calculation)
- Total overhead: ~18ms per submap

**Voxel Downsampling**:
- Reduces point count by ~70-80%
- Maintains geometric structure
- Speeds up KD-tree queries

### 5.2 Key Parameters

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Voxel size | 0.05 m | Matches wall point spacing |
| Max correspondence distance | 0.03 m | 3σ of odometry drift |
| Min fitness | 0.3 | At least 30% overlap required |
| Max RMSE | 0.05 m | 5cm maximum alignment error |
| Max iterations | 50 | Sufficient for convergence |
| LiDAR noise σ | 0.01 m | 1cm standard deviation |
| Min correspondences | 20 | Minimum for robust covariance |

### 5.3 Failure Modes and Handling

**Insufficient Points** (< 50):
- Reject submap entirely
- No update to global map

**ICP Convergence Failure** (fitness < 0.3 or RMSE > 0.05):
- Fall back to EKF pose (no ICP correction)
- Add points using identity transformation
- Log degraded accuracy in confidence tracker

**Singular Hessian**:
- Eigenvalue clamping prevents numerical issues
- Pseudoinverse as fallback if inversion fails

### 5.4 Output

**Pose Correction Dictionary**:
```python
{
    'dx': float,              # Translation X (meters)
    'dy': float,              # Translation Y (meters)
    'dtheta': float,          # Rotation (radians)
    'type': 'point_cloud_icp',
    'covariance': ndarray     # [3x3] Σ_x
}
```

**Logged Metrics**:
- Fitness: overlap fraction
- RMSE: alignment error (meters)
- Number of correspondences
- ICP correction: (dx, dy, dtheta)
- Covariance diagonal: (σ_x², σ_y², σ_θ²)

---

## 6. References

[1] **Censi, A.** (2007). "An accurate closed-form estimate of ICP's covariance." *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3167-3172.

[2] **Besl, P. J., & McKay, N. D.** (1992). "A method for registration of 3-D shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239-256.

[3] **Rusinkiewicz, S., & Levoy, M.** (2001). "Efficient variants of the ICP algorithm." *Proceedings of the Third International Conference on 3-D Digital Imaging and Modeling*, pp. 145-152.

[4] **Pomerleau, F., Colas, F., & Siegwart, R.** (2015). "A review of point cloud registration algorithms for mobile robotics." *Foundations and Trends in Robotics*, 4(1), 1-104.

[5] **Zhou, Q. Y., Park, J., & Koltun, V.** (2018). "Open3D: A modern library for 3D data processing." *arXiv preprint arXiv:1801.09847*.

---

## Appendix A: Mathematical Notation

| Symbol | Description |
|--------|-------------|
| S, T | Source and target point clouds |
| pᵢ, qᵢ | Corresponding points |
| R, t | Rotation matrix and translation vector |
| θ | Rotation angle (2D) |
| x = [tₓ, tᵧ, θ]ᵀ | 2D pose vector |
| J | Jacobian matrix |
| H | Hessian (Fisher Information) matrix |
| Σₓ | Pose covariance matrix |
| σ² | LiDAR measurement variance |
| ρ, α | Wall Hessian parameters |
| t_min, t_max | Wall tangential extents |

---

## Appendix B: Code Cross-Reference

| Section | File | Function | Lines |
|---------|------|----------|-------|
| ICP Alignment | `submap_stitcher.py` | `point_cloud_icp_align()` | 47-172 |
| Uncertainty Calculation | `submap_stitcher.py` | `point_cloud_icp_align()` | 126-153 |
| Wall Endpoint Correction | `submap_stitcher.py` | `_accumulate_walls()` | 174-243 |
| Submap Integration | `submap_stitcher.py` | `integrate_submap_to_global_map()` | 248-353 |

---

**End of Report**
