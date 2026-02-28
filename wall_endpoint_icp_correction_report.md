# Technical Report: Wall Endpoint Update After ICP Correction

**Author:** Analysis of Feature-Based EKF-SLAM System
**Date:** 2026-02-27
**Subject:** Mechanism for Updating Wall Endpoints Following ICP-Based Loop Closure

---

## Executive Summary

This report provides a detailed analysis of how wall landmark endpoints are updated following Iterative Closest Point (ICP)-based loop closure correction in the feature-based EKF-SLAM system. The system employs a **dual-representation architecture**: wall landmarks are stored both as Hessian normal form parameters (ρ, α) in the EKF state vector and as parametric extent descriptors (t_min, t_max) in the FeatureMap. The ICP correction process transforms wall endpoints geometrically for visualization and accumulation in the global map, while the EKF landmark states converge naturally through subsequent observations after the robot pose correction is applied.

---

## 1. System Architecture Overview

### 1.1 Dual Wall Representation

The system maintains two complementary representations of wall landmarks:

#### **EKF State Vector (LandmarkEKFSLAM)**
- **Location:** `ekf_update_feature.py`
- **Parameters:** (ρ, α) - Hessian normal form
  - ρ: Perpendicular distance from origin to wall
  - α: Angle of normal vector relative to map x-axis
- **Covariance:** Maintained in joint state covariance matrix P
- **Purpose:** Probabilistic state estimation with uncertainty propagation

#### **FeatureMap Extent Tracking (FeatureMap)**
- **Location:** `feature_map.py`
- **Parameters:** (ρ, α, t_min, t_max)
  - t_min, t_max: Parametric extent along wall tangent direction
  - Represents physical start/end points along the wall segment
- **Purpose:** Geometric representation for point cloud generation and visualization

#### **Global Wall Accumulator (SubmapStitcher)**
- **Location:** `submap_stitcher.py`
- **Storage:** `self.global_walls` dictionary
- **Parameters:** (ρ, α, t_min, t_max) in corrected global frame
- **Purpose:** Accumulate drift-corrected wall segments for final global map

---

## 2. ICP-Based Loop Closure Process

### 2.1 ICP Alignment Algorithm

**Method:** `point_cloud_icp_align()` in `submap_stitcher.py:40-127`

The ICP alignment computes a rigid transformation T that aligns the current local submap point cloud to the accumulated global map:

```python
reg_result = o3d.t.pipelines.registration.icp(
    source=source_tensor,        # Current submap (50 scans)
    target=target_tensor,        # Global map (all previous submaps)
    max_correspondence_distance=0.05,  # 5cm correspondence threshold
    init_source_to_target=init_transform,  # Usually identity
    estimation_method=TransformationEstimationPointToPoint(),
    criteria=ICPConvergenceCriteria(
        max_iteration=50,
        relative_fitness=1e-6,
        relative_rmse=1e-6
    )
)
```

**Quality Thresholds:**
- Fitness > 0.5 (at least 50% of points must have correspondences)
- Inlier RMSE < 0.05 m (5cm maximum alignment error)

**Outputs:**
- **Transformation:** 4×4 homogeneous matrix T
- **Pose Correction:** (dx, dy, dθ) extracted from T
- **Covariance:** 3×3 matrix computed via Censi method

---

### 2.2 Censi Covariance Estimation

**Method:** Lines 98-117 in `submap_stitcher.py`

The uncertainty of the ICP alignment is quantified using the **Censi method** (Fisher Information approach):

1. **Extract Point Correspondences:**
   ```python
   for i in range(len(source_transformed.points)):
       [k, idx, dist] = kdtree.search_knn_vector_3d(source_transformed.points[i], 1)
       if dist[0] < 0.03²:  # 3cm threshold
           correspondences.append((i, idx[0]))
   ```

2. **Build Fisher Information Matrix (Hessian):**

   For each correspondence (p_src, p_tgt), compute the Jacobian of the transformation:

   ```
   J = [ 1   0   -sin(θ)·p_x - cos(θ)·p_y ]
       [ 0   1    cos(θ)·p_x - sin(θ)·p_y ]
   ```

   Then accumulate: **H = Σ J^T · J**

3. **Compute Covariance:**

   **Σ_pose = σ_lidar² · H⁻¹**

   where σ_lidar = 0.01 m (LiDAR noise standard deviation)

**Eigenvalue Conditioning:** To prevent ill-conditioned inversions, eigenvalues are clamped:
```python
eigvals = np.maximum(eigvals, 1e-6)  # Minimum eigenvalue floor
```

**Result:** A 3×3 covariance matrix representing uncertainty in [dx, dy, dθ]

---

## 3. Wall Endpoint Transformation Mechanism

### 3.1 Geometric Transformation Process

**Method:** `_accumulate_walls()` in `submap_stitcher.py:129-185`

This is the **core mechanism** for updating wall endpoints after ICP correction. The process is executed for every wall landmark in the current submap's FeatureMap.

#### **Step-by-Step Algorithm:**

**Input:**
- `feature_map`: Current local submap's FeatureMap
- `R`: 2×2 rotation matrix from ICP correction
- `t`: 2D translation vector [dx, dy]

**For each wall landmark:**

**Step 1: Extract Source Wall Parameters (Robot Frame)**
```python
alpha_src = wall['alpha']      # Wall normal angle
rho_src = wall['rho']          # Perpendicular distance
t_min = wall['t_min']          # Start extent parameter
t_max = wall['t_max']          # End extent parameter
```

**Step 2: Compute Tangent and Normal Vectors**
```python
tangent_src = [-sin(α), cos(α)]   # Direction along wall
normal_src  = [cos(α), sin(α)]    # Perpendicular direction
```

**Step 3: Reconstruct Start and End Points**

The wall is represented as a line with parametric extent:
```
line_point = ρ · normal
start_raw = line_point + t_min · tangent
end_raw   = line_point + t_max · tangent
```

**Step 4: Apply ICP Transformation**
```python
start_corr = R @ start_raw + t
end_corr   = R @ end_raw + t
```

**Step 5: Compute Corrected Hessian Parameters**

After transformation, recompute (ρ, α) from the corrected endpoints:

```python
dtheta = arctan2(R[1,0], R[0,0])
alpha_corr = atan2(sin(alpha_src + dtheta), cos(alpha_src + dtheta))
normal_corr = [cos(alpha_corr), sin(alpha_corr)]
centroid_corr = 0.5 · (start_corr + end_corr)
rho_corr = dot(centroid_corr, normal_corr)
```

**Step 6: Enforce Hessian Normalization (ρ > 0)**
```python
if rho_corr < 0:
    rho_corr = -rho_corr
    alpha_corr = atan2(sin(alpha_corr + π), cos(alpha_corr + π))
    normal_corr = -normal_corr
    tangent_corr = -tangent_corr
```

**Step 7: Compute Parametric Extent in Corrected Frame**
```python
tangent_corr = [-sin(alpha_corr), cos(alpha_corr)]
t_s = dot(start_corr, tangent_corr)
t_e = dot(end_corr, tangent_corr)
t_min_corr = min(t_s, t_e)
t_max_corr = max(t_s, t_e)
```

---

### 3.2 Global Wall Accumulation

**Step 8: Merge into Global Wall Dictionary**

If the wall landmark is encountered for the first time:
```python
if lm_id not in self.global_walls:
    self.global_walls[lm_id] = {
        'rho': rho_corr,
        'alpha': alpha_corr,
        't_min': t_min_corr,
        't_max': t_max_corr
    }
```

If the wall landmark has been seen before (loop closure scenario):
```python
else:
    existing = self.global_walls[lm_id]
    alpha_e = existing['alpha']
    tangent_e = [-sin(alpha_e), cos(alpha_e)]

    # Project new endpoints onto existing tangent
    proj_s = dot(start_corr, tangent_e)
    proj_e = dot(end_corr, tangent_e)

    # Extend parametric bounds (union of extents)
    existing['t_min'] = min(existing['t_min'], proj_s, proj_e)
    existing['t_max'] = max(existing['t_max'], proj_s, proj_e)
```

**Key Insight:** The global wall accumulator preserves the **first-observed** orientation (α) and extends the parametric extent to encompass all observations. This prevents jitter from repeated updates while still accumulating geometric coverage.

---

## 4. Integration with EKF State

### 4.1 Robot Pose Correction

**Method:** `_apply_pose_correction()` in `local_submap_generator_feature.py:444-477`

After ICP succeeds, the pose correction is applied to the EKF:

```python
self.slam_manager.ekf.update(
    corrected_x, corrected_y, corrected_theta,
    measurement_covariance=pose_correction['covariance'],
    measurement_type='submap_alignment'
)
```

This EKF update treats the ICP-corrected pose as a **measurement** of the robot's global position, with the Censi covariance representing measurement uncertainty.

**State Update Equations:**
```
Innovation: z = [x_icp, y_icp, θ_icp] - [x_ekf, y_ekf, θ_ekf]
Kalman Gain: K = P · H^T · (H·P·H^T + R_icp)^(-1)
State Update: x_new = x_old + K · z
Covariance Update: P_new = (I - K·H) · P_old · (I - K·H)^T + K·R_icp·K^T  (Joseph form)
```

---

### 4.2 Wall Landmark State Propagation

**Critical Design Decision:** The EKF wall landmark states (ρ, α) are **NOT directly updated** with the ICP transformation.

**Rationale:**

1. **Indirect Correction via Pose Update:**
   - Once the robot pose is corrected, subsequent wall observations will naturally update the wall landmark states through the standard EKF measurement update cycle.
   - The robot pose correction propagates through the cross-correlation terms in the state covariance matrix P.

2. **Consistency with EKF Framework:**
   - Direct manipulation of landmark states outside the EKF update equations would violate the probabilistic consistency of the state estimate.
   - The covariance matrix P must remain synchronized with the state vector through proper Kalman update equations.

3. **Separation of Concerns:**
   - **EKF State:** Maintains probabilistic consistency for real-time localization
   - **Global Walls (Stitcher):** Accumulates drift-corrected geometry for final map visualization
   - **FeatureMap:** Tracks parametric extents for local submap point cloud generation

---

### 4.3 FeatureMap Synchronization

**Method:** `_process_matched_features()` in `feature_slam_manager.py:121-174`

After each EKF update cycle, the FeatureMap is synchronized with the latest EKF state:

```python
# Sync FeatureMap wall parameters with latest EKF state
for lm_id in self.feature_map.walls.keys():
    if lm_id in self.ekf.landmarks:
        lm_idx = self.ekf.landmarks[lm_id]['state_index']
        self.feature_map.update_wall_hessian(
            lm_id,
            rho=float(self.ekf.state[lm_idx]),
            alpha=float(self.ekf.state[lm_idx + 1])
        )
```

**Endpoint Extension (Normal Operation):**
```python
if feat_idx in extension_info:
    ext = extension_info[feat_idx]
    self.feature_map.update_wall_endpoints(
        landmark_id=landmark_id,
        new_start=ext['new_start'],
        new_end=ext['new_end']
    )
```

This mechanism continuously extends wall endpoints as new observations arrive, independent of ICP corrections.

---

## 5. Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│  SUBMAP GENERATION (Every 50 Scans)                             │
└───────────────────┬─────────────────────────────────────────────┘
                    │
                    ▼
    ┌───────────────────────────────────────────┐
    │  FeatureMap.generate_point_cloud()        │
    │  - Extract wall endpoints from (ρ,α,t_min,t_max)  │
    │  - Interpolate points at 5cm spacing      │
    │  - Output: Nx3 point array (local submap) │
    └───────────────┬───────────────────────────┘
                    │
                    ▼
    ┌───────────────────────────────────────────┐
    │  SubmapStitcher.integrate_submap()        │
    │  - Voxel downsample (5cm)                 │
    │  - Run ICP vs global map                  │
    └───────────┬─────────────┬─────────────────┘
                │             │
      ┌─────────▼─────────┐   │
      │   ICP SUCCESS?    │   │
      └─────────┬─────────┘   │
                │             │
        ┌───────┴────────┐    │
        │ YES            │    │ NO (identity transform)
        ▼                │    │
┌──────────────────────┐ │    │
│ point_cloud_icp_align│ │    │
│ Returns:             │ │    │
│ - T (4x4 matrix)     │ │    │
│ - dx, dy, dθ         │ │    │
│ - Σ_pose (3x3 cov)   │ │    │
└────────┬─────────────┘ │    │
         │               │    │
         ▼               │    │
┌──────────────────────┐ │    │
│ _accumulate_walls()  │ │    │
│ For each wall:       │ │    │
│ 1. Extract (ρ,α,t_min,t_max)   │
│ 2. Reconstruct endpoints       │
│ 3. Transform: R·p + t          │
│ 4. Recompute (ρ',α',t'_min,t'_max) │
│ 5. Store in global_walls       │
└────────┬─────────────┘ │    │
         │               │    │
         └───────┬───────┘    │
                 │            │
                 ▼            ▼
    ┌────────────────────────────────┐
    │  _apply_pose_correction()      │
    │  - EKF.update(x', y', θ', Σ)   │
    │  - Corrects robot pose         │
    │  - Propagates via P matrix     │
    └────────────┬───────────────────┘
                 │
                 ▼
    ┌────────────────────────────────┐
    │  SUBSEQUENT SCAN PROCESSING    │
    │  (Normal EKF Update Cycle)     │
    ├────────────────────────────────┤
    │ 1. Extract features            │
    │ 2. Data association            │
    │ 3. EKF update (wall/corner obs)│
    │ 4. Sync FeatureMap (ρ,α)       │
    │ 5. Extend wall endpoints       │
    └────────────┬───────────────────┘
                 │
                 ▼
    ┌────────────────────────────────┐
    │ EKF Wall States Converge       │
    │ - Natural correction via pose  │
    │ - Cross-correlation in P       │
    │ - No direct state manipulation │
    └────────────────────────────────┘
```

---

## 6. Example Scenario: Loop Closure Event

### Initial State (Before Loop Closure)

**EKF State:**
```
Robot: [x=10.0, y=5.0, θ=0.5]
Wall_42: [ρ=3.2, α=1.57]  (EKF estimate with drift)
```

**FeatureMap:**
```
Wall_42: {ρ: 3.2, α: 1.57, t_min: -2.0, t_max: 2.0}
```

**Global Walls (Stitcher):**
```
Wall_42: {ρ: 3.1, α: 1.58, t_min: -2.5, t_max: 2.5}  (from submap 0)
```

---

### ICP Alignment (Submap 5)

**Detected Transformation:**
```
dx = -0.15 m
dy = 0.08 m
dθ = -0.03 rad
R = [[cos(-0.03), -sin(-0.03)],
     [sin(-0.03),  cos(-0.03)]]
  ≈ [[0.9995, 0.0300],
     [-0.0300, 0.9995]]
```

**Censi Covariance:**
```
Σ_pose = [[0.0001, 0.0000, 0.0001],
          [0.0000, 0.0001, 0.0001],
          [0.0001, 0.0001, 0.0005]]
```

---

### Wall Endpoint Transformation

**Original Endpoints (Submap 5, before correction):**
```
Wall_42 in FeatureMap:
  ρ = 3.2, α = 1.57
  tangent = [-sin(1.57), cos(1.57)] = [-1.0, 0.0]
  normal  = [cos(1.57), sin(1.57)] = [0.0, 1.0]
  line_pt = 3.2 · [0.0, 1.0] = [0.0, 3.2]

  start = [0.0, 3.2] + (-2.0)·[-1.0, 0.0] = [2.0, 3.2]
  end   = [0.0, 3.2] + (2.0)·[-1.0, 0.0]  = [-2.0, 3.2]
```

**Apply Transformation:**
```
start_corr = [[0.9995, 0.0300],  @ [2.0]  + [-0.15]
              [-0.0300, 0.9995]]    [3.2]    [0.08]
           = [1.9990 + 0.0960 - 0.15, -0.0600 + 3.1984 + 0.08]
           = [1.945, 3.218]

end_corr   = [[0.9995, 0.0300],  @ [-2.0] + [-0.15]
              [-0.0300, 0.9995]]    [3.2]    [0.08]
           = [-1.9990 + 0.0960 - 0.15, 0.0600 + 3.1984 + 0.08]
           = [-2.055, 3.338]
```

**Recompute Hessian Parameters:**
```
centroid = 0.5 · ([1.945, 3.218] + [-2.055, 3.338])
         = [-0.055, 3.278]

alpha_corr = atan2(sin(1.57 + (-0.03)), cos(1.57 + (-0.03)))
           = atan2(sin(1.54), cos(1.54))
           ≈ 1.54 rad

normal_corr = [cos(1.54), sin(1.54)] ≈ [0.0300, 0.9995]
rho_corr = dot([-0.055, 3.278], [0.0300, 0.9995])
         = -0.00165 + 3.2764
         = 3.275
```

**Parametric Extent:**
```
tangent_corr = [-sin(1.54), cos(1.54)] ≈ [-0.9995, 0.0300]
t_s = dot([1.945, 3.218], [-0.9995, 0.0300]) = -1.9440 + 0.0965 ≈ -1.85
t_e = dot([-2.055, 3.338], [-0.9995, 0.0300]) = 2.0540 + 0.1001 ≈ 2.15

t_min_corr = -1.85
t_max_corr = 2.15
```

---

### Update Global Walls

```python
# Wall_42 already exists in global_walls from submap 0
existing = global_walls[42]  # {ρ: 3.1, α: 1.58, t_min: -2.5, t_max: 2.5}

# Project corrected endpoints onto existing tangent
tangent_e = [-sin(1.58), cos(1.58)] ≈ [-1.0, 0.0]
proj_s = dot([1.945, 3.218], [-1.0, 0.0]) = -1.945
proj_e = dot([-2.055, 3.338], [-1.0, 0.0]) = 2.055

# Extend bounds
existing['t_min'] = min(-2.5, -1.945, 2.055) = -2.5  (no change)
existing['t_max'] = max(2.5, -1.945, 2.055) = 2.5    (no change)
```

In this example, the corrected wall falls within the existing extent, so the bounds don't expand. The key update is the **geometrically corrected representation** stored for this submap.

---

### Robot Pose Correction

**EKF Update:**
```
Measurement: z = [10.0 - 0.15, 5.0 + 0.08, 0.5 - 0.03]
               = [9.85, 5.08, 0.47]

Current EKF State: [10.0, 5.0, 0.5, ... wall params ...]

Innovation: ν = z - h(x) ≈ [-0.15, 0.08, -0.03]

Kalman Gain: K = P · H^T · (H·P·H^T + Σ_pose)^(-1)

Updated State:
  x_new = 10.0 + K[0,0]·(-0.15) + ... ≈ 9.92
  y_new = 5.0 + K[1,0]·(0.08) + ...   ≈ 5.06
  θ_new = 0.5 + K[2,0]·(-0.03) + ...  ≈ 0.48
```

**Wall State (Indirect Update via Cross-Correlation):**

The EKF covariance matrix P contains cross-correlation terms between the robot pose and all landmarks. When the robot pose is updated, the landmark states also shift slightly due to these correlations:

```
Wall_42 state index: 3
ρ_new = ρ_old + K[3,0]·ν[0] + K[3,1]·ν[1] + K[3,2]·ν[2]
      ≈ 3.2 + (small correction) ≈ 3.18

α_new = α_old + K[4,0]·ν[0] + K[4,1]·ν[1] + K[4,2]·ν[2]
      ≈ 1.57 + (small correction) ≈ 1.56
```

The magnitude of this correction depends on the **correlation strength** between robot pose and wall landmark in the P matrix.

---

### Subsequent Observations

In future scans (e.g., scan 251, 252, ...), Wall_42 is observed again. The standard EKF update cycle:

1. **Data Association:** Wall observation matches landmark_id=42
2. **EKF Update:** Observed (ρ_obs, α_obs) vs predicted (ρ_pred, α_pred)
3. **Correction:** Kalman gain applies innovation to both robot pose and wall parameters
4. **FeatureMap Sync:** FeatureMap receives updated (ρ, α) from EKF state

Over several observations, the EKF wall state **converges** to the corrected geometry.

---

## 7. Key Design Principles

### 7.1 Separation of Representations

| Component | Purpose | Update Mechanism |
|-----------|---------|------------------|
| **EKF State** | Probabilistic localization & mapping | Kalman update (measurements) |
| **FeatureMap** | Local submap geometry | Synchronized from EKF state |
| **Global Walls** | Final drift-corrected map | ICP transformation accumulation |

### 7.2 Probabilistic Consistency

- **EKF state updates must preserve covariance consistency** (no direct manipulation)
- **ICP correction is treated as a measurement** with proper uncertainty (Σ_pose)
- **Cross-correlation in P matrix propagates corrections** from pose to landmarks

### 7.3 Gradual Convergence vs. Instant Correction

**Trade-off:**
- **Instant Correction:** Directly modify all landmark states with ICP transformation
  - ✓ Immediate map correction
  - ✗ Violates EKF probabilistic framework
  - ✗ Covariance becomes inconsistent
  - ✗ May introduce jumps in state estimates

- **Gradual Convergence (Implemented):**
  - ✓ Maintains probabilistic consistency
  - ✓ Smooth state evolution
  - ✓ Proper uncertainty propagation
  - ✗ Requires several observations to fully converge

**System Choice:** The implemented system prioritizes **probabilistic rigor** over immediate correction. This aligns with the thesis emphasis on uncertainty quantification.

---

## 8. Limitations and Future Work

### 8.1 Current Limitations

1. **No Direct EKF Landmark Correction:**
   - Wall landmark states (ρ, α) in the EKF are not directly updated with ICP transformation
   - Convergence requires subsequent observations, which may not occur if the robot doesn't revisit the area

2. **Global Walls Not Used for Planning:**
   - The `global_walls` dictionary in SubmapStitcher accumulates corrected geometry but is not fed back into the navigation module
   - Path planning uses the EKF FeatureMap, which may lag behind the corrected global map

3. **Single ICP Correction Per Submap:**
   - Each submap is aligned only once (when created)
   - Subsequent submaps may further refine the alignment, but previous submaps are not re-optimized

### 8.2 Proposed Enhancements

#### **A. Graph-Based Backend**

Transition to a **pose graph SLAM** framework (e.g., GTSAM, g2o):
- Represent each submap as a **pose node**
- ICP alignments become **loop closure edges**
- Batch optimization corrects **all poses and landmarks** globally
- Landmark states updated via graph optimization

**Benefits:**
- Global consistency across all landmarks
- Handles multiple loop closures
- Reduces linearization errors from EKF

#### **B. Explicit Landmark Correction with Covariance Update**

Develop a principled method to directly update landmark states while maintaining consistency:

1. **Compute landmark correction from pose correction:**
   ```
   For each landmark i:
     Δl_i = f(Δx_robot, P_robot-landmark)
   ```

2. **Update covariance using composite measurement:**
   ```
   P_new = update_with_correlated_measurement(P_old, Δx, Δl, Σ_pose)
   ```

3. **Theoretical Foundation:**
   - Use **Rauch-Tung-Striebel smoother** concepts
   - Treat ICP as a **joint measurement** of pose and landmarks
   - Construct augmented measurement model

#### **C. Hybrid Correction Strategy**

- **Real-time layer:** EKF for low-latency localization
- **Background optimization:** Periodic graph SLAM optimization
- **Bidirectional sync:** Feed optimized states back to EKF

---

## 9. Conclusion

The wall endpoint update mechanism following ICP correction demonstrates a **carefully balanced design** that prioritizes probabilistic consistency over immediate geometric correction. The system employs three complementary representations:

1. **EKF State:** Maintains statistical rigor with gradual convergence through observation-driven updates
2. **FeatureMap:** Provides real-time geometric representation synchronized with EKF state
3. **Global Walls:** Accumulates drift-corrected geometry for final map visualization

While the current implementation does not directly correct EKF landmark states with ICP transformations, this design choice ensures:
- Preservation of covariance consistency
- Smooth state evolution without discontinuous jumps
- Proper uncertainty propagation through the Kalman filter framework

The ICP-based loop closure effectively corrects the robot pose, which propagates to landmark estimates through cross-correlations in the state covariance matrix. Subsequent observations complete the convergence process, ensuring the map remains both geometrically accurate and statistically consistent.

For applications requiring immediate global consistency (e.g., large-scale multi-robot SLAM), transitioning to a graph-based backend would provide a more robust solution while preserving the strengths of the current feature-based approach.

---

## References

1. **Censi, A.** (2007). "An accurate closed-form estimate of ICP's covariance." *Proceedings of the IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3167-3172.

2. **Smith, R., Self, M., & Cheeseman, P.** (1990). "Estimating uncertain spatial relationships in robotics." *Autonomous Robot Vehicles*, pp. 167-193.

3. **Thrun, S., Burgard, W., & Fox, D.** (2005). *Probabilistic Robotics*. MIT Press. Chapter 10: SLAM.

4. **Bailey, T., & Durrant-Whyte, H.** (2006). "Simultaneous localization and mapping (SLAM): Part II." *IEEE Robotics & Automation Magazine*, 13(3), 108-117.

5. **Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W.** (2010). "A tutorial on graph-based SLAM." *IEEE Intelligent Transportation Systems Magazine*, 2(4), 31-43.

---

**Report End**
