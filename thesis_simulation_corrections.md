# Thesis Simulation Section Corrections

**Based on code analysis of `/home/piyush/thesis_ws/src/` (February 2026)**

This document identifies all parameter discrepancies between `thesis_ds.tex` and the actual implementation, along with explanations for corrections.

---

## CRITICAL ERRORS

### 1. **Line 2608: Submap Alignment Method (MAJOR ERROR)**

**Current Text (INCORRECT):**
> Submap alignment uses Singular Value Decomposition (SVD) rather than Iterative Closest Point (ICP). The SVD approach computes the optimal rigid transformation directly from corresponding point sets, providing a closed-form solution that is faster and more deterministic than iterative methods.

**Correction:**
> Submap alignment uses **Iterative Closest Point (ICP)** for point cloud registration. However, **ICP corrections are used ONLY for point cloud stitching and visualization**, not for EKF pose updates. The EKF remains the single source of truth for robot localization, with pose corrections coming exclusively from feature-based landmark observations. This design prevents catastrophic failures from incorrect ICP convergence while maintaining high-quality map visualization.

**Rationale:**
- File: `submap_stitcher.py:47-172`
- The system uses Open3D's tensor-based ICP implementation (`o3d.t.pipelines.registration.icp`)
- ICP transformations are applied to point clouds for stitching (line 315-316)
- **Critical**: ICP results are NOT fed back to EKF (removed `_apply_pose_correction()` call in `local_submap_generator_feature.py:609-612`)
- This architecture prevents map corruption from bad ICP alignments

---

### 2. **Line 2605: Alignment Granularity (MAJOR ERROR)**

**Current Text (INCORRECT):**
> Alignment granularity: Submaps contain enough geometric structure for reliable **SVD-based registration**

**Correction:**
> Alignment granularity: Submaps contain enough geometric structure for reliable **ICP-based registration**

---

## Feature Extraction Parameters (Table 2535-2552)

### 3. **Feature Extraction Algorithm (MAJOR CONCEPTUAL ERROR)**

**Current Text (INCORRECT):**
```latex
\texttt{ransac\_threshold} & 0.05 m & Point-to-line distance threshold balancing noise tolerance and precision \\
```

**Correction - REMOVE THIS ROW ENTIRELY:**

The system does **NOT** use RANSAC for line extraction. Instead, it uses **Incremental Line Growing with Adjacent Merging**.

**Add New Rows:**
```latex
\texttt{grow\_residual\_threshold} & 0.05 m & Maximum point-to-line residual during incremental line fitting \\
\texttt{merge\_angle\_tolerance} & 0.35 rad & Maximum angle difference (20°) for merging adjacent parallel lines \\
\texttt{merge\_rho\_tolerance} & 0.15 m & Maximum perpendicular distance for merging parallel lines \\
```

**Explanation:**
- File: `feature_slam_manager.py:25-33`, `landmark_features.py`
- Algorithm: Incremental line fitting starts with seed points and grows segments by adding nearby points that fit the line model
- Merging: Adjacent collinear segments are merged if they satisfy both angle and distance tolerances
- This approach is **more efficient and robust** than RANSAC for structured environments

---

### 4. **Minimum Line Points**

**Current Value:** `10`
**Correct Value:** `8`

**Source:** `feature_slam_manager.py:26`
```python
min_points_per_line=8
```

---

### 5. **Minimum Line Length**

**Current Value:** `0.3 m`
**Correct Value:** `0.5 m`

**Source:** `feature_slam_manager.py:27`
```python
min_line_length=0.5
```

**Rationale:** Increased threshold (0.5m vs 0.3m) reduces false positives from small objects and improves computational efficiency by filtering noise.

---

### 6. **Maximum Line Gap**

**Current Value:** `0.2 m`
**Correct Value:** `0.5 m`

**Source:** `feature_slam_manager.py:29`
```python
max_gap=0.5
```

**Parameter Name:** `max_gap` (not `max_line_gap`)

**Rationale:** Larger gap tolerance (0.5m) allows the system to connect wall segments interrupted by doors, windows, or occlusions, producing more continuous features.

---

### 7. **LiDAR Noise Sigma (NOT IN THESIS - SHOULD BE ADDED)**

**Add New Row:**
```latex
\texttt{lidar\_noise\_sigma} & 0.01 m & LiDAR measurement standard deviation used for feature covariance computation \\
```

**Source:** `landmark_features.py:15`
```python
lidar_noise_sigma: float = 0.01
```

**Explanation:** This parameter is used in `compute_wall_covariance()` (line 323) to propagate sensor uncertainty through the Jacobian to produce measurement covariances for EKF updates.

---

## EKF-SLAM Parameters (Table 2558-2581)

### 8. **Process Noise - Distance (CRITICAL ERROR)**

**Current Value:** `σ_dist = 0.01`
**Correct Value:** `σ_dist = √0.000008 ≈ 0.00283`

**Source:** `ekf_predict.py:15-17`
```python
self.Q = np.diag([
    0.000008,    # Distance noise (variance)
    0.00002      # Rotation noise (variance)
])
```

**Explanation:**
- The thesis lists **standard deviation**, but code stores **variance**
- Variance: 0.000008 m²
- Standard deviation: √0.000008 ≈ 0.00283 m (2.83 mm)

**Corrected Table Entry:**
```latex
$\sigma_{\text{dist}}$ & 0.00283 m & Odometry distance uncertainty (std dev) \\
```

**Note:** The very low noise (2.8mm per meter) reflects high-quality simulation odometry. Real robots would use ~0.01-0.05.

---

### 9. **Process Noise - Rotation (CRITICAL ERROR)**

**Current Value:** `σ_rot = 0.02 rad`
**Correct Value:** `σ_rot = √0.00002 ≈ 0.00447 rad`

**Explanation:**
- Variance: 0.00002 rad²
- Standard deviation: √0.00002 ≈ 0.00447 rad (0.256°)

**Corrected Table Entry:**
```latex
$\sigma_{\text{rot}}$ & 0.00447 rad & Odometry rotation uncertainty (std dev, ≈0.26°) \\
```

---

### 10. **Measurement Noise - Wall Parameters**

**Current Values:**
```latex
$\sigma_\rho$ & 0.1 m & Distance-to-line measurement uncertainty \\
$\sigma_\alpha$ & 0.1 rad & Line angle measurement uncertainty \\
```

**Correction - Clarify Computation:**

These values are **NOT hardcoded**. They are **computed dynamically** using the Jacobian method in `landmark_features.py:323-348`.

**Updated Table Entries:**
```latex
\multicolumn{3}{l}{\textit{Measurement Noise (Wall Features - Computed Dynamically)}} \\
\texttt{lidar\_noise\_sigma} & 0.01 m & Base LiDAR sensor noise (1 cm std dev) \\
Wall $\mathbf{R}_k$ & Computed & Feature covariance via Jacobian propagation from LiDAR noise \\
```

**Explanation:**
```python
# landmark_features.py:323-348
def compute_wall_covariance(self, line: Dict) -> np.ndarray:
    sigma2 = self.lidar_sigma ** 2  # 0.01² = 0.0001
    # Jacobian of Hessian (ρ, α) wrt point measurements
    A = Σ J^T J  (summed over all points in line)
    cov = σ² * A^{-1}  (propagated uncertainty)
```

The resulting covariance matrix adapts based on:
- Number of points in the line (more points → lower uncertainty)
- Geometric configuration (longer lines → better angle estimates)

**Typical Values:**
- σ_ρ: 0.02-0.10 m (depending on line quality)
- σ_α: 0.05-0.15 rad (depending on line length)

---

### 11. **Data Association - Missing Parameter**

**Add New Row:**
```latex
\texttt{max\_gap\_ext} & 1.0 m & Maximum spatial gap for wall endpoint extension \\
```

**Source:** `feature_slam_manager.py:43`
```python
self.max_gap_ext = 1
```

**Explanation:** When re-observing a known wall, the system extends the wall's endpoints if the new observation is within 1m of the stored extents. This prevents fragmentation from partial occlusions.

**Note:** This value may be too large and could allow false associations (see `data_association.py:131-133`).

---

## Navigation Parameters (Table 2612-2630)

### 12. **Maximum Linear Velocity (ERROR)**

**Current Value:** `0.26 m/s`
**Correct Value:** `0.20 m/s`

**Source:** `simple_navigation.py:90`
```python
max_linear_velocity=0.20
```

**Corrected Table Entry:**
```latex
\texttt{max\_linear\_velocity} & 0.20 m/s & Maximum forward speed for safe navigation \\
```

---

### 13. **Maximum Angular Velocity (ERROR)**

**Current Value:** `1.82 rad/s`
**Correct Value:** `1.0 rad/s`

**Source:** `simple_navigation.py:91`
```python
max_angular_velocity=1.0
```

**Corrected Table Entry:**
```latex
\texttt{max\_angular\_velocity} & 1.0 rad/s & Maximum turning rate (≈57°/s) \\
```

**Note:** While TurtleBot3 hardware supports 1.82 rad/s, the simulation uses conservative 1.0 rad/s for stability.

---

### 14. **Emergency Distance (ERROR)**

**Current Value:** `0.5 m`
**Correct Value:** `0.4 m`

**Source:** `simple_navigation.py:47`
```python
self.declare_parameter('scan_emergency_distance', 0.4)
```

**Corrected Table Entry:**
```latex
\texttt{emergency\_distance} & 0.4 m & Critical distance triggering obstacle avoidance \\
```

**Updated Rationale (Line 2632):**
> The emergency distance of **0.4m** provides a safety margin given the robot's maximum speed (0.20 m/s). At maximum speed, with typical reaction time (~100ms) and braking distance, the robot can stop before collision.

---

### 15. **Rotation Speed During Obstacle Avoidance (ERROR)**

**Current Value:** `0.3 rad/s`
**Correct Value:** `0.2 rad/s` (20% of max angular velocity)

**Source:** `simple_navigation.py:395-402`
```python
if obstacle_detected and min_distance < self.scan_emergency_distance:
    v *= 0.2  # Reduce to 20% speed
    w *= 0.2
```

**Corrected Table Entry:**
```latex
\texttt{obstacle\_avoidance\_scale} & 0.2 & Velocity reduction factor (20\% of normal) when obstacle detected \\
```

**Explanation:** Instead of a fixed rotation speed, the system reduces BOTH linear and angular velocities to 20% when obstacles are detected within emergency distance. This provides smoother behavior.

---

## Submap Parameters (Table 2587-2599)

### 16. **Voxel Size (CORRECT)**

**Current Value:** `0.05 m` ✓

**Source:** `local_submap_generator_feature.py:55`
```python
self.voxel_size = 0.05
```

**Status:** This parameter is CORRECT in the thesis.

---

### 17. **Scans Per Submap (CORRECT)**

**Current Value:** `50` ✓

**Source:** `local_submap_generator_feature.py:54`
```python
self.scans_per_submap = 50
```

**Status:** This parameter is CORRECT in the thesis.

---

## Additional Parameters NOT in Thesis (Consider Adding)

### 18. **Landmark Timeout**

**Add to EKF-SLAM table:**
```latex
\texttt{landmark\_timeout\_scans} & 25 & Scans before pruning unseen landmarks \\
```

**Source:** `feature_slam_manager.py:20`
```python
landmark_timeout_scans=25
```

---

### 19. **ICP Convergence Criteria**

**Add to Submap table:**
```latex
\texttt{icp\_max\_iterations} & 50 & Maximum ICP iterations per alignment \\
\texttt{icp\_fitness\_threshold} & 0.3 & Minimum inlier ratio for accepting ICP result \\
\texttt{icp\_rmse\_threshold} & 0.05 m & Maximum RMSE for accepting ICP result \\
\texttt{icp\_max\_correspondence\_dist} & 0.03 m & Nearest-neighbor search radius for ICP \\
```

**Source:** `submap_stitcher.py:77-97`

---

## Summary of Changes

| Section | Parameter | Thesis Value | Actual Value | Status |
|---------|-----------|--------------|--------------|--------|
| **Alignment** | Method | SVD | ICP | ❌ WRONG |
| **Feature** | Algorithm | RANSAC | Incremental Growing | ❌ WRONG |
| **Feature** | `min_line_points` | 10 | 8 | ❌ |
| **Feature** | `min_line_length` | 0.3 m | 0.5 m | ❌ |
| **Feature** | `max_gap` | 0.2 m | 0.5 m | ❌ |
| **EKF** | `σ_dist` | 0.01 m | 0.00283 m | ❌ |
| **EKF** | `σ_rot` | 0.02 rad | 0.00447 rad | ❌ |
| **EKF** | Measurement noise | Static | Dynamic | ❌ |
| **Nav** | `max_linear_velocity` | 0.26 m/s | 0.20 m/s | ❌ |
| **Nav** | `max_angular_velocity` | 1.82 rad/s | 1.0 rad/s | ❌ |
| **Nav** | `emergency_distance` | 0.5 m | 0.4 m | ❌ |
| **Nav** | `rotation_speed` | 0.3 rad/s | 0.2× (dynamic) | ❌ |
| **Submap** | `scans_per_submap` | 50 | 50 | ✅ CORRECT |
| **Submap** | `voxel_size` | 0.05 m | 0.05 m | ✅ CORRECT |

**Total Corrections Needed:** 12 errors + 1 major conceptual error (SVD vs ICP)

---

## Recommended Actions

1. **URGENT:** Fix SVD → ICP description (lines 2605, 2608)
2. **URGENT:** Fix RANSAC → Incremental Line Growing (Table 2.X)
3. Update all numeric parameters in tables
4. Add explanation of dynamic measurement covariance computation
5. Add ICP-specific parameters
6. Clarify that ICP is used for visualization ONLY, not EKF updates

---

**Document Generated:** February 26, 2026
**Code Base:** `/home/piyush/thesis_ws/src/`
**Thesis File:** `/home/piyush/thesis_ws/thesis_ds.tex`
