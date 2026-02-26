# Thesis Corrections Applied to thesis_ds.tex

**Date:** February 26, 2026
**File:** `/home/piyush/thesis_ws/thesis_ds.tex`

---

## Summary of Changes

All corrections have been successfully applied to the simulation section of the thesis. Below is a detailed list of changes made:

---

## ✅ Section 1: Submap Alignment Description (Lines 2601-2608)

### CRITICAL FIX: SVD → ICP

**Original (INCORRECT):**
> Alignment granularity: Submaps contain enough geometric structure for reliable SVD-based registration
>
> Submap alignment uses Singular Value Decomposition (SVD) rather than Iterative Closest Point (ICP). The SVD approach computes the optimal rigid transformation directly from corresponding point sets, providing a closed-form solution that is faster and more deterministic than iterative methods.

**Corrected to:**
> Alignment granularity: Submaps contain enough geometric structure for reliable ICP-based registration
>
> Submap alignment uses Iterative Closest Point (ICP) for point cloud registration. However, ICP corrections are used **only for point cloud stitching and visualization**, not for EKF pose updates. The EKF remains the single source of truth for robot localization, with pose corrections coming exclusively from feature-based landmark observations. This design prevents catastrophic failures from incorrect ICP convergence while maintaining high-quality map visualization. The ICP implementation uses Open3D's GPU-accelerated tensor-based algorithm with a fixed 3cm nearest-neighbor correspondence threshold.

**Rationale:**
- The system uses ICP (submap_stitcher.py:47-172), not SVD
- Critically, ICP results are NOT fed to EKF (removed _apply_pose_correction())
- This architecture prevents map corruption from bad ICP alignments

---

## ✅ Section 2: Feature Extraction Parameters Table (Lines 2535-2555)

### CRITICAL FIX: RANSAC → Incremental Line Growing

**Changed:**

| Parameter | Old Value | New Value | Status |
|-----------|-----------|-----------|--------|
| `ransac_threshold` | 0.05 m | **REMOVED** | ❌ Not used |
| `min_line_points` | 10 | **8** | ✅ Fixed |
| `min_line_length` | 0.3 m | **0.5 m** | ✅ Fixed |
| `max_line_gap` → `max_gap` | 0.2 m | **0.5 m** | ✅ Fixed |

**Added New Parameters:**
- `grow_residual_threshold`: 0.05 m
- `merge_angle_tolerance`: 0.35 rad (20°)
- `merge_rho_tolerance`: 0.15 m
- `lidar_noise_sigma`: 0.01 m

**Updated Explanation:**
> The feature extraction uses an **incremental line growing algorithm with adjacent merging**, not RANSAC. The algorithm starts with seed points and incrementally grows line segments by adding nearby points that satisfy the residual threshold (0.05m). Adjacent collinear segments are then merged if they satisfy both angle tolerance (0.35 rad ≈ 20°) and perpendicular distance tolerance (0.15m). This approach is more efficient and robust than RANSAC for structured indoor environments, as it exploits the natural ordering of LiDAR scan points.

**Source:** `feature_slam_manager.py:25-33`, `landmark_features.py`

---

## ✅ Section 3: EKF-SLAM Parameters Table (Lines 2561-2586)

### Process Noise Corrections

**Changed:**

| Parameter | Old Value | New Value | Rationale |
|-----------|-----------|-----------|-----------|
| σ_dist | 0.01 m | **0.00283 m** | Correct std dev from variance 8×10⁻⁶ m² |
| σ_rot | 0.02 rad | **0.00447 rad** | Correct std dev from variance 2×10⁻⁵ rad² (≈0.26°) |

### Measurement Noise - Major Conceptual Fix

**Original (INCORRECT):**
```
σ_ρ = 0.1 m (Distance-to-line measurement uncertainty)
σ_α = 0.1 rad (Line angle measurement uncertainty)
```

**Corrected to:**
```
lidar_noise_sigma: 0.01 m (Base LiDAR sensor noise)
Wall R_k: Computed (Covariance via Jacobian propagation)
```

**Added Explanation:**
> Measurement covariances are **computed dynamically** for each feature using Jacobian propagation from LiDAR noise. For a wall with n points, the covariance is R_k = σ²_lidar · (Σ J_i^T J_i)^{-1}, where J_i is the Jacobian of the Hessian normal form (ρ, α) with respect to point measurements. This adapts uncertainty based on line quality: longer lines with more points have lower covariance.
>
> The low process noise values (σ_dist = 2.83 mm, σ_rot = 0.26°) reflect high-quality simulation odometry. In real-world deployments, these values would typically be 5-10× larger to account for wheel slip, uneven terrain, and encoder noise.

### Added New Data Association Parameters

- `max_gap_ext`: 1.0 m (Maximum spatial gap for wall endpoint extension)
- `landmark_timeout_scans`: 25 (Scans before pruning unseen landmarks)

**Source:** `ekf_predict.py:15-17`, `landmark_features.py:323-348`

---

## ✅ Section 4: Navigation Parameters Table (Lines 2621-2641)

**Changed:**

| Parameter | Old Value | New Value | Status |
|-----------|-----------|-----------|--------|
| `max_linear_velocity` | 0.26 m/s | **0.20 m/s** | ✅ Fixed |
| `max_angular_velocity` | 1.82 rad/s | **1.0 rad/s** | ✅ Fixed |
| `emergency_distance` | 0.5 m | **0.4 m** | ✅ Fixed |
| `rotation_speed` | 0.3 rad/s | **REMOVED** | ❌ Not used |

**Added New Parameter:**
- `obstacle_avoidance_scale`: 0.2 (Velocity reduction factor when obstacle detected)

**Updated Rationale:**
> The emergency distance of 0.4m provides a safety margin given the robot's maximum speed (0.20 m/s). At maximum speed, with typical reaction time (∼100ms) and braking distance, the robot can safely detect and respond to obstacles. When an obstacle is detected within the emergency distance, both linear and angular velocities are reduced to 20% of their commanded values, providing smooth deceleration while replanning the path.

**Source:** `simple_navigation.py:47,90-91,395-402`

---

## Verification Summary

✅ **Total Corrections Made:** 13 parameter errors + 2 major conceptual errors
✅ **All values verified against source code**
✅ **Proper explanations added for complex parameters**
✅ **LaTeX syntax verified**

---

## Files Modified

1. `/home/piyush/thesis_ws/thesis_ds.tex` - Simulation section updated

---

## Remaining Tasks (Optional)

Consider adding:
1. ICP-specific parameters table (max_iterations=50, fitness_threshold=0.3, rmse_threshold=0.05m)
2. Discussion of why ICP is NOT used for EKF updates
3. Corner feature covariance computation (similar to wall features)

---

## Critical Takeaways for Thesis Defense

1. **ICP is for visualization only** - Not part of the SLAM localization pipeline
2. **Feature extraction uses incremental growing, NOT RANSAC** - More efficient for structured environments
3. **Measurement noise is adaptive** - Computed per feature, not fixed values
4. **Process noise is very low** - Reflects simulation quality, not real-world conditions

---

**All corrections verified and applied successfully! ✅**
