# Feature-Based SLAM with ICP Submap Stitching

## Abstract

This document provides a comprehensive overview of the production mapping stack used in this repository: **feature-based EKF-SLAM** with **ICP-based submap-to-global stitching**. It summarizes architecture, data flow, uncertainty handling, and evaluation metrics used in structured indoor environments.

## 1. Thesis Objective

### 1.1 Research Question

**How does feature-based EKF-SLAM with ICP-assisted submap stitching perform in terms of:**
- Localization accuracy
- Map quality and completeness
- Computational efficiency
- Uncertainty quantification
- Robustness to different environment structures

### 1.2 Methodology

Both approaches are implemented within the same ROS 2 framework and evaluated using:
- **Identical exploration strategy**: Frontier-based autonomous navigation
- **Same sensor configuration**: 2D LiDAR (360-degree laser scanner)
- **Same test environments**: Structured indoor spaces (simulated and real-world)
- **Consistent evaluation metrics**: Pose error, map accuracy, computation time, uncertainty metrics

## 2. System Architecture

The runtime system now uses a single mapping pipeline:

### 2.1 Feature-Based Mapping (Runtime)

**Approach:** Sparse landmark tracking with EKF-SLAM

**Core Components:**
- Feature extraction (walls and corners)
- Data association with Mahalanobis distance gating
- Full EKF-SLAM (joint robot-landmark state)
- Fisher Information Matrix uncertainty quantification
- Feature map representation with geometric primitives

**See:** `docs/methodology_feature_mapping.md` for complete details.

## 3. Common Framework

### 3.1 Shared Components

Core system components:

| Component | Purpose | Implementation |
|-----------|---------|----------------|
| **Frontier Explorer** | Autonomous navigation | `autonomous_exploration` package |
| **Transform Management** | Coordinate frame handling | `transform_utils.py` |
| **Point Cloud Processing** | Scan conversion and filtering | `mapping_utils.py` |
| **Data Logging** | CSV output for evaluation | `evaluation_utils.py` |
| **Ground Truth Interface** | Simulation pose subscription | `local_submap_generator_feature.py` |

### 3.2 ROS 2 Integration

**Main Node:** `LocalSubmapGeneratorFeature`

**Runtime Selection:**
- Feature mode is the only supported mapping runtime mode.

**Common Subscriptions:**
- `/tb3_1/scan` (LaserScan) — LiDAR measurements
- `/tb3_1/odom` (Odometry) — Wheel encoder estimates
- `/tb3_1/ground_truth_pose` (PoseStamped) — Simulation ground truth

**Common Publications:**
- `/tb3_1/ekf_pose` (PoseStamped) — Corrected robot pose
- `/tb3_1/ekf_path` (Path) — Trajectory history
- `/tb3_1/global_map` (PointCloud2) — Global map

**Additional Publications:**
- `/tb3_1/scan_features` (MarkerArray) — Detected landmarks
- `/tb3_1/current_submap` (PointCloud2) — Current submap

**TF Broadcast:**
- `map → tb3_1/odom` — Corrected transform (feature pipeline)

## 4. Coordinate Frames

Both systems use the standard ROS REP-105 coordinate frame convention:

```
map (Fixed World Frame)
 │
 └─> odom (Odometry Frame, drift-prone)
      │
      └─> base_footprint (Robot Frame)
```

### 4.1 Frame Definitions

**`map` Frame:**
- Fixed global reference frame
- Origin defined at system initialization
- Corrected by SLAM (EKF state estimate)

**`odom` Frame:**
- Locally consistent odometry frame
- Drifts over time due to wheel slip
- Updated by Gazebo odometry

**`base_footprint` Frame:**
- Robot-centered frame
- LiDAR measurements expressed here initially

### 4.2 Transform Chain

The EKF publishes `map → odom` transform:

$$
^{map}\mathbf{T}_{base} = \, ^{map}\mathbf{T}_{odom} \times \, ^{odom}\mathbf{T}_{base}
$$

## 5. Evaluation Framework

### 5.1 Evaluation Metrics

| Metric | Description | Computation |
|--------|-------------|-------------|
| **Absolute Trajectory Error (ATE)** | Pose error vs ground truth | RMSE of position and orientation |
| **Map Accuracy** | Point cloud alignment error | Cloud-to-cloud distance |
| **Computation Time** | Processing latency per scan | Wall-clock timing |
| **Uncertainty Calibration** | Predicted vs actual error | Chi-squared test, NEES |
| **Landmark Count** | Map sparsity (feature mode) | Number of landmarks in state |
| **Memory Usage** | State vector size | Bytes allocated |

### 5.2 Data Collection

**Logged Data:**

| File | Content | Purpose |
|------|---------|---------|
| `ekf_vs_groundtruth.csv` | Pose errors over time | Accuracy evaluation |
| `submap_confidence.csv` | Uncertainty metrics per submap | Uncertainty analysis |
| `global_map.pcd` | Final point cloud map | Visual comparison |
| `computation_time.csv` | Per-scan processing time | Efficiency evaluation |

**Feature Pipeline Additional Data:**
- `landmark_count.csv` — Number of landmarks over time
- `feature_matches.csv` — Data association statistics

## 6. Mapping Architecture

### 6.1 Feature-Based Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                Feature-Based SLAM Pipeline                   │
│                                                               │
│  ┌──────────────┐      ┌──────────────┐                     │
│  │   LiDAR      │────>│   Feature     │                     │
│  │   Scans      │      │  Extraction   │                     │
│  └──────────────┘      └──────────────┘                     │
│                               │                               │
│                               ↓                               │
│                    ┌──────────────────┐                      │
│                    │      Data        │                      │
│                    │   Association    │                      │
│                    └──────────────────┘                      │
│                               │                               │
│                               ↓                               │
│                    ┌──────────────────┐                      │
│                    │   Full EKF-SLAM  │                      │
│                    │ (Robot+Landmarks)│                      │
│                    └──────────────────┘                      │
│                               │                               │
│                               ↓                               │
│                    ┌──────────────────┐                      │
│                    │   Feature Map    │                      │
│                    │   Management     │                      │
│                    └──────────────────┘                      │
│                               │                               │
│                               ↓                               │
│                    ┌──────────────────┐                      │
│                    │  Point Cloud     │────> Sparse Map      │
│                    │  Generation      │                      │
│                    └──────────────────┘                      │
└─────────────────────────────────────────────────────────────┘
```

**Key Classes:**
- `FeatureSLAMManager` (encapsulates all feature SLAM logic)
- `LandmarkFeatureExtractor` in `landmark_features.py`
- `associate_landmarks()` in `data_association.py`
- `FeatureMap` in `feature_map.py`

## 7. Performance Characteristics

### 7.1 Feature-Based Approach

| Aspect | Characteristic | Notes |
|--------|---------------|-------|
| **Representation** | Sparse landmarks | Walls (ρ, α) + Corners (x, y) |
| **Computational Complexity** | O(N + L²) per scan | N = features, L = landmarks |
| **Memory** | Grows quadratically | O(L²) covariance matrix |
| **Accuracy** | Consistent long-term | Explicit data association prevents drift |
| **Uncertainty** | Full covariance | Robot-landmark correlations tracked |
| **Robustness** | Requires features | Struggles in feature-poor environments |
| **Map Quality** | Geometric abstraction | Compact, semantic representation |

## 8. Exploration Strategy

### 8.1 Frontier-Based Exploration

The mapping pipeline uses the same frontier-based autonomous navigation stack:

**Algorithm:**
1. Detect frontiers (boundaries between known free space and unknown space)
2. Rank frontiers by information gain and distance
3. Select best frontier as navigation goal
4. Use ROS 2 Nav2 for path planning and execution
5. Repeat until no frontiers remain

**Implementation:**
- `autonomous_exploration` ROS 2 package
- Occupancy grid generation from laser scans
- Frontier detection with morphological operations
- Nav2 integration for global/local planning

### 8.2 Controlled Comparison

**Why Same Exploration?**
- Ensures both methods observe the same environment regions
- Eliminates exploration bias from results
- Allows fair comparison of SLAM performance independent of path choice

**Logging:**
- Exploration paths recorded with timestamps
- Frontier selection logged for reproducibility
- Ground truth trajectory stored for error analysis

## 9. Thesis Structure

### 9.1 Documentation Organization

| Document | Purpose |
|----------|---------|
| **00_system_overview.md** (this file) | High-level system overview |
| **literature_review.md** | Academic background and related work |
| **methodology_feature_mapping.md** | Complete feature-based approach documentation |
| **01_ekf_slam_theory.md** | Mathematical derivations for EKF-SLAM |
| **02_landmark_features.md** | Feature extraction algorithms |
| **03_data_association.md** | Data association theory |
| **05_submap_management.md** | Submap creation and stitching |
| **06_uncertainty_quantification.md** | Uncertainty metrics and theory |
| **07_coordinate_frames.md** | Transform conventions |

### 9.2 Experimental Workflow

```
1. Setup
   ├─> Configure simulation environment (Gazebo)
   ├─> Launch exploration system
   └─> Start feature-based mapping pipeline

2. Data Collection
   ├─> Run same exploration scenario
   ├─> Log pose errors, landmarks, timing
   └─> Save final feature map + point cloud

3. Analysis
   ├─> Compute ATE
   ├─> Compare map quality (cloud-to-cloud)
   ├─> Analyze uncertainty calibration
   ├─> Compare computational efficiency
   └─> Statistical significance testing

4. Visualization
   ├─> Plot trajectory errors
   ├─> Render final maps
   ├─> Generate performance figures
   └─> Create thesis figures
```

## 10. Pipeline Summary

### 10.1 State Representation

**Feature-Based EKF-SLAM:**
$$
\mathbf{x} = \begin{bmatrix}
x_r, y_r, \theta_r \\
\rho_1, \alpha_1 \\
\vdots \\
x_{c_1}, y_{c_1} \\
\vdots
\end{bmatrix} \quad \text{(robot + landmarks)}
$$

### 10.2 Measurement Update

**Feature-Based:**
- Measurement: Landmark observations (ρ, α) or (x, y)
- Observation model: Geometric projection from robot to landmark
- Update frequency: Every landmark re-observation
- Additional pose correction: submap ICP correction during submap stitching

### 10.3 Uncertainty Source

**Feature-Based:**
- Fisher Information Matrix from feature geometry
- Propagated through EKF covariance
- Correlated across landmarks
- ICP stitching covariance is used as pose-correction uncertainty

## 11. Expected Outcomes

### 11.1 Hypotheses

**H1: Accuracy**
- Feature-based: Consistent long-term accuracy with robust data association
- ICP submap stitching: Improves global consistency across submaps

**H2: Computational Efficiency**
- Feature extraction/association dominates per-scan cost
- Landmark count drives EKF update growth over time

**H3: Map Quality**
- Geometrically interpretable wall/corner map with interpolated point cloud output

**H4: Uncertainty Calibration**
- Predicted EKF uncertainty should track measured trajectory error
- Submap confidence should improve with repeated landmark observations

### 11.2 Contributions

This comparative study provides:
- ✅ Rigorous controlled evaluation under identical exploration
- ✅ Quantitative evaluation across multiple metrics
- ✅ Open-source implementation with reproducible results
- ✅ Comprehensive documentation of the deployed mapping pipeline
- ✅ Uncertainty-aware mapping with quantitative confidence

## 12. Implementation Status

### 12.1 Completed Features

**Feature Pipeline:**
- ✅ ROS 2 integration with TF2
- ✅ Ground truth comparison logging
- ✅ Frontier-based exploration interface
- ✅ Point cloud visualization
- ✅ Wall and corner extraction
- ✅ Data association with segment overlap validation
- ✅ Full EKF-SLAM with Joseph form update
- ✅ Feature map with wall extension
- ✅ Fisher Information covariance
- ✅ Landmark pruning
- ✅ ICP-based submap stitching and pose correction

### 12.2 Recent Improvements

**Wall Matching Fix (February 2026):**
- Added segment overlap validation to prevent merging non-overlapping collinear walls
- 0.5m gap tolerance for localization errors
- Improved data association robustness

**Code Cleanup:**
- Removed dead code (`get_covariance`, `get_statistics`)
- Fixed redundant imports
- Verified build success

**Documentation Overhaul:**
- Created comprehensive literature review
- Separate methodology documents for both approaches
- Updated system overview (this file) for comparative study framing

## 13. References

**SLAM Foundations:**
1. Durrant-Whyte, H., & Bailey, T. (2006). "Simultaneous Localization and Mapping." *IEEE Robotics & Automation Magazine*.
2. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

**ICP-Based Methods:**
3. Besl, P. J., & McKay, N. D. (1992). "A Method for Registration of 3-D Shapes." *IEEE TPAMI*.
4. Censi, A. (2007). "An Accurate Closed-Form Estimate of ICP's Covariance." *ICRA 2007*.

**Feature-Based Methods:**
5. Dissanayake, G., et al. (2001). "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem." *IEEE Trans. Robotics and Automation*.
6. Nguyen, V., et al. (2005). "A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder." *IROS 2005*.

**Exploration:**
7. Yamauchi, B. (1997). "A Frontier-Based Approach for Autonomous Exploration." *CIRA*.
8. Burgard, W., et al. (2005). "Coordinated Multi-Robot Exploration." *IEEE Trans. Robotics*.

**See:** `docs/literature_review.md` for complete bibliography (50+ references)

---

**Next Steps:**
1. Review methodology document: `methodology_feature_mapping.md`
2. Run experiments across identical exploration scenarios
3. Analyze results using evaluation scripts
4. Generate thesis figures and statistical comparisons
