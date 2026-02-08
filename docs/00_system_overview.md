# Hybrid ICP-Landmark SLAM System: Overview

## Abstract

This document provides a comprehensive overview of the hybrid SLAM (Simultaneous Localization and Mapping) system implemented for autonomous mobile robots. The system combines **landmark-based EKF-SLAM** with **ICP (Iterative Closest Point)** scan matching to achieve robust localization and uncertainty-aware mapping. This hybrid approach leverages the complementary strengths of both paradigms: sparse, feature-based representation for long-term consistency and dense point cloud matching for short-term accuracy.

## 1. System Architecture

### 1.1 High-Level Design

```
┌─────────────────────────────────────────────────────────────────┐
│                     Local Submap Generator                       │
│                                                                   │
│  ┌──────────────┐      ┌──────────────┐      ┌──────────────┐  │
│  │   LiDAR      │────>│   Feature     │────>│     Data      │  │
│  │   Scans      │      │  Extraction   │      │ Association   │  │
│  └──────────────┘      └──────────────┘      └──────────────┘  │
│         │                                              │          │
│         │                                              ↓          │
│         │                                   ┌──────────────────┐ │
│         │                                   │   EKF-SLAM       │ │
│         │                                   │   State Update   │ │
│         │                                   └──────────────────┘ │
│         ↓                                              │          │
│  ┌──────────────┐                                     │          │
│  │     ICP      │                                     │          │
│  │  Scan-to-Map │                                     │          │
│  │   Matching   │                                     │          │
│  └──────────────┘                                     │          │
│         │                                              │          │
│         └──────────────────┬───────────────────────────┘          │
│                            ↓                                       │
│                  ┌──────────────────┐                            │
│                  │  Robot Pose      │                            │
│                  │  Estimation      │                            │
│                  └──────────────────┘                            │
│                            │                                       │
│                            ↓                                       │
│                  ┌──────────────────┐                            │
│                  │  Submap          │                            │
│                  │  Creation        │                            │
│                  └──────────────────┘                            │
│                            │                                       │
│                            ↓                                       │
│                  ┌──────────────────┐                            │
│                  │  Submap          │────> Global Map            │
│                  │  Stitching       │                            │
│                  └──────────────────┘                            │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Core Components

| Component | Purpose | Implementation File |
|-----------|---------|-------------------|
| **EKF-SLAM** | Probabilistic state estimation with landmarks | `ekf_slam.py` |
| **Feature Extraction** | Extract walls and corners from LiDAR scans | `landmark_features.py` |
| **Data Association** | Match observations to existing landmarks | `data_association.py` |
| **ICP Matching** | Dense point cloud alignment | `mapping_utils.py` |
| **Submap Stitching** | Global map construction | `submap_stitcher.py` |
| **Uncertainty Tracking** | Confidence quantification | `evaluation_utils.py` |

## 2. Mathematical Framework

### 2.1 State Representation

The system maintains a joint state vector containing the robot pose and all landmark positions:

$$
\mathbf{x} = \begin{bmatrix}
\mathbf{x}_r \\
\mathbf{x}_{l_1} \\
\mathbf{x}_{l_2} \\
\vdots \\
\mathbf{x}_{l_N}
\end{bmatrix}
$$

Where:
- $\mathbf{x}_r = [x, y, \theta]^T$ is the robot pose (position and orientation)
- $\mathbf{x}_{l_i}$ is the $i$-th landmark representation
  - For walls: $\mathbf{x}_{l_i} = [\rho_i, \alpha_i]^T$ (Hessian normal form)
  - For corners: $\mathbf{x}_{l_i} = [x_i, y_i]^T$ (Cartesian coordinates)

### 2.2 Covariance Matrix

The system maintains a full covariance matrix $\mathbf{P}$ that captures both:
1. **Uncertainty** in robot pose and landmark positions (diagonal blocks)
2. **Correlation** between robot and landmarks (off-diagonal blocks)

$$
\mathbf{P} = \begin{bmatrix}
\mathbf{P}_{rr} & \mathbf{P}_{rl_1} & \mathbf{P}_{rl_2} & \cdots \\
\mathbf{P}_{l_1r} & \mathbf{P}_{l_1l_1} & \mathbf{P}_{l_1l_2} & \cdots \\
\mathbf{P}_{l_2r} & \mathbf{P}_{l_2l_1} & \mathbf{P}_{l_2l_2} & \cdots \\
\vdots & \vdots & \vdots & \ddots
\end{bmatrix}
$$

This full covariance tracking is what enables **uncertainty-aware mapping** — we can query the confidence of any part of the map at any time.

## 3. Dual Correction Strategy

### 3.1 Landmark-Based Correction

**Frequency:** Every scan (10 Hz)
**Purpose:** Long-term consistency, loop closure
**Method:** EKF update with landmark observations

When landmarks are re-observed:
1. Predict observation from current state
2. Compute innovation (measurement - prediction)
3. Update state and covariance using Kalman gain

### 3.2 ICP-Based Correction

**Frequency:** Every scan (when sufficient points accumulated)
**Purpose:** Short-term accuracy, drift correction
**Method:** Point cloud registration

When scan-to-map ICP succeeds:
1. Align current scan to accumulated submap
2. Compute pose correction
3. Apply as measurement to EKF

### 3.3 Complementary Benefits

| Aspect | Landmark-Based | ICP-Based |
|--------|---------------|-----------|
| **Representation** | Sparse (efficient) | Dense (complete) |
| **Robustness** | Feature-dependent | Works in all environments |
| **Long-term** | Excellent (explicit correspondences) | Drift accumulation |
| **Short-term** | Noisy (discrete observations) | High accuracy |
| **Uncertainty** | Explicit covariance | Estimated from residuals |

## 4. Information Flow

### 4.1 Per-Scan Processing

```
1. Odometry Update (PREDICT)
   ├─> Update robot pose estimate
   ├─> Propagate covariance
   └─> Process noise injection

2. Feature Extraction
   ├─> Convert LiDAR scan to Cartesian points
   ├─> Split-and-merge line segmentation
   ├─> Corner detection from line intersections
   └─> Compute feature covariances

3. Data Association
   ├─> Spatial gating (distance threshold)
   ├─> Mahalanobis distance gating (statistical)
   ├─> Nearest neighbor assignment
   └─> Separate matched/unmatched features

4. EKF Update (CORRECT)
   ├─> For matched landmarks:
   │   ├─> Compute observation Jacobian
   │   ├─> Calculate Kalman gain
   │   └─> Update state and covariance
   └─> For unmatched landmarks:
       ├─> Initialize new landmark
       └─> Augment state and covariance

5. ICP Alignment (OPTIONAL)
   ├─> Accumulate scan points in local frame
   ├─> Align to submap using ICP
   ├─> Compute pose correction
   └─> Apply as EKF measurement

6. Landmark Pruning
   └─> Remove landmarks not seen for N scans
```

### 4.2 Per-Submap Processing

After collecting 50 scans:

```
1. Compute Submap Confidence
   ├─> Extract robot pose covariance
   ├─> Compute information matrix
   ├─> Calculate confidence score
   └─> Log to CSV for analysis

2. Transform to Global Frame
   ├─> Use submap start pose
   └─> Transform all points to map frame

3. Submap Stitching
   ├─> Align to global map (ICP)
   ├─> Apply pose correction (if needed)
   └─> Integrate points into global map

4. Reset for Next Submap
   ├─> Clear local point buffer
   └─> Set new submap start pose
```

## 5. Coordinate Frames

The system maintains three coordinate frames following ROS REP-105 standard:

```
map (Fixed World Frame)
 │
 └─> odom (Odometry Frame, drift-prone)
      │
      └─> base_footprint (Robot Frame)
```

### 5.1 Frame Definitions

**`map` Frame:**
- Origin: Fixed, defined at system initialization
- Purpose: Global, consistent reference frame
- Updated by: EKF-SLAM (corrects drift)

**`odom` Frame:**
- Origin: Matches `map` at initialization
- Purpose: Locally consistent, smooth odometry
- Updated by: Wheel encoders / visual odometry (drifts over time)

**`base_footprint` Frame:**
- Origin: Robot center
- Purpose: Sensor measurements reference
- Updated by: Gazebo simulation (ground truth) or real robot

### 5.2 Transform Chain

The EKF publishes the `map → odom` transform to correct accumulated odometry drift:

$$
^{map}\mathbf{T}_{base} = \, ^{map}\mathbf{T}_{odom} \times \, ^{odom}\mathbf{T}_{base}
$$

Where:
- $^{map}\mathbf{T}_{base}$ is the corrected robot pose (from EKF)
- $^{odom}\mathbf{T}_{base}$ is the odometry estimate (from encoders)
- $^{map}\mathbf{T}_{odom}$ is the correction transform (computed and published)

## 6. Uncertainty Quantification

### 6.1 Information-Theoretic Confidence

For each submap, we compute a confidence score based on the **information matrix**:

$$
\mathbf{I} = \mathbf{P}^{-1}
$$

Where $\mathbf{P}$ is the robot pose covariance. The information matrix represents how much information the system has about its state:
- High information → low uncertainty → high confidence
- Low information → high uncertainty → low confidence

### 6.2 Confidence Metric

The normalized confidence score is:

$$
\text{confidence} = 1 - \exp\left(-\frac{\text{tr}(\mathbf{I})}{10}\right)
$$

This metric is logged for every submap, providing:
- Temporal analysis: How confidence evolves
- Spatial analysis: Which map regions are reliable
- System validation: Quantitative quality measure

## 7. Key Design Decisions

### 7.1 Landmark Representation

**Walls: Hessian Normal Form**
- Parameters: $(\rho, \alpha)$ where $\rho$ is distance from origin, $\alpha$ is normal angle
- Advantages: Orientation-normalized, 2D state per wall
- Limitation: Treats walls as infinite lines (requires spatial gating)

**Corners: Cartesian Coordinates**
- Parameters: $(x, y)$ in map frame
- Advantages: Intuitive, direct position representation
- Natural for point features

### 7.2 Hybrid Architecture Rationale

**Why not pure ICP?**
- No explicit landmark tracking → no loop closure
- No uncertainty correlation between robot and map
- Memory intensive for large environments

**Why not pure landmark SLAM?**
- Feature-poor environments problematic
- Discrete observations → noisy between detections
- Requires good feature extraction

**Why hybrid?**
- ICP provides dense, accurate short-term corrections
- Landmarks provide sparse, consistent long-term structure
- Full covariance enables uncertainty-aware decisions

### 7.3 Submap Strategy

**Why submaps?**
- Bounded computational cost per ICP iteration
- Natural checkpointing for confidence evaluation
- Enables hierarchical mapping

**Submap size: 50 scans**
- Trade-off between locality and coverage
- Sufficient for ICP convergence
- Manageable memory footprint

## 8. Implementation Details

### 8.1 ROS 2 Integration

**Node:** `LocalSubmapGenerator`
**Key Subscriptions:**
- `/tb3_1/scan` (LaserScan) — LiDAR measurements
- `/tb3_1/odom` (Odometry) — Wheel encoder estimates
- `/tb3_1/ground_truth_pose` (PoseStamped) — Simulation ground truth

**Key Publications:**
- `/tb3_1/ekf_pose` (PoseStamped) — Corrected robot pose
- `/tb3_1/ekf_path` (Path) — Trajectory history
- `/tb3_1/global_map` (PointCloud2) — Stitched global map
- `/tb3_1/scan_features` (MarkerArray) — Detected landmarks

**TF Broadcast:**
- `map → tb3_1/odom` — EKF correction transform

### 8.2 Data Logging

For thesis analysis, the system logs:

| File | Content | Purpose |
|------|---------|---------|
| `ekf_vs_groundtruth.csv` | Pose errors over time | Accuracy evaluation |
| `submap_confidence.csv` | Confidence metrics per submap | Uncertainty analysis |
| `global_map.pcd` | Final point cloud map | Visualization |

## 9. Performance Characteristics

### 9.1 Computational Complexity

**Per Scan:**
- Feature extraction: $O(n)$ where $n$ = number of scan points
- Data association: $O(m \cdot k)$ where $m$ = features, $k$ = landmarks
- EKF update: $O(d^3)$ where $d$ = state dimension
- ICP (when triggered): $O(n \cdot \log n)$

**Per Submap:**
- Submap ICP: $O(N \cdot \log M)$ where $N$ = submap points, $M$ = global map points

### 9.2 Memory Requirements

- State vector: $d \times 1$ where $d = 3 + 2k$ (robot + landmarks)
- Covariance: $d \times d$ (grows quadratically with landmarks)
- Submap buffer: $\sim$50 scans $\times$ 360 points $\sim$ 18k points
- Global map: Cumulative, voxel-filtered (manageable)

## 10. Extensions and Future Work

### 10.1 Potential Improvements

1. **Loop Closure Detection**
   - Currently relies on continuous landmark tracking
   - Could add place recognition for large-scale loops

2. **Endpoint Representation for Walls**
   - Replace Hessian form with $[x_1, y_1, x_2, y_2]$
   - Eliminates infinite line ambiguity
   - Full endpoint uncertainty in covariance

3. **Multi-Robot Extension**
   - Share landmark observations
   - Distributed map merging using confidence weights

4. **Active SLAM**
   - Use confidence metric to drive exploration
   - Navigate toward uncertain regions

### 10.2 Thesis Contributions

This implementation provides:
- ✅ Full robot-map covariance tracking
- ✅ Quantitative confidence metric per submap
- ✅ Hybrid approach combining dense and sparse methods
- ✅ Comprehensive evaluation data (ground truth comparison)
- ✅ Reproducible, documented codebase

## 11. References

1. Durrant-Whyte, H., & Bailey, T. (2006). "Simultaneous Localization and Mapping: Part I." *IEEE Robotics & Automation Magazine*.

2. Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

3. Besl, P. J., & McKay, N. D. (1992). "A Method for Registration of 3-D Shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*.

4. Nguyen, V., Martinelli, A., Tomatis, N., & Siegwart, R. (2005). "A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder for Indoor Mobile Robotics." *IROS 2005*.

5. Barfoot, T. D. (2017). *State Estimation for Robotics*. Cambridge University Press.

---

**Next Documents:**
- `01_ekf_slam_theory.md` — Complete EKF-SLAM mathematical derivation
- `02_landmark_features.md` — Feature extraction algorithms and geometry
- `03_data_association.md` — Mahalanobis distance and gating theory
- `04_icp_alignment.md` — ICP algorithm and convergence analysis
- `05_submap_management.md` — Submap creation and stitching
- `06_uncertainty_quantification.md` — Information theory and confidence metrics
- `07_coordinate_frames.md` — Transform management and frame conventions
