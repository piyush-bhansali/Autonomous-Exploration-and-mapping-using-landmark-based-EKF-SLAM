# Feature-Based SLAM Methodology

## 1. Overview

This document describes the feature-based SLAM methodology implemented for the comparative SLAM study. The feature-based approach uses Extended Kalman Filter (EKF) with wall and corner landmarks extracted from LiDAR scans, providing sparse probabilistic maps with explicit uncertainty quantification and natural loop closure capabilities.

## 2. System Architecture

### 2.1 Pipeline Overview

```
┌──────────────────────────────────────────────────────────────┐
│                 Feature-Based SLAM Pipeline                   │
│                                                                │
│  LiDAR Scan                                                   │
│      │                                                         │
│      ├──> Odometry Prediction ──> EKF Prediction             │
│      │                                  │                      │
│      │                                  ↓                      │
│      ├──> Feature Extraction ──> Walls & Corners             │
│      │         │                        │                      │
│      │         │                        ↓                      │
│      │         │              Data Association                │
│      │         │                        │                      │
│      │         │              ┌─────────┴─────────┐           │
│      │         │              │                   │           │
│      │         │              ↓                   ↓           │
│      │         │         Matched            Unmatched        │
│      │         │              │                   │           │
│      │         │              ↓                   ↓           │
│      │         │      EKF Landmark         Add New           │
│      │         │         Update           Landmarks          │
│      │         │              │                   │           │
│      │         │              └─────────┬─────────┘           │
│      │         │                        ↓                      │
│      │         │              Landmark Pruning                │
│      │         │                        │                      │
│      │         │                        ↓                      │
│      │         └──>          Current Pose Estimate            │
│      │                                  │                      │
│      └──────────────────────────────────┘                     │
│                                          │                      │
│                                          ↓                      │
│                                  Submap Complete?              │
│                                          │                      │
│                                          ├── Yes ──> Generate  │
│                                          │          Point Cloud│
│                                          │              │       │
│                                          │              ↓       │
│                                          │        Submap        │
│                                          │        Stitching     │
│                                          │              │       │
│                                          │              ↓       │
│                                          │         Global Map   │
│                                          │                      │
│                                          └── No ──> Continue    │
│                                                                │
└──────────────────────────────────────────────────────────────┘
```

### 2.2 Core Components

| Component | Purpose | Implementation |
|-----------|---------|----------------|
| **Feature Extraction** | Extract walls and corners | `landmark_features.py` |
| **Data Association** | Match observations to landmarks | `data_association.py` |
| **EKF-SLAM** | Probabilistic state estimation | `ekf_update_feature.py` |
| **Feature Map** | Store landmark parameters | `feature_map.py` |
| **SLAM Manager** | Coordinate SLAM operations | `feature_slam_manager.py` |
| **Point Cloud Generation** | Interpolate map from features | `feature_map.generate_point_cloud()` |

## 3. Feature Extraction

### 3.1 Pipeline

```
Raw LiDAR Scan
     │
     ↓
Cartesian Conversion
     │
     ↓
Gap Detection (split on occlusions)
     │
     ↓
Incremental Line Growing
     │
     ↓
Adjacent Segment Merging
     │
     ├──> Wall Features (Hessian form from endpoint geometry)
     │
     └──> Corner Features (from adjacent merged segments)
```

### 3.2 Scan to Cartesian Conversion

Convert polar LiDAR data to Cartesian coordinates:

$$
\begin{align}
x_i &= r_i \cos(\theta_i) \\
y_i &= r_i \sin(\theta_i)
\end{align}
$$

where:
- $r_i$: Range measurement
- $\theta_i = \theta_{min} + i \cdot \theta_{increment}$

**Filtering:**
```python
valid_mask = (ranges >= range_min) &
             (ranges <= range_max) &
             np.isfinite(ranges)
```

### 3.3 Gap-Based Segmentation

Split point cloud at large gaps (occlusions, object boundaries):

$$
\text{gap}_i = ||p_{i+1} - p_i|| > \text{threshold}_{gap}
$$

**Parameter:** `max_gap = 0.2m`

This prevents joining disconnected surfaces across gaps.

### 3.4 Incremental Line Growing

Within each gap-separated cluster, line segments are grown sequentially from ordered scan points:

1. Initialize candidate line growth from the first 2 points.
2. Extend the candidate by one point.
3. Compute the maximum perpendicular residual to the endpoint-defined line.
4. If residual exceeds `grow_residual_threshold`, finalize the previous segment and restart.

Segment finalization enforces:
- minimum point count (`min_points_per_line`)
- minimum geometric extent (`min_line_length`)

This produces deterministic endpoint-based segments without recursive split logic.

### 3.5 Adjacent Segment Merge

Consecutive grown segments are merged when all of the following are satisfied:
- acute direction difference below `merge_angle_tolerance`
- endpoint gap below `max_gap`
- merged candidate residual below `merge_rho_tolerance`

This preserves wall continuity while preventing merges across disconnected geometry (e.g., door openings).

### 3.6 Wall Representation (Hessian Normal Form)

Walls are represented in Hessian normal form:

$$
\rho = x \cos\alpha + y \sin\alpha
$$

where:
- $\rho$: Perpendicular distance from origin
- $\alpha$: Angle of normal vector

**Conversion from endpoint geometry:**
1. Direction from segment endpoints: $\hat{d} = \frac{p_{end} - p_{start}}{\|p_{end} - p_{start}\|}$
2. Normal vector: $\mathbf{n} = [-d_y, d_x]$
3. Midpoint: $\mathbf{m} = \frac{1}{2}(p_{start} + p_{end})$
4. Distance: $\rho = \mathbf{n} \cdot \mathbf{m}$, then enforce $\rho > 0$ by flipping sign if needed
5. Angle: $\alpha = \text{atan2}(n_y, n_x)$

**Advantages:**
- No redundancy (unique representation)
- Direct geometric interpretation
- Efficient EKF updates

### 3.7 Corner Representation

Corners are represented in Cartesian coordinates:

$$
\mathbf{l}_corner = [x, y]^T
$$

Located at breakpoints between wall segments.

**Corner quality metric:**
- Sharpness: Angle between adjacent walls
- Sharper corners (closer to 90°) provide better constraints

## 4. Feature Uncertainty Quantification

### 4.1 Wall Covariance (Hessian-Based)

For wall features, covariance is computed using Fisher Information Matrix:

$$
\Sigma_{wall} = \sigma^2 (J^T J)^{-1}
$$

where $\sigma = 0.01m$ (LiDAR noise) and $J$ is the Jacobian matrix.

**Jacobian for point $i$ on wall:**
$$
J_i = \begin{bmatrix}
\frac{\partial r_i}{\partial \rho} & \frac{\partial r_i}{\partial \alpha}
\end{bmatrix} = \begin{bmatrix}
1 & x_i\sin\alpha - y_i\cos\alpha
\end{bmatrix}
$$

where $r_i = \rho - (x_i\cos\alpha + y_i\sin\alpha)$ is the point-to-line residual.

**Information accumulation:**
$$
H = \sum_{i=1}^N J_i^T J_i
$$

**Covariance:**
$$
\Sigma_{wall} = \sigma^2 H^{-1}
$$

**Interpretation:**
- More points → smaller covariance (more information)
- Longer walls → better angle constraint
- Perpendicular distance well-constrained by all points

### 4.2 Corner Covariance (PCA-Based)

For corners, covariance reflects point scatter in neighboring region:

1. Collect neighboring points (window = 6 points each side)
2. Compute PCA on neighbors
3. Eigenvalues represent scatter variance
4. Covariance in PCA basis:

$$
\Sigma_{corner} = \sigma^2 \frac{1}{N_{eff}} V \Lambda V^T
$$

where:
- $\Lambda$: Diagonal matrix of eigenvalues (scatter)
- $V$: Eigenvectors (principal directions)
- $N_{eff} = \min(N, 30)$: Effective observation count

**Interpretation:**
- Large scatter → large covariance (uncertain position)
- Small scatter → small covariance (well-defined corner)
- More observations → lower uncertainty

## 5. Data Association

### 5.1 Overview

Data association matches observed features to existing landmarks using:
1. Type consistency (wall-to-wall, corner-to-corner)
2. Mahalanobis distance gating
3. Euclidean distance gating
4. **Segment overlap validation (walls only)**

### 5.2 Mahalanobis Distance Gating

For each observation-landmark pair, compute innovation:

$$
\mathbf{\nu} = \mathbf{z} - h(\mathbf{x})
$$

Innovation covariance:

$$
S = H P H^T + R
$$

Mahalanobis distance:

$$
d_M = \sqrt{\mathbf{\nu}^T S^{-1} \mathbf{\nu}}
$$

**Gating threshold:** $d_M < 5.99$ (χ² 95% confidence, 2 DOF)

**Rationale:**
- Accounts for both landmark and measurement uncertainty
- Statistical consistency with Gaussian assumption
- Rejects outliers beyond confidence bound

### 5.3 Euclidean Distance Gating

Additional spatial constraint for robustness:

**For walls:**
$$
d_{wall} = |\rho_{landmark} - (x_r \cos\alpha + y_r \sin\alpha)| < 2.0m
$$

**For corners:**
$$
d_{corner} = ||\mathbf{l}_{landmark} - \mathbf{x}_r||_2 < 2.0m
$$

This prevents matching distant features even if Mahalanobis distance is small (due to large uncertainty).

### 5.4 Wall Segment Overlap Validation

**Critical innovation:** Prevents matching collinear walls separated by gaps.

**Algorithm:**
1. Transform observed wall endpoints to map frame:
   $$
   \begin{bmatrix} x_{map} \\ y_{map} \end{bmatrix} = R(\theta_r) \begin{bmatrix} x_{robot} \\ y_{robot} \end{bmatrix} + \begin{bmatrix} x_r \\ y_r \end{bmatrix}
   $$

2. Compute wall tangent direction:
   $$
   \mathbf{t} = [-\sin\alpha, \cos\alpha]^T
   $$

3. Project all endpoints onto tangent:
   $$
   \begin{align}
   p_{existing,start} &= \mathbf{t} \cdot \mathbf{s}_{existing} \\
   p_{existing,end} &= \mathbf{t} \cdot \mathbf{e}_{existing} \\
   p_{obs,start} &= \mathbf{t} \cdot \mathbf{s}_{obs} \\
   p_{obs,end} &= \mathbf{t} \cdot \mathbf{e}_{obs}
   \end{align}
   $$

4. Check overlap or adjacency:
   $$
   \text{overlap} = \max(\min(p_{existing}), \min(p_{obs})) \leq \min(\max(p_{existing}), \max(p_{obs}))
   $$

   OR gap is small:
   $$
   \text{gap} = \max(\min(p_{existing}), \min(p_{obs})) - \min(\max(p_{existing}), \max(p_{obs})) \leq 0.5m
   $$

**Gap tolerance:** 0.5m (accounts for localization errors)

**Effect:**
- Collinear walls with large gaps → separate landmarks
- Collinear walls touching/overlapping → same landmark (extended)
- Prevents incorrect map merging

### 5.5 Best Match Selection

```python
best_landmark = None
best_mahalanobis = infinity

for each landmark:
    if type_matches and euclidean_ok and mahalanobis_ok:
        if wall: check_segment_overlap()
        if passes_all_tests and mahalanobis < best_mahalanobis:
            best_landmark = landmark
            best_mahalanobis = mahalanobis
```

Returns: `(matched_pairs, unmatched_observations)`

## 6. EKF-SLAM

### 6.1 State Vector

The EKF maintains a joint state of robot pose and all landmark parameters:

$$
\mathbf{x} = \begin{bmatrix}
x_r \\ y_r \\ \theta_r \\
\rho_1 \\ \alpha_1 \\
x_{c1} \\ y_{c1} \\
\vdots \\
\rho_M \\ \alpha_M \\
x_{cN} \\ y_{cN}
\end{bmatrix}
$$

where:
- $(x_r, y_r, \theta_r)$: Robot pose
- $(\rho_i, \alpha_i)$: Wall $i$ in Hessian form
- $(x_{cj}, y_{cj})$: Corner $j$ in Cartesian

### 6.2 Covariance Matrix

Full covariance captures correlations:

$$
\mathbf{P} = \begin{bmatrix}
\mathbf{P}_{rr} & \mathbf{P}_{rl_1} & \cdots & \mathbf{P}_{rl_N} \\
\mathbf{P}_{l_1r} & \mathbf{P}_{l_1l_1} & \cdots & \mathbf{P}_{l_1l_N} \\
\vdots & \vdots & \ddots & \vdots \\
\mathbf{P}_{l_Nr} & \mathbf{P}_{l_Nl_1} & \cdots & \mathbf{P}_{l_Nl_N}
\end{bmatrix}
$$

**Key property:** Correlations enable consistent map updates when robot pose is corrected.

### 6.3 Prediction Step

Using odometry $\mathbf{u} = [\Delta d, \Delta\theta]^T$:

**Robot pose update:**
$$
\begin{align}
x_r' &= x_r + \Delta d \cos(\theta_r + \Delta\theta/2) \\
y_r' &= y_r + \Delta d \sin(\theta_r + \Delta\theta/2) \\
\theta_r' &= \theta_r + \Delta\theta
\end{align}
$$

**Jacobian (robot pose only changes):**
$$
F = \begin{bmatrix}
I_3 + F_{motion} & 0 \\
0 & I_{landmarks}
\end{bmatrix}
$$

where:
$$
F_{motion} = \begin{bmatrix}
0 & 0 & -\Delta d \sin(\theta_r + \Delta\theta/2) \\
0 & 0 & \Delta d \cos(\theta_r + \Delta\theta/2) \\
0 & 0 & 0
\end{bmatrix}
$$

**Control Jacobian:**
$$
G = \begin{bmatrix}
G_{odom} \\
0_{landmarks \times 2}
\end{bmatrix}
$$

**Covariance update:**
$$
\mathbf{P}' = F \mathbf{P} F^T + G Q G^T
$$

### 6.4 Update Step (Wall Observation)

**Observation model** (wall in robot frame):
$$
\begin{bmatrix} \rho_{obs} \\ \alpha_{obs} \end{bmatrix} = \begin{bmatrix}
\rho - (x_r \cos\alpha + y_r \sin\alpha) \\
\alpha - \theta_r
\end{bmatrix} + \mathbf{v}
$$

**Jacobian:**
$$
H = \begin{bmatrix}
-\cos\alpha & -\sin\alpha & 0 & 1 & x_r\sin\alpha - y_r\cos\alpha \\
0 & 0 & -1 & 0 & 1
\end{bmatrix}
$$

(Extended to full state size with zeros for other landmarks)

**Innovation:**
$$
\mathbf{\nu} = \mathbf{z}_{obs} - h(\mathbf{x}^-)
$$

**Kalman gain:**
$$
K = \mathbf{P}^- H^T (H \mathbf{P}^- H^T + R)^{-1}
$$

**State update:**
$$
\mathbf{x} = \mathbf{x}^- + K \mathbf{\nu}
$$

**Covariance update (Joseph form):**
$$
\mathbf{P} = (I - KH) \mathbf{P}^- (I - KH)^T + K R K^T
$$

### 6.5 Update Step (Corner Observation)

**Observation model** (corner in robot frame):
$$
\begin{bmatrix} x_{obs} \\ y_{obs} \end{bmatrix} = R(-\theta_r) \begin{bmatrix} x_c - x_r \\ y_c - y_r \end{bmatrix} + \mathbf{v}
$$

**Jacobian:**
$$
H = \begin{bmatrix}
-\cos\theta_r & -\sin\theta_r & -\Delta x \sin\theta_r - \Delta y \cos\theta_r & \cos\theta_r & -\sin\theta_r \\
\sin\theta_r & -\cos\theta_r & -\Delta x \cos\theta_r + \Delta y \sin\theta_r & \sin\theta_r & \cos\theta_r
\end{bmatrix}
$$

where $\Delta x = x_c - x_r$, $\Delta y = y_c - y_r$.

**EKF update:** Same as walls (Kalman gain, state/covariance update)

### 6.6 Landmark Initialization

**For new wall:**
1. Add to EKF in robot frame: $[\rho_{robot}, \alpha_{robot}]$
2. Transform to map frame:
   $$
   \begin{align}
   \alpha_{map} &= \alpha_{robot} + \theta_r \\
   \rho_{map} &= \rho_{robot} + x_r \cos\alpha_{map} + y_r \sin\alpha_{map}
   \end{align}
   $$

3. Compute initial covariance:
   $$
   \mathbf{P}_{wall} = H_r \mathbf{P}_{rr} H_r^T + H_z R H_z^T
   $$

4. Augment state and covariance
5. Register in FeatureMap for geometric tracking

**For new corner:**
1. Add to EKF in robot frame: $[x_{robot}, y_{robot}]$
2. Transform to map frame:
   $$
   \begin{bmatrix} x_{map} \\ y_{map} \end{bmatrix} = R(\theta_r) \begin{bmatrix} x_{robot} \\ y_{robot} \end{bmatrix} + \begin{bmatrix} x_r \\ y_r \end{bmatrix}
   $$

3. Compute initial covariance and augment
4. Register in FeatureMap

### 6.7 Landmark Pruning

Remove landmarks that haven't been observed recently:

```python
scans_since_seen = current_scan - last_seen

if scans_since_seen > timeout_scans:
    remove_landmark()
```

**Parameters:**
- Timeout: 50 scans
- Minimum observations for persistence: 2

**Pruning process:**
1. Remove from state vector
2. Remove rows/columns from covariance matrix
3. Update indices of remaining landmarks
4. Synchronize with FeatureMap

**Rationale:**
- Prevents unbounded state growth
- Removes spurious detections
- Maintains computational efficiency

### 6.7 Covariance Conditioning

To maintain numerical stability:

$$
\mathbf{P}_{conditioned} = V \cdot \text{clip}(\Lambda, \lambda_{min}, \lambda_{max}) \cdot V^T
$$

where eigenvalues are clipped:
- $\lambda_{min} = 10^{-6}$ (prevents singularity)
- $\lambda_{max} = 100$ (prevents unbounded uncertainty)

Applied after each prediction and update.

## 7. Feature Map Management

### 7.1 Purpose

FeatureMap stores geometric parameters separate from EKF state:
- Wall Hessian parameters $(\rho, \alpha)$ and tangential extents $(t_{\min}, t_{\max})$
- Corner positions
- Observation counts

**Why separate?**
- EKF stores parametric representation ($\rho, \alpha$ for walls)
- FeatureMap stores geometric **extents** for visualization and overlap checks
- Enables segment overlap validation
- Facilitates point cloud generation

### 7.2 Wall Extension

When a wall is re-observed and matched:

1. Get existing extents $(t_{\min}, t_{\max})$ along the wall tangent
2. Transform new observation endpoints to map frame and project onto the wall tangent
3. Take min/max projections:
   $$
   \begin{align}
   p_{min} &= \min(p_{s,old}, p_{e,old}, p_{s,new}, p_{e,new}) \\
   p_{max} &= \max(p_{s,old}, p_{e,old}, p_{s,new}, p_{e,new})
   \end{align}
   $$
4. Update $(t_{\min}, t_{\max})$ with the new min/max

**Effect:** Walls grow as robot explores, capturing full extent while remaining consistent with the current $(\rho, \alpha)$.

### 7.3 Point Cloud Generation

For visualization and map export:

```python
def generate_point_cloud(spacing=0.05):
    points = []

    # Interpolate along walls
    for wall in walls:
        # Reconstruct endpoints from (rho, alpha, t_min, t_max)
        length = ||end - start||
        num_points = ceil(length / spacing)
        for i in range(num_points):
            t = i / (num_points - 1)
            point = start + t * (end - start)
            points.append(point)

    # Add corners
    for corner in corners:
        points.append(corner.position)

    return points
```

**Spacing:** 5cm (dense enough for visualization, not excessive)

**Result:** Sparse landmark map → dense point cloud for compatibility with planning/visualization tools

## 8. Performance Characteristics

### 8.1 Computational Complexity

| Operation | Complexity | Notes |
|-----------|-----------|-------|
| Feature extraction | $O(N)$ | $N$ = scan points |
| PCA per segment | $O(M)$ | $M$ = points per segment |
| Data association | $O(L \cdot F)$ | $L$ = landmarks, $F$ = features |
| EKF prediction | $O(L^2)$ | Covariance propagation |
| EKF update | $O(L^2)$ | Per landmark observation |
| Landmark pruning | $O(L)$ | Occasional operation |

**Total per scan:** $O(N + L^2)$

**Scaling:**
- Feature extraction: Linear with scan size
- EKF updates: Quadratic with landmark count
- Typical: $L \approx 10-50$ landmarks in local area

### 8.2 Memory Requirements

- **Per landmark:** ~100 bytes (state + covariance rows)
- **EKF covariance:** $O(L^2)$ where $L$ = 3 + 2×(num landmarks)
- **FeatureMap:** ~200 bytes per wall, ~50 bytes per corner

**Example:**
- 20 walls, 10 corners: ~5KB for landmarks
- EKF state size: 3 + 40 + 20 = 63 dimensions
- Covariance: 63×63×8 bytes ≈ 32KB

**Scaling:** $O(L^2)$ but with small constant (bounded landmark count)

### 8.3 Accuracy vs. Computational Cost

**Feature extraction trade-offs:**
- Smaller angle threshold: More corners detected, more computation
- Larger angle threshold: Miss some corners, less computation
- Optimal: 50° (balanced)

**Data association trade-offs:**
- Tighter gates: Fewer matches, more new landmarks, higher memory
- Looser gates: More matches, risk of incorrect associations
- Optimal: Mahalanobis 5.99 + Euclidean 2.0m

**EKF update frequency:**
- Update every landmark: Most accurate, expensive
- Update selectively: Faster, slight accuracy loss
- Current: Update all matched landmarks (ensures consistency)

## 9. Advantages and Limitations

### 9.1 Advantages

✅ **Explicit probabilistic uncertainty**
- Full covariance matrix
- Theoretically grounded confidence estimates
- Uncertainty-aware decision making

✅ **Sparse, efficient representation**
- Memory scales with landmark count, not environment size
- Typical: 10-50 landmarks vs. millions of grid cells

✅ **Natural loop closure**
- Re-observing landmarks updates entire map through correlations
- No additional place recognition needed for small loops

✅ **Bounded computational complexity**
- Landmark count bounded by environment structure
- Pruning prevents unbounded growth

✅ **Interpretable map**
- Walls and corners have semantic meaning
- Useful for high-level planning

✅ **Well-studied theoretical properties**
- Convergence guarantees (Dissanayake et al., 2001)
- Observability analysis (Huang et al., 2010)

### 9.2 Limitations

❌ **Requires reliable features**
- Struggles in feature-poor environments (long corridors, warehouses)
- Needs structured geometry

❌ **Discrete observations**
- Gaps between feature detections
- Pose uncertainty grows between landmarks

❌ **Data association challenges**
- Incorrect matches corrupt map irrecoverably
- Conservative gating may miss valid matches

❌ **Linearization errors**
- EKF assumes local linearity
- Large motions/rotations can cause inconsistency

❌ **Computational cost scales with landmarks**
- $O(L^2)$ updates
- Bounded but grows with map complexity

❌ **Feature extraction overhead**
- Incremental segment growth, merge validation, and corner extraction required
- More processing than raw ICP

## 10. Parameters and Configuration

### 10.1 Feature Extraction Parameters

```python
feature_params = {
    'min_points_per_line': 5,
    'min_line_length': 0.3,  # meters
    'corner_angle_threshold': 50.0,  # degrees
    'max_gap': 0.2,  # meters
    'merge_angle_tolerance': 0.15,  # radians
    'merge_rho_tolerance': 0.15,  # meters
    'grow_residual_threshold': 0.03,  # meters
    'lidar_noise_sigma': 0.01,  # meters
}
```

### 10.2 Data Association Parameters

```python
association_params = {
    'max_mahalanobis_dist': 5.99,  # χ² 95%, 2-DOF
    'max_euclidean_dist': 6.0,  # meters
    'wall_angle_tolerance': 0.22,  # radians
    'wall_rho_tolerance': 0.3,  # meters
}
```

### 10.3 EKF Parameters

```python
ekf_params = {
    'max_landmark_range': 5.0,  # meters
    'landmark_timeout_scans': 50,
    'min_observations_for_init': 2,
    'process_noise_distance': 0.01,
    'process_noise_rotation': 0.005,
    'min_noise_floor': 0.0001,
}
```

### 10.4 Submap Parameters

```python
submap_params = {
    'scans_per_submap': 50,
    'point_cloud_spacing': 0.05,  # meters
}
```

## 11. Experimental Validation

### 11.1 Evaluation Metrics

**Localization Accuracy:**
- Absolute Trajectory Error (ATE)
- Relative Pose Error (RPE)

**Map Quality:**
- Landmark count (sparsity)
- Coverage (% of environment represented)
- Precision/Recall vs. ground truth

**Computational Performance:**
- Processing time per scan
- Memory usage (EKF state size)
- Landmark count over time

**Uncertainty Calibration:**
- Normalized Estimation Error Squared (NEES)
- Confidence interval coverage (should be ~95%)

### 11.2 Comparison with ICP

| Aspect | Feature-Based | ICP-Based |
|--------|---------------|-----------|
| Map representation | Sparse landmarks | Dense point cloud |
| Memory | $O(L^2)$ | $O(environment)$ |
| Computation per scan | $O(L^2)$ | $O(N \log M)$ |
| Feature-poor environments | Struggles | Robust |
| Uncertainty | Explicit covariance | Hessian-based |
| Loop closure | Natural (landmark re-observation) | Requires global ICP |

## 12. Implementation Architecture

### 12.1 Class Structure

```
FeatureSLAMManager
    ├── EKF-SLAM (ekf)
    ├── FeatureExtractor (feature_extractor)
    ├── FeatureMap (feature_map)
    └── Methods:
        ├── process_scan()
        ├── initialize_pose()
        ├── predict_motion()
        ├── get_robot_pose()
        └── generate_point_cloud()
```

**Design rationale:**
- Encapsulates all feature SLAM logic
- Clean interface for ROS2 node
- Testable independent of ROS2
- Modular components

### 12.2 ROS2 Integration

```python
class LocalSubmapGenerator(Node):
    def __init__(self):
        self.slam_manager = FeatureSLAMManager(...)

    def scan_callback(self, msg):
        stats = self.slam_manager.process_scan(msg)
```

### 12.3 Key Files

| File | Purpose |
|------|---------|
| `feature_slam_manager.py` | Main SLAM orchestrator |
| `ekf_predict.py` | EKF prediction (odometry) |
| `ekf_update_feature.py` | EKF update (landmarks + pose correction) |
| `landmark_features.py` | Feature extraction |
| `data_association.py` | Matching algorithm |
| `feature_map.py` | Geometric feature storage |
| `local_submap_generator_feature.py` | ROS2 mapping node |

## 13. Conclusion

The feature-based SLAM methodology provides a probabilistic framework for sparse, uncertainty-aware mapping. Its strengths lie in explicit covariance representation, natural loop closure through landmark re-observation, and efficient memory usage through sparse representation.

The primary trade-offs are dependence on structured environments with reliable features and computational cost scaling with landmark count. These characteristics make feature-based SLAM well-suited for structured indoor environments where landmarks are abundant and probabilistic uncertainty quantification is valuable.

The comparative evaluation against ICP-based SLAM will empirically quantify these trade-offs across different environment types, exploration scenarios, and performance metrics.

## References

Arras, K. O., Castellanos, J. A., Schilt, M., & Siegwart, R. (2001). Feature-based multi-hypothesis localization and tracking using geometric constraints. *Robotics and Autonomous Systems*, 44(1), 41-53.

Bailey, T., & Durrant-Whyte, H. (2006). Simultaneous localization and mapping (SLAM): Part II. *IEEE Robotics & Automation Magazine*, 13(3), 108-117.

Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001). A solution to the simultaneous localization and map building (SLAM) problem. *IEEE Transactions on Robotics and Automation*, 17(3), 229-241.

Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010). Observability-based rules for designing consistent EKF SLAM estimators. *The International Journal of Robotics Research*, 29(5), 502-528.

Neira, J., & Tardós, J. D. (2001). Data association in stochastic mapping using the joint compatibility test. *IEEE Transactions on Robotics and Automation*, 17(6), 890-897.

Nguyen, V., Gächter, S., Martinelli, A., Tomatis, N., & Siegwart, R. (2005). A comparison of line extraction algorithms using 2D range data for indoor mobile robotics. *Autonomous Robots*, 23(2), 97-111.

Pavlidis, T., & Horowitz, S. L. (1974). Segmentation of plane curves. *IEEE Transactions on Computers*, C-23(8), 860-870.

Siegwart, R., & Nourbakhsh, I. R. (2011). *Introduction to autonomous mobile robots* (2nd ed.). MIT Press.

Smith, R., Self, M., & Cheeseman, P. (1990). Estimating uncertain spatial relationships in robotics. In *Autonomous Robot Vehicles* (pp. 167-193). Springer.

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press.
