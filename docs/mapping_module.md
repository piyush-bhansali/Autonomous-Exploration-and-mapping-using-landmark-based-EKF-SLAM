# Mapping Module: Theory, Implementation, and Data Flow

**GeoSLAM — Feature-Based EKF-SLAM for Autonomous Indoor Mapping**
*Thesis Documentation — Comprehensive Module Reference*

---

## Table of Contents

1. [Module Architecture and Data Flow](#1-module-architecture-and-data-flow)
2. [Coordinate Frames](#2-coordinate-frames)
3. [Feature Extraction](#3-feature-extraction)
   - 3.1 Scan-to-Cartesian Conversion
   - 3.2 Gap-Based Segmentation
   - 3.3 Incremental Line Growing (TLS)
   - 3.4 Adjacent Segment Merging
   - 3.5 Hessian Normal Form Parameterisation
   - 3.6 Wall Covariance via Fisher Information
   - 3.7 Corner Detection
   - 3.8 Corner Covariance via Jacobian Propagation
4. [EKF State and Prediction](#4-ekf-state-and-prediction)
   - 4.1 State Vector and Covariance
   - 4.2 Motion Model (Midpoint Integration)
   - 4.3 Process Noise Jacobians
   - 4.4 Covariance Prediction and Conditioning
5. [Landmark Initialisation](#5-landmark-initialisation)
   - 5.1 Wall Landmark Augmentation
   - 5.2 Corner Landmark Augmentation
   - 5.3 Cross-Covariance Seeding
6. [Data Association](#6-data-association)
   - 6.1 Type and Euclidean Gating
   - 6.2 Predicted Observation and Jacobian
   - 6.3 Mahalanobis Distance Gate
   - 6.4 Wall Segment Overlap/Gap Validation
   - 6.5 Best-Match Selection
7. [EKF Update](#7-ekf-update)
   - 7.1 Innovation Computation
   - 7.2 Outlier Rejection
   - 7.3 Kalman Gain
   - 7.4 State Update
   - 7.5 Rho Normalisation (Cross-Landmark)
   - 7.6 Joseph-Form Covariance Update
8. [Landmark Pruning](#8-landmark-pruning)
9. [Feature Map](#9-feature-map)
   - 9.1 Wall Storage with Arc-Length Extents
   - 9.2 Hessian Synchronisation
   - 9.3 Point Cloud Generation
10. [SLAM Manager Orchestration](#10-slam-manager-orchestration)
    - 10.1 Per-Scan Processing Pipeline
    - 10.2 Matched Feature Processing and Full Hessian Sync
    - 10.3 Unmatched Feature Insertion
11. [Submap Management and Stitching](#11-submap-management-and-stitching)
    - 11.1 Submap Trigger and Point Cloud Generation
    - 11.2 Persistent Wall Extents Across Submap Boundaries
    - 11.3 Global Wall Registry
    - 11.4 Feature-Based SVD Alignment
    - 11.5 SVD Covariance and Degeneracy Detection
    - 11.6 EKF Pose Correction Feedback
    - 11.7 Wall Accumulation
12. [TF and Pose Publishing](#12-tf-and-pose-publishing)
13. [Parameter Reference](#13-parameter-reference)

---

## 1. Module Architecture and Data Flow

The mapping module is implemented across six Python files in the `map_generation` package. The central ROS 2 node is `LocalSubmapGeneratorFeature` (`local_submap_generator_feature.py`), which orchestrates all sub-systems at 10 Hz driven by incoming LiDAR scans and odometry messages.

```
                       ┌──────────────────────────────────────┐
    /tb3_1/scan ──────►│  LocalSubmapGeneratorFeature (Node)  │
    /tb3_1/odom ──────►│                                      │
                       └──────────────┬───────────────────────┘
                                      │
             ┌────────────────────────▼────────────────────────────────┐
             │              FeatureSLAMManager                         │
             │  (feature_slam_manager.py — orchestration layer)        │
             │                                                          │
             │   ┌──────────────────┐  ┌──────────────────┐           │
             │   │ LandmarkFeature  │  │  LandmarkEKFSLAM │           │
             │   │  Extractor       │  │  (ekf_update_     │           │
             │   │ (landmark_       │  │   feature.py)     │           │
             │   │  features.py)    │  │                   │           │
             │   └────────┬─────────┘  └─────────┬─────────┘           │
             │            │ observed_features      │ state, P           │
             │            └──────────┬────────────┘                    │
             │                       ▼                                  │
             │              data_association.py                         │
             │           (matched, unmatched, extension_info)           │
             │                       │                                  │
             │            ┌──────────▼──────────┐                      │
             │            │    FeatureMap        │                      │
             │            │  (feature_map.py)    │                      │
             │            │  walls: {rho, alpha, │                      │
             │            │   t_min, t_max}      │                      │
             │            └─────────────────────-┘                      │
             └──────────────────────┬──────────────────────────────────┘
                                    │ point cloud (map frame)
                                    ▼
                           SubmapStitcher
                        (submap_stitcher.py)
                        SVD alignment → pose_correction
                                    │
                                    ▼
                        EKF pose update + global map
```

**Per-scan execution sequence:**
1. `odom_callback` → compute `(Δd, Δθ)` → `predict_motion()`
2. `scan_callback` → `_process_scan_feature_mode()`
3. `FeatureSLAMManager.process_scan()`:
   a. `LandmarkFeatureExtractor.extract_features()` → `observed_features`
   b. `associate_landmarks()` → `matched`, `unmatched`, `extension_info`
   c. `_process_matched_features()` → EKF update + endpoint extension + full Hessian sync
   d. `_process_unmatched_features()` → add new landmarks
   e. `_prune_old_landmarks()`
4. Every 50 scans: `create_submap_from_features()` → `SubmapStitcher.integrate_submap_to_global_map()`

---

## 2. Coordinate Frames

The system operates in three distinct coordinate frames. Understanding their relationships is essential for interpreting every transformation in the code.

### 2.1 Robot Frame

Origin at the robot centre, x-axis pointing forward, y-axis pointing left. All raw LiDAR measurements are expressed in this frame. Features (walls, corners) are extracted in robot frame.

A point at range $r$ and bearing $\phi$ maps to robot-frame Cartesian coordinates:
$$x_R = r\cos\phi, \quad y_R = r\sin\phi$$

### 2.2 EKF Map Frame

A fixed, world-aligned inertial frame initialised at the robot's starting pose. The EKF state vector and all stored landmark parameters live in this frame. All published poses and the `FeatureMap` are in this frame.

The rigid-body transformation from robot frame to map frame is:
$$\mathbf{T}_{\text{map}}^{\text{robot}} = \begin{bmatrix} \cos\theta_r & -\sin\theta_r & x_r \\ \sin\theta_r & \cos\theta_r & y_r \\ 0 & 0 & 1 \end{bmatrix}$$

where $(x_r, y_r, \theta_r)$ is the EKF robot state.

### 2.3 Global Wall Registry (`global_walls`)

The `SubmapStitcher` maintains a persistent registry `global_walls` — a dictionary keyed by landmark ID storing the Hessian parameters $(\rho, \alpha)$ and arc-length extents $(t_{\min}, t_{\max})$ as they stood after the most recent SVD correction. This registry is **not a separate coordinate frame**; it lives in the same map frame as `feature_map.walls`. The small numerical difference between the two structures arises from odometric drift accumulated over the current 50-scan window: `feature_map.walls` holds the EKF's current (slightly drifted) estimate, while `global_walls` holds the last corrected snapshot. The SVD alignment each cycle measures this drift and feeds the correction back into the EKF, after which both structures represent the same physical reality. The distinction is therefore one of **metadata freshness** — current estimate vs. last corrected reference — not a difference in coordinate frame.

### 2.4 Odom Frame and TF Chain

ROS 2 navigation requires the chain:

$$\texttt{map} \xrightarrow{\text{TF}} \texttt{tb3\_1/odom} \xrightarrow{\text{wheel enc.}} \texttt{tb3\_1/base\_footprint}$$

The `map→odom` transform is computed by:
$$T_{\text{map}\to\text{odom}} = T_{\text{map}\to\text{base}} \cdot T_{\text{odom}\to\text{base}}^{-1}$$

where the closed-form 2D inverse is:
$$T^{-1} = \begin{bmatrix} c & s & -(cx+sy) \\ -s & c & sx-cy \\ 0 & 0 & 1 \end{bmatrix}$$

This is computed in `transform_utils.py:compute_map_to_odom_transform` and broadcast at 10 Hz via `_publish_tf_callback`.

---

## 3. Feature Extraction

**File:** `landmark_features.py` — `class LandmarkFeatureExtractor`

The feature extractor converts a raw `LaserScan` message into a list of wall and corner feature dictionaries. Each feature carries its geometric parameters, covariance, and supporting point data.

### 3.1 Scan-to-Cartesian Conversion

```python
# landmark_features.py:69 — scan_to_cartesian()
angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, num_points)
valid_mask = (ranges >= range_min) & (ranges <= range_max) & np.isfinite(ranges)
x = valid_ranges * np.cos(valid_angles)
y = valid_ranges * np.sin(valid_angles)
```

The LDS-01 provides 360 uniformly-spaced range samples over $[−\pi, \pi]$ at each 10 Hz scan. Readings outside $[r_{\min}, r_{\max}]$ and `inf`/`nan` values are masked out. The resulting point set $\mathcal{P} = \{(x_i, y_i)\}$ is in the robot frame.

### 3.2 Gap-Based Segmentation

```python
# landmark_features.py:117 — split_on_gaps()
dists = np.linalg.norm(points[1:] - points[:-1], axis=1)
gap_indices = np.where(dists > self.max_gap)[0]
```

Consecutive LiDAR returns are separated if the Euclidean distance between adjacent points exceeds `max_gap = 0.2 m`. This splits the point set into continuous segments corresponding to separate surfaces. Each segment requires at least `min_points = 10` points to proceed to line fitting. This pre-processing prevents fitting lines across free space and ensures each wall candidate corresponds to a single continuous reflective surface.

### 3.3 Incremental Line Growing (Total Least Squares)

```python
# landmark_features.py:138 — grow_lines_incremental()
while i < n:
    candidate = points[start:i + 1]
    residual = self.segment_residual_tls(candidate)
    if residual <= self.grow_residual_threshold:
        i += 1          # accept point, grow segment
    else:
        finalize(points[start:i])
        start = max(i - 1, start + 1)   # restart with shared boundary
        i = start + 1
```

**Total Least Squares (TLS) Fit:** For a candidate segment $\{(x_i, y_i)\}_{i=1}^n$, compute the centroid $\bar{\mathbf{p}} = \frac{1}{n}\sum_i \mathbf{p}_i$ and the centred matrix:

$$\mathbf{C} = \begin{bmatrix} (x_1 - \bar{x}) & (y_1 - \bar{y}) \\ \vdots & \vdots \\ (x_n - \bar{x}) & (y_n - \bar{y}) \end{bmatrix} \in \mathbb{R}^{n \times 2}$$

Apply SVD: $\mathbf{C} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^\top$. The first row of $\mathbf{V}^\top$ (the right singular vector corresponding to the largest singular value $\sigma_1$) is the **principal axis** $\hat{\mathbf{d}}$ — the direction along which point variance is maximum. The normal vector is $\hat{\mathbf{n}} = [-d_y, d_x]^\top$.

The TLS residual for a candidate segment is the maximum perpendicular distance from any point to the best-fit line:

$$r_{\text{TLS}} = \max_{i} \left| (\mathbf{p}_i - \bar{\mathbf{p}}) \cdot \hat{\mathbf{n}} \right|$$

implemented in `segment_residual_tls()` at line 182. A point is added to the growing segment if and only if $r_{\text{TLS}} \leq \delta_{\text{grow}} = 0.03\,\text{m}$. When this threshold is violated, the current segment is finalised (if it satisfies the minimum requirements) and a new segment begins with the last accepted point as the shared boundary, preserving geometric continuity at segment junctions.

**Rationale for TLS over endpoint-based residuals:** Endpoint-based methods compute the perpendicular distance from each intermediate point to the line defined by the two outermost points. At grazing angles — common when the robot is nearly parallel to a wall — the outermost endpoints are the noisiest samples, making the reference line highly unstable. TLS uses all points symmetrically, yielding a statistically optimal fit under isotropic Gaussian noise.

### 3.4 Adjacent Segment Merging

```python
# landmark_features.py:203 — merge_adjacent_lines()
if (angle_diff <= merge_angle_tol          # 0.22 rad ≈ 12.6°
        and endpoint_gap <= max_gap        # 0.2 m
        and residual <= merge_residual_tol):  # 0.15 m
    merged[-1] = np.vstack([current, seg])
```

After incremental growing, a single physical wall segment may have been split by a momentary noise spike or a minor surface irregularity. The merge step repairs these splits by joining adjacent segments that satisfy three simultaneous conditions:

1. **Angular consistency:** $|\angle(\hat{\mathbf{d}}_a, \hat{\mathbf{d}}_b)| \leq \delta_\theta = 0.22\,\text{rad}$
   where the angle is computed as $\arccos(|\hat{\mathbf{d}}_a \cdot \hat{\mathbf{d}}_b|)$ (acute angle, direction-invariant).

2. **Spatial continuity:** $\|\mathbf{p}_{a,\text{last}} - \mathbf{p}_{b,\text{first}}\| \leq \delta_{\text{gap}} = 0.2\,\text{m}$

3. **Combined residual:** The TLS residual of the merged candidate does not exceed $\delta_{\text{merge}} = 0.15\,\text{m}$, ensuring that merging two compatible segments does not produce an implausibly curved line.

A segment qualifies as a valid line if it contains at least `min_points = 10` points and spans a minimum length of `min_length = 0.3 m`.

### 3.5 Hessian Normal Form Parameterisation

Each valid segment is converted to the Hessian normal form $(\rho, \alpha)$:

$$\rho = \mathbf{p} \cdot \hat{\mathbf{n}}, \quad \alpha = \text{atan2}(\hat{n}_y, \hat{n}_x)$$

where $\hat{\mathbf{n}}$ is the TLS normal from Section 3.3 and $\mathbf{p}$ is any point on the line (the centroid is used). The uniqueness constraint $\rho \geq 0$ is enforced: if $\rho < 0$, both $\rho$ and $\hat{\mathbf{n}}$ are negated (flipping $\alpha$ by $\pi$), yielding the canonical form.

```python
# landmark_features.py:246 — convert_line_to_hessian()
centroid, _, normal = self.fit_line_tls(points)
rho = float(np.dot(centroid, normal))
if rho < 0.0:
    rho = -rho
    normal = -normal
alpha = float(np.arctan2(normal[1], normal[0]))
```

The Hessian form is non-redundant (only two parameters define the infinite line), compact, and analytically differentiable — properties essential for EKF Jacobian computation. The full geometric structure of the segment is recovered via the arc-length parameterisation:

$$\mathbf{p}(t) = \rho\hat{\mathbf{n}} + t\hat{\boldsymbol{\tau}}, \quad \hat{\boldsymbol{\tau}} = [-\sin\alpha, \cos\alpha]^\top$$

Wall spatial extent is stored as the scalar arc-length bounds $[t_{\min}, t_{\max}]$ along $\hat{\boldsymbol{\tau}}$ (Section 9.1), making endpoint geometry invariant to EKF parameter updates.

### 3.6 Wall Covariance via Fisher Information

The measurement covariance for a wall feature is derived from the Cramér–Rao Lower Bound (CRLB). Under the noise model that each LiDAR point $\mathbf{p}_i = (x_i, y_i)$ is corrupted by isotropic Gaussian noise with variance $\sigma^2 = (0.01\,\text{m})^2$, the constraint equation is:

$$f(\mathbf{p}_i; \rho, \alpha) = x_i\cos\alpha + y_i\sin\alpha - \rho = 0$$

Differentiating with respect to the wall parameters:

$$\frac{\partial f}{\partial \rho} = -1, \quad \frac{\partial f}{\partial \alpha} = -x_i\sin\alpha + y_i\cos\alpha \equiv t_i$$

where $t_i$ is the arc-length (tangential position) of point $i$ along the wall. The per-point Jacobian is:

$$\mathbf{J}_i = \begin{bmatrix} 1 & t_i \end{bmatrix} \in \mathbb{R}^{1 \times 2}$$

The Fisher Information Matrix accumulates contributions from all $n$ supporting points:

$$\mathbf{A} = \sum_{i=1}^{n} \mathbf{J}_i^\top \mathbf{J}_i = \begin{bmatrix} n & \sum t_i \\ \sum t_i & \sum t_i^2 \end{bmatrix}$$

The CRLB gives the measurement covariance as:

$$\boldsymbol{\Sigma}_{\text{wall}} = \sigma^2 \mathbf{A}^{-1}$$

```python
# landmark_features.py:326 — compute_wall_covariance()
for px, py in points:
    dr_d_alpha = px * sin_a - py * cos_a   # = t_i
    J = np.array([[1.0, dr_d_alpha]])
    A += J.T @ J
cov = sigma2 * A_inv
```

The matrix $\mathbf{A}$ has a clear geometric interpretation: the diagonal element $\sum t_i^2$ represents the second moment of the point distribution along the wall tangent. A short wall (small spread) yields a large $\alpha$-covariance, reflecting the fact that the normal angle of a short line is poorly constrained. Numerically, the minimum eigenvalue of $\mathbf{A}$ is clamped to $\sigma^2 \cdot 0.1$ to prevent inversion of near-singular matrices when points are concentrated near the midpoint.

### 3.7 Corner Detection

```python
# landmark_features.py:299 — extract_corners_from_adjacent_lines()
for i in range(1, len(lines)):
    left, right = lines[i-1], lines[i]
    angle = acute_angle_between_dirs(left['direction'], right['direction'])
    if angle < self.corner_angle:    # 50° threshold
        continue
    corner_pos = compute_line_intersection(left, right)
```

Corners are extracted only between **adjacent** wall segments (consecutive in the scan ordering). This restriction is intentional: non-adjacent walls may be spatially close but structurally unrelated, and attempting to find their intersection produces spurious corner candidates. The constraint that the two lines share a common measurement arc (there are scan points on both sides of the corner) is a necessary condition for the intersection to correspond to an actual architectural corner.

A corner is detected when the acute angle between the two wall directions exceeds the threshold $\delta_{\text{corner}} = 50°$, which eliminates near-collinear walls while accepting T-junctions, L-corners, and obtuse corners.

**Line Intersection:** Given walls $(\rho_a, \alpha_a)$ and $(\rho_b, \alpha_b)$, the intersection solves the $2 \times 2$ linear system:

$$\begin{bmatrix} \cos\alpha_a & \sin\alpha_a \\ \cos\alpha_b & \sin\alpha_b \end{bmatrix} \begin{bmatrix} x_c \\ y_c \end{bmatrix} = \begin{bmatrix} \rho_a \\ \rho_b \end{bmatrix}$$

The determinant $\det(\mathbf{A}) = \cos\alpha_a\sin\alpha_b - \sin\alpha_a\cos\alpha_b = \sin(\alpha_b - \alpha_a)$ is checked against a threshold of $10^{-6}$; nearly parallel walls (small angle difference) are rejected. If the intersection fails, the corner position defaults to the midpoint between the two segment endpoints.

### 3.8 Corner Covariance via Jacobian Propagation

The corner position $\mathbf{c} = [x_c, y_c]^\top$ is a function of four wall parameters $\boldsymbol{\theta} = [\rho_a, \alpha_a, \rho_b, \alpha_b]^\top$. Given $\mathbf{c} = \mathbf{A}^{-1}\mathbf{b}$ where $\mathbf{A}$ depends on $\alpha_a, \alpha_b$ and $\mathbf{b} = [\rho_a, \rho_b]^\top$, the partial derivatives are:

$$\frac{\partial \mathbf{c}}{\partial \rho_a} = \mathbf{A}^{-1}\mathbf{e}_1, \quad
\frac{\partial \mathbf{c}}{\partial \rho_b} = \mathbf{A}^{-1}\mathbf{e}_2$$

$$\frac{\partial \mathbf{c}}{\partial \alpha_a} = -\mathbf{A}^{-1}\frac{\partial \mathbf{A}}{\partial \alpha_a}\mathbf{c}, \quad
\frac{\partial \mathbf{c}}{\partial \alpha_b} = -\mathbf{A}^{-1}\frac{\partial \mathbf{A}}{\partial \alpha_b}\mathbf{c}$$

using the matrix differentiation identity $\partial(\mathbf{A}^{-1}\mathbf{b})/\partial\theta_k = \mathbf{A}^{-1}(\partial\mathbf{b}/\partial\theta_k) - \mathbf{A}^{-1}(\partial\mathbf{A}/\partial\theta_k)\mathbf{c}$.

The partial derivatives of $\mathbf{A}$ are:
$$\frac{\partial \mathbf{A}}{\partial \alpha_a} = \begin{bmatrix} -\sin\alpha_a & \cos\alpha_a \\ 0 & 0 \end{bmatrix}, \quad
\frac{\partial \mathbf{A}}{\partial \alpha_b} = \begin{bmatrix} 0 & 0 \\ -\sin\alpha_b & \cos\alpha_b \end{bmatrix}$$

The full $2 \times 4$ Jacobian is assembled column-wise and the corner covariance computed by first-order error propagation:

$$\boldsymbol{\Sigma}_c = \mathbf{J} \begin{bmatrix} \boldsymbol{\Sigma}_a & \mathbf{0} \\ \mathbf{0} & \boldsymbol{\Sigma}_b \end{bmatrix} \mathbf{J}^\top$$

```python
# landmark_features.py:386 — compute_corner_covariance()
dx_dalpha1 = -A_inv @ (dA_dalpha1 @ x)
dx_dalpha2 = -A_inv @ (dA_dalpha2 @ x)
J = np.column_stack([dx_drho1, dx_dalpha1, dx_drho2, dx_dalpha2])
cov_corner = J @ Sigma_theta @ J.T
```

Numerical conditioning: the determinant of $\mathbf{A}$ is checked against $0.1$ (stricter than the intersection threshold) before inversion, rejecting poorly-conditioned corner estimates. The output covariance is symmetrised and its eigenvalues are clamped from below at $\sigma^4$.

---

## 4. EKF State and Prediction

**Files:** `ekf_predict.py` — `class BaseEKF`, `ekf_update_feature.py` — `class LandmarkEKFSLAM(BaseEKF)`

### 4.1 State Vector and Covariance

The EKF maintains a joint state vector over robot pose and all active landmarks:

$$\mathbf{x} = \begin{bmatrix} x_r \\ y_r \\ \theta_r \\ \rho_1 \\ \alpha_1 \\ \vdots \\ \rho_M \\ \alpha_M \\ x_{c_1} \\ y_{c_1} \\ \vdots \\ x_{c_N} \\ y_{c_N} \end{bmatrix} \in \mathbb{R}^{3+2M+2N}$$

where $M$ is the number of active wall landmarks and $N$ the number of active corner landmarks. The full covariance matrix $\mathbf{P} \in \mathbb{R}^{n \times n}$ (where $n = 3 + 2M + 2N$) captures all pairwise uncertainties, including **cross-covariances** between robot pose and each landmark and between different landmarks. These cross-terms are essential: a single landmark observation corrects not only that landmark's state but also, through the Kalman update, the robot pose and all correlated landmarks.

Initial pose covariance: $\mathbf{P}_0 = 0.01 \cdot \mathbf{I}_3$ (small, reflecting known initial conditions).

### 4.2 Motion Model (Midpoint Integration)

The motion model takes wheel odometry increments $(\Delta d, \Delta\theta)$ from consecutive odometry messages. Rather than applying the heading $\theta_r$ at the start of the motion step, the **midpoint heading** $\theta_{\text{mid}} = \theta_r + \Delta\theta/2$ is used. This second-order integration scheme reduces the linearisation error for large angular increments:

$$x_r' = x_r + \Delta d \cos\left(\theta_r + \frac{\Delta\theta}{2}\right)$$
$$y_r' = y_r + \Delta d \sin\left(\theta_r + \frac{\Delta\theta}{2}\right)$$
$$\theta_r' = \theta_r + \Delta\theta$$

```python
# ekf_predict.py:39 — predict_with_relative_motion()
theta_mid = theta + delta_theta / 2.0
x_new = x + delta_d * np.cos(theta_mid)
y_new = y + delta_d * np.sin(theta_mid)
```

The angle is immediately normalised: $\theta \mapsto \text{atan2}(\sin\theta, \cos\theta)$ to prevent wrap-around errors.

### 4.3 Process Noise Jacobians

The linearised motion model requires two Jacobians:

**$\mathbf{F}$: Jacobian of $f$ with respect to the full state** (only the robot pose rows are non-trivial; landmarks are constant):

$$\mathbf{F} = \begin{bmatrix} 1 & 0 & -\Delta d\sin\theta_{\text{mid}} & 0 & \cdots \\ 0 & 1 & \phantom{-}\Delta d\cos\theta_{\text{mid}} & 0 & \cdots \\ 0 & 0 & 1 & 0 & \cdots \\ \vdots & & & \mathbf{I}_{2M+2N} \end{bmatrix}$$

**$\mathbf{G}$: Jacobian of $f$ with respect to the noise inputs $(\delta_d, \delta_\theta)$:**

$$\mathbf{G}_{[3 \times 2]} = \begin{bmatrix} \cos\theta_{\text{mid}} & -\frac{\Delta d}{2}\sin\theta_{\text{mid}} \\ \sin\theta_{\text{mid}} & \phantom{-}\frac{\Delta d}{2}\cos\theta_{\text{mid}} \\ 0 & 1 \end{bmatrix}$$

(Only the robot-pose rows of $\mathbf{G}$ are non-zero; landmark rows are zero.)

### 4.4 Covariance Prediction and Conditioning

The process noise is scaled by the magnitude of the motion to avoid artificially growing uncertainty during stationary periods:

$$\sigma_d^2 = q_d \cdot (\Delta d)^2 + \epsilon_d, \quad \sigma_\theta^2 = q_\theta \cdot (\Delta\theta)^2 + \epsilon_\theta$$

with minimum variance floors $\epsilon_d = (0.01\,\text{m})^2$, $\epsilon_\theta = (0.001\,\text{rad})^2$ to ensure the EKF remains observable even when stationary. The noise coefficients are $q_d = 0.01$, $q_\theta = 0.005$.

The predicted covariance:
$$\mathbf{P}^- = \mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{G}\mathbf{Q}_{\text{scaled}}\mathbf{G}^\top$$

After each prediction step, the covariance is **conditioned**:
1. Symmetrised: $\mathbf{P} \leftarrow (\mathbf{P} + \mathbf{P}^\top)/2$
2. Eigendecomposed: $\mathbf{P} = \mathbf{V}\boldsymbol{\Lambda}\mathbf{V}^\top$
3. Eigenvalues clamped: $\lambda_i \leftarrow \text{clip}(\lambda_i, 10^{-6}, 100)$
4. Regularised: $\mathbf{P} \leftarrow \mathbf{V}\boldsymbol{\Lambda}_{\text{clamped}}\mathbf{V}^\top + 10^{-9}\mathbf{I}$

This maintains strict positive-definiteness, prevents numerically singular covariances, and bounds the maximum representable uncertainty.

---

## 5. Landmark Initialisation

**File:** `ekf_update_feature.py:29` — `add_landmark()`

When a feature is unmatched by data association (Section 6), it is added to the EKF state as a new landmark. The state vector and covariance matrix are augmented to accommodate two new states.

### 5.1 Wall Landmark Augmentation

A wall observation $(\rho_r, \alpha_r)$ in robot frame is transformed to map frame using the current EKF robot state $(x_r, y_r, \theta_r)$:

$$\alpha_m = \text{atan2}(\sin(\alpha_r + \theta_r), \cos(\alpha_r + \theta_r))$$
$$\rho_m = \rho_r + x_r\cos\alpha_m + y_r\sin\alpha_m$$

with the normalisation $\rho_m \geq 0$ enforced as usual. This implements `robot_wall_to_map_frame()` in `transform_utils.py`.

The **initialisation Jacobian with respect to robot pose** $\mathbf{H}_r \in \mathbb{R}^{2 \times 3}$:

$$\mathbf{H}_r = \begin{bmatrix} \cos\alpha_m & \sin\alpha_m & -x_r\sin\alpha_m + y_r\cos\alpha_m \\ 0 & 0 & 1 \end{bmatrix}$$

where the partial $\partial\rho_m / \partial\theta_r = -x_r\sin\alpha_m + y_r\cos\alpha_m$ arises from differentiating the angle transformation.

The **Jacobian with respect to the observation** $\mathbf{H}_z \in \mathbb{R}^{2 \times 2}$:

$$\mathbf{H}_z = \begin{bmatrix} 1 & -x_r\sin\alpha_m + y_r\cos\alpha_m \\ 0 & 1 \end{bmatrix}$$

### 5.2 Corner Landmark Augmentation

A corner at robot-frame position $(z_x, z_y)$ maps to map frame by a full 2D rigid transform:

$$\begin{bmatrix} l_x \\ l_y \end{bmatrix} = \begin{bmatrix} x_r \\ y_r \end{bmatrix} + \begin{bmatrix} \cos\theta_r & -\sin\theta_r \\ \sin\theta_r & \cos\theta_r \end{bmatrix}\begin{bmatrix} z_x \\ z_y \end{bmatrix}$$

Jacobian with respect to robot pose $\mathbf{H}_r \in \mathbb{R}^{2 \times 3}$:
$$\mathbf{H}_r = \begin{bmatrix} 1 & 0 & -\sin\theta_r z_x - \cos\theta_r z_y \\ 0 & 1 & \phantom{-}\cos\theta_r z_x - \sin\theta_r z_y \end{bmatrix}$$

Jacobian with respect to observation $\mathbf{H}_z \in \mathbb{R}^{2 \times 2}$:
$$\mathbf{H}_z = \begin{bmatrix} \cos\theta_r & -\sin\theta_r \\ \sin\theta_r & \phantom{-}\cos\theta_r \end{bmatrix}$$

### 5.3 Cross-Covariance Seeding

The new landmark's marginal covariance (capturing uncertainty from both the robot pose uncertainty and the measurement noise) is:

$$\mathbf{P}_{ll} = \mathbf{H}_r \mathbf{P}_{rr} \mathbf{H}_r^\top + \mathbf{H}_z \mathbf{R} \mathbf{H}_z^\top$$

where $\mathbf{P}_{rr} = \mathbf{P}[0:3, 0:3]$ is the current robot pose covariance and $\mathbf{R}$ is the feature measurement covariance from Section 3.6 or 3.8.

The cross-covariance between the new landmark and all existing states:

$$\mathbf{P}_{xl} = \mathbf{P}[:, 0:3] \cdot \mathbf{H}_r^\top$$

The augmented covariance matrix is assembled by block expansion:

$$\mathbf{P}' = \begin{bmatrix} \mathbf{P} & \mathbf{P}_{xl} \\ \mathbf{P}_{xl}^\top & \mathbf{P}_{ll} \end{bmatrix}$$

```python
# ekf_update_feature.py:90 — add_landmark()
P_lm = H_r @ P_rr @ H_r.T + H_z @ R_obs @ H_z.T
P_rl = self.P[:, 0:3] @ H_r.T
P_new[0:n_old, n_old:n_old+2] = P_rl
P_new[n_old:n_old+2, n_old:n_old+2] = P_lm
```

The landmark is registered with `observations = 1`. It will not qualify for data association until `min_observations_for_init = 2` have accumulated.

---

## 6. Data Association

**File:** `data_association.py` — `associate_landmarks()`

Data association maps each observed feature to an existing EKF landmark (or flags it as unmatched). A correct association is critical: an incorrect match inserts a false innovation into the EKF, corrupting both the pose and map estimates in ways that may not be recoverable. The implementation uses a multi-stage nearest-neighbour pipeline with four sequential gates.

### 6.1 Type and Euclidean Gating

```python
# data_association.py:39
if feature['type'] != lm_data['feature_type']:
    continue

if feature['type'] == 'wall':
    rho_pred = lm_rho - (x_r * cos(lm_alpha) + y_r * sin(lm_alpha))
    if abs(rho_pred) > max_euclidean_dist:   # 6.0 m
        continue
    if alpha_diff > wall_angle_tolerance:    # 0.349 rad ≈ 20°
        continue
    if abs(lm_rho - obs_rho_map) > wall_rho_tolerance:  # 0.5 m
        continue
```

**Type gate:** An observed wall is only compared against wall landmarks and an observed corner against corner landmarks.

**Wall pre-filters:** Before computing the full Mahalanobis distance (which requires a matrix inversion), three cheap scalar gates reduce the candidate set:
1. **Range gate:** The predicted observation range $|\rho_{\text{pred}}|$ must be within 6.0 m.
2. **Angle gate:** The angular difference $|\alpha_{\text{obs}} - \alpha_{\text{pred}}|$ must be within 20°. This prevents matching walls with similar distance but different orientation.
3. **Map-frame rho gate:** The observed wall, transformed to map frame, must have $|\rho_m^{\text{obs}} - \rho_m^{\text{stored}}| \leq 0.5\,\text{m}$. This eliminates walls that are angularly similar but on opposite sides of the environment.

### 6.2 Predicted Observation and Jacobian

For each landmark that passes the pre-filter, the predicted observation and full Jacobian are computed via `_build_observation()`.

**Wall prediction** (map frame → robot frame):
$$\hat{\rho}_r = \rho_m - (x_r\cos\alpha_m + y_r\sin\alpha_m)$$
$$\hat{\alpha}_r = \text{atan2}(\sin(\alpha_m - \theta_r), \cos(\alpha_m - \theta_r))$$

Wall observation Jacobian $\mathbf{H} \in \mathbb{R}^{2 \times n}$:

$$\mathbf{H}_{[0, :]} = \begin{bmatrix} \underbrace{-\cos\alpha_m, & -\sin\alpha_m, & 0}_{\partial/\partial(x_r, y_r, \theta_r)}, & \cdots, & \underbrace{1, & \partial\rho/\partial\alpha_m}_{\partial/\partial(\rho_m, \alpha_m)}, & \cdots \end{bmatrix}$$

$$\mathbf{H}_{[1, :]} = \begin{bmatrix} 0, & 0, & -1, & \cdots, & 0, & 1, & \cdots \end{bmatrix}$$

where $\partial\rho_r/\partial\alpha_m = x_r\sin\alpha_m - y_r\cos\alpha_m$.

**Corner prediction** (map frame → robot frame):
$$\hat{\mathbf{z}} = \mathbf{R}(-\theta_r)(\mathbf{l} - \mathbf{r})$$

where $\mathbf{l} = [l_x, l_y]^\top$ is the landmark map position and $\mathbf{r} = [x_r, y_r]^\top$ is the robot position. The rotation matrix is evaluated at $-\theta_r$ (map to robot).

Corner observation Jacobian, using $c = \cos\theta_r$, $s = \sin\theta_r$, $dx = l_x - x_r$, $dy = l_y - y_r$:

$$\mathbf{H}_{[0, :3]} = \begin{bmatrix} -c, & -s, & -s\,dx + c\,dy \end{bmatrix}$$
$$\mathbf{H}_{[1, :3]} = \begin{bmatrix} \phantom{-}s, & -c, & -c\,dx - s\,dy \end{bmatrix}$$

with landmark columns $\mathbf{H}_{[:, \text{lm\_idx}]} = \mathbf{R}(-\theta_r)$.

### 6.3 Mahalanobis Distance Gate

The Mahalanobis distance squared:

$$D_M^2 = \boldsymbol{\nu}^\top \mathbf{S}^{-1} \boldsymbol{\nu}$$

where the innovation $\boldsymbol{\nu} = \mathbf{z}_{\text{obs}} - \hat{\mathbf{z}}$ (with angle-wrap for walls) and the innovation covariance $\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$.

The gate threshold $D_M^2 < 5.99$ corresponds to the 95th percentile of the $\chi^2(2)$ distribution (2 degrees of freedom). This gate is **joint**: it accounts for both the observation uncertainty $\mathbf{R}$ and the landmark prediction uncertainty $\mathbf{H}\mathbf{P}^-\mathbf{H}^\top$, making it robust to large landmark uncertainty during initialisation.

### 6.4 Wall Segment Overlap/Gap Validation

```python
# data_association.py:99 — gap check
if wall_data and wall_data.get('t_min') is not None:
    sto_lo = wall_data['t_min']       # stored arc-length min (tangent coords)
    sto_hi = wall_data['t_max']       # stored arc-length max
    obs_lo = min(dot(obs_s, tangent), dot(obs_e, tangent))
    obs_hi = max(dot(obs_s, tangent), dot(obs_e, tangent))
    gap = max(0.0, max(obs_lo - sto_hi, sto_lo - obs_hi))
    if gap > max_gap_ext:             # 0.5 m
        continue
```

Even a wall that passes the Mahalanobis gate may be a different physical wall if the two observations are spatially disjoint. This step checks whether the observed wall segment's arc-length projection onto the stored wall's tangent direction overlaps or is within `max_gap_ext = 0.5 m` of the stored extent. The stored extent $[t_{\min}, t_{\max}]$ is directly available in the `feature_map.walls` dictionary (no re-projection needed, since the refactored storage uses tangential scalars). The observed endpoints are first transformed to map frame using the current robot pose, then projected onto the stored wall tangent.

This check is critical in multi-room environments where two parallel walls at the same distance but separated by a doorway would otherwise both pass the Mahalanobis gate.

### 6.5 Best-Match Selection

For each observed feature, the candidate landmark with the **lowest Mahalanobis distance** among all that pass all gates is selected. This implements the nearest-neighbour principle. If no candidate passes all gates, the feature is placed in the unmatched list. Each observed feature is matched to at most one landmark; no exclusion zone prevents two features from matching the same landmark (an approximation that works well in practice for the density of features in structured indoor environments).

---

## 7. EKF Update

**File:** `ekf_update_feature.py:189` — `update_landmark_observation()`

### 7.1 Innovation Computation

$$\boldsymbol{\nu} = \mathbf{z}_{\text{obs}} - \hat{\mathbf{z}}_{\text{pred}}$$

For wall observations, the angular component $\nu_\alpha$ is normalised: $\nu_\alpha \leftarrow \text{atan2}(\sin\nu_\alpha, \cos\nu_\alpha)$. This prevents the innovation from exceeding $\pi$ in magnitude when the true difference is small but the raw subtraction crosses the $\pm\pi$ boundary.

### 7.2 Outlier Rejection

Before computing the Kalman gain, a second Mahalanobis test is applied at a wider threshold:

$$D_M^2 = \boldsymbol{\nu}^\top \mathbf{S}^{-1}\boldsymbol{\nu} > 13.8 \implies \text{reject}$$

The threshold 13.8 corresponds to the 99.7th percentile of $\chi^2(2)$, roughly equivalent to a $3\sigma$ test in 2D. This serves as a final safeguard against data association errors that passed the 95% gate.

### 7.3 Kalman Gain

$$\mathbf{S} = \mathbf{H}\mathbf{P}^-\mathbf{H}^\top + \mathbf{R}$$
$$\mathbf{K} = \mathbf{P}^-\mathbf{H}^\top\mathbf{S}^{-1}$$

$\mathbf{K} \in \mathbb{R}^{n \times 2}$. The full state-size Kalman gain is necessary because a single observation updates all $n$ states through the cross-covariance structure of $\mathbf{P}$.

### 7.4 State Update

$$\mathbf{x} \leftarrow \mathbf{x}^- + \mathbf{K}\boldsymbol{\nu}$$

The robot heading is immediately normalised after the update to prevent wrap-around accumulation.

### 7.5 Rho Normalisation (Cross-Landmark)

After the state update, the EKF applies rho normalisation to **all** wall landmarks:

```python
# ekf_update_feature.py:252 — normalise all walls after every update
for lm_data in self.landmarks.values():
    if lm_data['feature_type'] == 'wall':
        i = lm_data['state_index']
        if self.state[i] < 0.0:
            self.state[i] = -self.state[i]
            self.state[i+1] = normalize_angle(self.state[i+1] + np.pi)
```

This is a critical correctness requirement. The EKF update modifies the entire state vector, including wall landmarks that were **not** directly observed. Through cross-covariance coupling, updating wall landmark $j$ can perturb the $\rho$ value of wall landmark $k$, potentially driving it negative. Without normalisation, $k$'s $\alpha$ value would be off by $\pi$ on the next scan, causing a 180° angle prediction error that fails the angle gate in data association and forces $k$ to be treated as a new landmark. This was the root cause of landmark IDs reaching 800–900 in the unpatched system.

The cross-landmark normalisation loop runs after every EKF update and is $O(M)$ — negligible cost for typical landmark counts of 10–50.

### 7.6 Joseph-Form Covariance Update

The standard update $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$ is numerically fragile: small floating-point errors in $\mathbf{K}$ can accumulate and render $\mathbf{P}$ non-positive-semidefinite. The Joseph form avoids this:

$$\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}\mathbf{K}^\top$$

This form is equivalent to the standard update when $\mathbf{K}$ is the optimal Kalman gain but guarantees $\mathbf{P} \succeq 0$ for any $\mathbf{K}$, providing robustness against numerical errors and approximate gains.

```python
# ekf_update_feature.py:261
I_KH = np.eye(n) - K @ H
self.P = I_KH @ self.P @ I_KH.T + K @ measurement_covariance @ K.T
self.P = self._condition_covariance(self.P)
```

After the Joseph form update, covariance conditioning (Section 4.4) is applied to maintain strict positive-definiteness.

---

## 8. Landmark Pruning

**File:** `ekf_update_feature.py:269` — `prune_landmarks()`

Two conditions trigger removal of a landmark:

1. **Timeout:** `current_scan - last_seen > landmark_timeout_scans = 25`. A landmark not re-observed within 25 scans (2.5 seconds at 10 Hz) is considered out of the sensor's field of view or spurious.

2. **Provisional timeout:** `observations < min_observations_for_init = 2` and `current_scan - last_seen > 10`. A newly added landmark that has not been confirmed by a second independent observation within 10 scans is removed. This prevents fleeting reflections and noise spikes from polluting the landmark map.

Removal is performed in **reverse state-index order** to avoid index-shifting side effects when deleting rows and columns from $\mathbf{P}$:

```python
# ekf_update_feature.py:281
to_remove.sort(key=lambda lm_id: self.landmarks[lm_id]['state_index'], reverse=True)
for lm_id in to_remove:
    self._remove_landmark(lm_id)
```

Each removal deletes 2 rows and 2 columns from $\mathbf{P}$ and 2 elements from the state vector, then decrements the `state_index` of all subsequent landmarks by 2. The corresponding entry is also removed from the `FeatureMap` via `feature_map.remove_landmark()` in the SLAM manager.

---

## 9. Feature Map

**File:** `feature_map.py` — `class FeatureMap`

The `FeatureMap` is a geometric layer that stores the spatial extent of each wall and corner landmark independently of the EKF state vector. While the EKF tracks only the Hessian parameters $(\rho, \alpha)$ and the Cartesian position $(x_c, y_c)$, the `FeatureMap` also tracks the observable length of each wall through its arc-length extents.

### 9.1 Wall Storage with Arc-Length Extents

Each wall is stored as:
```python
walls[landmark_id] = {
    'rho':   rho,     # Hessian distance (EKF-corrected)
    'alpha': alpha,   # Hessian angle (EKF-corrected)
    't_min': t_min,   # arc-length of near endpoint along tangent τ̂
    't_max': t_max,   # arc-length of far endpoint along tangent τ̂
    'observation_count': k
}
```

The tangential extents $t_{\min}, t_{\max}$ are defined relative to the wall tangent vector $\hat{\boldsymbol{\tau}} = [-\sin\alpha, \cos\alpha]^\top$. A 2D endpoint is recovered as:

$$\mathbf{p}_{\text{start}} = \rho\hat{\mathbf{n}} + t_{\min}\hat{\boldsymbol{\tau}}, \quad \mathbf{p}_{\text{end}} = \rho\hat{\mathbf{n}} + t_{\max}\hat{\boldsymbol{\tau}}$$

This representation is **invariant to Hessian updates**: when `update_wall_hessian()` modifies $(\rho, \alpha)$, the extents $t_{\min}, t_{\max}$ remain unchanged. This is the key property that eliminates the wall endpoint drift problem — previously, storing 2D endpoint vectors meant that updating $(\rho, \alpha)$ left the stored endpoints inconsistent with the new line equation, causing geometric bias in overlap checks and point cloud generation.

When a new observation extends the wall:
```python
# feature_map.py:51 — update_wall_endpoints()
new_t_lo = min(dot(new_start, tangent), dot(new_end, tangent))
new_t_hi = max(dot(new_start, tangent), dot(new_end, tangent))
wall['t_min'] = min(wall['t_min'], new_t_lo)
wall['t_max'] = max(wall['t_max'], new_t_hi)
```

The incoming endpoints (in map frame) are projected onto the wall's current tangent to obtain their arc-length positions, and the stored extent is expanded to the union. **Extents are never reset**: they accumulate monotonically across all submap boundaries, giving the most complete and up-to-date picture of each wall's observed spatial coverage at all times.

### 9.2 Hessian Synchronisation

After every batch of EKF updates, the SLAM manager syncs the EKF state back to the `FeatureMap`:

```python
# feature_slam_manager.py:172 — full sync after update batch
for lm_id, lm_data in self.ekf.landmarks.items():
    if lm_data['feature_type'] == 'wall' and lm_id in self.feature_map.walls:
        lm_idx = lm_data['state_index']
        self.feature_map.update_wall_hessian(
            lm_id,
            rho=float(self.ekf.state[lm_idx]),
            alpha=float(self.ekf.state[lm_idx + 1])
        )
```

This full sync (not just the matched wall) is necessary because the cross-landmark rho normalisation (Section 7.5) may have modified the $(\rho, \alpha)$ parameters of unmatched walls during the EKF update. Without this sync, `feature_map.walls` would retain stale Hessian parameters for those walls, causing a 180° angle prediction error on the next scan.

### 9.3 Point Cloud Generation

`generate_point_cloud(spacing=0.05)` reconstructs 2D line segments from the stored $(\rho, \alpha, t_{\min}, t_{\max})$ tuples and samples them at 5 cm spacing:

```python
# feature_map.py:105
tangent = np.array([-np.sin(alpha), np.cos(alpha)])
normal  = np.array([ np.cos(alpha), np.sin(alpha)])
line_pt = rho * normal
start   = line_pt + t_min * tangent
end     = line_pt + t_max * tangent
```

Walls with `t_min is None` are skipped — this guard covers only the brief cold-start period before a wall's very first observation; it never fires at submap boundaries since extents are never reset. Corner positions are appended as single points. The result is an $(N_{\text{pts}} \times 3)$ array suitable for passing directly to `SubmapStitcher`.

---

## 10. SLAM Manager Orchestration

**File:** `feature_slam_manager.py` — `class FeatureSLAMManager`

### 10.1 Per-Scan Processing Pipeline

```python
# feature_slam_manager.py:59 — process_scan()
observed_features = self.feature_extractor.extract_features(scan_msg)
matched, unmatched, extension_info = associate_landmarks(
    observed_features, self.ekf, ..., return_extension_info=True,
    feature_map=self.feature_map, max_gap_ext=self.max_gap_ext
)
self._process_matched_features(observed_features, matched, extension_info)
self._process_unmatched_features(observed_features, unmatched)
self._prune_old_landmarks()
```

### 10.2 Matched Feature Processing and Full Hessian Sync

For each matched `(feat_idx, landmark_id)` pair:

1. **EKF update:** `update_landmark_observation()` with the feature's measurement covariance.
2. **Corner map update:** The EKF-corrected corner position is read back from the state vector and stored in the `FeatureMap` (the EKF is the authoritative source for corner positions; the `FeatureMap` stores only the most recent EKF estimate).
3. **Wall endpoint extension:** If `feat_idx` is in `extension_info`, the new map-frame start and end points (pre-computed in `data_association.py`) are passed to `update_wall_endpoints()`.

After all matched features have been processed (not inside the loop), a full Hessian sync is performed to propagate any rho-normalisation flips to the `FeatureMap` (Section 9.2).

### 10.3 Unmatched Feature Insertion

```python
# feature_slam_manager.py:193 — _add_wall_landmark()
landmark_id = self.ekf.add_landmark(z_x=rho, z_y=alpha, ...)
rho_map, alpha_map = robot_wall_to_map_frame(rho, alpha, x_r, y_r, theta_r)
start_map = rotate_point_2d(feature['start_point'], x_r, y_r, theta_r)
end_map   = rotate_point_2d(feature['end_point'],   x_r, y_r, theta_r)
self.feature_map.add_wall(landmark_id, rho_map, alpha_map, start_map, end_map)
```

For wall landmarks: the robot-frame Hessian parameters are transformed to map frame (Section 5.1), and the robot-frame endpoints are rotated to map frame using `rotate_point_2d()`. Both are inserted into the `FeatureMap`, which internally converts the 2D endpoints to $t_{\min}, t_{\max}$ arc-lengths.

For corner landmarks: the EKF state index is immediately read back after `add_landmark()` to obtain the map-frame position (already computed internally by the EKF during initialisation).

---

## 11. Submap Management and Stitching

**Files:** `local_submap_generator_feature.py`, `submap_stitcher.py`

### 11.1 Submap Trigger and Point Cloud Generation

Every `scans_per_submap = 50` scan callbacks that produce at least one feature, `create_submap_from_features()` is triggered. The submap boundary condition is based purely on scan count, not on distance travelled, ensuring a consistent update rate regardless of robot velocity.

```python
# local_submap_generator_feature.py:439
points_map_frame = self.slam_manager.generate_point_cloud(spacing=0.05)
```

The point cloud is generated entirely from the `FeatureMap` (Section 9.3). Points are already in the EKF map frame — no additional transformation is required.

### 11.2 Persistent Wall Extents Across Submap Boundaries

Wall arc-length extents ($t_{\min}$, $t_{\max}$) in `feature_map.walls` are **never reset** at submap boundaries. They accumulate monotonically across the full operational lifetime of each landmark: every new observation of a wall can only expand its stored extent, never shrink it.

This design has three direct benefits:

1. **Gap check is always active.** The spatial proximity check in `data_association.py` (Section 6.4) requires valid extents to function. With persistent extents, this check is active from the very first re-observation of any wall onward — there is no vulnerability window at submap boundaries where the check is silently bypassed.

2. **SVD alignment has geometry immediately.** The overlap computation in `_compute_overlap_correspondences()` requires $t_{\min}$/$t_{\max}$ from both `feature_map.walls` (source) and `global_walls` (target). With persistent extents, overlap correspondences are computable from the first scan of a new submap window, not only after several scans have rebuilt the extents.

3. **Simplified architecture.** The distinction between `feature_map.walls` and `global_walls` is now purely temporal: `feature_map.walls` holds the EKF's current (potentially drifted) estimate of each wall, and `global_walls` holds the last SVD-corrected reference. The `t_min is None` guard in `generate_point_cloud()` and `data_association.py` remains as a cold-start guard for the initial period before any wall has been observed at all, but never fires during normal submap transitions.

Confidence values attached to each submap are computed from the summed landmark information (trace of each landmark covariance inverse) at the submap boundary — they are independent of wall extent geometry and are unaffected by this design choice.

### 11.3 Global Wall Registry

`SubmapStitcher.global_walls` maps `landmark_id → {rho, alpha, t_min, t_max}`, storing the last SVD-corrected landmark parameters for each wall. This registry is not a separate coordinate frame — it lives in the same map frame as `feature_map.walls`. It persists across all submap boundaries. For the first submap, walls are seeded directly from the `FeatureMap` (no correction applied yet). For subsequent submaps, the registry is updated (extents extended or new walls inserted) after applying the SVD drift correction.

The registry enables feature-based alignment without re-matching: landmark IDs are permanent within the EKF lifetime, so the intersection `feature_map.walls.keys() ∩ global_walls.keys()` immediately identifies shared walls without a descriptor search.

### 11.4 Feature-Based SVD Alignment

**File:** `submap_stitcher.py:138` — `feature_align_submaps()`

For each shared wall ID, `_compute_overlap_correspondences()` generates point pairs $({\mathbf{p}}^{\text{src}}_k, {\mathbf{p}}^{\text{tgt}}_k)$ from the overlapping section:

1. **Target interval:** $[t_{\min}^{\text{tgt}}, t_{\max}^{\text{tgt}}]$ is directly available in `global_walls`.

2. **Source interval projected onto target tangent:** The source 2D endpoints (reconstructed from `feature_map.walls`) are projected onto the target wall's tangent $\hat{\boldsymbol{\tau}}_{\text{tgt}}$ to express the source extent in the same 1D coordinate as the target. The overlap interval is:

   $$[t_{\text{lo}}, t_{\text{hi}}] = [\max(\min(t_s^{\text{src,lo}}, t_s^{\text{src,hi}}), t_{\min}^{\text{tgt}}), \min(\max(\ldots), t_{\max}^{\text{tgt}})]$$

3. **Require $t_{\text{hi}} - t_{\text{lo}} \geq 0.3\,\text{m}$** to provide sufficient constraint.

4. **Generate $n_s = 8$ uniformly-spaced pairs.** Crucially, each point is generated using its wall's **own** geometry:

   $$\mathbf{p}^{\text{tgt}}_k = \rho_{\text{tgt}}\hat{\mathbf{n}}_{\text{tgt}} + t_k\hat{\boldsymbol{\tau}}_{\text{tgt}}$$
   $$\mathbf{p}^{\text{src}}_k = \rho_{\text{src}}\hat{\mathbf{n}}_{\text{src}} + t_k\hat{\boldsymbol{\tau}}_{\text{src}}$$

   This ensures that $\mathbf{p}^{\text{src}}_k$ lies exactly on the source wall and $\mathbf{p}^{\text{tgt}}_k$ exactly on the target wall, eliminating the cross-frame geometric bias that arose from the previous shared-tangent scheme.

### 11.5 SVD Procrustes Alignment

**File:** `submap_stitcher.py:104` — `_svd_align()`

Given all accumulated point pairs $\{(\mathbf{p}^{\text{src}}_k, \mathbf{p}^{\text{tgt}}_k)\}$, the optimal 2D rigid body transform is found by minimising:

$$E(\mathbf{R}, \mathbf{t}) = \sum_k \|\mathbf{R}\mathbf{p}^{\text{src}}_k + \mathbf{t} - \mathbf{p}^{\text{tgt}}_k\|^2$$

**Step 1:** Compute centroids and centred point sets:
$$\bar{\mathbf{p}}^{\text{src}} = \frac{1}{N}\sum_k \mathbf{p}^{\text{src}}_k, \quad \tilde{\mathbf{p}}^{\text{src}}_k = \mathbf{p}^{\text{src}}_k - \bar{\mathbf{p}}^{\text{src}}$$

**Step 2:** Compute the $2 \times 2$ cross-covariance matrix:
$$\mathbf{H} = \sum_k \tilde{\mathbf{p}}^{\text{src}}_k (\tilde{\mathbf{p}}^{\text{tgt}}_k)^\top$$

**Step 3:** SVD: $\mathbf{H} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^\top$

**Step 4:** Optimal rotation (with reflection guard):
$$\mathbf{R}^* = \mathbf{V}\begin{bmatrix}1 & 0 \\ 0 & \det(\mathbf{V}\mathbf{U}^\top)\end{bmatrix}\mathbf{U}^\top$$

The determinant guard ensures $\det(\mathbf{R}^*) = +1$ (proper rotation), preventing mirror-image solutions.

**Step 5:** Optimal translation:
$$\mathbf{t}^* = \bar{\mathbf{p}}^{\text{tgt}} - \mathbf{R}^*\bar{\mathbf{p}}^{\text{src}}$$

### 11.5 SVD Covariance and Degeneracy Detection

The pose correction covariance is derived from the SVD singular values, which measure the geometric informativeness of the point correspondences:

$$\sigma_{xy}^2 = \frac{\sigma_{\text{lidar}}^2}{\Sigma_1 + \epsilon}, \quad \sigma_\theta^2 = \frac{\sigma_{\text{lidar}}^2}{N\bar{d}^2 + \epsilon}$$

where $\Sigma_1$ is the smallest singular value of $\mathbf{H}$ (controlling translational constraint tightness) and $\bar{d}^2$ is the mean squared distance of source points from their centroid (controlling rotational constraint tightness).

In a feature-poor environment such as a featureless corridor, all shared walls are parallel. In this case, one singular value of $\mathbf{H}$ is near zero (the direction along the corridor is unconstrained), producing a high condition number:

$$\kappa = \Sigma_0 / \Sigma_1 > 100 \implies \text{alignment rejected}$$

This **degeneracy check** prevents the SVD from returning an unreliable correction when the geometry is insufficient to constrain the rigid transform. The EKF integrates the submap without correction in this case, and the naturally larger EKF covariance reflects the degraded accuracy.

### 11.6 EKF Pose Correction Feedback

```python
# local_submap_generator_feature.py:492
self._apply_pose_correction(
    dx=pose_correction['dx'],
    dy=pose_correction['dy'],
    dtheta=pose_correction['dtheta'],
    measurement_covariance=pose_correction['covariance']
)
```

The SVD correction $(\delta x, \delta y, \delta\theta)$ is injected into the EKF as a direct pose observation via `ekf.update()`, which uses a $3 \times n$ observation model $\mathbf{H}_{[0:3, 0:3]} = \mathbf{I}_3$ (observing only the robot pose states). The correction propagates to all landmark states through the off-diagonal cross-covariance blocks in $\mathbf{P}$, providing a global consistency correction without an explicit pose-graph optimisation layer.

### 11.7 Wall Accumulation

`_accumulate_walls()` applies the SVD correction to all walls in the current submap's `FeatureMap` and upserts them into `global_walls`:

1. **Reconstruct 2D endpoints** from stored $(\rho, \alpha, t_{\min}, t_{\max})$.
2. **Apply correction:** `start_corr = R @ start_raw + t`.
3. **Compute corrected Hessian:** $\alpha_{\text{corr}} = \alpha_{\text{src}} + \delta\theta$, $\rho_{\text{corr}} = \bar{\mathbf{p}}_{\text{corr}} \cdot \hat{\mathbf{n}}_{\text{corr}}$ (centroid of corrected endpoints dotted with corrected normal).
4. **Store as scalars:** Project corrected endpoints onto corrected tangent → $t_{\min}^{\text{corr}}, t_{\max}^{\text{corr}}$.
5. **Extend existing entry:** For walls already in `global_walls`, project corrected endpoints onto the existing entry's tangent and extend its $[t_{\min}, t_{\max}]$ to the union.

---

## 12. TF and Pose Publishing

The node publishes the following at 10 Hz:

| Topic | Message Type | Frame | Content |
|-------|-------------|-------|---------|
| `/{robot}/ekf_pose` | `PoseStamped` | `map` | EKF-corrected robot pose |
| `/{robot}/ekf_path` | `Path` | `map` | Full trajectory (sampled at ≥0.1 m steps) |
| `/{robot}/scan_features` | `MarkerArray` | `map` | Extracted wall/corner visualisation |
| `/{robot}/current_submap` | `PointCloud2` | `map` | Current submap point cloud |
| `/{robot}/global_map` | `PointCloud2` | `map` | Accumulated global map (1 Hz timer) |

The `map→odom` TF is broadcast at 10 Hz, enabling the navigation stack to use `odom`-frame odometry while receiving global corrections from the SLAM system.

---

## 13. Parameter Reference

| Parameter | Value | Location | Description |
|-----------|-------|----------|-------------|
| `min_points_per_line` | 10 | `FeatureSLAMManager` | Min scan points to accept a line segment |
| `min_line_length` | 0.3 m | `FeatureSLAMManager` | Min spatial length of a valid wall |
| `corner_angle_threshold` | 50° | `FeatureSLAMManager` | Min angle between adjacent walls for corner |
| `max_gap` | 0.2 m | `FeatureSLAMManager` | Max gap in point sequence to continue segment |
| `merge_angle_tolerance` | 0.22 rad | `FeatureSLAMManager` | Max angular difference for segment merge |
| `merge_rho_tolerance` | 0.15 m | `FeatureSLAMManager` | Max TLS residual after merging |
| `grow_residual_threshold` | 0.03 m | `FeatureSLAMManager` | Max TLS residual to continue growing |
| `lidar_noise_sigma` | 0.01 m | `LandmarkFeatureExtractor` | LDS-01 range noise standard deviation |
| `landmark_timeout_scans` | 25 | `FeatureSLAMManager` | Scans without observation before pruning |
| `min_observations_for_init` | 2 | `FeatureSLAMManager` | Min observations to retain provisional landmark |
| `chi_sq_gate` | 5.99 | `FeatureSLAMManager` | Data association gate (χ²₂, 95%) |
| `chi_sq_update_gate` | 13.8 | `LandmarkEKFSLAM` | EKF update outlier gate (χ²₂, 99.7%) |
| `max_euclidean_dist` | 6.0 m | `FeatureSLAMManager` | Max range for wall pre-filter |
| `wall_angle_tolerance` | 0.349 rad | `FeatureSLAMManager` | Max angle difference for wall pre-filter (20°) |
| `wall_rho_tolerance` | 0.5 m | `FeatureSLAMManager` | Max map-frame rho difference for wall pre-filter |
| `max_gap_ext` | 0.5 m | `FeatureSLAMManager` | Max spatial gap for wall overlap check |
| `q_d` | 0.01 | `BaseEKF` | Distance noise coefficient |
| `q_θ` | 0.005 | `BaseEKF` | Rotation noise coefficient |
| `scans_per_submap` | 50 | `LocalSubmapGeneratorFeature` | Scans per submap boundary |
| `voxel_size` | 0.05 m | `SubmapStitcher` | Voxel downsampling resolution |
| `n_samples` | 8 | `SubmapStitcher` | Sample points per overlapping wall pair |
| `min_overlap` | 0.3 m | `SubmapStitcher` | Min overlap length for SVD correspondence |
| `condition_threshold` | 100.0 | `SubmapStitcher` | Max condition number before rejecting alignment |
| `min_submap_points` | 50 | `SubmapStitcher` | Min points in submap to proceed with stitching |

---

*End of Mapping Module Documentation*
