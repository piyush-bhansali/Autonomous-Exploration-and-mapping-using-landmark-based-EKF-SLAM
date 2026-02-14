# Landmark Feature Extraction: Theory and Implementation

> **Note:** This document provides detailed mathematical derivations for feature extraction used in **Feature-based SLAM mode**. For complete system context, see `methodology_feature_mapping.md`.

## Table of Contents
1. [Introduction](#1-introduction)
2. [LiDAR Scan Processing](#2-lidar-scan-processing)
3. [Line Segmentation](#3-line-segmentation)
4. [Hessian Normal Form](#4-hessian-normal-form)
5. [Corner Detection](#5-corner-detection)
6. [Covariance Estimation](#6-covariance-estimation)
7. [Feature Quality Assessment](#7-feature-quality-assessment)
8. [Implementation](#8-implementation)
9. [Visualization and Debugging](#9-visualization-and-debugging)

---

## 1. Introduction

Feature extraction is the process of converting raw LiDAR scan data into **geometric primitives** (walls and corners) that can serve as landmarks for SLAM. Good features should be:
- **Distinctive:** Easily distinguishable from other features
- **Stable:** Consistently detectable across multiple observations
- **Geometric:** Parameterizable with few degrees of freedom
- **Efficient:** Fast to extract and match

### 1.1 Why Walls and Corners?

**Walls (Lines):**
- Common in structured environments (buildings, corridors)
- 2 parameters in Hessian form (compact)
- Orientation-invariant representation
- Stable over long distances

**Corners:**
- Provide point constraints (complementary to lines)
- Natural at wall intersections, furniture edges
- 2 parameters (x, y)
- High information density

### 1.2 Literature Review: Line Extraction Methods

The problem of extracting linear features from 2D range data has been extensively studied. **Nguyen et al. (2005)** conducted a comprehensive comparison of line extraction algorithms using 2D laser rangefinders for indoor mobile robotics, evaluating four major approaches:

**1. Split-and-Merge (Pavlidis & Horowitz, 1974)**
- **Method**: Recursive subdivision based on perpendicular distance threshold
- **Advantages**: Fast ($O(n \log n)$), robust to noise, few parameters
- **Disadvantages**: Sensitive to initial split point selection
- **Best for**: Structured environments with clear wall segments

**2. Incremental (Duda & Hart, 1973)**
- **Method**: Sequential point addition while residuals remain below threshold
- **Advantages**: Very fast ($O(n)$), deterministic
- **Disadvantages**: Sensitive to point order, can miss small gaps
- **Best for**: Real-time applications with computational constraints

**3. Hough Transform (Duda & Hart, 1972)**
- **Method**: Parameter space voting for detecting collinear points
- **Advantages**: Robust to occlusions, finds parallel lines
- **Disadvantages**: Computationally expensive, requires discretization
- **Best for**: Cluttered environments with multiple parallel structures

**4. RANSAC (Fischler & Bolles, 1981)**
- **Method**: Random sample consensus for outlier-robust fitting
- **Advantages**: Excellent outlier rejection, statistically principled
- **Disadvantages**: Non-deterministic, requires many iterations
- **Best for**: Highly noisy data with significant outliers

**Experimental Comparison Results** (Nguyen et al., 2005, Table 2):

| Algorithm | Execution Time | Detection Rate | Precision | Outlier Robustness |
|-----------|---------------|----------------|-----------|-------------------|
| Split-and-Merge | 3.2 ms | 94% | 0.89 | Good |
| Incremental | 1.1 ms | 88% | 0.76 | Fair |
| Hough Transform | 12.7 ms | 96% | 0.92 | Excellent |
| RANSAC | 8.4 ms | 95% | 0.91 | Excellent |

**Conclusion**: The current implementation uses an **incremental line-growing + adjacent merge** pipeline for deterministic real-time extraction while retaining geometric consistency checks.

### 1.3 Geometric Primitives in SLAM

**Siegwart & Nourbakhsh (2011, Chapter 5)** discuss feature selection for localization:

- **Point features**: High precision but lack orientation information
- **Line features**: Orientation + distance constraints, redundant representation issues
- **Hessian form**: Minimal parameterization avoiding redundancy

**Arras et al. (2001)** demonstrated that line-based features achieve **2-3× better localization accuracy** than point-based features in structured environments, with similar computational cost.

---

## 2. LiDAR Scan Processing

### 2.1 Raw Scan Data

A 2D LiDAR scan consists of range measurements at fixed angular intervals:

$$
\mathcal{S} = \{(r_i, \theta_i) \mid i = 1, \ldots, N\}
$$

Where:
- $r_i \in [r_{\min}, r_{\max}]$: Range to obstacle
- $\theta_i = \theta_{\min} + i \cdot \Delta\theta$: Bearing angle
- $\Delta\theta = \frac{\theta_{\max} - \theta_{\min}}{N-1}$: Angular resolution

**Typical Values (TurtleBot3):**
- $N = 360$ samples
- $\theta_{\min} = 0°$, $\theta_{\max} = 360°$
- $\Delta\theta = 1°$
- $r_{\min} = 0.12$ m, $r_{\max} = 3.5$ m

### 2.2 Cartesian Conversion

Convert polar coordinates to Cartesian:

$$
\begin{bmatrix}
x_i \\
y_i
\end{bmatrix} = \begin{bmatrix}
r_i \cos(\theta_i) \\
r_i \sin(\theta_i)
\end{bmatrix}
$$

**Filtering:** Remove invalid measurements:
- $r_i < r_{\min}$ or $r_i > r_{\max}$
- $r_i = \text{NaN}$ or $r_i = \infty$

**Result:** Point cloud $\mathcal{P} = \{(x_i, y_i) \mid i \in \text{valid}\}$

### 2.3 Gap Detection

LiDAR scans have **gaps** (discontinuities) where:
- Objects are occluded
- Sensor range is exceeded
- Open doorways exist

**Gap Detection Criterion:**

$$
\|p_{i+1} - p_i\| > \tau_{\text{gap}}
$$

Where $\tau_{\text{gap}} = 0.2$ m (typical).

**Purpose:** Prevents connecting points across gaps into incorrect lines.

---

## 3. Line Segmentation

### 3.1 Incremental Line Growing

The current pipeline grows line segments over ordered scan points:

1. Start from the first 2 points in the ordered segment.
2. Add one point at a time.
3. Evaluate max perpendicular residual to the endpoint-defined line.
4. If residual exceeds `grow_residual_threshold`, finalize the previous segment and restart.

Segment acceptance requires:
- `min_points_per_line`
- `min_line_length`

This avoids recursive splitting while preserving deterministic behavior.

### 3.2 Adjacent Segment Merge

After growth, adjacent segments are merged when they remain geometrically consistent:
- acute direction difference below `merge_angle_tolerance`
- endpoint gap below `max_gap`
- residual of merged candidate below `merge_rho_tolerance`

This keeps long wall continuity while preventing merges across discontinuities.

### 3.3 Endpoint-Line Residual

Given segment endpoints $p_s, p_e$ and a candidate point $p_i$, the perpendicular residual is:

$$
r_i = \frac{|(p_i - p_s) \times (p_e - p_s)|}{\|p_e - p_s\|}
$$

The segment quality metric is:

$$
r_{\max} = \max_i r_i
$$

A candidate remains part of the segment while $r_{\max}$ is below the configured threshold.

---

## 4. Hessian Normal Form

### 4.1 Definition

The **Hessian normal form** represents a line using:

$$
\rho = x \cos(\alpha) + y \sin(\alpha)
$$

Where:
- $\rho \geq 0$: Perpendicular distance from origin to line
- $\alpha \in [-\pi, \pi]$: Angle of the normal vector $\mathbf{n} = [\cos\alpha, \sin\alpha]^T$

**Advantages:**
1. **Uniqueness:** Each line has exactly one representation (with $\rho \geq 0$)
2. **Compactness:** Only 2 parameters
3. **Orientation-invariant:** $\alpha$ is absolute (not relative to robot)

### 4.2 Conversion from Endpoint-Defined Segment

Given a segment with endpoints $p_s = [x_s, y_s]^T$ and $p_e = [x_e, y_e]^T$:

**Step 1: Compute Normal Vector**

Compute direction and rotate by 90°:

$$
\hat{d} = \frac{p_e - p_s}{\|p_e - p_s\|}, \quad
\mathbf{n} = \begin{bmatrix}
-d_y \\
d_x
\end{bmatrix}
$$

**Step 2: Compute Midpoint and Distance $\rho$**

Use the segment midpoint $\mathbf{m} = \frac{1}{2}(p_s + p_e)$:

$$
\rho_{\text{signed}} = \mathbf{n}^T \mathbf{m}
$$

**Step 3: Ensure $\rho \geq 0$**

If $\rho_{\text{signed}} < 0$, flip the normal:

$$
\rho = |\rho_{\text{signed}}|, \quad \mathbf{n} \leftarrow \text{sign}(\rho_{\text{signed}}) \cdot \mathbf{n}
$$

**Step 4: Compute Angle $\alpha$**

$$
\alpha = \text{atan2}(n_y, n_x)
$$

### 4.3 Geometric Interpretation

```
         y
         │
         │      Wall (line)
         │     /
         │    /
         │   /
         │  /
         │ /  α (normal angle)
         │/____________________
    ─────•────────────────────> x
         │←─ ρ ─→
       Origin
```

The point on the line closest to the origin is:

$$
p_{\text{closest}} = \rho \cdot \begin{bmatrix}
\cos\alpha \\
\sin\alpha
\end{bmatrix}
$$

### 4.4 Line Properties

**Any point $p = [x, y]^T$ on the line satisfies:**

$$
x \cos\alpha + y \sin\alpha = \rho
$$

**Signed distance** from point $p$ to line:

$$
d_{\text{signed}}(p) = x \cos\alpha + y \sin\alpha - \rho
$$

- $d_{\text{signed}} = 0$: Point on line
- $d_{\text{signed}} > 0$: Point on normal side
- $d_{\text{signed}} < 0$: Point on opposite side

**Perpendicular distance:**

$$
d_{\text{perp}}(p) = |d_{\text{signed}}(p)|
$$

---

## 5. Corner Detection

### 5.1 Definition

A **corner** is the intersection point of two non-parallel lines.

### 5.2 Line Intersection

Given two lines in Hessian form:
- Line A: $\rho_A = x \cos\alpha_A + y \sin\alpha_A$
- Line B: $\rho_B = x \cos\alpha_B + y \sin\alpha_B$

**System of equations:**

$$
\begin{bmatrix}
\cos\alpha_A & \sin\alpha_A \\
\cos\alpha_B & \sin\alpha_B
\end{bmatrix} \begin{bmatrix}
x \\
y
\end{bmatrix} = \begin{bmatrix}
\rho_A \\
\rho_B
\end{bmatrix}
$$

**Solution (if lines not parallel):**

$$
\begin{bmatrix}
x \\
y
\end{bmatrix} = \frac{1}{\det(\mathbf{A})} \begin{bmatrix}
\sin\alpha_B & -\sin\alpha_A \\
-\cos\alpha_B & \cos\alpha_A
\end{bmatrix} \begin{bmatrix}
\rho_A \\
\rho_B
\end{bmatrix}
$$

Where:

$$
\det(\mathbf{A}) = \cos\alpha_A \sin\alpha_B - \sin\alpha_A \cos\alpha_B = \sin(\alpha_B - \alpha_A)
$$

**Parallel check:** If $|\det(\mathbf{A})| < \epsilon$ (e.g., $\epsilon = 10^{-6}$), lines are parallel.

### 5.3 Corner Detection Algorithm

**Input:** List of line segments $\{\ell_i\}_{i=1}^M$

**Output:** List of corner positions $\{c_j\}$

```
corners ← []
for i ← 1 to M:
    for j ← i+1 to M:
        // Check angle difference
        angle_diff ← |α_j - α_i|
        if angle_diff < threshold_angle:
            continue  // Lines too parallel

        // Check spatial proximity of endpoints
        min_dist ← min_endpoint_distance(ℓ_i, ℓ_j)
        if min_dist > threshold_proximity:
            continue  // Lines too far apart

        // Compute intersection
        p_intersect ← line_intersection(ℓ_i, ℓ_j)

        // Validate intersection is near endpoints
        if distance(p_intersect, endpoints(ℓ_i, ℓ_j)) < threshold:
            if not is_duplicate(p_intersect, corners):
                corners.append(p_intersect)

return corners
```

**Parameters:**
- `threshold_angle`: Minimum angle between lines (e.g., 45°)
- `threshold_proximity`: Maximum distance between line endpoints (e.g., 1.0 m)
- `threshold_duplicate`: Minimum distance between corners (e.g., 0.1 m)

### 5.4 Corner Validation

Not all line intersections are valid corners. We validate by checking:

1. **Angle constraint:** $\theta_{\min} < |\alpha_B - \alpha_A| < 180° - \theta_{\min}$
2. **Proximity constraint:** Intersection must be near both line segments
3. **Support constraint:** Sufficient scan points nearby

---

## 6. Covariance Estimation

### 6.1 Wall Covariance (Hessian Form)

The uncertainty in wall parameters $(\rho, \alpha)$ depends on the point cloud fit quality.

**Residual-Based Estimation:**

Given points $\{p_i\}$ and fitted line $(\rho, \alpha)$, compute residuals:

$$
r_i = x_i \cos\alpha + y_i \sin\alpha - \rho
$$

**Jacobian w.r.t. parameters:**

$$
\mathbf{J}_i = \begin{bmatrix}
\frac{\partial r_i}{\partial \rho} & \frac{\partial r_i}{\partial \alpha}
\end{bmatrix} = \begin{bmatrix}
-1 & -x_i \sin\alpha + y_i \cos\alpha
\end{bmatrix}
$$

**Stacked Jacobian:**

$$
\mathbf{J} = \begin{bmatrix}
\mathbf{J}_1 \\
\mathbf{J}_2 \\
\vdots \\
\mathbf{J}_N
\end{bmatrix}_{N \times 2}
$$

**Normal Equations:**

$$
\mathbf{H} = \mathbf{J}^T \mathbf{J} = \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i
$$

**Residual Variance:**

$$
\sigma_r^2 = \frac{1}{N - 2} \sum_{i=1}^N r_i^2
$$

(Degrees of freedom = $N - 2$ for 2 parameters)

**Covariance Estimate:**

$$
\mathbf{P}_{(\rho, \alpha)} = \sigma_r^2 \cdot \mathbf{H}^{-1}
$$

**Add Sensor Noise Floor:**

$$
\mathbf{P}_{\text{total}} = \mathbf{P}_{(\rho, \alpha)} + \mathbf{P}_{\text{sensor}}
$$

Where:

$$
\mathbf{P}_{\text{sensor}} = \begin{bmatrix}
\sigma_{\text{range}}^2 & 0 \\
0 & \sigma_{\text{bearing}}^2
\end{bmatrix} = \begin{bmatrix}
(0.01)^2 & 0 \\
0 & (0.5°)^2
\end{bmatrix}
$$

### 6.2 Corner Covariance

For corners, we compute covariance from neighboring points.

**Sample Covariance:**

Given neighbor points $\{p_i\}_{i=1}^M$ near the corner $\bar{c}$:

$$
\mathbf{C} = \frac{1}{M - 1} \sum_{i=1}^M (p_i - \bar{c})(p_i - \bar{c})^T
$$

**With Sensor Noise:**

$$
\mathbf{P}_{\text{corner}} = \mathbf{C} + \mathbf{P}_{\text{sensor}}
$$

Where:

$$
\mathbf{P}_{\text{sensor}} = \sigma_{\text{range}}^2 \cdot \mathbf{I}_{2 \times 2}
$$

### 6.3 Uncertainty Interpretation

**Low covariance trace** $\text{tr}(\mathbf{P})$:
- Many supporting points
- Low residuals (good fit)
- Precise feature → reliable landmark

**High covariance trace:**
- Few supporting points
- High residuals (noisy data)
- Uncertain feature → unreliable landmark

**Eigenvalues of $\mathbf{P}$:**
- Large eigenvalue direction: high uncertainty
- Small eigenvalue direction: well-constrained

---

## 7. Feature Quality Assessment

### 7.1 Quality Metrics from Literature

**Nguyen et al. (2005)** established quantitative metrics for evaluating line extraction quality:

**1. Detection Rate (Recall)**

Fraction of ground truth lines successfully detected:

$$
\text{Detection Rate} = \frac{\text{True Positives}}{\text{True Positives} + \text{False Negatives}}
$$

**Threshold**: Detection rate > 90% considered good

**2. Precision**

Fraction of detected lines that are correct:

$$
\text{Precision} = \frac{\text{True Positives}}{\text{True Positives} + \text{False Positives}}
$$

**Threshold**: Precision > 0.85 considered good

**3. Geometric Accuracy**

Root mean square error between detected and ground truth lines:

$$
\text{RMSE}_{\text{line}} = \sqrt{\frac{1}{M}\sum_{i=1}^M \left[(\rho_i - \rho_i^{\text{gt}})^2 + d^2 (\alpha_i - \alpha_i^{\text{gt}})^2\right]}
$$

where $d$ is a normalization distance (typical: $d = 1$ m)

**Threshold**: RMSE < 0.05 m considered excellent

### 7.2 Feature Stability Analysis

**Arras et al. (2001)** studied feature repeatability across multiple scans:

**Temporal Stability**: Fraction of features re-detected across consecutive scans

$$
\text{Stability}(t) = \frac{|\mathcal{F}_{t} \cap \mathcal{F}_{t+1}|}{|\mathcal{F}_t|}
$$

**Experimental Results** (Arras et al., Table 3):
- Wall features: 96% stability over 1s intervals
- Corner features: 82% stability over 1s intervals

**Implication**: Walls are more reliable landmarks for SLAM

### 7.3 Information Content

**Zhang & Ghosh (2000)** quantified information gain from different feature types:

**Fisher Information Matrix** for line observation:

$$
\mathbf{I}_{\text{wall}} = \mathbf{H}^T \mathbf{R}^{-1} \mathbf{H}
$$

where $\mathbf{H}$ is the observation Jacobian (see Section 6 of `01_ekf_slam_theory.md`)

**Information Gain** (determinant of FIM):

$$
\text{Info}_{\text{wall}} = \det(\mathbf{I}_{\text{wall}})
$$

**Comparative Information Gains**:
- Single wall observation: 1-2 DOF constraint (orientation + perpendicular distance)
- Single corner observation: 2 DOF constraint (x, y position)
- Two perpendicular walls: 3 DOF constraint (full pose observability)

### 7.4 Line Fitting Quality Indicators

**Linearity Ratio** (from PCA):

$$
\text{Linearity} = \frac{\lambda_1}{\lambda_1 + \lambda_2}
$$

**Interpretation**:
- $> 0.95$: Excellent linear structure
- $0.90 - 0.95$: Good linear structure
- $0.80 - 0.90$: Marginal (consider rejecting)
- $< 0.80$: Poor fit (reject)

**Residual Statistics**:

$$
\text{RMS}_{\text{residual}} = \sqrt{\frac{1}{N}\sum_{i=1}^N r_i^2}
$$

**Acceptance Criteria** (based on sensor noise):
- TurtleBot3 LiDAR: $\sigma_{\text{range}} \approx 0.01$ m
- Accept if: $\text{RMS}_{\text{residual}} < 3\sigma_{\text{range}} = 0.03$ m

**Support Count**:

Minimum number of supporting points for statistical significance:

$$
N_{\text{min}} = \frac{k \cdot p}{s}
$$

where:
- $k = 2$: parameters to estimate (ρ, α)
- $p = 5$: points per parameter (rule of thumb)
- $s = 1$: sampling factor

**Typical**: $N_{\text{min}} = 10$ points

---

## 8. Implementation

### 8.1 Feature Extraction Pipeline

```python
def extract_features(scan):
    # 1. Convert to Cartesian
    points = scan_to_cartesian(scan)

    # 2. Detect gaps
    segments = split_on_gaps(points, max_gap=0.2)

    # 3. Line segmentation (per continuous segment)
    lines = []
    for segment in segments:
        lines_in_segment = split_and_merge(segment)
        lines.extend(lines_in_segment)

    # 4. Convert to Hessian form
    walls = []
    for line in lines:
        rho, alpha = convert_to_hessian(line['points'])
        covariance = compute_wall_covariance(line['points'], rho, alpha)
        walls.append({
            'type': 'wall',
            'rho': rho,
            'alpha': alpha,
            'covariance': covariance,
            'num_points': len(line['points'])
        })

    # 5. Corner detection
    corners = detect_corners_from_lines(lines)
    for corner in corners:
        covariance = compute_corner_covariance(corner['neighbors'])
        corner['covariance'] = covariance
        corner['type'] = 'corner'

    return walls + corners
```

### 8.2 Code Mapping

| Algorithm | File | Function |
|-----------|------|----------|
| Scan to Cartesian | `landmark_features.py` | `_scan_to_cartesian()` |
| Gap detection | `landmark_features.py` | `_split_on_gaps()` |
| Incremental line growing | `landmark_features.py` | `_grow_lines_incremental()` |
| Adjacent segment merge | `landmark_features.py` | `_merge_adjacent_lines()` |
| Hessian conversion | `landmark_features.py` | `_convert_line_to_hessian()` |
| Corner detection | `landmark_features.py` | `_extract_corners_from_adjacent_lines()` |
| Wall covariance | `landmark_features.py` | `_compute_wall_covariance()` |
| Corner covariance | `landmark_features.py` | `_compute_corner_covariance()` |

### 8.3 Parameter Tuning

| Parameter | Typical Value | Effect |
|-----------|--------------|--------|
| `min_points_per_line` | 10 | More points → more stable lines, fewer detections |
| `grow_residual_threshold` | 0.03 m | Lower → more splitting, shorter segments |
| `min_line_length` | 0.5 m | Filters out small noise segments |
| `corner_angle_threshold` | 45° | Minimum angle for valid corner |
| `max_gap` | 0.2 m | Gap detection sensitivity |
| `merge_angle_tolerance` | 0.15 rad | Controls directional compatibility during merge |
| `merge_rho_tolerance` | 0.15 m | Controls merged-segment residual tolerance |

**Tuning Strategy:**
1. Set `grow_residual_threshold` to ~2-3× expected sensor noise
2. Set `min_points_per_line` to ensure statistical significance
3. Tune merge tolerances to preserve long walls without over-merging disconnected surfaces

---

## 9. Visualization and Debugging

### 9.1 RViz Markers

For thesis visualization, publish features as RViz markers:

**Walls (LINE_STRIP):**
```python
marker = Marker()
marker.type = Marker.LINE_STRIP
marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0  # Green
marker.scale.x = 0.02  # Line width

# Compute endpoints from Hessian form
p0 = rho * [cos(alpha), sin(alpha)]
direction = [-sin(alpha), cos(alpha)]
start = p0 - L/2 * direction
end = p0 + L/2 * direction

marker.points = [start, end]
```

**Corners (SPHERE):**
```python
marker = Marker()
marker.type = Marker.SPHERE
marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0  # Red
marker.scale.x = marker.scale.y = marker.scale.z = 0.1  # Sphere radius
marker.pose.position.x, marker.pose.position.y = corner['position']
```

### 9.2 Quality Metrics

Log feature quality for analysis:

| Metric | Formula | Interpretation |
|--------|---------|----------------|
| Linearity | $\lambda_1 / (\lambda_1 + \lambda_2)$ | > 0.95 → good line |
| Residual RMS | $\sqrt{\frac{1}{N}\sum r_i^2}$ | < 0.05 m → good fit |
| Support | Number of points | > 10 → stable feature |
| Length | Euclidean distance | > 0.5 m → significant |

---

## References

### Feature Extraction Algorithms

1. **Nguyen, V., Martinelli, A., Tomatis, N., & Siegwart, R. (2005).** "A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder for Indoor Mobile Robotics." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 1929-1934.
   - Comprehensive comparison of Split-and-Merge, Incremental, Hough Transform, and RANSAC
   - Experimental evaluation on real robot data with ground truth

2. **Siadat, A., Kaske, A., Klausmann, S., Dufaut, M., & Husson, R. (1997).** "An Optimized Segmentation Method for a 2D Laser-Scanner Applied to Mobile Robot Navigation." *Proceedings of 3rd IFAC Symposium on Intelligent Components and Instruments for Control Applications*, pp. 153-158.
   - Line segment-based map building for mobile robots
   - Hessian form representation for walls

3. **Pfister, S. T., Kriechbaum, K. L., Roumeliotis, S. I., & Burdick, J. W. (2002).** "Weighted Line Fitting Algorithms for Mobile Robot Map Building and Efficient Data Representation." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 1304-1311.
   - Weighted least squares for line fitting with uncertainty
   - Covariance propagation from scan points to line parameters

### Geometric Representations

4. **Duda, R. O., & Hart, P. E. (1972).** "Use of the Hough Transformation to Detect Lines and Curves in Pictures." *Communications of the ACM*, 15(1), 11-15.
   - Foundational paper introducing Hough transform for line detection

5. **Duda, R. O., & Hart, P. E. (1973).** "Pattern Classification and Scene Analysis." Wiley.
   - Chapter 8: Incremental line extraction algorithm

6. **Zhang, L., & Ghosh, B. K. (2000).** "Line Segment Based Map Building and Localization Using 2D Laser Rangefinder." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 2538-2543.
   - Information-theoretic analysis of line features for localization

### Feature-Based Localization

7. **Arras, K. O., Tomatis, N., Jensen, B. T., & Siegwart, R. (2001).** "Multisensor On-the-Fly Localization: Precision and Reliability for Applications." *Robotics and Autonomous Systems*, 34(2-3), 131-143.
   - Feature stability analysis across multiple scans
   - Demonstrated 2-3× improvement with line features vs. point features

8. **Castellanos, J. A., Montiel, J. M. M., Neira, J., & Tardós, J. D. (1999).** "The SPmap: A Probabilistic Framework for Simultaneous Localization and Map Building." *IEEE Transactions on Robotics and Automation*, 15(5), 948-952.
   - Stochastic mapping with geometric features

### Outlier Rejection and Robustness

9. **Fischler, M. A., & Bolles, R. C. (1981).** "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381-395.
   - RANSAC algorithm for robust line fitting in presence of outliers

10. **Pavlidis, T., & Horowitz, S. L. (1974).** "Segmentation of Plane Curves." *IEEE Transactions on Computers*, C-23(8), 860-870.
    - Split-and-merge algorithm for curve segmentation

### Mathematical Foundations

11. **Jolliffe, I. T. (2002).** *Principal Component Analysis* (2nd ed.). Springer.
    - Chapter 2: Properties of Population Principal Components
    - Mathematical basis for PCA-based line fitting

12. **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011).** *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
    - Chapter 5: Mobile Robot Localization
    - Chapter 4: Perception (feature extraction from range data)

---

**Next:** `03_data_association.md` — Landmark matching using Mahalanobis distance and statistical gating
