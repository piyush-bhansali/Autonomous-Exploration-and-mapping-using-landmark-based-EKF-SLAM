# Landmark Feature Extraction: Theory and Implementation

## Table of Contents
1. [Introduction](#1-introduction)
2. [LiDAR Scan Processing](#2-lidar-scan-processing)
3. [Line Segmentation](#3-line-segmentation)
4. [Hessian Normal Form](#4-hessian-normal-form)
5. [Corner Detection](#5-corner-detection)
6. [Covariance Estimation](#6-covariance-estimation)
7. [Implementation](#7-implementation)

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

### 3.1 Split-and-Merge Algorithm

The split-and-merge algorithm recursively segments the point cloud into line segments.

**Algorithm:**

```
function SPLIT_AND_MERGE(points):
    if length(points) < min_points:
        return []

    // Fit line to all points
    line ← FIT_LINE_PCA(points)

    // Find point with maximum perpendicular distance
    distances ← [DISTANCE_TO_LINE(p, line) for p in points]
    max_idx ← argmax(distances)
    max_dist ← distances[max_idx]

    // Check if split is needed
    if max_dist > threshold AND can_split(points, max_idx):
        // Recursively split
        left ← SPLIT_AND_MERGE(points[0:max_idx+1])
        right ← SPLIT_AND_MERGE(points[max_idx:end])
        return concat(left, right)
    else:
        // Accept as single segment
        if length(points) >= min_points:
            return [line]
        else:
            return []
```

**Parameters:**
- `min_points`: Minimum points per segment (e.g., 10)
- `threshold`: Maximum perpendicular distance (e.g., 0.05 m)
- `min_line_length`: Minimum Euclidean distance (e.g., 0.5 m)

### 3.2 PCA-Based Line Fitting

Principal Component Analysis (PCA) finds the best-fit line through a set of points.

**Given:** Points $\{p_i\}_{i=1}^N$ where $p_i = [x_i, y_i]^T$

**Step 1: Compute Centroid**

$$
\bar{p} = \frac{1}{N} \sum_{i=1}^N p_i = \begin{bmatrix}
\bar{x} \\
\bar{y}
\end{bmatrix}
$$

**Step 2: Center Points**

$$
\tilde{p}_i = p_i - \bar{p}
$$

**Step 3: Compute Covariance Matrix**

$$
\mathbf{C} = \frac{1}{N-1} \sum_{i=1}^N \tilde{p}_i \tilde{p}_i^T = \begin{bmatrix}
\sigma_x^2 & \sigma_{xy} \\
\sigma_{xy} & \sigma_y^2
\end{bmatrix}
$$

**Step 4: Eigendecomposition**

$$
\mathbf{C} = \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T
$$

Where:
- $\mathbf{V} = [v_1 \mid v_2]$: Eigenvectors (principal axes)
- $\boldsymbol{\Lambda} = \text{diag}(\lambda_1, \lambda_2)$: Eigenvalues (variances)
- $\lambda_1 \geq \lambda_2$

**Step 5: Line Direction**

The line direction is the first principal component:

$$
\hat{d} = v_1 = \begin{bmatrix}
d_x \\
d_y
\end{bmatrix}
$$

**Line Parameterization:**

$$
\ell(t) = \bar{p} + t \cdot \hat{d}, \quad t \in \mathbb{R}
$$

**Quality Metric:**

$$
\text{linearity} = \frac{\lambda_1}{\lambda_1 + \lambda_2}
$$

Values close to 1 indicate strong linear structure.

### 3.3 Point-to-Line Distance

The perpendicular distance from point $p$ to line $\ell$ defined by centroid $\bar{p}$ and direction $\hat{d}$:

$$
d(p, \ell) = \|(p - \bar{p}) - \langle p - \bar{p}, \hat{d} \rangle \hat{d}\|
$$

**Geometric Interpretation:**
1. Project $(p - \bar{p})$ onto $\hat{d}$: $\text{proj} = \langle p - \bar{p}, \hat{d} \rangle \hat{d}$
2. Compute perpendicular component: $\text{perp} = (p - \bar{p}) - \text{proj}$
3. Distance is magnitude: $d = \|\text{perp}\|$

**Simplified Form:**

$$
d(p, \ell) = \left| (p - \bar{p}) - \hat{d} \langle p - \bar{p}, \hat{d} \rangle \right|
$$

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

### 4.2 Conversion from PCA Line

Given a line defined by centroid $\bar{p} = [\bar{x}, \bar{y}]^T$ and direction $\hat{d} = [d_x, d_y]^T$:

**Step 1: Compute Normal Vector**

The normal is perpendicular to the direction (90° rotation):

$$
\mathbf{n} = \begin{bmatrix}
-d_y \\
d_x
\end{bmatrix}
$$

**Step 2: Compute Distance $\rho$**

The signed distance from origin to line is:

$$
\rho_{\text{signed}} = \mathbf{n}^T \bar{p} = -d_y \bar{x} + d_x \bar{y}
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

## 7. Implementation

### 7.1 Feature Extraction Pipeline

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

### 7.2 Code Mapping

| Algorithm | File | Function |
|-----------|------|----------|
| Scan to Cartesian | `landmark_features.py` | `_scan_to_cartesian()` |
| Gap detection | `landmark_features.py` | `_split_on_gaps()` |
| Split-and-merge | `landmark_features.py` | `_extract_lines()` |
| PCA line fitting | `landmark_features.py` | `_fit_line_pca()` |
| Hessian conversion | `landmark_features.py` | `_convert_line_to_hessian()` |
| Corner detection | `landmark_features.py` | `_detect_corners_from_lines()` |
| Wall covariance | `landmark_features.py` | `_compute_wall_covariance()` |
| Corner covariance | `landmark_features.py` | `_compute_corner_covariance()` |

### 7.3 Parameter Tuning

| Parameter | Typical Value | Effect |
|-----------|--------------|--------|
| `min_points_per_line` | 10 | More points → more stable lines, fewer detections |
| `line_fit_threshold` | 0.05 m | Lower → more splitting, shorter segments |
| `min_line_length` | 0.5 m | Filters out small noise segments |
| `corner_angle_threshold` | 45° | Minimum angle for valid corner |
| `max_gap` | 0.2 m | Gap detection sensitivity |

**Tuning Strategy:**
1. Set `line_fit_threshold` to ~2× expected sensor noise
2. Set `min_points_per_line` to ensure statistical significance
3. Adjust `corner_angle_threshold` to match environment (sharp vs. rounded corners)

---

## 8. Visualization and Debugging

### 8.1 RViz Markers

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

### 8.2 Quality Metrics

Log feature quality for analysis:

| Metric | Formula | Interpretation |
|--------|---------|----------------|
| Linearity | $\lambda_1 / (\lambda_1 + \lambda_2)$ | > 0.95 → good line |
| Residual RMS | $\sqrt{\frac{1}{N}\sum r_i^2}$ | < 0.05 m → good fit |
| Support | Number of points | > 10 → stable feature |
| Length | Euclidean distance | > 0.5 m → significant |

---

## References

1. **Nguyen, V., Martinelli, A., Tomatis, N., & Siegwart, R. (2005).** "A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder for Indoor Mobile Robotics." *IROS 2005*.

2. **Arras, K. O., Tomatis, N., Jensen, B. T., & Siegwart, R. (2001).** "Multisensor On-the-Fly Localization: Precision and Reliability for Applications." *Robotics and Autonomous Systems*, 34(2-3), 131-143.

3. **Duda, R. O., & Hart, P. E. (1972).** "Use of the Hough Transformation to Detect Lines and Curves in Pictures." *Communications of the ACM*, 15(1), 11-15.

4. **Jolliffe, I. T. (2002).** *Principal Component Analysis*. Springer. Chapter 2: Properties of Population Principal Components.

5. **Pavlidis, T., & Horowitz, S. L. (1974).** "Segmentation of Plane Curves." *IEEE Transactions on Computers*, C-23(8), 860-870.

---

**Next:** `03_data_association.md` — Landmark matching using Mahalanobis distance and statistical gating
