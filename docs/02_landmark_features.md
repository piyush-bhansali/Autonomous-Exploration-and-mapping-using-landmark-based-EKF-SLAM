# Landmark Feature Extraction: Theory and Implementation

## Table of Contents
1. [Introduction](#1-introduction)
2. [Scan Pre-processing](#2-scan-pre-processing)
3. [Line Segmentation](#3-line-segmentation)
4. [Hessian Normal Form](#4-hessian-normal-form)
5. [Corner Detection](#5-corner-detection)
6. [Covariance Estimation](#6-covariance-estimation)
7. [Implementation Reference](#7-implementation-reference)

---

## 1. Introduction

Feature extraction converts a raw 2D LiDAR scan into geometric primitives — walls and corners — that serve as landmarks for EKF-SLAM. Indoor environments are dominated by planar surfaces. Walls are abundant, stable across scans, and easily parameterised. Corners complement walls by providing point constraints that fully determine a 2D position.

### 1.1 Choice of Line Extraction Method

Four principal methods extract line segments from 2D range data (Nguyen et al., 2005):

| Method | Complexity | Deterministic | Notes |
|---|---|---|---|
| Split-and-Merge (Pavlidis & Horowitz, 1974) | $O(n \log n)$ | Yes | Recursive subdivision on perpendicular distance |
| Incremental (Duda & Hart, 1973) | $O(n)$ | Yes | Sequential point addition while residual is bounded |
| Hough Transform (Duda & Hart, 1972) | $O(n^2)$ | Yes | Parameter-space voting; robust to occlusions |
| RANSAC (Fischler & Bolles, 1981) | $O(n)$ amortised | No | Outlier-robust random sampling |

This system uses an **incremental growing + adjacent merge** pipeline. Incremental growing is $O(n)$, deterministic,   A subsequent merge step recovers long walls that were split by threshold effects during growth.

---

## 2. Scan Pre-processing

### 2.1 Polar to Cartesian Conversion

A 2D LiDAR scan is a set of range-bearing measurements $\{(r_i, \varphi_i)\}$. Valid readings (within the sensor range $[r_{\min}, r_{\max}]$ and free of NaN/Inf) are converted to Cartesian coordinates:

$$
x_i = r_i \cos\varphi_i, \qquad y_i = r_i \sin\varphi_i.
$$

For the TurtleBot3 LDS-01: $r_{\min} = 0.12\,\mathrm{m}$, $r_{\max} = 3.5\,\mathrm{m}$, 360 samples at $1°$ resolution.

### 2.2 Gap Detection

Consecutive points that belong to different surfaces are separated by a spatial gap. Two adjacent points $p_i$ and $p_{i+1}$ are split into different segments when

$$
\|p_{i+1} - p_i\| > \tau_{\mathrm{gap}},
$$

with $\tau_{\mathrm{gap}} = 0.2\,\mathrm{m}$. This prevents connecting points across doorways or occlusion boundaries into a single incorrect segment.

---

## 3. Line Segmentation

### 3.1 Total Least Squares Line Fit

All residuals in the pipeline — both for deciding when to split a growing segment and for deciding whether two adjacent segments should be merged — are computed from a **total least squares (TLS)** fit.

Given a set of candidate points, centre them at their mean $\bar{\mathbf{p}}$:

$$
\tilde{\mathbf{p}}_i = \mathbf{p}_i - \bar{\mathbf{p}}.
$$

Form the $N \times 2$ centred data matrix and compute its thin SVD:

$$
\tilde{\mathbf{P}} = \mathbf{U}\,\boldsymbol{\Sigma}\,\mathbf{V}^\top.
$$

The first right-singular vector $\mathbf{v}_1$ (corresponding to the largest singular value) is the principal axis of the point cloud — the best-fit line direction. The unit normal to the line is

$$
\mathbf{n} = [-v_{1y},\; v_{1x}]^\top.
$$

The perpendicular distance from any point $\mathbf{p}_i$ to this line is

$$
d_i = |(\mathbf{p}_i - \bar{\mathbf{p}}) \cdot \mathbf{n}|.
$$

The segment residual used throughout the pipeline is $r = \max_i d_i$.

**Rationale.** An endpoint-to-endpoint residual uses only the two noisiest measurements in the segment (LiDAR points measured at grazing angles have higher range noise). Noise on a single endpoint can bias the reference line enough to cause valid interior points to exceed the residual threshold, splitting one physical wall into multiple segments. TLS uses all points equally, so no single noisy measurement dominates.

### 3.2 Incremental Line Growing

The algorithm processes the ordered points in each gap-separated segment:

1. Set `start = 0`.
2. Add points one at a time, maintaining the current candidate segment `points[start:i]`.
3. Compute the TLS residual of the candidate.
4. If the residual exceeds `grow_residual_threshold` (default: 0.03 m):
   - Attempt to finalise `points[start:i]` (requires $\geq$ `min_points` and minimum length).
   - Restart from `start = i − 1` so the boundary point is shared with the new segment.
5. Finalise the last segment when all points are consumed.

The threshold $\tau = 0.03\,\mathrm{m} \approx 3\sigma_{\mathrm{LiDAR}}$ is set at three times the LiDAR range noise.

### 3.3 Adjacent Segment Merge

After growing, consecutive segment pairs are evaluated for merging. Two segments are merged when:

1. Their angular difference is below `merge_angle_tol` (default: 0.35 rad $\approx 20°$).
2. The endpoint gap between the last point of the first segment and the first point of the second segment is below `max_gap`.
3. The TLS residual of the combined candidate is below `merge_rho_tolerance` (default: 0.15 m).

The merge pass is applied iteratively until no more merges occur. This step recovers walls that were split during growth because a gap or noisy point momentarily pushed the residual above the threshold.

---

## 4. Hessian Normal Form

### 4.1 Definition

A line is represented in **Hessian normal form** as $(\rho, \alpha)$, satisfying

$$
x\cos\alpha + y\sin\alpha = \rho,
$$

where $\rho \geq 0$ is the perpendicular distance from the origin to the line, and $\alpha \in [-\pi, \pi]$ is the angle of the outward normal $\mathbf{n} = [\cos\alpha,\; \sin\alpha]^\top$.

The constraint $\rho \geq 0$ makes the representation unique: every line has exactly one $(\rho, \alpha)$ pair. This is the canonical form used throughout the EKF state.

The foot of the perpendicular from the origin to the line is

$$
\mathbf{p}_{\mathrm{foot}} = \rho\,[\cos\alpha,\; \sin\alpha]^\top.
$$

### 4.2 Conversion from Segmented Points

Given the set of scan points assigned to a segment, the Hessian parameters are computed from the TLS fit:

1. Compute the centroid $\bar{\mathbf{p}}$ and the TLS normal $\mathbf{n}$ (Section 3.1).
2. Compute the signed distance: $\rho_{\mathrm{signed}} = \bar{\mathbf{p}} \cdot \mathbf{n}$.
3. Enforce $\rho \geq 0$: if $\rho_{\mathrm{signed}} < 0$, set $\rho = -\rho_{\mathrm{signed}}$ and $\mathbf{n} \leftarrow -\mathbf{n}$.
4. Set $\alpha = \mathrm{atan2}(n_y, n_x)$.

Using the TLS centroid for $\rho$ and the TLS normal for $\alpha$ is more accurate than the previous practice of using the segment midpoint (average of endpoints) and the endpoint-to-endpoint direction, because the TLS estimates are statistically optimal under isotropic Gaussian noise.

### 4.3 Coordinate Frame Transformations

To transform a wall $(\rho_r, \alpha_r)$ in the robot frame to the map frame:

$$
\alpha_m = \alpha_r + \theta_r, \qquad \rho_m = \rho_r + x_r\cos\alpha_m + y_r\sin\alpha_m.
$$

The inverse — predicting the robot-frame observation from a map-frame landmark — is:

$$
\hat{\rho}_r = \rho_m - (x_r\cos\alpha_m + y_r\sin\alpha_m), \qquad \hat{\alpha}_r = \alpha_m - \theta_r.
$$

---

## 5. Corner Detection

### 5.1 Definition

A corner is the intersection of two non-parallel line segments. It appears at wall junctions and provides a 2D point constraint.

### 5.2 Detection from Adjacent Lines

Corners are extracted from **adjacent** line segments only — pairs of segments that are consecutive in the ordered scan and share a common endpoint region. Checking all $\binom{M}{2}$ pairs would introduce spurious matches between distant, unrelated walls.

For each adjacent pair $(\ell_a, \ell_b)$:

1. **Angle check.** Compute the angle between the two lines. Skip if $|\alpha_b - \alpha_a| < \theta_{\min}$ (default $\theta_{\min} = 50°$). Near-parallel lines produce a geometrically unstable intersection.
2. **Proximity check.** Verify that the nearest endpoints of the two segments are within `max_endpoint_dist` of each other.
3. **Intersection.** Solve the $2 \times 2$ linear system (Section 5.3).
4. **Proximity to endpoints.** Accept the corner only if the intersection point lies near the actual segment endpoints.

### 5.3 Line Intersection

Given lines $(\rho_a, \alpha_a)$ and $(\rho_b, \alpha_b)$, the intersection solves

$$
\mathbf{A}\,\mathbf{c} = \mathbf{b}, \qquad
\mathbf{A} = \begin{bmatrix}\cos\alpha_a & \sin\alpha_a \\ \cos\alpha_b & \sin\alpha_b\end{bmatrix}, \quad
\mathbf{b} = \begin{bmatrix}\rho_a \\ \rho_b\end{bmatrix}.
$$

The solution exists when $|\det\mathbf{A}| = |\sin(\alpha_b - \alpha_a)| > \epsilon$. The implementation uses $\epsilon = 0.1$ ($\approx \sin(5.7°)$) as a numerical stability threshold, which is tighter than the geometric threshold of $50°$ but guards against near-singular matrix inversion.

$$
\mathbf{c} = \mathbf{A}^{-1}\mathbf{b}.
$$

---

## 6. Covariance Estimation

### 6.1 Wall Covariance

The uncertainty in wall parameters $(\rho, \alpha)$ is derived from the Cramér–Rao lower bound for line fitting under isotropic Gaussian noise.

**Measurement model.** Each scan point $(p_x^i, p_y^i)$ lies on the wall with additive noise $\boldsymbol{\delta}_i \sim \mathcal{N}(\mathbf{0}, \sigma^2\mathbf{I})$. The constraint $h_i = \rho - p_x^i\cos\alpha - p_y^i\sin\alpha$ has noise variance

$$
\mathrm{Var}(h_i) = \sigma^2(\cos^2\alpha + \sin^2\alpha) = \sigma^2,
$$

independent of point position or wall orientation. The per-point residual Jacobian with respect to $(\rho, \alpha)$ is

$$
\mathbf{J}_i = \begin{bmatrix} 1 & p_x^i\sin\alpha - p_y^i\cos\alpha \end{bmatrix} \in \mathbb{R}^{1\times 2}.
$$

Note: $\partial h_i/\partial\rho = 1$ and $\partial h_i/\partial\alpha = p_x^i\sin\alpha - p_y^i\cos\alpha$ is the tangential coordinate of point $i$ along the wall.

**Fisher information matrix.** Summing over all $N$ supporting points:

$$
\mathbf{A} = \frac{1}{\sigma^2}\sum_{i=1}^N \mathbf{J}_i^\top \mathbf{J}_i
= \frac{1}{\sigma^2}\begin{bmatrix} N & \sum_i t_i \\ \sum_i t_i & \sum_i t_i^2 \end{bmatrix},
$$

where $t_i = p_x^i\sin\alpha - p_y^i\cos\alpha$ is the signed arc-length of point $i$ along the wall measured from the foot of the perpendicular.

**Cramér–Rao lower bound.** The covariance is

$$
\mathrm{Cov}(\rho, \alpha) = \sigma^2\,\mathbf{A}^{-1}\bigl(\sigma^2\mathbf{A}\bigr)^{-1}\,\sigma^2
\quad \Longrightarrow \quad
\boxed{\mathrm{Cov}(\rho, \alpha) = \sigma^2\,\mathbf{A}^{-1}},
$$

where $\sigma = 0.01\,\mathrm{m}$ is fixed from the LDS-01 manufacturer specification.

Using a fixed $\sigma$ rather than estimating it from residuals is important. A residual-based estimate $\hat{\sigma}^2 = \sum r_i^2/(N-2)$ decreases as more points are added, making the filter progressively overconfident in long-wall observations. The fixed-$\sigma$ approach separates sensor physics ($\sigma^2$) from geometric information ($\mathbf{A}^{-1}$), so adding more points increases $\mathbf{A}$ (more information) while $\sigma^2$ stays calibrated (Censi, 2007).

**Physical interpretation.**

$$
\mathrm{Var}(\rho) \approx \frac{\sigma^2}{N}, \qquad
\mathrm{Var}(\alpha) \approx \frac{\sigma^2}{\sum_i t_i^2}.
$$

$\mathrm{Var}(\rho)$ decreases with more points. $\mathrm{Var}(\alpha)$ decreases with a longer wall (larger spread of $t_i$). A short wall observed by few points is uncertain in both parameters; a long wall observed by many points is nearly certain.

**Robustness.** When $\mathbf{A}$ is near-singular (very short segment, insufficient point spread), the eigenvalues of $\mathbf{A}$ are clamped to a minimum before inversion, and the output covariance eigenvalues are clamped to a minimum. This prevents overconfidence in degenerate extractions.

### 6.2 Corner Covariance

A corner at $\mathbf{c} = (c_x, c_y)$ is the intersection of two walls. Its position depends on four wall parameters $\boldsymbol{\theta} = [\rho_a, \alpha_a, \rho_b, \alpha_b]^\top$. First-order error propagation gives

$$
\mathrm{Cov}(\mathbf{c}) = \mathbf{J}\,\boldsymbol{\Sigma}_\theta\,\mathbf{J}^\top,
$$

where $\mathbf{J} = \partial \mathbf{c}/\partial\boldsymbol{\theta} \in \mathbb{R}^{2\times 4}$ and $\boldsymbol{\Sigma}_\theta = \mathrm{blkdiag}(\mathrm{Cov}_a, \mathrm{Cov}_b)$ is the block-diagonal covariance of the two parent walls (assumed independent).

**Jacobian derivation.** From $\mathbf{c} = \mathbf{A}^{-1}\mathbf{b}$:

$$
\frac{\partial \mathbf{c}}{\partial \rho_a} = \mathbf{A}^{-1}\mathbf{e}_1, \qquad
\frac{\partial \mathbf{c}}{\partial \rho_b} = \mathbf{A}^{-1}\mathbf{e}_2,
$$

$$
\frac{\partial \mathbf{c}}{\partial \alpha_k} = -\mathbf{A}^{-1}\!\left(\frac{\partial \mathbf{A}}{\partial \alpha_k}\right)\!\mathbf{c},
$$

where $\partial\mathbf{A}/\partial\alpha_a = [[-\sin\alpha_a, \cos\alpha_a],\,[0,0]]^\top$ and similarly for $\alpha_b$. This is the standard formula for differentiating a matrix inverse: $\partial(\mathbf{A}^{-1}\mathbf{b})/\partial\alpha = \mathbf{A}^{-1}(\partial\mathbf{b}/\partial\alpha) - \mathbf{A}^{-1}(\partial\mathbf{A}/\partial\alpha)\mathbf{A}^{-1}\mathbf{b}$. Since $\partial\mathbf{b}/\partial\alpha_k = \mathbf{0}$, only the second term survives.

The full Jacobian is assembled as

$$
\mathbf{J} = \bigl[\,\partial_{\rho_a}\mathbf{c}\;\big|\;\partial_{\alpha_a}\mathbf{c}\;\big|\;\partial_{\rho_b}\mathbf{c}\;\big|\;\partial_{\alpha_b}\mathbf{c}\,\bigr].
$$

**Output conditioning.** The result $\mathbf{J}\boldsymbol{\Sigma}_\theta\mathbf{J}^\top$ is symmetrised and eigenvalue-clamped to guarantee positive definiteness, as numerical cancellation near the singular geometry boundary can produce small negative eigenvalues.

---

## 7. Implementation Reference

| Function | File | Description |
|---|---|---|
| `_scan_to_cartesian` | `landmark_features.py` | Polar-to-Cartesian conversion with range filtering |
| `_split_on_gaps` | `landmark_features.py` | Gap detection and segment splitting |
| `_fit_line_tls` | `landmark_features.py` | SVD-based TLS line fit; returns centroid, direction, normal |
| `_segment_residual_tls` | `landmark_features.py` | Max perpendicular distance to TLS line |
| `_grow_lines_incremental` | `landmark_features.py` | Incremental line growing with TLS residual |
| `_merge_adjacent_lines` | `landmark_features.py` | Adjacent segment merge with TLS residual check |
| `_convert_line_to_hessian` | `landmark_features.py` | TLS centroid and normal → $(\rho, \alpha)$ |
| `_extract_corners_from_adjacent_lines` | `landmark_features.py` | Corner detection from adjacent pairs only |
| `_compute_wall_covariance` | `landmark_features.py` | Fisher information / CRLB wall covariance |
| `_compute_corner_covariance` | `landmark_features.py` | Jacobian propagation through intersection formula |
| `extract_features` | `landmark_features.py` | Complete scan → walls + corners pipeline |

### 7.1 Key Parameters

| Parameter | Default | Effect |
|---|---|---|
| `grow_residual_threshold` | 0.03 m | Split threshold during incremental growth ($\approx 3\sigma$) |
| `min_points_per_line` | 10 | Minimum supporting points for a valid segment |
| `min_line_length` | 0.5 m | Minimum segment length |
| `merge_angle_tol` | 0.35 rad | Maximum angular difference for merge |
| `merge_rho_tolerance` | 0.15 m | Maximum TLS residual of merged candidate |
| `corner_angle_threshold` | 50° | Minimum intersection angle for a valid corner |
| `max_gap` | 0.2 m | Maximum gap for gap detection and merge proximity |

---

## References

1. **Nguyen, V., Martinelli, A., Tomatis, N., & Siegwart, R. (2005).** "A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder for Indoor Mobile Robotics." *Proceedings of IEEE/RSJ IROS*, pp. 1929–1934.

2. **Pavlidis, T., & Horowitz, S. L. (1974).** "Segmentation of Plane Curves." *IEEE Transactions on Computers*, C-23(8), 860–870.

3. **Duda, R. O., & Hart, P. E. (1972).** "Use of the Hough Transformation to Detect Lines and Curves in Pictures." *Communications of the ACM*, 15(1), 11–15.

4. **Duda, R. O., & Hart, P. E. (1973).** *Pattern Classification and Scene Analysis*. Wiley.

5. **Fischler, M. A., & Bolles, R. C. (1981).** "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

6. **Censi, A. (2007).** "An Accurate Closed-Form Estimate of ICP's Covariance." *Proceedings of IEEE ICRA*, pp. 3167–3172.

7. **Pfister, S. T., Kriechbaum, K. L., Roumeliotis, S. I., & Burdick, J. W. (2002).** "Weighted Line Fitting Algorithms for Mobile Robot Map Building and Efficient Data Representation." *Proceedings of IEEE ICRA*, pp. 1304–1311.

8. **Jolliffe, I. T. (2002).** *Principal Component Analysis* (2nd ed.). Springer.

9. **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011).** *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

10. **Castellanos, J. A., Montiel, J. M. M., Neira, J., & Tardós, J. D. (1999).** "The SPmap: A Probabilistic Framework for Simultaneous Localization and Map Building." *IEEE Transactions on Robotics and Automation*, 15(5), 948–952.
