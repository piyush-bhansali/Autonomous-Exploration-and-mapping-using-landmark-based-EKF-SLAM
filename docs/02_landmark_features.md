# Landmark Feature Extraction: Theory and Implementation

## Table of Contents

1. [Introduction](#1-introduction)
2. [Scan Pre-processing](#2-scan-pre-processing)
3. [Line Segmentation](#3-line-segmentation)
4. [Hessian Normal Form](#4-hessian-normal-form)
5. [Corner Detection](#5-corner-detection)
6. [Measurement Uncertainty](#6-measurement-uncertainty)
7. [Implementation Reference](#7-implementation-reference)

---

## 1. Introduction

Feature-based SLAM requires reducing the raw sensor stream to a compact set of geometric primitives whose positions can be maintained in a probabilistic state vector. For a 2D LiDAR operating in a structured indoor environment, the dominant surface geometry is planar: walls, doors, and columns all produce straight line segments in the scan plane. Two feature types are extracted from each scan:

- **Wall landmarks** — infinite-line abstractions of planar surfaces, parameterised in Hessian normal form $(\rho, \alpha)$.
- **Corner landmarks** — point features at wall junctions, parameterised as Cartesian coordinates $(c_x, c_y)$.

Walls provide bearing and distance constraints to linear surfaces and are abundant in indoor environments. Corners complement walls by providing point constraints, fully determining a 2D position from a single feature observation when the corner is well-conditioned geometrically.

### 1.1 Feature Type Comparison

| Property | Wall $(\rho, \alpha)$ | Corner $(c_x, c_y)$ |
|---|---|---|
| Constraint type | 1-DOF (distance to line) | 2-DOF (point position) |
| Abundance | High | Moderate |
| Geometric stability | Requires $\geq$ 2 non-parallel walls | Single feature sufficient |
| Observability | Degenerates in corridor environments | Stable at wall junctions |

### 1.2 Choice of Line Extraction Method

Four principal algorithms extract line segments from 2D range data (Nguyen et al., 2005):

| Method | Time Complexity | Deterministic | Notes |
|---|---|---|---|
| Split-and-Merge (Pavlidis & Horowitz, 1974) | $O(n \log n)$ | Yes | Recursive subdivision; sensitive to noise at segment boundaries |
| **Incremental** (Duda & Hart, 1973) | $O(n)$ | Yes | Sequential point admission; used here |
| Hough Transform (Duda & Hart, 1972) | $O(n^2)$ | Yes | Robust to occlusions; expensive for online use |
| RANSAC (Fischler & Bolles, 1981) | $O(n)$ amortised | No | Outlier-robust; non-deterministic |

This system uses an **incremental growing + adjacent merge** pipeline. Incremental growing is $O(n)$ and deterministic, making it suitable for real-time operation at LiDAR scan rates (5 Hz). A subsequent merge pass recovers long walls that were split by threshold effects during growth. All geometric decisions are based on a **Total Least Squares (TLS)** residual rather than endpoint-to-endpoint distances, for reasons explained in Section 3.1.

---

## 2. Scan Pre-processing

### 2.1 Polar-to-Cartesian Conversion

A 2D LiDAR scan delivers $M$ range-bearing measurements:

$$
\mathcal{S} = \{(r_i,\,\varphi_i)\}_{i=1}^{M}, \qquad \varphi_i = \varphi_{\min} + (i-1)\,\Delta\varphi.
$$

Readings that fall outside the sensor's valid range interval $[r_{\min}, r_{\max}]$ or contain non-finite values are discarded. Valid readings are converted to Cartesian coordinates in the robot frame:

$$
\mathbf{p}_i = \begin{bmatrix} x_i \\ y_i \end{bmatrix} = r_i \begin{bmatrix} \cos\varphi_i \\ \sin\varphi_i \end{bmatrix}.
$$

For the TurtleBot3 LDS-01 sensor: $r_{\min} = 0.12\,\mathrm{m}$, $r_{\max} = 3.5\,\mathrm{m}$, $M = 360$ samples at $\Delta\varphi = 1°$.

### 2.2 Gap-Based Segmentation

Points on distinct surfaces are separated by a spatial discontinuity in the ordered scan sequence. Two consecutive valid points $\mathbf{p}_i$ and $\mathbf{p}_{i+1}$ are assigned to different segments when

$$
\|\mathbf{p}_{i+1} - \mathbf{p}_i\| > \tau_{\mathrm{gap}},
$$

with $\tau_{\mathrm{gap}} = 0.2\,\mathrm{m}$. The result is a partition of the valid point set into $K$ **continuous segments** $\{\mathcal{C}_k\}_{k=1}^K$, each containing points from a single connected surface.

Segments with fewer than $N_{\min} = 10$ points are discarded before further processing, as they do not provide statistically reliable line fits.

---

## 3. Line Segmentation

### 3.1 Total Least Squares Line Fit

All geometric quality decisions throughout the pipeline — residual-based splitting during incremental growth, and residual-based acceptance during the merge pass — are computed from a **Total Least Squares (TLS)** line fit.

**Definition.** Given a set of $N$ points $\{\mathbf{p}_i\}$, the TLS line minimises the sum of squared *perpendicular* distances from each point to the fitted line, rather than the squared vertical distances used by ordinary least squares. This is the statistically correct criterion when both coordinates are corrupted by isotropic noise, as is the case for 2D LiDAR returns.

**Algorithm.** Compute the centroid:

$$
\bar{\mathbf{p}} = \frac{1}{N}\sum_{i=1}^N \mathbf{p}_i.
$$

Centre the data and form the $N \times 2$ matrix $\widetilde{\mathbf{P}}$, whose $i$-th row is $(\mathbf{p}_i - \bar{\mathbf{p}})^\top$. Compute the thin SVD:

$$
\widetilde{\mathbf{P}} = \mathbf{U}\,\boldsymbol{\Sigma}\,\mathbf{V}^\top, \qquad \mathbf{V} = [\mathbf{v}_1 \;\; \mathbf{v}_2].
$$

The first right singular vector $\mathbf{v}_1$ (associated with the largest singular value $\sigma_1$) is the principal axis of the point cloud, i.e., the best-fit line direction. The unit normal to the line is the second right singular vector:

$$
\hat{\mathbf{n}} = \mathbf{v}_2 = [-v_{1y},\; v_{1x}]^\top.
$$

The signed perpendicular distance from any point $\mathbf{p}_i$ to this line is:

$$
d_i = (\mathbf{p}_i - \bar{\mathbf{p}}) \cdot \hat{\mathbf{n}}.
$$

**Segment residual.** The scalar quality metric used throughout the pipeline is the maximum absolute perpendicular deviation:

$$
r_{\mathrm{TLS}} = \max_{i \in \{1,\ldots,N\}} |d_i|.
$$

**Rationale for TLS over endpoint-based residuals.** An endpoint-to-endpoint reference line uses only the two noisiest measurements in a segment (LiDAR points near grazing incidence angles have higher range noise). Noise on a single endpoint biases the reference line, causing valid interior points to exceed the threshold and splitting one physical wall into multiple short segments. TLS uses all $N$ points equally, making the fit statistically optimal and robust to single-point outliers.

### 3.2 Incremental Line Growing

Each continuous segment $\mathcal{C}_k$ is processed by the incremental growing algorithm, which produces a list of line segments that satisfy the TLS residual threshold $\tau_r$.

**Algorithm.** Let the points of $\mathcal{C}_k$ be ordered $\mathbf{p}_0, \mathbf{p}_1, \ldots, \mathbf{p}_{n-1}$.

1. Set $\text{start} \leftarrow 0$, $i \leftarrow 1$.
2. Compute the TLS residual $r_{\mathrm{TLS}}$ of the candidate window $\{\mathbf{p}_{\text{start}}, \ldots, \mathbf{p}_i\}$.
3. If $r_{\mathrm{TLS}} \leq \tau_r$: advance $i \leftarrow i+1$ and repeat from step 2.
4. If $r_{\mathrm{TLS}} > \tau_r$: the point $\mathbf{p}_i$ violates the residual bound.
   - Attempt to finalise the candidate $\{\mathbf{p}_{\text{start}}, \ldots, \mathbf{p}_{i-1}\}$ (requires $N \geq N_{\min}$ and $\ell \geq \ell_{\min}$).
   - Set $\text{start} \leftarrow \max(i-1,\,\text{start}+1)$ — retaining $\mathbf{p}_{i-1}$ as the first point of the next candidate ensures continuity.
   - Set $i \leftarrow \text{start} + 1$ and repeat from step 2.
5. Finalise the remaining window $\{\mathbf{p}_{\text{start}}, \ldots, \mathbf{p}_{n-1}\}$ when $i = n$.

A candidate segment is accepted as a valid line only if:

$$
N \geq N_{\min} = 10, \qquad \ell = \|\mathbf{p}_{\text{last}} - \mathbf{p}_{\text{first}}\| \geq \ell_{\min} = 0.3\,\mathrm{m}.
$$

The threshold $\tau_r = 0.03\,\mathrm{m}$ is set at three times the LiDAR range standard deviation $\sigma = 0.01\,\mathrm{m}$, i.e., $\tau_r \approx 3\sigma$, so a wall point is admitted if its perpendicular deviation is within the $3\sigma$ noise band.

### 3.3 Adjacent Segment Merge

Incremental growing can split a single physical wall into two or three segments when a gap or momentary noise spike pushes the residual above $\tau_r$ at an interior point. The merge pass examines consecutive segment pairs and recovers these false splits.

**Merge criterion.** Two adjacent segments $\mathcal{L}_a$ and $\mathcal{L}_b$ (in that order) are merged when all three of the following hold:

1. **Angular similarity:**
$$
\Delta\theta_{ab} = \arccos\bigl(|\hat{\mathbf{d}}_a \cdot \hat{\mathbf{d}}_b|\bigr) \leq \tau_\theta = 0.22\,\mathrm{rad}\;(\approx 12.6°),
$$
where $\hat{\mathbf{d}}_a$ and $\hat{\mathbf{d}}_b$ are the unit direction vectors of each segment (endpoint-to-endpoint).

2. **Endpoint proximity:**
$$
\|\mathbf{p}_{\mathrm{last}}(\mathcal{L}_a) - \mathbf{p}_{\mathrm{first}}(\mathcal{L}_b)\| \leq \tau_{\mathrm{gap}} = 0.2\,\mathrm{m}.
$$

3. **Joint residual:**
$$
r_{\mathrm{TLS}}\bigl(\mathcal{L}_a \cup \mathcal{L}_b\bigr) \leq \tau_{\mathrm{merge}} = 0.15\,\mathrm{m}.
$$

All three conditions must hold simultaneously. The merge pass is applied iteratively until no further merges occur. This recovers long walls regardless of how many times they were split during growth.

---

## 4. Hessian Normal Form

### 4.1 Definition

Every valid line segment is abstracted as an infinite line represented in **Hessian normal form**:

$$
\boxed{x\cos\alpha + y\sin\alpha = \rho,}
$$

where:
- $\rho \geq 0$ is the perpendicular distance from the coordinate origin to the line.
- $\alpha \in (-\pi, \pi]$ is the angle of the outward unit normal $\hat{\mathbf{n}} = [\cos\alpha,\;\sin\alpha]^\top$ with respect to the positive $x$-axis.

The constraint $\rho \geq 0$ makes the representation unique: every line in the plane has exactly one $(\rho, \alpha)$ pair. This uniqueness is essential for consistent landmark storage in the EKF state vector.

The foot of the perpendicular from the origin to the line is the closest point on the wall to the sensor origin:

$$
\mathbf{p}_{\perp} = \rho\,[\cos\alpha,\;\sin\alpha]^\top = \rho\,\hat{\mathbf{n}}.
$$

The wall tangent vector (parallel to the wall) is:

$$
\hat{\mathbf{t}} = [-\sin\alpha,\;\cos\alpha]^\top, \qquad \hat{\mathbf{t}} \perp \hat{\mathbf{n}}.
$$

Any point on the wall can be written as:

$$
\mathbf{p}(t) = \rho\,\hat{\mathbf{n}} + t\,\hat{\mathbf{t}}, \qquad t \in \mathbb{R},
$$

where $t$ is the signed arc-length from $\mathbf{p}_\perp$ along the wall. The scalar $t$ is used throughout the endpoint management system to represent wall spatial extent without redundancy.

### 4.2 Conversion from TLS Line Fit

Given the TLS centroid $\bar{\mathbf{p}}$ and normal $\hat{\mathbf{n}}$ computed from Section 3.1:

1. Compute the signed distance: $\rho_s = \bar{\mathbf{p}} \cdot \hat{\mathbf{n}}$.
2. Enforce $\rho \geq 0$:

$$
(\rho,\,\hat{\mathbf{n}}) \leftarrow \begin{cases} (\rho_s,\;\hat{\mathbf{n}}) & \text{if } \rho_s \geq 0 \\ (-\rho_s,\;-\hat{\mathbf{n}}) & \text{if } \rho_s < 0. \end{cases}
$$

3. Set $\alpha = \mathrm{atan2}(n_y, n_x)$.

Using the TLS centroid for computing $\rho$ is statistically optimal: any point on the true wall satisfies $\mathbf{p} \cdot \hat{\mathbf{n}} = \rho$, so the centroid of wall points gives the minimum-variance estimate of $\rho$ under isotropic Gaussian noise. The endpoint-based midpoint $\tfrac{1}{2}(\mathbf{p}_{\mathrm{first}} + \mathbf{p}_{\mathrm{last}})$ is less accurate because it depends only on the two (potentially noisiest) boundary measurements.

### 4.3 Coordinate Frame Transformations

Let the robot pose in the map frame be $\mathbf{x}_r = (x_r,\,y_r,\,\theta_r)^\top$.

**Robot frame to map frame.** A wall observed in robot frame as $(\rho_r, \alpha_r)$ is expressed in map frame as:

$$
\alpha_m = \alpha_r + \theta_r, \qquad
\rho_m = \rho_r + x_r\cos\alpha_m + y_r\sin\alpha_m.
$$

**Derivation.** In the map frame the wall equation is $\mathbf{p}_m \cdot \hat{\mathbf{n}}_m = \rho_m$. The same physical point in robot frame is $\mathbf{p}_r = \mathbf{R}_r^\top(\mathbf{p}_m - \mathbf{t}_r)$ where $\mathbf{R}_r$ is the robot rotation matrix and $\mathbf{t}_r = [x_r, y_r]^\top$. Substituting into the robot-frame wall equation and expanding gives the expressions above.

**Map frame to robot frame (EKF measurement prediction).** From a stored map-frame landmark $(\rho_m, \alpha_m)$ and robot pose $(x_r, y_r, \theta_r)$, the predicted robot-frame observation is:

$$
\hat{\rho}_r = \rho_m - \bigl(x_r\cos\alpha_m + y_r\sin\alpha_m\bigr), \qquad
\hat{\alpha}_r = \alpha_m - \theta_r.
$$

These prediction equations are used verbatim in the EKF observation model $h(\mathbf{x})$ for wall landmarks.

---

## 5. Corner Detection

### 5.1 Definition and Role

A corner is the intersection point $\mathbf{c} \in \mathbb{R}^2$ of two non-parallel line segments. It represents a wall junction — a stable, compact feature that provides a 2D point constraint in the EKF update. While a single wall observation constrains only one degree of freedom (the perpendicular distance), a single corner observation constrains both translational degrees of freedom.

### 5.2 Adjacent-Only Detection Strategy

Corner candidates are extracted only from **adjacent** segment pairs — consecutive segments in the ordered scan that share a common endpoint neighbourhood. Checking all $\binom{M}{2}$ pairs (where $M$ is the number of extracted walls) would introduce spurious matches between geometrically unrelated walls, particularly in corridor environments where many parallel walls are simultaneously visible.

The adjacency constraint is geometrically motivated: a physical wall junction produces two consecutive segments in the scan because the LiDAR sweeps across both walls in order. A true corner's two parent walls are always adjacent in the scan sequence.

### 5.3 Corner Validity Criterion

For each adjacent pair $(\mathcal{L}_a, \mathcal{L}_b)$, a corner is accepted only if the two segments meet at a sufficiently large angle. The angle between the two line directions is:

$$
\Delta\phi_{ab} = \arccos\bigl(|\hat{\mathbf{d}}_a \cdot \hat{\mathbf{d}}_b|\bigr).
$$

The pair is accepted as a corner candidate if:

$$
\Delta\phi_{ab} \geq \phi_{\min} = 50°.
$$

Near-parallel segments ($\Delta\phi_{ab} < \phi_{\min}$) are skipped. The intersection of nearly parallel walls is geometrically ill-conditioned: a small angular uncertainty $\delta\alpha$ produces a large positional uncertainty $\delta c \approx \rho / \sin(\Delta\phi_{ab})$, which diverges as $\Delta\phi_{ab} \to 0$.

### 5.4 Line Intersection

Given two lines in Hessian normal form $(\rho_a, \alpha_a)$ and $(\rho_b, \alpha_b)$, the intersection solves the $2 \times 2$ linear system:

$$
\mathbf{A}\,\mathbf{c} = \mathbf{b}, \qquad
\mathbf{A} = \begin{bmatrix}\cos\alpha_a & \sin\alpha_a \\ \cos\alpha_b & \sin\alpha_b\end{bmatrix}, \quad
\mathbf{b} = \begin{bmatrix}\rho_a \\ \rho_b\end{bmatrix}.
$$

The system has a unique solution when $\det\mathbf{A} = \sin(\alpha_b - \alpha_a) \neq 0$. An additional numerical stability check rejects near-singular systems:

$$
|\det\mathbf{A}| = |\sin(\alpha_b - \alpha_a)| > \epsilon_{\mathrm{det}} = 0.1 \quad (\approx \sin 5.7°).
$$

Note that this threshold ($5.7°$) is tighter than the geometric acceptance threshold ($50°$). The geometric check rejects any pair with $\Delta\phi < 50°$; for accepted pairs, the determinant check provides a secondary guard against numerical failure in the matrix solve.

The corner position is:

$$
\mathbf{c} = \mathbf{A}^{-1}\mathbf{b} = \frac{1}{\det\mathbf{A}}\begin{bmatrix}\sin\alpha_b & -\sin\alpha_a \\ -\cos\alpha_b & \cos\alpha_a\end{bmatrix}\begin{bmatrix}\rho_a \\ \rho_b\end{bmatrix}.
$$

If the matrix solve fails numerically (singular $\mathbf{A}$), the midpoint of the shared endpoint region, $\tfrac{1}{2}(\mathbf{p}_{\mathrm{last}}(\mathcal{L}_a) + \mathbf{p}_{\mathrm{first}}(\mathcal{L}_b))$, is used as a fallback.

---

## 6. Measurement Uncertainty

A core requirement of the EKF is that each observation $\mathbf{z}$ is accompanied by a measurement covariance $\mathbf{R} = \mathrm{Cov}(\mathbf{z})$ that accurately reflects extraction noise. Overestimating $\mathbf{R}$ causes the filter to be overly conservative; underestimating it causes the filter to trust observations more than is warranted and can lead to divergence.

The noise source is the LiDAR range measurement: $r_i = r_i^* + \epsilon_i$, where $r_i^*$ is the true range and $\epsilon_i \sim \mathcal{N}(0, \sigma^2)$ with $\sigma = 0.01\,\mathrm{m}$ (LDS-01 specification). Under this model, the Cartesian noise of each point is approximately isotropic and Gaussian:

$$
\mathbf{p}_i = \mathbf{p}_i^* + \boldsymbol{\delta}_i, \qquad \boldsymbol{\delta}_i \sim \mathcal{N}(\mathbf{0},\,\sigma^2\mathbf{I}).
$$

### 6.1 Wall Covariance — Cramér–Rao Lower Bound

The uncertainty of the extracted Hessian parameters $(\rho, \alpha)$ is derived from the Fisher Information Matrix (FIM) evaluated at the true wall.

**Observation model.** For a point $\mathbf{p}_i = [p_x^i, p_y^i]^\top$ on the wall, the constraint $h_i = \rho - p_x^i\cos\alpha - p_y^i\sin\alpha = 0$ holds exactly. With additive point noise $\boldsymbol{\delta}_i$, the noisy constraint is $h_i \sim \mathcal{N}(0, \sigma_h^2)$ where:

$$
\sigma_h^2 = \hat{\mathbf{n}}^\top(\sigma^2\mathbf{I})\hat{\mathbf{n}} = \sigma^2\bigl(\cos^2\alpha + \sin^2\alpha\bigr) = \sigma^2.
$$

The constraint noise variance equals $\sigma^2$ regardless of wall orientation or point position — a useful property of the Hessian parameterisation.

**Per-point Jacobian.** The gradient of $h_i$ with respect to $(\rho, \alpha)$:

$$
\mathbf{J}_i = \frac{\partial h_i}{\partial[\rho,\,\alpha]} = \begin{bmatrix} 1 & \;p_x^i\sin\alpha - p_y^i\cos\alpha \end{bmatrix} = \begin{bmatrix} 1 & t_i \end{bmatrix},
$$

where $t_i = p_x^i\sin\alpha - p_y^i\cos\alpha = \mathbf{p}_i \cdot \hat{\mathbf{t}}$ is the signed arc-length of point $i$ along the wall tangent, measured from the foot of the perpendicular $\mathbf{p}_\perp$.

**Fisher Information Matrix.** Summing over all $N$ supporting points:

$$
\mathbf{F} = \frac{1}{\sigma^2}\sum_{i=1}^N \mathbf{J}_i^\top\mathbf{J}_i
= \frac{1}{\sigma^2}\begin{bmatrix} N & \displaystyle\sum_i t_i \\[6pt] \displaystyle\sum_i t_i & \displaystyle\sum_i t_i^2 \end{bmatrix}.
$$

**Cramér–Rao Lower Bound.** The minimum achievable covariance of any unbiased estimator of $(\rho, \alpha)$ is:

$$
\boxed{\mathrm{Cov}(\rho,\alpha) \geq \mathbf{F}^{-1} = \sigma^2\left(\sum_{i=1}^N \mathbf{J}_i^\top\mathbf{J}_i\right)^{-1}}.
$$

**Physical interpretation.** Expanding $\mathbf{F}^{-1}$ in the centred coordinate system where $\sum_i t_i = 0$ (which holds exactly when the centroid lies at the foot of perpendicular):

$$
\mathrm{Var}(\rho) \approx \frac{\sigma^2}{N}, \qquad \mathrm{Var}(\alpha) \approx \frac{\sigma^2}{\displaystyle\sum_i t_i^2}.
$$

- $\mathrm{Var}(\rho)$ decreases as $1/N$: more points give a better distance estimate.
- $\mathrm{Var}(\alpha)$ decreases as the wall grows longer (larger $\sum t_i^2 = N\cdot\overline{t^2}$, where $\overline{t^2}$ is the mean squared arc-length spread). A short wall observed by few points is uncertain in both parameters; a long wall observed by many points is nearly certain in angle.

**Why sensor-specified $\sigma$, not a residual estimate.** A residual-based estimate $\hat{\sigma}^2 = \sum r_i^2 / (N-2)$ decreases as more points are added, making the filter progressively overconfident in long-wall observations. Using the manufacturer-specified $\sigma = 0.01\,\mathrm{m}$ separates sensor physics (which is constant) from geometric information content (which is captured by $\mathbf{F}^{-1}$), so the covariance correctly reflects both the sensor noise floor and the information gained from a longer wall (Censi, 2007).

**Numerical conditioning.** When $\mathbf{F}$ is near-singular — as occurs for very short segments with fewer than $\sim$3 distinct tangential positions — the eigenvalues of $\mathbf{F}$ are clamped to a minimum value $\lambda_{\min} = \sigma^2 \cdot 0.1$ before inversion, and the output covariance eigenvalues are clamped to $\sigma^2 \cdot 10^{-4}$ to prevent overconfidence.

### 6.2 Corner Covariance — Jacobian Error Propagation

A corner $\mathbf{c} = [c_x, c_y]^\top$ is a deterministic function of the four wall parameters:

$$
\boldsymbol{\theta} = [\rho_a,\;\alpha_a,\;\rho_b,\;\alpha_b]^\top, \qquad \mathbf{c} = \mathbf{A}(\boldsymbol{\theta})^{-1}\,\mathbf{b}(\boldsymbol{\theta}).
$$

First-order (linearised) error propagation gives the corner covariance:

$$
\boxed{\mathrm{Cov}(\mathbf{c}) = \mathbf{J}\,\boldsymbol{\Sigma}_\theta\,\mathbf{J}^\top},
$$

where $\mathbf{J} = \partial\mathbf{c}/\partial\boldsymbol{\theta} \in \mathbb{R}^{2\times 4}$ and $\boldsymbol{\Sigma}_\theta = \mathrm{blkdiag}(\mathrm{Cov}_a,\,\mathrm{Cov}_b)$ is block-diagonal (the two parent walls are assumed independent).

**Jacobian computation.** Using the matrix differentiation identity $\partial(\mathbf{A}^{-1}\mathbf{b})/\partial\theta = \mathbf{A}^{-1}(\partial\mathbf{b}/\partial\theta) - \mathbf{A}^{-1}(\partial\mathbf{A}/\partial\theta)\mathbf{c}$:

*Partial with respect to $\rho_a$ and $\rho_b$* (only $\mathbf{b}$ depends on $\rho_k$):

$$
\frac{\partial\mathbf{c}}{\partial\rho_a} = \mathbf{A}^{-1}\begin{bmatrix}1\\0\end{bmatrix}, \qquad
\frac{\partial\mathbf{c}}{\partial\rho_b} = \mathbf{A}^{-1}\begin{bmatrix}0\\1\end{bmatrix}.
$$

*Partial with respect to $\alpha_a$* (only $\mathbf{A}$ depends on $\alpha_a$, $\partial\mathbf{b}/\partial\alpha_a = \mathbf{0}$):

$$
\frac{\partial\mathbf{A}}{\partial\alpha_a} = \begin{bmatrix}-\sin\alpha_a & \cos\alpha_a \\ 0 & 0\end{bmatrix},
\qquad
\frac{\partial\mathbf{c}}{\partial\alpha_a} = -\mathbf{A}^{-1}\!\left(\frac{\partial\mathbf{A}}{\partial\alpha_a}\right)\mathbf{c}.
$$

*Partial with respect to $\alpha_b$* (analogously):

$$
\frac{\partial\mathbf{A}}{\partial\alpha_b} = \begin{bmatrix} 0 & 0 \\ -\sin\alpha_b & \cos\alpha_b\end{bmatrix},
\qquad
\frac{\partial\mathbf{c}}{\partial\alpha_b} = -\mathbf{A}^{-1}\!\left(\frac{\partial\mathbf{A}}{\partial\alpha_b}\right)\mathbf{c}.
$$

The full $2 \times 4$ Jacobian is:

$$
\mathbf{J} = \left[\;\frac{\partial\mathbf{c}}{\partial\rho_a}\;\Bigg|\;\frac{\partial\mathbf{c}}{\partial\alpha_a}\;\Bigg|\;\frac{\partial\mathbf{c}}{\partial\rho_b}\;\Bigg|\;\frac{\partial\mathbf{c}}{\partial\alpha_b}\;\right].
$$

**Geometric interpretation.** The corner uncertainty $\mathrm{Cov}(\mathbf{c})$ is large when:
- Either parent wall is short (large $\mathrm{Cov}_a$ or $\mathrm{Cov}_b$).
- The intersection angle $\Delta\phi_{ab}$ is small, which makes $|\det\mathbf{A}|$ small and $\|\mathbf{A}^{-1}\|$ large — geometrically, a shallow intersection amplifies wall parameter uncertainty into large positional uncertainty.

**Numerical conditioning.** The result $\mathbf{J}\boldsymbol{\Sigma}_\theta\mathbf{J}^\top$ is symmetrised as $\tfrac{1}{2}(\mathbf{C} + \mathbf{C}^\top)$ and its eigenvalues are clamped to $\sigma^2 \cdot 10^{-4}$ to guarantee positive definiteness, as numerical cancellation near the singular geometry boundary can produce small negative eigenvalues.

---

## 7. Implementation Reference

### 7.1 Pipeline Summary

```
LaserScan
    │
    ▼
scan_to_cartesian()          Polar → Cartesian, range filtering
    │
    ▼
split_on_gaps()              Gap detection → continuous segments
    │
    ▼  (per segment)
grow_lines_incremental()     TLS residual-based incremental growing
    │
    ▼
merge_adjacent_lines()       Angle + gap + TLS residual merge criterion
    │
    ▼
make_line_dict()             TLS Hessian fit + wall covariance
    │
    ├──► Wall features: {type, rho, alpha, covariance, start_point, end_point}
    │
    ▼
extract_corners_from_adjacent_lines()
    │
    └──► Corner features: {type, position, covariance}
```

### 7.2 Function Reference

| Function | File | Description |
|---|---|---|
| `scan_to_cartesian` | `landmark_features.py` | Polar-to-Cartesian with validity filtering |
| `split_on_gaps` | `landmark_features.py` | Euclidean gap detection → segment list |
| `fit_line_tls` | `landmark_features.py` | SVD-based TLS: returns centroid, direction, normal |
| `segment_residual_tls` | `landmark_features.py` | $\max_i |d_i|$ perpendicular residual |
| `grow_lines_incremental` | `landmark_features.py` | Incremental growing with $\tau_r$ threshold |
| `merge_adjacent_lines` | `landmark_features.py` | Angle + proximity + TLS residual merge pass |
| `convert_line_to_hessian` | `landmark_features.py` | TLS centroid + normal $\to (\rho, \alpha)$ with $\rho \geq 0$ |
| `make_line_dict` | `landmark_features.py` | Full wall descriptor: Hessian + covariance + endpoints |
| `compute_line_intersection` | `landmark_features.py` | $2\times 2$ Hessian system solve for corner position |
| `extract_corners_from_adjacent_lines` | `landmark_features.py` | Adjacent-only corner detection with angle gate |
| `compute_wall_covariance` | `landmark_features.py` | FIM / CRLB: $\mathbf{R}_w = \sigma^2\mathbf{F}^{-1}$ |
| `compute_corner_covariance` | `landmark_features.py` | $\mathbf{R}_c = \mathbf{J}\boldsymbol{\Sigma}_\theta\mathbf{J}^\top$ |
| `extract_features` | `landmark_features.py` | Full scan $\to$ walls + corners pipeline |

### 7.3 Parameters

| Parameter | Symbol | Default | Role |
|---|---|---|---|
| `grow_residual_threshold` | $\tau_r$ | 0.03 m | TLS residual split threshold ($\approx 3\sigma$) |
| `min_points_per_line` | $N_{\min}$ | 10 | Minimum supporting points for a valid segment |
| `min_line_length` | $\ell_{\min}$ | 0.3 m | Minimum accepted segment length |
| `max_gap` | $\tau_{\mathrm{gap}}$ | 0.2 m | Gap detection and merge proximity threshold |
| `merge_angle_tolerance` | $\tau_\theta$ | 0.22 rad | Maximum angular difference for merge |
| `merge_rho_tolerance` | $\tau_{\mathrm{merge}}$ | 0.15 m | Maximum joint TLS residual for merge |
| `corner_angle_threshold` | $\phi_{\min}$ | 50° | Minimum intersection angle for corner validity |
| `lidar_noise_sigma` | $\sigma$ | 0.01 m | LiDAR range noise standard deviation |

---

## References

1. **Nguyen, V., Martinelli, A., Tomatis, N., & Siegwart, R. (2005).** "A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder for Indoor Mobile Robotics." *Proceedings of IEEE/RSJ IROS*, pp. 1929–1934.

2. **Pavlidis, T., & Horowitz, S. L. (1974).** "Segmentation of Plane Curves." *IEEE Transactions on Computers*, C-23(8), 860–870.

3. **Duda, R. O., & Hart, P. E. (1972).** "Use of the Hough Transformation to Detect Lines and Curves in Pictures." *Communications of the ACM*, 15(1), 11–15.

4. **Duda, R. O., & Hart, P. E. (1973).** *Pattern Classification and Scene Analysis*. Wiley.

5. **Fischler, M. A., & Bolles, R. C. (1981).** "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381–395.

6. **Censi, A. (2007).** "An accurate closed-form estimate of ICP's covariance." *Proceedings of IEEE ICRA*, pp. 3167–3172.

7. **Pfister, S. T., Kriechbaum, K. L., Roumeliotis, S. I., & Burdick, J. W. (2002).** "Weighted Line Fitting Algorithms for Mobile Robot Map Building and Efficient Data Representation." *Proceedings of IEEE ICRA*, pp. 1304–1311.

8. **Jolliffe, I. T. (2002).** *Principal Component Analysis* (2nd ed.). Springer.

9. **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011).** *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

10. **Castellanos, J. A., Montiel, J. M. M., Neira, J., & Tardós, J. D. (1999).** "The SPmap: A Probabilistic Framework for Simultaneous Localization and Map Building." *IEEE Transactions on Robotics and Automation*, 15(5), 948–952.

11. **Kay, S. M. (1993).** *Fundamentals of Statistical Signal Processing: Estimation Theory*. Prentice Hall.
