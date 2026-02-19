# Data Association: Theory and Implementation

## Table of Contents
1. [Introduction](#1-introduction)
2. [Mahalanobis Distance](#2-mahalanobis-distance)
3. [Chi-Squared Gating](#3-chi-squared-gating)
4. [Nearest-Neighbour Association](#4-nearest-neighbour-association)
5. [Wall Association](#5-wall-association)
6. [Corner Association](#6-corner-association)
7. [Rho Sign Normalisation](#7-rho-sign-normalisation)
8. [Implementation Reference](#8-implementation-reference)

---

## 1. Introduction

Data association is the problem of deciding which observed feature corresponds to which landmark already in the map. Every scan produces a set of observed walls and corners. Each must either be matched to an existing landmark (and used to update it via the EKF) or declared new (and used to initialise a new landmark). Incorrect associations corrupt the map and can cause filter divergence (Neira & Tardós, 2001).

### 1.1 Design Philosophy

The system uses nearest-neighbour association with a two-stage gate:

1. **Spatial gate** — fast Euclidean pre-filter, $O(1)$ per candidate.
2. **Mahalanobis gate** — statistically principled filter based on the full innovation covariance $\mathbf{S}$.

False positives (wrong match accepted) are more harmful than false negatives (correct match missed). A false negative leads to a duplicate landmark that will eventually be pruned. A false positive introduces a systematic error that can cause divergence (Bailey, 2002). The Mahalanobis gate at 95% confidence is therefore the primary filter; the spatial gate only reduces the candidate set.

The spatial gate threshold must be at least as large as the maximum sensor range. A threshold smaller than the sensor range will reject valid observations before the Mahalanobis test can run, producing a continuous stream of duplicate landmarks.

---

## 2. Mahalanobis Distance

Given an observed feature with predicted value $\hat{\mathbf{z}}$ and innovation covariance $\mathbf{S}$, the squared Mahalanobis distance of the actual observation $\mathbf{z}$ is

$$
D_M^2(\mathbf{z}, \hat{\mathbf{z}}) = (\mathbf{z} - \hat{\mathbf{z}})^\top \mathbf{S}^{-1} (\mathbf{z} - \hat{\mathbf{z}}).
$$

Unlike Euclidean distance, $D_M^2$ is dimensionless and accounts for different variances in each component and for correlations between components. Points on the ellipsoid $D_M^2 = c^2$ are equally likely under the Gaussian $\mathcal{N}(\hat{\mathbf{z}}, \mathbf{S})$.

The innovation covariance is

$$
\mathbf{S} = \mathbf{H}\,\bar{\mathbf{P}}\,\mathbf{H}^\top + \mathbf{R},
$$

where $\mathbf{H}$ is the observation Jacobian, $\bar{\mathbf{P}}$ is the predicted state covariance, and $\mathbf{R}$ is the observation noise covariance (the landmark covariance from extraction).

If $\mathbf{z} \sim \mathcal{N}(\hat{\mathbf{z}}, \mathbf{S})$ — that is, if the observation genuinely comes from the predicted landmark — then

$$
D_M^2 \sim \chi^2(d),
$$

where $d$ is the dimension of the observation (here $d = 2$ for both walls and corners).

---

## 3. Chi-Squared Gating

An observation is accepted as a candidate match if

$$
D_M^2 \leq \chi^2_\alpha(d),
$$

where $\chi^2_\alpha(d)$ is the critical value of the chi-squared distribution with $d$ degrees of freedom at confidence level $1 - \alpha$.

For 2-DOF features at 95% confidence: $\chi^2_{0.05}(2) = 5.991$.

This means a correct association falls inside the gate with probability 95%. The probability of accepting a spurious association (false positive) depends on the distance between unrelated features and is typically negligible for well-separated landmarks.

| Confidence | $\chi^2(2)$ threshold |
|---|---|
| 90% | 4.605 |
| 95% | 5.991 |
| 99% | 9.210 |
| 99.9% | 13.816 |

**Implementation note.** The gate compares $D_M^2$ (the squared distance) directly to the threshold $5.991$. Do not compare $D_M^2$ to $5.991^2 = 35.88$; that would make the gate more than six times too loose and would accept associations at nearly $\chi^2(2)_{0.001}$ confidence (i.e., 35.88 corresponds to a $p$-value of $\approx 0.0000002$, far outside any reasonable statistical meaning).

---

## 4. Nearest-Neighbour Association

For each observed feature, the algorithm finds the existing landmark that minimises $D_M$ subject to passing the spatial and chi-squared gates:

```
for each observed feature z:
    best_id   ← None
    best_dist ← ∞

    for each existing landmark m_j of matching type:
        1. Spatial gate: if distance(robot, m_j) > τ_spatial, skip
        1.5. Wall gap gate: if wall extents exist and gap > max_gap_ext, skip
        2. Compute predicted observation ẑ_j = h(x_r, m_j)
        3. Compute innovation  ν = z − ẑ_j  (angles wrapped to [−π, π])
        4. Compute Jacobian H and innovation covariance S = H P H^T + R
        5. Compute D²_M = ν^T S^{-1} ν
        6. Chi-squared gate: if D²_M > 5.991, skip
        7. If D_M < best_dist: best_id ← j, best_dist ← D_M

    if best_id is not None: mark as matched to m_{best_id}
    else:                   mark as new landmark
```

Matched features update their corresponding landmark via the EKF. Unmatched features initialise new landmarks.

---

## 5. Wall Association

### 5.1 Spatial Gate

The signed perpendicular distance from the robot to the map-frame wall $(\rho_m, \alpha_m)$ is

$$
d_\perp = \rho_m - (x_r\cos\alpha_m + y_r\sin\alpha_m).
$$

The wall is a candidate if $|d_\perp| \leq \tau_{\mathrm{spatial}}$ (default: 6.0 m, larger than the LiDAR range of 3.5 m).

**Additional wall gates (implementation defaults):**
- **Angle gate:** $|\hat{\alpha}_r - \alpha_r| \leq 0.349066$ rad ($\approx 20^\circ$).
- **Rho gate:** $|\rho_m - \rho_m^{\mathrm{obs}}| \leq 0.5$ m, where $\rho_m^{\mathrm{obs}}$ is the observed wall transformed to map frame.
 - **Gap gate (extent overlap):** if the wall has stored extents, the observed segment must overlap or be within a small gap of the existing extent. The implementation uses a maximum allowable gap of `max_gap_ext = 0.5 m` along the wall tangent. Segments beyond this gap are treated as **new landmarks** to avoid bridging doorways/openings.

**Extent representation.** Wall extents are stored as scalar limits $(t_{\min}, t_{\max})$ along the wall tangent $\hat{\mathbf{t}} = [-\sin\alpha,\; \cos\alpha]^\top$. When an observed wall segment is matched, its endpoints are transformed to map frame and projected onto $\hat{\mathbf{t}}$ for the gap check and for endpoint extension.

### 5.2 Predicted Observation

$$
\hat{\rho}_r = \rho_m - (x_r\cos\alpha_m + y_r\sin\alpha_m), \qquad
\hat{\alpha}_r = \alpha_m - \theta_r.
$$

### 5.3 Observation Jacobian

With columns indexed by $[\ldots, x_r, y_r, \theta_r, \ldots, \rho_m, \alpha_m, \ldots]$:

$$
\mathbf{H}_{\mathrm{wall}} = \begin{bmatrix}
-\cos\alpha_m & -\sin\alpha_m & 0 & \cdots & 1 & x_r\sin\alpha_m - y_r\cos\alpha_m & \cdots \\
0 & 0 & -1 & \cdots & 0 & 1 & \cdots
\end{bmatrix}.
$$

This is the same Jacobian used in the EKF update (Section 5.1 of `01_ekf_slam_theory.md`). Using the same Jacobian in both data association and EKF update ensures that the Mahalanobis distance is computed in the same linearised space as the subsequent update.

---

## 6. Corner Association

### 6.1 Spatial Gate

Euclidean distance from the robot to the map-frame corner $(x_m, y_m)$:

$$
d = \sqrt{(x_m - x_r)^2 + (y_m - y_r)^2} \leq \tau_{\mathrm{spatial}}.
$$

### 6.2 Predicted Observation

Let $s = \sin\theta_r$, $c = \cos\theta_r$, $\Delta x = x_m - x_r$, $\Delta y = y_m - y_r$:

$$
\hat{z}_x = c\,\Delta x + s\,\Delta y, \qquad \hat{z}_y = -s\,\Delta x + c\,\Delta y.
$$

### 6.3 Observation Jacobian

Columns indexed by $[\ldots, x_r, y_r, \theta_r, \ldots, x_m, y_m, \ldots]$:

$$
\mathbf{H}_{\mathrm{corner}} = \begin{bmatrix}
-c & -s & -s\,\Delta x + c\,\Delta y & \cdots & c & s & \cdots \\
 s & -c & -c\,\Delta x - s\,\Delta y & \cdots & -s & c & \cdots
\end{bmatrix}.
$$

**Derivation of $H[0,2]$.** $\hat{z}_x = c\,\Delta x + s\,\Delta y$ where $c = \cos\theta_r$ and $s = \sin\theta_r$.

$$
\frac{\partial \hat{z}_x}{\partial \theta_r}
= -s\,\Delta x + c\,\Delta y.
$$

**Derivation of $H[1,2]$.** $\hat{z}_y = -s\,\Delta x + c\,\Delta y$.

$$
\frac{\partial \hat{z}_y}{\partial \theta_r}
= -c\,\Delta x - s\,\Delta y.
$$

These match the Jacobians in the EKF update (`ekf_update_feature.py:203–207`). An earlier version of the code used $\mathrm{cos\_theta} = \cos(-\theta_r) = c$ and $\mathrm{sin\_theta} = \sin(-\theta_r) = -s$, which flipped the signs of $H[0,2]$ and $H[1,2]$, producing an incorrect innovation covariance $\mathbf{S}$ during association.

---

## 7. Rho Sign Normalisation

The observation model assumes the map-frame landmark has $\rho_m \geq 0$. When transforming the observed robot-frame $(\rho_r, \alpha_r)$ to the map frame for comparison:

$$
\alpha_m^{\mathrm{obs}} = \alpha_r + \theta_r, \qquad
\rho_m^{\mathrm{obs}} = \rho_r + x_r\cos\alpha_m^{\mathrm{obs}} + y_r\sin\alpha_m^{\mathrm{obs}}.
$$

If $\rho_m^{\mathrm{obs}} < 0$, the canonical form requires:

$$
\rho_m^{\mathrm{obs}} \leftarrow -\rho_m^{\mathrm{obs}}, \qquad
\alpha_m^{\mathrm{obs}} \leftarrow \alpha_m^{\mathrm{obs}} + \pi \pmod{2\pi} \text{ (wrapped to } [-\pi, \pi]).
$$

Both $\rho$ and $\alpha$ must be adjusted together. Negating only $\rho$ without flipping $\alpha$ leaves an inconsistent pair: the rho-difference check would use the correct magnitude, but the alpha-difference check in the Mahalanobis computation would use the wrong direction, producing a biased $\mathbf{S}$.

---

## 8. Implementation Reference

| Component | File | Notes |
|---|---|---|
| `associate_landmarks` | `data_association.py` | Main association function |
| Wall Jacobian | `data_association.py:84–97` | Matches `ekf_update_feature.py` |
| Corner Jacobian | `data_association.py:141–163` | Corrected signs; $H[0,2] = -s\Delta x + c\Delta y$ |
| Chi-sq gate | `data_association.py:170` | `mahal_dist_sq < 5.991` (not `5.991**2`) |
| Rho normalisation | `data_association.py:75–85` | Both $\rho$ and $\alpha$ flipped when $\rho < 0$ |

### 8.1 Parameters

| Parameter | Default | Justification |
|---|---|---|
| `max_mahalanobis_dist` | 5.991 | $\chi^2_{0.05}(2)$: 95% confidence, 2-DOF |
| `max_euclidean_dist` | 5.0 m | $\geq$ LiDAR range (3.5 m) + margin |

---

## References

1. **Mahalanobis, P. C. (1936).** "On the Generalized Distance in Statistics." *Proceedings of the National Institute of Sciences of India*, 2(1), 49–55.

2. **Neira, J., & Tardós, J. D. (2001).** "Data Association in Stochastic Mapping Using the Joint Compatibility Test." *IEEE Transactions on Robotics and Automation*, 17(6), 890–897.

3. **Bailey, T. (2002).** "Mobile Robot Localisation and Mapping in Extensive Outdoor Environments." PhD Thesis, University of Sydney.

4. **Bar-Shalom, Y., & Fortmann, T. E. (1988).** *Tracking and Data Association*. Academic Press.

5. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.

6. **Barfoot, T. D. (2017).** *State Estimation for Robotics*. Cambridge University Press.
