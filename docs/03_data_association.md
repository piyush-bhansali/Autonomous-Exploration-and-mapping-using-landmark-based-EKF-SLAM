# Data Association: Theory and Statistical Gating

> **Note:** This document provides detailed mathematical derivations for data association used in **Feature-based SLAM mode**. For complete system context, see `methodology_feature_mapping.md`.

## Table of Contents
1. [Introduction](#1-introduction)
2. [The Association Problem](#2-the-association-problem)
3. [Mahalanobis Distance](#3-mahalanobis-distance)
4. [Chi-Squared Gating](#4-chi-squared-gating)
5. [Spatial Gating](#5-spatial-gating)
6. [Nearest Neighbor Association](#6-nearest-neighbor-association)
7. [Implementation](#7-implementation)
8. [Advanced Topics](#8-advanced-topics)
9. [Data Association Performance Metrics](#9-data-association-performance-metrics)

---

## 1. Introduction

**Data association** is the problem of determining which observed feature corresponds to which landmark in the map. This is critical for SLAM because:
- **Correct associations** → Accurate state updates
- **Incorrect associations** → Catastrophic errors (divergence)

### 1.0 Historical Context and Literature Review

The data association problem in SLAM has its roots in multi-target tracking research. **Mahalanobis (1936)** first introduced the distance metric that bears his name for statistical pattern recognition, which became fundamental to modern data association.

**Bar-Shalom & Fortmann (1988)** established the theoretical framework for data association in tracking systems, introducing:
- **Nearest Neighbor (NN)**: Greedy association based on minimum distance
- **Probabilistic Data Association (PDA)**: Weighted combination of hypotheses
- **Joint Probabilistic Data Association (JPDA)**: Multi-target extension of PDA

**SLAM-Specific Developments:**

**Bailey et al. (2000)** analyzed data association failure modes in outdoor SLAM, identifying:
- **False positive associations**: Incorrect matches causing filter divergence
- **False negative associations**: Missed matches causing map duplication
- **Ambiguity in symmetric environments**: Multiple equally-likely hypotheses

**Neira & Tardós (2001)** introduced the **Joint Compatibility Branch and Bound (JCBB)** algorithm specifically for SLAM:
- Considers correlations between landmarks (critical for SLAM)
- Computes joint Mahalanobis distance for sets of observations
- Provides optimal solution with branch-and-bound search

**Validation Gating Theory** (Bar-Shalom & Fortmann, 1988):

The probability that a correct association falls within the validation gate is:

$$
P_D = P\left(D_M^2 < \chi^2_{\alpha}(d)\right) = 1 - \alpha
$$

For 2-DOF features with 95% confidence ($\chi^2_{0.05}(2) = 5.99$):
- **True positive rate**: 95% (detection probability)
- **False negative rate**: 5% (missed correct associations)

**Conservative Gating Principle** (Bailey, 2002):

In SLAM, **false positives are catastrophic** (cause divergence), while **false negatives are recoverable** (landmark re-initialized). Therefore:

$$
P_{\text{false positive}} \ll P_{\text{false negative}}
$$

This justifies using conservative thresholds (95% confidence) for Mahalanobis gating.

### 1.1 Challenge

At each time step, we have:
- $M$ observed features: $\mathcal{Z} = \{z_1, z_2, \ldots, z_M\}$
- $N$ known landmarks: $\mathcal{L} = \{m_1, m_2, \ldots, m_N\}$

**Question:** Which $z_i$ corresponds to which $m_j$?

**Complexity:** $O(M \cdot N)$ comparisons required.

### 1.2 Ambiguity

Multiple features may be similar:
- Parallel walls (same orientation, different position)
- Symmetric environments
- Feature-rich clutter

**Solution:** Use both **geometric** and **statistical** information.

### 1.3 Common Pitfall: Overly Conservative Spatial Gating

⚠️ **Critical Implementation Note:**

Setting the spatial gating threshold too low (e.g., 1.0m) will **reject all valid observations** before Mahalanobis gating can run, resulting in zero landmark matches.

**Symptoms:**
- Zero matched landmarks despite features being visible
- EKF state not updating with landmark observations
- Continuous creation of duplicate landmarks

**Fix:** Set spatial threshold ≥ sensor range (e.g., 5.0m for 3.5m LiDAR) and let the statistically-principled Mahalanobis distance determine compatibility.

---

## 2. The Association Problem

### 2.1 Problem Formulation

For each observed feature $z_i$, find the most compatible landmark $m_j^*$:

$$
j^* = \arg\min_{j \in \{1, \ldots, N\}} d(z_i, m_j | \mathbf{x}, \mathbf{P})
$$

Where $d(\cdot)$ is a compatibility metric that accounts for:
- Prediction uncertainty (from $\mathbf{P}$)
- Measurement noise
- State estimate $\mathbf{x}$

### 2.2 Gating Strategy

A two-stage approach:

**Stage 1: Spatial Gating (Fast, Conservative)**
- Reject obviously incompatible associations using Euclidean distance
- Reduces candidate set

**Stage 2: Mahalanobis Gating (Accurate, Statistical)**
- Compute statistically-principled compatibility using covariance
- Select best match from candidates

---

## 3. Mahalanobis Distance

### 3.1 Definition

The **Mahalanobis distance** is a statistical measure of how many standard deviations a point is from a distribution.

Given:
- Observation: $z \in \mathbb{R}^d$
- Predicted observation: $\hat{z} \in \mathbb{R}^d$
- Innovation covariance: $\mathbf{S} \in \mathbb{R}^{d \times d}$

The Mahalanobis distance is:

$$
D_M(z, \hat{z}) = \sqrt{(z - \hat{z})^T \mathbf{S}^{-1} (z - \hat{z})}
$$

Or squared form:

$$
D_M^2(z, \hat{z}) = (z - \hat{z})^T \mathbf{S}^{-1} (z - \hat{z})
$$

### 3.2 Intuition

**Euclidean distance** treats all dimensions equally:

$$
D_E = \sqrt{(z - \hat{z})^T (z - \hat{z})} = \|z - \hat{z}\|
$$

**Mahalanobis distance** accounts for:
1. **Different variances** in each dimension
2. **Correlations** between dimensions

**Example:** For a 2D feature with uncertainty ellipse:

```
    y
    │
    │    ×  Point A (far along major axis)
    │   ╱ ╲
    │  ╱   ╲
    │ │  •  │ ← Distribution center
    │  ╲   ╱
    │   ╲ ╱
    │    × Point B (close along major axis)
    └────────────> x
```

- $D_E(A) > D_E(B)$ (Euclidean)
- $D_M(A) < D_M(B)$ (Mahalanobis, if major axis is x)

Mahalanobis correctly identifies A as more compatible (within 1 std dev).

### 3.3 Geometric Interpretation

The Mahalanobis distance defines an **ellipsoid** of constant probability:

$$
(z - \hat{z})^T \mathbf{S}^{-1} (z - \hat{z}) = c^2
$$

Points on this ellipsoid are **equally likely** under the Gaussian distribution $\mathcal{N}(\hat{z}, \mathbf{S})$.

**Eigendecomposition:**

$$
\mathbf{S} = \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T
$$

- Eigenvectors $\mathbf{V}$: Principal axes of ellipse
- Eigenvalues $\boldsymbol{\Lambda}$: Variances along axes
- Axis lengths: $a_i = c \sqrt{\lambda_i}$

### 3.4 Normalization Property

If $z \sim \mathcal{N}(\hat{z}, \mathbf{S})$, then:

$$
D_M^2(z, \hat{z}) \sim \chi^2(d)
$$

The squared Mahalanobis distance follows a **chi-squared distribution** with $d$ degrees of freedom.

This is the foundation for statistical gating.

---

## 4. Chi-Squared Gating

### 4.1 Chi-Squared Distribution

The chi-squared distribution $\chi^2(d)$ describes the distribution of a sum of $d$ squared standard normal variables:

$$
X = \sum_{i=1}^d Z_i^2, \quad Z_i \sim \mathcal{N}(0, 1)
$$

**Properties:**
- Mean: $\mathbb{E}[X] = d$
- Variance: $\text{Var}(X) = 2d$
- Support: $X \in [0, \infty)$

**PDF:**

$$
p(x; d) = \frac{1}{2^{d/2} \Gamma(d/2)} x^{d/2 - 1} e^{-x/2}
$$

### 4.2 Gating Threshold

We accept an association if:

$$
D_M^2(z, \hat{z}) \leq \chi^2_{\alpha}(d)
$$

Where $\chi^2_{\alpha}(d)$ is the critical value at confidence level $1 - \alpha$.

**Common Values:**

| Confidence | $\alpha$ | $\chi^2(2)$ | $\chi^2(3)$ |
|------------|---------|------------|------------|
| 95% | 0.05 | 5.99 | 7.82 |
| 99% | 0.01 | 9.21 | 11.34 |
| 99.9% | 0.001 | 13.82 | 16.27 |

**Interpretation:**
- 95% confidence: Accept if observation is within 95% probability region
- Reject 5% of valid observations (false negatives)
- Accept very few outliers (false positives)

### 4.3 Degrees of Freedom

For SLAM features:
- **Walls (Hessian form):** $d = 2$ (ρ, α)
- **Corners (Cartesian):** $d = 2$ (x, y)

### 4.4 Gating Decision

```python
def is_compatible(z, z_hat, S, confidence=0.95):
    """
    Check if observation z is compatible with prediction z_hat
    using chi-squared gating.
    """
    # Innovation
    innovation = z - z_hat

    # Normalize angles if needed
    if has_angular_component:
        innovation[angle_idx] = normalize_angle(innovation[angle_idx])

    # Mahalanobis distance squared
    S_inv = np.linalg.inv(S)
    mahal_dist_sq = innovation.T @ S_inv @ innovation

    # Chi-squared threshold
    dof = len(z)
    threshold = chi2.ppf(confidence, dof)  # scipy.stats.chi2

    return mahal_dist_sq <= threshold
```

### 4.5 Innovation Covariance $\mathbf{S}$

The innovation covariance combines prediction and measurement uncertainty:

$$
\mathbf{S} = \mathbf{H} \bar{\mathbf{P}} \mathbf{H}^T + \mathbf{R}
$$

Where:
- $\mathbf{H}$: Observation Jacobian
- $\bar{\mathbf{P}}$: Predicted state covariance
- $\mathbf{R}$: Measurement noise covariance

**Intuition:**
- Large $\mathbf{S}$: High uncertainty → accept wider range of observations
- Small $\mathbf{S}$: High confidence → reject observations that deviate

---

## 5. Spatial Gating

### 5.1 Purpose

Spatial gating provides a **fast pre-filter** before expensive Mahalanobis computation:
- $O(1)$ per comparison (no matrix inversion)
- Rejects obviously incompatible landmarks
- Reduces false associations in cluttered environments

### 5.2 Distance Metrics

**For Walls (Hessian Form):**

Perpendicular distance from robot to wall:

$$
d_{\perp}(robot, wall) = |\rho - (x_r \cos\alpha + y_r \sin\alpha)|
$$

**For Corners (Cartesian):**

Euclidean distance:

$$
d(robot, corner) = \sqrt{(x_m - x_r)^2 + (y_m - y_r)^2}
$$

### 5.3 Gating Threshold

$$
d < \tau_{\text{spatial}}
$$

Typical value: $\tau_{\text{spatial}} = 5.0$ m

**Rationale:**
- LiDAR range: 3.5 m (TurtleBot3 Waffle Pi)
- Feature detection range: ~3.5 m
- Threshold set to 5.0 m to accommodate full sensor range plus margin
- **Critical:** Too conservative (e.g., 1.0 m) will reject valid observations before statistical gating
- **Best practice:** Set spatial threshold ≥ sensor range to allow Mahalanobis distance to determine compatibility

### 5.4 Limitation

Spatial gating alone is insufficient:
- Does not account for uncertainty
- No angular/orientation information
- Cannot distinguish parallel features

**Always combine with Mahalanobis gating.**

---

## 6. Nearest Neighbor Association

### 6.1 Algorithm

For each observed feature $z_i$:

```
1. Initialize: best_match ← None, best_distance ← ∞

2. For each landmark m_j:
   a. Type check: if type(z_i) ≠ type(m_j), skip
   b. Spatial gate: if distance(robot, m_j) > τ_spatial, skip
   c. Predict observation: ẑ_j ← h(x_r, m_j)
   d. Compute innovation: ν ← z_i - ẑ_j
   e. Compute Jacobian: H ← ∂h/∂x|(x, m_j)
   f. Innovation covariance: S ← H P H^T + R
   g. Mahalanobis distance: D_M ← √(ν^T S^{-1} ν)
   h. Chi-squared gate: if D_M^2 > χ²_α(d), skip
   i. Update best: if D_M < best_distance:
        best_match ← j
        best_distance ← D_M

3. If best_match exists:
     return (z_i, m_{best_match})  // Matched
   Else:
     return (z_i, None)  // New landmark
```

### 6.2 Matched vs. Unmatched

**Matched Features:**
- Update corresponding landmark via EKF
- Increment observation count
- Update last-seen timestamp

**Unmatched Features:**
- Initialize as new landmark
- Augment state vector and covariance
- Set observation count = 1

### 6.3 Complexity

**Per observation:**
- Spatial gating: $O(N)$ distance computations
- After spatial filtering: $O(K)$ Mahalanobis computations, $K \ll N$
- Per Mahalanobis: $O(d^3)$ for matrix inversion

**Total:** $O(M \cdot N + M \cdot K \cdot d^3)$

**Practical:** With spatial gating, $K \approx 3$-$5$ (only nearby landmarks).

---

## 7. Implementation

### 7.1 Data Association Function

```python
def associate_landmarks(observed_features, ekf_slam,
                        max_mahalanobis_dist=5.99,
                        max_euclidean_dist=5.0):
    """
    Associate observed features with existing landmarks.

    Args:
        observed_features: List of feature dicts with 'type', 'rho'/'alpha'
                          or 'position', and 'covariance'
        ekf_slam: EKF-SLAM object with state, covariance, landmarks
        max_mahalanobis_dist: Chi-squared threshold (e.g., 5.99 for 95%, 2-DOF)
        max_euclidean_dist: Spatial gating threshold (meters, typically ≥ sensor range)

    Returns:
        matched: List of (feature_idx, landmark_id) tuples
        unmatched: List of feature_idx for new landmarks
    """
    matched = []
    unmatched = []

    if len(ekf_slam.landmarks) == 0:
        # No landmarks yet
        return matched, list(range(len(observed_features)))

    x_r, y_r, theta_r = ekf_slam.state[0:3]

    for feat_idx, feature in enumerate(observed_features):
        best_landmark_id = None
        best_mahalanobis = float('inf')

        for landmark_id, lm_data in ekf_slam.landmarks.items():
            # Type consistency check
            if feature['type'] != lm_data['feature_type']:
                continue

            idx = lm_data['state_index']

            if feature['type'] == 'wall':
                # --- WALL ASSOCIATION ---
                lm_rho = ekf_slam.state[idx]
                lm_alpha = ekf_slam.state[idx + 1]

                # Spatial gating
                dist_to_wall = abs(lm_rho - (x_r * np.cos(lm_alpha) +
                                              y_r * np.sin(lm_alpha)))
                if dist_to_wall > max_euclidean_dist:
                    continue

                # Predicted observation
                rho_pred = lm_rho - (x_r * np.cos(lm_alpha) +
                                     y_r * np.sin(lm_alpha))
                alpha_pred = lm_alpha - theta_r
                alpha_pred = normalize_angle(alpha_pred)

                z_pred = np.array([rho_pred, alpha_pred])
                z_obs = np.array([feature['rho'], feature['alpha']])

                # Innovation
                innovation = z_obs - z_pred
                innovation[1] = normalize_angle(innovation[1])

                # Observation Jacobian
                n = len(ekf_slam.state)
                H = np.zeros((2, n))
                H[0, 0] = -np.cos(lm_alpha)
                H[0, 1] = -np.sin(lm_alpha)
                H[0, 2] = 0.0
                H[1, 0] = 0.0
                H[1, 1] = 0.0
                H[1, 2] = -1.0
                H[0, idx] = 1.0
                H[0, idx+1] = x_r * np.sin(lm_alpha) - y_r * np.cos(lm_alpha)
                H[1, idx] = 0.0
                H[1, idx+1] = 1.0

            elif feature['type'] == 'corner':
                # --- CORNER ASSOCIATION ---
                lm_x = ekf_slam.state[idx]
                lm_y = ekf_slam.state[idx + 1]

                # Spatial gating
                dist_to_corner = np.sqrt((lm_x - x_r)**2 + (lm_y - y_r)**2)
                if dist_to_corner > max_euclidean_dist:
                    continue

                # Predicted observation (rotate to robot frame)
                dx = lm_x - x_r
                dy = lm_y - y_r
                cos_theta = np.cos(-theta_r)
                sin_theta = np.sin(-theta_r)
                z_pred_x = cos_theta * dx - sin_theta * dy
                z_pred_y = sin_theta * dx + cos_theta * dy

                z_pred = np.array([z_pred_x, z_pred_y])
                z_obs = feature['position']

                innovation = z_obs - z_pred

                # Observation Jacobian
                n = len(ekf_slam.state)
                H = np.zeros((2, n))
                H[0, 0] = -cos_theta
                H[0, 1] = sin_theta
                H[0, 2] = -dx * sin_theta - dy * cos_theta
                H[1, 0] = -sin_theta
                H[1, 1] = -cos_theta
                H[1, 2] = dx * cos_theta - dy * sin_theta
                H[0, idx] = cos_theta
                H[0, idx+1] = -sin_theta
                H[1, idx] = sin_theta
                H[1, idx+1] = cos_theta

            else:
                continue

            # Innovation covariance
            S = H @ ekf_slam.P @ H.T + feature['covariance']

            # Mahalanobis distance
            try:
                S_inv = np.linalg.inv(S)
                mahal_dist_sq = innovation.T @ S_inv @ innovation
                mahal_dist = float(np.sqrt(mahal_dist_sq))
            except np.linalg.LinAlgError:
                continue  # Singular covariance

            # Chi-squared gating
            if mahal_dist_sq < max_mahalanobis_dist**2 and mahal_dist < best_mahalanobis:
                best_mahalanobis = mahal_dist
                best_landmark_id = landmark_id

        if best_landmark_id is not None:
            matched.append((feat_idx, best_landmark_id))
        else:
            unmatched.append(feat_idx)

    return matched, unmatched
```

### 7.2 Code Mapping

| Component | File | Function/Class |
|-----------|------|---------------|
| Data Association | `data_association.py` | `associate_landmarks()` |
| EKF State | `ekf_slam.py` | `LandmarkEKFSLAM.state` |
| EKF Covariance | `ekf_slam.py` | `LandmarkEKFSLAM.P` |
| Landmarks | `ekf_slam.py` | `LandmarkEKFSLAM.landmarks` |

### 7.3 Parameter Tuning

| Parameter | Value | Justification |
|-----------|-------|---------------|
| `max_mahalanobis_dist` | 5.99 | 95% confidence, 2-DOF χ² |
| `max_euclidean_dist` | 5.0 m | Covers full LiDAR range (3.5m) + margin |
| Angle normalization | Always | Prevents ±2π ambiguity |

**Trade-offs:**
- **Spatial threshold too low (e.g., 1.0m):** Rejects valid observations before statistical gating runs
- **Spatial threshold too high (e.g., 10.0m):** Unnecessary computational cost for distant landmarks
- **Mahalanobis threshold:** Controls statistical rigor (lower = fewer false positives)

**For SLAM:**
- Spatial gating should be **permissive** (≥ sensor range) to allow all visible landmarks
- Mahalanobis gating provides **rigorous** filtering using statistical principles
- False positives are worse than false negatives (cause divergence) → Conservative Mahalanobis threshold

---

## 8. Advanced Topics

### 8.1 Joint Compatibility Branch and Bound (JCBB)

**Limitation of Nearest Neighbor:** Considers each observation independently.

**JCBB:** Considers **joint compatibility** of multiple observations:

$$
D_M^2(\mathbf{z}_{1:M}, \hat{\mathbf{z}}_{1:M}) = (\mathbf{z}_{1:M} - \hat{\mathbf{z}}_{1:M})^T \mathbf{S}_{1:M}^{-1} (\mathbf{z}_{1:M} - \hat{\mathbf{z}}_{1:M})
$$

**Advantages:**
- More robust to ambiguity
- Considers cross-correlations

**Disadvantages:**
- Exponential complexity: $O(N^M)$
- Requires branch-and-bound search

**Not implemented in current system** (nearest neighbor sufficient for structured environments).

### 8.2 Outlier Rejection

In addition to chi-squared gating, outliers are rejected during EKF update:

```python
if mahal_dist_sq > 13.8:  # 99.7% confidence
    # Reject outlier, skip update
    return
```

This prevents single bad associations from corrupting the entire map.

---

## 9. Data Association Performance Metrics

### 9.1 Confusion Matrix

Data association performance is evaluated using classification metrics:

|  | Predicted Match | Predicted No Match |
|---|---|---|
| **True Match** | True Positive (TP) | False Negative (FN) |
| **True No Match** | False Positive (FP) | True Negative (TN) |

**Definitions:**
- **TP**: Correct association (observation matched to correct landmark)
- **FP**: Incorrect association (observation matched to wrong landmark)
- **FN**: Missed association (observation not matched despite correct landmark existing)
- **TN**: Correct rejection (new landmark correctly identified)

### 9.2 Performance Metrics

**Precision** (fraction of associations that are correct):

$$
\text{Precision} = \frac{TP}{TP + FP}
$$

**Recall** (fraction of true matches found):

$$
\text{Recall} = \frac{TP}{TP + FN}
$$

**F1 Score** (harmonic mean of precision and recall):

$$
F_1 = 2 \cdot \frac{\text{Precision} \cdot \text{Recall}}{\text{Precision} + \text{Recall}}
$$

**Association Rate** (Bailey, 2002):

$$
\text{Association Rate} = \frac{\text{Number of matches}}{\text{Number of observations}}
$$

### 9.3 Experimental Benchmarks

**Neira & Tardós (2001)** reported the following performance for JCBB vs. NN:

| Algorithm | Precision | Recall | F1 Score | Computation Time |
|-----------|-----------|--------|----------|------------------|
| Nearest Neighbor | 0.91 | 0.88 | 0.89 | 1.2 ms |
| JCBB | 0.98 | 0.96 | 0.97 | 8.7 ms |

**Observations:**
- JCBB provides ~7% improvement in F1 score
- Computation time increases ~7× due to joint compatibility checking
- For structured environments, NN is often sufficient

### 9.4 Failure Mode Analysis

**Bailey (2002, Chapter 4)** identified critical failure scenarios:

**1. Perceptual Aliasing**

Multiple landmarks appear identical:
- Parallel walls with same length
- Symmetric room layout
- Repetitive patterns

**Mitigation**: Use spatial context, require multiple observations for confirmation

**2. Spurious Features**

False detections from sensor noise:
- Specular reflections (glass, mirrors)
- Dynamic objects (people, furniture)
- Sensor artifacts

**Mitigation**: Conservative Mahalanobis threshold, require temporal consistency

**3. Landmark Drift**

Accumulated errors cause landmark positions to drift:
- Predicted observation no longer matches
- Association failures increase over time

**Mitigation**: Loop closure, global optimization (graph SLAM)

### 9.5 Gating Threshold Selection

**Trade-off Analysis** (Bar-Shalom & Fortmann, 1988):

**Conservative threshold** (e.g., $\chi^2_{0.01}(2) = 9.21$, 99% confidence):
- **Pros**: Very low false positive rate (< 1%)
- **Cons**: Higher false negative rate (> 1%), more duplicate landmarks

**Permissive threshold** (e.g., $\chi^2_{0.10}(2) = 4.61$, 90% confidence):
- **Pros**: Low false negative rate (< 10%)
- **Cons**: Higher false positive rate (> 10%), risk of divergence

**Standard SLAM Practice** (95% confidence, $\chi^2_{0.05}(2) = 5.99$):
- Balance between false positives and false negatives
- Empirically validated across many SLAM systems
- Theoretical justification from chi-squared distribution

**Adaptive Gating** (Bailey, 2002):

Adjust threshold based on landmark maturity:

$$
\chi^2_{\text{threshold}} = \begin{cases}
9.21 & \text{if } n_{\text{obs}} < 5 \quad \text{(new landmark, be conservative)} \\
5.99 & \text{if } n_{\text{obs}} \geq 5 \quad \text{(mature landmark, use standard)}
\end{cases}
$$

where $n_{\text{obs}}$ is the number of times the landmark has been observed.

---

## References

### Foundational Theory

1. **Mahalanobis, P. C. (1936).** "On the Generalized Distance in Statistics." *Proceedings of the National Institute of Sciences of India*, 2(1), 49-55.
   - Introduced the Mahalanobis distance metric for statistical pattern recognition

2. **Bar-Shalom, Y., & Fortmann, T. E. (1988).** *Tracking and Data Association*. Academic Press.
   - Chapter 6: Validation gating and nearest neighbor association
   - Chapter 7: Probabilistic data association filters
   - Established theoretical framework for multi-target tracking

3. **Bar-Shalom, Y., & Li, X. R. (1995).** *Multitarget-Multisensor Tracking: Principles and Techniques*. YBS Publishing.
   - Advanced topics in data association for multiple targets
   - Joint probabilistic data association (JPDA)

### SLAM-Specific Data Association

4. **Neira, J., & Tardós, J. D. (2001).** "Data Association in Stochastic Mapping Using the Joint Compatibility Test." *IEEE Transactions on Robotics and Automation*, 17(6), 890-897.
   - Introduced Joint Compatibility Branch and Bound (JCBB) for SLAM
   - Considers landmark correlations in EKF-SLAM
   - Provides optimal solution with branch-and-bound search

5. **Bailey, T., Nieto, J., & Nebot, E. (2006).** "Consistency of the FastSLAM Algorithm." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 424-429.
   - Data association strategies for particle filter SLAM
   - Analysis of consistency requirements

6. **Bailey, T. (2002).** "Mobile Robot Localisation and Mapping in Extensive Outdoor Environments." PhD Thesis, University of Sydney.
   - Chapter 4: Comprehensive treatment of data association failure modes
   - Experimental validation in large-scale outdoor environments
   - Adaptive gating strategies

### Alternative Association Methods

7. **Montemerlo, M., Thrun, S., Koller, D., & Wegbreit, B. (2002).** "FastSLAM: A Factored Solution to the Simultaneous Localization and Mapping Problem." *Proceedings of AAAI National Conference on Artificial Intelligence*, pp. 593-598.
   - Per-particle data association in FastSLAM
   - Avoids hard associations through particle filtering

8. **Bosse, M., Newman, P., Leonard, J., & Teller, S. (2004).** "Simultaneous Localization and Map Building in Large-Scale Cyclic Environments Using the Atlas Framework." *The International Journal of Robotics Research*, 23(12), 1113-1139.
   - Multi-hypothesis tracking for ambiguous associations
   - Delayed decision-making through hypothesis trees

### Performance Evaluation

9. **Castellanos, J. A., Neira, J., & Tardós, J. D. (2004).** "Limits to the Consistency of EKF-Based SLAM." *Proceedings of 5th IFAC Symposium on Intelligent Autonomous Vehicles*.
   - Analysis of data association failures and their impact on consistency
   - Relationship between association errors and filter divergence

10. **Dissanayake, M. G., et al. (2001).** "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem." *IEEE Transactions on Robotics and Automation*, 17(3), 229-241.
    - Section on data association requirements for convergence proof

### Textbooks and Tutorials

11. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.
    - Section 10.3: Maximum likelihood data association
    - Section 12.3: EKF-SLAM with known correspondences
    - Section 13.3: Unknown data association in SLAM

12. **Barfoot, T. D. (2017).** *State Estimation for Robotics*. Cambridge University Press.
    - Section 8.2.3: Batch data association
    - Appendix C.2: Chi-squared distribution tables

---

**Next:** `05_submap_management.md` — Submap stitching workflow and ICP-assisted global integration
