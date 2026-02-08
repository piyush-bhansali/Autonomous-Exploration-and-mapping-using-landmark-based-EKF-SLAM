# Extended Kalman Filter SLAM: Mathematical Theory

## Table of Contents
1. [Introduction](#1-introduction)
2. [Problem Formulation](#2-problem-formulation)
3. [EKF Prediction Step](#3-ekf-prediction-step)
4. [EKF Update Step](#4-ekf-update-step)
5. [Landmark Initialization](#5-landmark-initialization)
6. [Landmark Observation Models](#6-landmark-observation-models)
7. [Covariance Management](#7-covariance-management)
8. [Implementation Details](#8-implementation-details)

---

## 1. Introduction

Extended Kalman Filter SLAM (EKF-SLAM) is a probabilistic approach to simultaneously estimate the robot's pose and build a map of the environment. The key idea is to maintain a **joint probability distribution** over the robot pose and all landmark positions, represented as a Gaussian with mean $\mathbf{x}$ and covariance $\mathbf{P}$.

###

 1.1 Why EKF-SLAM?

**Advantages:**
- **Explicit uncertainty:** Full covariance matrix captures correlations
- **Incremental updates:** Online, real-time operation
- **Theoretically grounded:** Optimal under Gaussian assumptions

**Limitations:**
- **Computational cost:** $O(n^3)$ for $n$ landmarks
- **Linearization errors:** EKF assumes local linearity
- **Data association:** Requires correct feature matching

---

## 2. Problem Formulation

### 2.1 State Vector

The system state at time $t$ consists of:

$$
\mathbf{x}_t = \begin{bmatrix}
\mathbf{x}_r \\
\mathbf{m}_1 \\
\mathbf{m}_2 \\
\vdots \\
\mathbf{m}_N
\end{bmatrix}
$$

Where:
- $\mathbf{x}_r = [x, y, \theta]^T \in \mathbb{R}^3$ is the robot pose
  - $(x, y)$: position in the map frame
  - $\theta$: heading angle
- $\mathbf{m}_i \in \mathbb{R}^{d_i}$ is the $i$-th landmark state
  - For walls (Hessian form): $\mathbf{m}_i = [\rho_i, \alpha_i]^T \in \mathbb{R}^2$
  - For corners (Cartesian): $\mathbf{m}_i = [x_i, y_i]^T \in \mathbb{R}^2$

**Dimension:** $\dim(\mathbf{x}_t) = 3 + \sum_{i=1}^N d_i$

### 2.2 Covariance Matrix

The covariance matrix represents the uncertainty and correlations:

$$
\mathbf{P}_t = \begin{bmatrix}
\Sigma_{rr} & \Sigma_{rm_1} & \Sigma_{rm_2} & \cdots & \Sigma_{rm_N} \\
\Sigma_{m_1r} & \Sigma_{m_1m_1} & \Sigma_{m_1m_2} & \cdots & \Sigma_{m_1m_N} \\
\Sigma_{m_2r} & \Sigma_{m_2m_1} & \Sigma_{m_2m_2} & \cdots & \Sigma_{m_2m_N} \\
\vdots & \vdots & \vdots & \ddots & \vdots \\
\Sigma_{m_Nr} & \Sigma_{m_Nm_1} & \Sigma_{m_Nm_2} & \cdots & \Sigma_{m_Nm_N}
\end{bmatrix}
$$

**Interpretation:**
- $\Sigma_{rr}$: Robot pose uncertainty
- $\Sigma_{m_im_i}$: Landmark $i$ uncertainty
- $\Sigma_{rm_i}$: **Cross-correlation** between robot and landmark $i$
  - This correlation is what enables map correction when robot pose is updated!

### 2.3 Gaussian Assumption

We assume the posterior distribution is Gaussian:

$$
p(\mathbf{x}_t | \mathbf{z}_{1:t}, \mathbf{u}_{1:t}) = \mathcal{N}(\mathbf{x}_t; \boldsymbol{\mu}_t, \mathbf{P}_t)
$$

Where:
- $\mathbf{z}_{1:t}$: All observations up to time $t$
- $\mathbf{u}_{1:t}$: All control inputs up to time $t$
- $\boldsymbol{\mu}_t \equiv \mathbf{x}_t$: Mean (our state estimate)
- $\mathbf{P}_t$: Covariance

---

## 3. EKF Prediction Step

The prediction step propagates the state estimate forward using the **motion model**.

### 3.1 Motion Model

Given a control input $\mathbf{u}_t = [\delta_d, \delta_\theta]^T$ (change in distance and orientation from odometry), the robot state evolves as:

$$
\mathbf{x}_r^+ = f(\mathbf{x}_r, \mathbf{u}_t) = \begin{bmatrix}
x + \delta_d \cos(\theta + \delta_\theta / 2) \\
y + \delta_d \sin(\theta + \delta_\theta / 2) \\
\theta + \delta_\theta
\end{bmatrix}
$$

**Rationale:** We use the **midpoint** heading $\theta + \delta_\theta / 2$ to better approximate the arc trajectory.

**Landmarks** do not move (static world assumption):

$$
\mathbf{m}_i^+ = \mathbf{m}_i \quad \forall i
$$

### 3.2 State Prediction

The predicted state is:

$$
\bar{\mathbf{x}}_t = \begin{bmatrix}
f(\mathbf{x}_r, \mathbf{u}_t) \\
\mathbf{m}_1 \\
\mathbf{m}_2 \\
\vdots \\
\mathbf{m}_N
\end{bmatrix}
$$

### 3.3 Jacobian of Motion Model

To propagate uncertainty, we linearize the motion model. The Jacobian is:

$$
\mathbf{F}_t = \frac{\partial f}{\partial \mathbf{x}} \bigg|_{\mathbf{x}_{t-1}, \mathbf{u}_t}
$$

For our motion model:

$$
\mathbf{F}_t = \begin{bmatrix}
\mathbf{F}_{rr} & \mathbf{0} & \mathbf{0} & \cdots \\
\mathbf{0} & \mathbf{I} & \mathbf{0} & \cdots \\
\mathbf{0} & \mathbf{0} & \mathbf{I} & \cdots \\
\vdots & \vdots & \vdots & \ddots
\end{bmatrix}
$$

Where the robot-to-robot block is:

$$
\mathbf{F}_{rr} = \begin{bmatrix}
1 & 0 & -\delta_d \sin(\theta_{\text{mid}}) \\
0 & 1 & \delta_d \cos(\theta_{\text{mid}}) \\
0 & 0 & 1
\end{bmatrix}
$$

With $\theta_{\text{mid}} = \theta + \delta_\theta / 2$.

**Derivation:**

$$
\frac{\partial}{\partial x}\left[x + \delta_d \cos(\theta_{\text{mid}})\right] = 1
$$

$$
\frac{\partial}{\partial \theta}\left[x + \delta_d \cos(\theta_{\text{mid}})\right] = -\delta_d \sin(\theta_{\text{mid}}) \cdot \frac{1}{2}
$$

(Similarly for $y$ component)

### 3.4 Control Noise Jacobian

The motion uncertainty depends on the control input. Define:

$$
\mathbf{G}_t = \frac{\partial f}{\partial \mathbf{u}} \bigg|_{\mathbf{x}_{t-1}, \mathbf{u}_t}
$$

For our model:

$$
\mathbf{G}_t = \begin{bmatrix}
\mathbf{G}_r \\
\mathbf{0} \\
\mathbf{0} \\
\vdots
\end{bmatrix}_{(3+2N) \times 2}
$$

Where:

$$
\mathbf{G}_r = \begin{bmatrix}
\cos(\theta_{\text{mid}}) & -\frac{\delta_d}{2} \sin(\theta_{\text{mid}}) \\
\sin(\theta_{\text{mid}}) & \frac{\delta_d}{2} \cos(\theta_{\text{mid}}) \\
0 & 1
\end{bmatrix}
$$

### 3.5 Process Noise

The control noise covariance is proportional to motion:

$$
\mathbf{Q}_t = \begin{bmatrix}
\sigma_d^2 \cdot |\delta_d|^2 + \sigma_{\min,d}^2 & 0 \\
0 & \sigma_\theta^2 \cdot |\delta_\theta|^2 + \sigma_{\min,\theta}^2
\end{bmatrix}
$$

Where:
- $\sigma_d, \sigma_\theta$: Motion noise coefficients (tunable)
- $\sigma_{\min,d}, \sigma_{\min,\theta}$: Minimum noise (prevents singularity when stationary)

**Rationale:** Larger motions → larger uncertainty.

### 3.6 Covariance Prediction

The predicted covariance is:

$$
\bar{\mathbf{P}}_t = \mathbf{F}_t \mathbf{P}_{t-1} \mathbf{F}_t^T + \mathbf{G}_t \mathbf{Q}_t \mathbf{G}_t^T
$$

**Interpretation:**
1. $\mathbf{F}_t \mathbf{P}_{t-1} \mathbf{F}_t^T$: Propagate existing uncertainty
2. $\mathbf{G}_t \mathbf{Q}_t \mathbf{G}_t^T$: Add new process noise

**Key Property:** Prediction **always increases** uncertainty (entropy increases).

---

## 4. EKF Update Step

The update step **corrects** the state estimate using an observation $\mathbf{z}_t$.

### 4.1 Observation Model

An observation of landmark $j$ is modeled as:

$$
\mathbf{z}_t = h_j(\mathbf{x}_r, \mathbf{m}_j) + \mathbf{v}_t
$$

Where:
- $h_j$: Observation function (converts landmark to sensor measurement)
- $\mathbf{v}_t \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_t)$: Measurement noise

The specific forms of $h_j$ depend on the landmark type (see Section 6).

### 4.2 Predicted Observation

Given the current state estimate $\bar{\mathbf{x}}_t$, the predicted observation is:

$$
\hat{\mathbf{z}}_t = h_j(\bar{\mathbf{x}}_r, \bar{\mathbf{m}}_j)
$$

### 4.3 Innovation

The **innovation** (or residual) is the difference between actual and predicted observations:

$$
\boldsymbol{\nu}_t = \mathbf{z}_t - \hat{\mathbf{z}}_t
$$

**For angular measurements,** we must normalize angles to $[-\pi, \pi]$:

$$
\nu_\theta = \text{atan2}(\sin(\nu_\theta), \cos(\nu_\theta))
$$

### 4.4 Observation Jacobian

Linearize the observation model:

$$
\mathbf{H}_t = \frac{\partial h_j}{\partial \mathbf{x}} \bigg|_{\bar{\mathbf{x}}_t}
$$

This is a sparse matrix of size $(d_z \times \dim(\mathbf{x}))$ where $d_z$ is the observation dimension:

$$
\mathbf{H}_t = \begin{bmatrix}
\frac{\partial h_j}{\partial \mathbf{x}_r} & \mathbf{0} & \cdots & \frac{\partial h_j}{\partial \mathbf{m}_j} & \cdots & \mathbf{0}
\end{bmatrix}
$$

Only the robot and observed landmark blocks are non-zero.

### 4.5 Innovation Covariance

The innovation covariance is:

$$
\mathbf{S}_t = \mathbf{H}_t \bar{\mathbf{P}}_t \mathbf{H}_t^T + \mathbf{R}_t
$$

**Interpretation:** Total uncertainty in the observation prediction, combining:
1. Propagated state uncertainty: $\mathbf{H}_t \bar{\mathbf{P}}_t \mathbf{H}_t^T$
2. Measurement noise: $\mathbf{R}_t$

### 4.6 Kalman Gain

The Kalman gain determines how much to trust the measurement vs. the prediction:

$$
\mathbf{K}_t = \bar{\mathbf{P}}_t \mathbf{H}_t^T \mathbf{S}_t^{-1}
$$

**Interpretation:**
- If $\mathbf{S}_t$ is large (noisy measurement or high prediction uncertainty): $\mathbf{K}_t$ is small → trust prediction
- If $\mathbf{S}_t$ is small (precise measurement and low prediction uncertainty): $\mathbf{K}_t$ is large → trust measurement

### 4.7 State Update

The corrected state estimate is:

$$
\mathbf{x}_t = \bar{\mathbf{x}}_t + \mathbf{K}_t \boldsymbol{\nu}_t
$$

**Key Insight:** This updates **all** elements of the state vector, including:
- Robot pose (directly observed)
- Observed landmark (directly observed)
- **All other landmarks** (via cross-correlations in $\mathbf{P}$!)

This is why maintaining the full covariance matrix is crucial.

### 4.8 Covariance Update

The **Joseph form** of the covariance update (numerically stable):

$$
\mathbf{P}_t = (\mathbf{I} - \mathbf{K}_t \mathbf{H}_t) \bar{\mathbf{P}}_t (\mathbf{I} - \mathbf{K}_t \mathbf{H}_t)^T + \mathbf{K}_t \mathbf{R}_t \mathbf{K}_t^T
$$

**Simpler form** (if numerical stability is ensured):

$$
\mathbf{P}_t = (\mathbf{I} - \mathbf{K}_t \mathbf{H}_t) \bar{\mathbf{P}}_t
$$

**Key Property:** Update **always decreases** uncertainty (information gain).

---

## 5. Landmark Initialization

When a new landmark is observed for the first time, it must be added to the state.

### 5.1 Inverse Observation Model

Given an observation $\mathbf{z}$ in the robot frame and current robot pose $\mathbf{x}_r$, we convert to the map frame:

$$
\mathbf{m}_{\text{new}} = g(\mathbf{x}_r, \mathbf{z})
$$

The function $g$ is the **inverse** of the observation model $h$.

### 5.2 State Augmentation

The new state vector is:

$$
\mathbf{x}^+ = \begin{bmatrix}
\mathbf{x} \\
\mathbf{m}_{\text{new}}
\end{bmatrix}
$$

### 5.3 Jacobians for Initialization

Linearize the inverse model:

$$
\mathbf{H}_r = \frac{\partial g}{\partial \mathbf{x}_r}, \quad \mathbf{H}_z = \frac{\partial g}{\partial \mathbf{z}}
$$

### 5.4 New Landmark Covariance

The uncertainty in the new landmark position is:

$$
\mathbf{P}_{mm} = \mathbf{H}_r \mathbf{P}_{rr} \mathbf{H}_r^T + \mathbf{H}_z \mathbf{R} \mathbf{H}_z^T
$$

Where:
- $\mathbf{P}_{rr}$: Current robot pose covariance
- $\mathbf{R}$: Observation noise covariance

**Interpretation:** Landmark uncertainty comes from:
1. Robot pose uncertainty (we don't know exactly where we are)
2. Measurement noise (sensor imperfection)

### 5.5 Cross-Covariance

The correlation between the new landmark and the rest of the state:

$$
\mathbf{P}_{xm} = \mathbf{P}_{x} \begin{bmatrix}
\mathbf{H}_r^T \\
\mathbf{0}
\end{bmatrix}
$$

### 5.6 Augmented Covariance Matrix

The new covariance is:

$$
\mathbf{P}^+ = \begin{bmatrix}
\mathbf{P} & \mathbf{P}_{xm} \\
\mathbf{P}_{xm}^T & \mathbf{P}_{mm}
\end{bmatrix}
$$

---

## 6. Landmark Observation Models

### 6.1 Wall Landmarks (Hessian Normal Form)

**Representation:** A wall is parameterized by $[\rho, \alpha]$ where:
- $\rho \geq 0$: Perpendicular distance from origin to the line
- $\alpha \in [-\pi, \pi]$: Angle of the normal vector

**Line Equation:**

$$
\rho = x \cos(\alpha) + y \sin(\alpha)
$$

#### 6.1.1 Forward Model (Map → Robot Frame)

Given:
- Robot pose: $(x_r, y_r, \theta_r)$
- Wall in map: $(\rho_m, \alpha_m)$

The observation in robot frame is:

$$
\mathbf{z} = \begin{bmatrix}
\rho_r \\
\alpha_r
\end{bmatrix} = \begin{bmatrix}
\rho_m - (x_r \cos(\alpha_m) + y_r \sin(\alpha_m)) \\
\alpha_m - \theta_r
\end{bmatrix}
$$

**Intuition:**
- $\rho_r$: Distance from robot to wall = map distance minus robot's projection onto normal
- $\alpha_r$: Normal angle in robot frame = map angle minus robot heading

#### 6.1.2 Observation Jacobian

$$
\mathbf{H} = \begin{bmatrix}
-\cos(\alpha_m) & -\sin(\alpha_m) & 0 & 1 & x_r \sin(\alpha_m) - y_r \cos(\alpha_m) \\
0 & 0 & -1 & 0 & 1
\end{bmatrix}
$$

Where columns correspond to: $[x_r, y_r, \theta_r, \rho_m, \alpha_m]$

**Derivation:**

$$
\frac{\partial \rho_r}{\partial x_r} = -\cos(\alpha_m)
$$

$$
\frac{\partial \rho_r}{\partial \alpha_m} = x_r \sin(\alpha_m) - y_r \cos(\alpha_m)
$$

(From chain rule and product rule)

#### 6.1.3 Inverse Model (Robot → Map Frame)

Given observation $(\rho_r, \alpha_r)$ in robot frame:

$$
\begin{bmatrix}
\rho_m \\
\alpha_m
\end{bmatrix} = \begin{bmatrix}
\rho_r + x_r \cos(\alpha_r + \theta_r) + y_r \sin(\alpha_r + \theta_r) \\
\alpha_r + \theta_r
\end{bmatrix}
$$

### 6.2 Corner Landmarks (Cartesian)

**Representation:** A corner is simply $(x_m, y_m)$ in the map frame.

#### 6.2.1 Forward Model

Given:
- Robot pose: $(x_r, y_r, \theta_r)$
- Corner in map: $(x_m, y_m)$

The observation in robot frame (after rotation by $-\theta_r$):

$$
\mathbf{z} = \begin{bmatrix}
x_r' \\
y_r'
\end{bmatrix} = \begin{bmatrix}
\cos(\theta_r) & \sin(\theta_r) \\
-\sin(\theta_r) & \cos(\theta_r)
\end{bmatrix} \begin{bmatrix}
x_m - x_r \\
y_m - y_r
\end{bmatrix}
$$

Expanded:

$$
\begin{bmatrix}
x_r' \\
y_r'
\end{bmatrix} = \begin{bmatrix}
(x_m - x_r) \cos(\theta_r) + (y_m - y_r) \sin(\theta_r) \\
-(x_m - x_r) \sin(\theta_r) + (y_m - y_r) \cos(\theta_r)
\end{bmatrix}
$$

#### 6.2.2 Observation Jacobian

Let $\Delta x = x_m - x_r$, $\Delta y = y_m - y_r$.

$$
\mathbf{H} = \begin{bmatrix}
-\cos(\theta_r) & -\sin(\theta_r) & -\Delta x \sin(\theta_r) + \Delta y \cos(\theta_r) & \cos(\theta_r) & \sin(\theta_r) \\
\sin(\theta_r) & -\cos(\theta_r) & -\Delta x \cos(\theta_r) - \Delta y \sin(\theta_r) & -\sin(\theta_r) & \cos(\theta_r)
\end{bmatrix}
$$

Columns: $[x_r, y_r, \theta_r, x_m, y_m]$

#### 6.2.3 Inverse Model

Given observation $(x_r', y_r')$ in robot frame:

$$
\begin{bmatrix}
x_m \\
y_m
\end{bmatrix} = \begin{bmatrix}
x_r \\
y_r
\end{bmatrix} + \begin{bmatrix}
\cos(\theta_r) & -\sin(\theta_r) \\
\sin(\theta_r) & \cos(\theta_r)
\end{bmatrix} \begin{bmatrix}
x_r' \\
y_r'
\end{bmatrix}
$$

---

## 7. Covariance Management

### 7.1 Covariance Conditioning

To prevent numerical issues, we enforce bounds on eigenvalues:

$$
\lambda_{\min} \leq \lambda_i(\mathbf{P}) \leq \lambda_{\max}
$$

**Algorithm:**
1. Symmetrize: $\mathbf{P} \leftarrow (\mathbf{P} + \mathbf{P}^T) / 2$
2. Eigendecomposition: $\mathbf{P} = \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T$
3. Clamp eigenvalues: $\tilde{\lambda}_i = \text{clip}(\lambda_i, \lambda_{\min}, \lambda_{\max})$
4. Reconstruct: $\mathbf{P} \leftarrow \mathbf{V} \text{diag}(\tilde{\lambda}) \mathbf{V}^T$

**Typical values:** $\lambda_{\min} = 10^{-6}$, $\lambda_{\max} = 100$

### 7.2 Landmark Pruning

To manage computational complexity, landmarks are removed if:
1. Not observed for $N_{\text{timeout}}$ scans (e.g., 50)
2. Fewer than $N_{\min}$ observations (e.g., 2) after initialization

**Removal procedure:**
1. Identify landmark at index $i$ to remove
2. Delete rows/columns $i$ and $i+1$ from $\mathbf{x}$ and $\mathbf{P}$
3. Update all landmark indices $j > i$ by decrementing by 2

**Important:** Remove in reverse order (highest index first) to avoid invalidating indices.

---

## 8. Implementation Details

### 8.1 Algorithm Summary

```
Initialize:
  x ← [x_r, y_r, θ_r]  (robot pose only)
  P ← I * 0.01  (small initial uncertainty)
  landmarks ← {}  (empty map)

For each timestep t:

  // 1. PREDICT (Motion Model)
  Δd, Δθ ← get_odometry_delta()
  x_r ← motion_model(x_r, Δd, Δθ)
  F ← compute_motion_jacobian(x_r, Δd, Δθ)
  G ← compute_control_jacobian(x_r, Δd, Δθ)
  Q ← compute_process_noise(Δd, Δθ)
  P ← F * P * F^T + G * Q * G^T

  // 2. OBSERVE (Feature Extraction)
  features ← extract_features(scan)

  // 3. ASSOCIATE (Data Association)
  For each feature z:
    matched_id ← find_matching_landmark(z, x, P)

    If matched_id exists:
      // UPDATE (Measurement Model)
      ẑ ← predict_observation(x_r, landmarks[matched_id])
      ν ← z - ẑ  (innovation)
      H ← compute_observation_jacobian(x_r, landmarks[matched_id])
      S ← H * P * H^T + R  (innovation covariance)
      K ← P * H^T * inv(S)  (Kalman gain)
      x ← x + K * ν  (state update)
      P ← (I - K*H) * P * (I - K*H)^T + K*R*K^T  (covariance update)

    Else:
      // INITIALIZE (New Landmark)
      m_new ← inverse_observation_model(x_r, z)
      x ← [x; m_new]  (augment state)
      P ← augment_covariance(P, x_r, z, R)

  // 4. PRUNE (Landmark Management)
  remove_old_landmarks()
```

### 8.2 Code Mapping

| Algorithm Step | Implementation File | Function |
|---------------|-------------------|----------|
| Prediction | `ekf_slam.py` | `predict_with_relative_motion()` |
| Observation | `landmark_features.py` | `extract_features()` |
| Association | `data_association.py` | `associate_landmarks()` |
| Update | `ekf_slam.py` | `update_landmark_observation()` |
| Initialization | `ekf_slam.py` | `add_landmark()` |
| Pruning | `ekf_slam.py` | `prune_landmarks()` |

### 8.3 Numerical Considerations

**1. Angle Normalization:**
Always wrap angles to $[-\pi, \pi]$ after updates:
```python
theta = np.arctan2(np.sin(theta), np.cos(theta))
```

**2. Singularity Handling:**
Check condition number before matrix inversion:
```python
if np.linalg.cond(S) < 1e10:
    K = P @ H.T @ np.linalg.inv(S)
else:
    # Use pseudoinverse or skip update
```

**3. Symmetry Enforcement:**
After each update:
```python
P = (P + P.T) / 2.0
```

---

## References

1. **Smith, R., Self, M., & Cheeseman, P. (1990).** "Estimating Uncertain Spatial Relationships in Robotics." *Autonomous Robot Vehicles*, pp. 167-193.

2. **Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001).** "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem." *IEEE Transactions on Robotics and Automation*, 17(3), 229-241.

3. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press. Chapter 10: EKF-SLAM.

4. **Barfoot, T. D. (2017).** *State Estimation for Robotics*. Cambridge University Press. Chapter 8: Batch and Recursive Estimation.

5. **Bailey, T., & Durrant-Whyte, H. (2006).** "Simultaneous Localization and Mapping (SLAM): Part II." *IEEE Robotics & Automation Magazine*, 13(3), 108-117.

---

**Next:** `02_landmark_features.md` — Feature extraction algorithms and Hessian normal form geometry
