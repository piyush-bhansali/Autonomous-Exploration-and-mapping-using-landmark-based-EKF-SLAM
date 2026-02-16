# EKF-SLAM: Theory and Implementation

## Table of Contents
1. [Problem Formulation](#1-problem-formulation)
2. [Prediction Step](#2-prediction-step)
3. [Update Step](#3-update-step)
4. [Landmark Initialisation](#4-landmark-initialisation)
5. [Observation Models](#5-observation-models)
6. [Landmark Normalisation](#6-landmark-normalisation)
7. [Covariance Management](#7-covariance-management)
8. [Convergence and Consistency](#8-convergence-and-consistency)

---

## 1. Problem Formulation

### 1.1 State Vector

The filter maintains a joint state over the robot pose and all active landmarks:

$$
\mathbf{x} = \begin{bmatrix} x_r \\ y_r \\ \theta_r \\ \mathbf{m}_1 \\ \vdots \\ \mathbf{m}_N \end{bmatrix}
$$

where $\mathbf{x}_r = [x_r, y_r, \theta_r]^\top$ is the robot pose in the map frame and $\mathbf{m}_i$ is the $i$-th landmark.

- **Wall:** $\mathbf{m}_i = [\rho_i, \alpha_i]^\top \in \mathbb{R}^2$ (Hessian normal form)
- **Corner:** $\mathbf{m}_i = [x_i, y_i]^\top \in \mathbb{R}^2$ (Cartesian)

The dimension of the state is $3 + 2N$ for $N$ landmarks.

### 1.2 Covariance Matrix

$$
\mathbf{P} = \begin{bmatrix}
\boldsymbol{\Sigma}_{rr} & \boldsymbol{\Sigma}_{rm_1} & \cdots & \boldsymbol{\Sigma}_{rm_N} \\
\boldsymbol{\Sigma}_{m_1 r} & \boldsymbol{\Sigma}_{m_1 m_1} & \cdots & \boldsymbol{\Sigma}_{m_1 m_N} \\
\vdots & \vdots & \ddots & \vdots \\
\boldsymbol{\Sigma}_{m_N r} & \boldsymbol{\Sigma}_{m_N m_1} & \cdots & \boldsymbol{\Sigma}_{m_N m_N}
\end{bmatrix}
$$

The off-diagonal blocks $\boldsymbol{\Sigma}_{m_i m_j}$ capture correlations between landmarks. These correlations arise because all landmarks are observed from a common, uncertain robot position. They are essential for consistent estimation: an update to the robot pose propagates corrections to every landmark through these cross-covariance terms (Smith & Cheeseman, 1987; Dissanayake et al., 2001).

### 1.3 Bayesian Formulation

The filter tracks the Gaussian posterior

$$
p(\mathbf{x}_t \mid \mathbf{z}_{1:t},\, \mathbf{u}_{1:t}) = \mathcal{N}(\mathbf{x}_t;\, \boldsymbol{\mu}_t,\, \mathbf{P}_t).
$$

---

## 2. Prediction Step

### 2.1 Motion Model

Odometry provides incremental motion $(\delta_d, \delta_\theta)$: a change in forward distance and heading. The robot pose is propagated by midpoint integration:

$$
\mathbf{x}_r^+ = f(\mathbf{x}_r,\, \delta_d,\, \delta_\theta) = \begin{bmatrix}
x + \delta_d \cos(\theta_{\mathrm{mid}}) \\
y + \delta_d \sin(\theta_{\mathrm{mid}}) \\
\theta + \delta_\theta
\end{bmatrix}, \qquad \theta_{\mathrm{mid}} = \theta + \tfrac{\delta_\theta}{2}.
$$

Midpoint integration reduces the first-order heading error compared to using the initial heading. Landmarks are assumed static:

$$
\mathbf{m}_i^+ = \mathbf{m}_i \quad \forall\, i.
$$

### 2.2 State Jacobian

The full state Jacobian $\mathbf{F}$ is block-diagonal:

$$
\mathbf{F} = \begin{bmatrix}
\mathbf{F}_{rr} & \mathbf{0} & \cdots \\
\mathbf{0} & \mathbf{I} & \cdots \\
\vdots & \vdots & \ddots
\end{bmatrix},
$$

where the robot-to-robot block is

$$
\mathbf{F}_{rr} = \frac{\partial f}{\partial \mathbf{x}_r} = \begin{bmatrix}
1 & 0 & -\delta_d \sin(\theta_{\mathrm{mid}}) \\
0 & 1 &  \delta_d \cos(\theta_{\mathrm{mid}}) \\
0 & 0 & 1
\end{bmatrix}.
$$

**Derivation of $F_{rr}[0,2]$:**

$$
\frac{\partial}{\partial \theta}\bigl[x + \delta_d \cos(\theta_{\mathrm{mid}})\bigr]
= -\delta_d \sin(\theta_{\mathrm{mid}}) \cdot \frac{\partial \theta_{\mathrm{mid}}}{\partial \theta}
= -\delta_d \sin(\theta_{\mathrm{mid}}) \cdot 1
= -\delta_d \sin(\theta_{\mathrm{mid}}).
$$

Note: $\partial \theta_{\mathrm{mid}}/\partial \theta = 1$ because $\theta_{\mathrm{mid}} = \theta + \delta_\theta/2$ and $\delta_\theta$ is a fixed input, not a state. The $1/2$ factor appears only in the noise Jacobian $\mathbf{G}$ (with respect to $\delta_\theta$), not in $\mathbf{F}$ (with respect to $\theta$).

### 2.3 Noise Jacobian

The noise Jacobian maps the motion noise $\mathbf{q} = [\delta_d^{\mathrm{noise}},\, \delta_\theta^{\mathrm{noise}}]^\top$ into the state:

$$
\mathbf{G} = \frac{\partial f}{\partial \mathbf{q}}\bigg|_{\mathbf{x}_r},
\qquad
\mathbf{G}_{r} = \begin{bmatrix}
\cos(\theta_{\mathrm{mid}}) & -\tfrac{\delta_d}{2} \sin(\theta_{\mathrm{mid}}) \\
\sin(\theta_{\mathrm{mid}}) & \phantom{-}\tfrac{\delta_d}{2} \cos(\theta_{\mathrm{mid}}) \\
0 & 1
\end{bmatrix}.
$$

The $1/2$ factor in column 2 comes from $\partial \theta_{\mathrm{mid}}/\partial \delta_\theta = 1/2$.

### 2.4 Process Noise

The control noise covariance uses a two-component model:

$$
\mathbf{Q}(\delta_d, \delta_\theta) = \begin{bmatrix}
\alpha_d\,\delta_d^2 + \sigma_{d,\min}^2 & 0 \\
0 & \alpha_\theta\,\delta_\theta^2 + \sigma_{\theta,\min}^2
\end{bmatrix},
$$

where $\alpha_d$ and $\alpha_\theta$ are distance- and rotation-noise coefficients, and $\sigma_{\min}^2$ terms prevent a zero process noise when the robot is stationary. Odometry error is empirically proportional to distance travelled (Borenstein & Feng, 1996; Martinelli et al., 2007), so scaling process noise with $\delta_d^2$ and $\delta_\theta^2$ is physically motivated.

### 2.5 Covariance Prediction

$$
\bar{\mathbf{P}} = \mathbf{F}\,\mathbf{P}\,\mathbf{F}^\top + \mathbf{G}\,\mathbf{Q}\,\mathbf{G}^\top.
$$

Prediction always increases uncertainty (or leaves it unchanged). Landmarks are not directly affected by the motion model, but their cross-covariance with the robot — captured in the off-diagonal blocks of $\mathbf{P}$ — increases as robot pose uncertainty grows.

---

## 3. Update Step

### 3.1 Observation Model

An observation of landmark $j$ from robot pose $\mathbf{x}_r$ is modelled as

$$
\mathbf{z} = h_j(\mathbf{x}_r,\, \mathbf{m}_j) + \mathbf{v}, \qquad \mathbf{v} \sim \mathcal{N}(\mathbf{0}, \mathbf{R}).
$$

The specific forms of $h_j$ are given in Section 5.

### 3.2 Observation Jacobian

$$
\mathbf{H} = \frac{\partial h_j}{\partial \mathbf{x}}\bigg|_{\bar{\mathbf{x}}}.
$$

$\mathbf{H}$ is sparse: only the columns corresponding to $\mathbf{x}_r$ and $\mathbf{m}_j$ are non-zero.

### 3.3 Innovation

$$
\boldsymbol{\nu} = \mathbf{z} - h_j(\bar{\mathbf{x}}_r,\, \bar{\mathbf{m}}_j).
$$

Angular components of $\boldsymbol{\nu}$ are wrapped to $[-\pi, \pi]$.

### 3.4 Innovation Covariance

$$
\mathbf{S} = \mathbf{H}\,\bar{\mathbf{P}}\,\mathbf{H}^\top + \mathbf{R}.
$$

### 3.5 Kalman Gain

$$
\mathbf{K} = \bar{\mathbf{P}}\,\mathbf{H}^\top\,\mathbf{S}^{-1}.
$$

### 3.6 State and Covariance Update

$$
\mathbf{x} = \bar{\mathbf{x}} + \mathbf{K}\,\boldsymbol{\nu},
$$

$$
\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\,\bar{\mathbf{P}}\,(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\,\mathbf{R}\,\mathbf{K}^\top.
$$

The Joseph form is used for numerical stability. It guarantees a symmetric positive semi-definite result even with finite-precision arithmetic, unlike the simpler $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\bar{\mathbf{P}}$ form.

A key property of the joint-state formulation is that $\mathbf{K}$ is non-zero for every block of the state, not just the observed landmark. The update therefore corrects all landmark estimates simultaneously through the cross-covariance terms.

---

## 4. Landmark Initialisation

### 4.1 Inverse Observation Model

When an observed feature does not match any existing landmark, it is added to the map. The map-frame parameters are obtained by inverting the observation model:

$$
\mathbf{m}_{\mathrm{new}} = g(\mathbf{x}_r,\, \mathbf{z}).
$$

The specific forms of $g$ are given in Section 5.

### 4.2 State Augmentation

The state and covariance are extended by two elements:

$$
\mathbf{x}^+ = \begin{bmatrix} \mathbf{x} \\ \mathbf{m}_{\mathrm{new}} \end{bmatrix}.
$$

### 4.3 New Landmark Covariance

Linearising $g$ around the current estimate gives Jacobians $\mathbf{H}_r = \partial g / \partial \mathbf{x}_r$ and $\mathbf{H}_z = \partial g / \partial \mathbf{z}$. The new landmark covariance is

$$
\boldsymbol{\Sigma}_{mm} = \mathbf{H}_r\,\mathbf{P}_{rr}\,\mathbf{H}_r^\top + \mathbf{H}_z\,\mathbf{R}\,\mathbf{H}_z^\top.
$$

The first term captures uncertainty from not knowing the robot pose exactly. The second term captures measurement noise.

### 4.4 Cross-Covariance

The new landmark is correlated with the existing state through the robot pose:

$$
\boldsymbol{\Sigma}_{xm} = \mathbf{P}_x\,\begin{bmatrix} \mathbf{H}_r^\top \\ \mathbf{0} \end{bmatrix}.
$$

### 4.5 Augmented Covariance

$$
\mathbf{P}^+ = \begin{bmatrix}
\mathbf{P} & \boldsymbol{\Sigma}_{xm} \\
\boldsymbol{\Sigma}_{xm}^\top & \boldsymbol{\Sigma}_{mm}
\end{bmatrix}.
$$

---

## 5. Observation Models

### 5.1 Wall Landmarks

A wall in the map frame is $(\rho_m, \alpha_m)$. Observed from robot pose $(x_r, y_r, \theta_r)$, the predicted robot-frame observation is:

$$
\hat{\mathbf{z}} = \begin{bmatrix} \hat{\rho}_r \\ \hat{\alpha}_r \end{bmatrix}
= \begin{bmatrix}
\rho_m - (x_r \cos\alpha_m + y_r \sin\alpha_m) \\
\alpha_m - \theta_r
\end{bmatrix}.
$$

**Derivation.** Any point $(x, y)$ on the map-frame wall satisfies $x\cos\alpha_m + y\sin\alpha_m = \rho_m$. Expressing this in the robot frame, the perpendicular distance from the robot's origin to the wall is $\rho_r = \rho_m - x_r\cos\alpha_m - y_r\sin\alpha_m$. The wall normal angle in the robot frame is $\alpha_r = \alpha_m - \theta_r$.

**Observation Jacobian.** Columns indexed by $[x_r, y_r, \theta_r, \rho_m, \alpha_m]$:

$$
\mathbf{H} = \begin{bmatrix}
-\cos\alpha_m & -\sin\alpha_m & 0 & 1 & x_r\sin\alpha_m - y_r\cos\alpha_m \\
0 & 0 & -1 & 0 & 1
\end{bmatrix}.
$$

**Inverse model.** Given $(\rho_r, \alpha_r)$ in the robot frame:

$$
\rho_m = \rho_r + x_r\cos(\alpha_r + \theta_r) + y_r\sin(\alpha_r + \theta_r), \qquad \alpha_m = \alpha_r + \theta_r.
$$

**Jacobian of inverse model wrt robot pose** $\mathbf{H}_r$ (2×3):

$$
\mathbf{H}_r = \begin{bmatrix}
\cos\alpha_m & \sin\alpha_m & -x_r\sin\alpha_m + y_r\cos\alpha_m \\
0 & 0 & 1
\end{bmatrix}.
$$

**Jacobian wrt observation** $\mathbf{H}_z$ (2×2):

$$
\mathbf{H}_z = \begin{bmatrix}
1 & -x_r\sin\alpha_m + y_r\cos\alpha_m \\
0 & 1
\end{bmatrix}.
$$

### 5.2 Corner Landmarks

A corner in the map frame is $(x_m, y_m)$. Observed from robot pose $(x_r, y_r, \theta_r)$:

$$
\hat{\mathbf{z}} = \begin{bmatrix} \hat{z}_x \\ \hat{z}_y \end{bmatrix}
= \begin{bmatrix}
 \cos\theta_r & \sin\theta_r \\
-\sin\theta_r & \cos\theta_r
\end{bmatrix}
\begin{bmatrix} x_m - x_r \\ y_m - y_r \end{bmatrix}.
$$

Let $\Delta x = x_m - x_r$, $\Delta y = y_m - y_r$. The observation Jacobian with columns $[x_r, y_r, \theta_r, x_m, y_m]$ is:

$$
\mathbf{H} = \begin{bmatrix}
-\cos\theta_r & -\sin\theta_r & -\sin\theta_r\,\Delta x + \cos\theta_r\,\Delta y
  & \cos\theta_r & \sin\theta_r \\
 \sin\theta_r & -\cos\theta_r & -\cos\theta_r\,\Delta x - \sin\theta_r\,\Delta y
  & -\sin\theta_r & \cos\theta_r
\end{bmatrix}.
$$

**Inverse model.** Given $(z_x, z_y)$ in the robot frame:

$$
\begin{bmatrix} x_m \\ y_m \end{bmatrix}
= \begin{bmatrix} x_r \\ y_r \end{bmatrix}
+ \begin{bmatrix} \cos\theta_r & -\sin\theta_r \\ \sin\theta_r & \cos\theta_r \end{bmatrix}
\begin{bmatrix} z_x \\ z_y \end{bmatrix}.
$$

---

## 6. Landmark Normalisation

After each EKF update, every wall landmark is checked for sign consistency. The EKF update $\mathbf{x} \leftarrow \mathbf{x} + \mathbf{K}\boldsymbol{\nu}$ places no constraint on the sign of $\rho$. If $\rho$ drifts negative, the Hessian normal points in the opposite direction. Future observations of the same wall will then compute $\hat{\alpha}_r = \alpha_m - \theta_r$ with a flipped $\alpha_m$, causing the innovation to exceed the association gate and the wall to be duplicated.

Normalisation restores the canonical form $\rho \geq 0$: if $\rho < 0$, flip both $\rho \leftarrow -\rho$ and $\alpha \leftarrow \alpha + \pi$ (wrapped to $[-\pi, \pi]$).

---

## 7. Covariance Management

### 7.1 Numerical Conditioning

After each update the covariance is symmetrised and eigenvalue-clamped:

1. Symmetrise: $\mathbf{P} \leftarrow (\mathbf{P} + \mathbf{P}^\top)/2$
2. Eigendecompose: $\mathbf{P} = \mathbf{V}\boldsymbol{\Lambda}\mathbf{V}^\top$
3. Clamp: $\tilde{\lambda}_i = \mathrm{clip}(\lambda_i,\; \lambda_{\min},\; \lambda_{\max})$
4. Reconstruct: $\mathbf{P} \leftarrow \mathbf{V}\,\mathrm{diag}(\tilde{\boldsymbol{\lambda}})\,\mathbf{V}^\top$

Typical bounds: $\lambda_{\min} = 10^{-6}$, $\lambda_{\max} = 100$.

### 7.2 Landmark Pruning

Landmarks are removed if they have not been observed for $T_{\mathrm{timeout}}$ consecutive scans (implemented: 50 scans). When a landmark at state index $i$ is removed, rows and columns $i$ and $i+1$ are deleted from both $\mathbf{x}$ and $\mathbf{P}$, and all subsequent landmark indices are decremented. Landmarks are removed in reverse index order to preserve index validity during the process.

---

## 8. Convergence and Consistency

### 8.1 Convergence (Dissanayake et al., 2001)

Three theorems govern the long-run behaviour of EKF-SLAM:

1. **Monotonic reduction.** The determinant of any submatrix of the map covariance decreases monotonically as observations are incorporated.
2. **Full correlation.** In the limit, all landmark estimates become fully correlated: $\lim_{k\to\infty} \mathrm{corr}(\mathbf{m}_i, \mathbf{m}_j) = 1$.
3. **Bounded absolute uncertainty.** The absolute position uncertainty of every landmark is bounded below by the initial robot-pose uncertainty.

The second theorem means that the relative geometry of landmarks converges to a deterministic estimate even though the absolute map position remains uncertain.

### 8.2 Consistency

The EKF linearises the motion and observation models at the current state estimate. This introduces a known inconsistency: the linearised system has only two unobservable degrees of freedom (global $x$ and $y$), whereas the true nonlinear system has three (global $x$, $y$, and $\theta$). As a result, the filter gains spurious orientation information and underestimates angular uncertainty over time (Huang et al., 2010).

The First Estimates Jacobian (FEJ) approach corrects this by computing Jacobians at the first-ever estimate of each variable rather than the current estimate (Huang et al., 2010). This restores the correct three-dimensional unobservable subspace. The current implementation uses standard EKF Jacobians for simplicity; FEJ is a recommended extension for long-duration operation.

---

## References

1. **Smith, R. C., & Cheeseman, P. (1987).** "On the Representation and Estimation of Spatial Uncertainty." *International Journal of Robotics Research*, 5(4), 56–68.

2. **Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001).** "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem." *IEEE Transactions on Robotics and Automation*, 17(3), 229–241.

3. **Durrant-Whyte, H., & Bailey, T. (2006).** "Simultaneous Localization and Mapping: Part I." *IEEE Robotics & Automation Magazine*, 13(2), 99–110.

4. **Bailey, T., & Durrant-Whyte, H. (2006).** "Simultaneous Localization and Mapping (SLAM): Part II." *IEEE Robotics & Automation Magazine*, 13(3), 108–117.

5. **Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010).** "Observability-based Rules for Designing Consistent EKF SLAM Estimators." *International Journal of Robotics Research*, 29(5), 502–528.

6. **Borenstein, J., & Feng, L. (1996).** "Measurement and Correction of Systematic Odometry Errors in Mobile Robots." *IEEE Transactions on Robotics and Automation*, 12(6), 869–880.

7. **Martinelli, A., Tomatis, N., & Siegwart, R. (2007).** "Simultaneous Localization and Odometry Self-Calibration for Mobile Robot." *Autonomous Robots*, 22(1), 75–85.

8. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.

9. **Barfoot, T. D. (2017).** *State Estimation for Robotics*. Cambridge University Press.
