# Extended Kalman Filter SLAM: Mathematical Theory

> **⚠️ LEGACY DOCUMENTATION NOTICE**
> This document contains detailed mathematical derivations for EKF-SLAM components. It was originally written for a hybrid ICP-landmark SLAM system. The thesis has since been refocused as a **comparative study between ICP-based and Feature-based SLAM** (see updated documentation).
>
> **Primary Documentation (Updated for Comparative Study):**
> - `methodology_icp_mapping.md` — Complete ICP-based SLAM methodology
> - `methodology_feature_mapping.md` — Complete feature-based SLAM methodology
> - `literature_review.md` — Academic background for comparative study
> - `00_system_overview.md` — System architecture overview
>
> **Status of this document:**
> - Sections 1-4.8: EKF-SLAM theory (applies to **Feature mode**)
> - Sections 4.9-4.13: Hybrid ICP+landmark updates (**not used in current comparative study**)
> - Sections 5-10: General EKF theory (supplementary reference)
>
> For the current system architecture, refer to the primary documentation listed above.

---

## Table of Contents
1. [Introduction](#1-introduction)
2. [Problem Formulation](#2-problem-formulation)
3. [EKF Prediction Step](#3-ekf-prediction-step)
4. [EKF Update Step](#4-ekf-update-step)
   - 4.1-4.8: Landmark-Based Updates
   - 4.9: Hybrid Update Strategy
   - 4.10: ICP-Based Pose Measurement Update
   - 4.11: Hessian-Based Measurement Uncertainty for ICP
   - 4.12: Implementation of Hessian-Based ICP Covariance
   - 4.13: Unified Hessian-Based Uncertainty Framework
5. [Landmark Initialization](#5-landmark-initialization)
6. [Landmark Observation Models](#6-landmark-observation-models)
7. [Covariance Management](#7-covariance-management)
8. [Convergence Theory and Correlation Growth](#8-convergence-theory-and-correlation-growth)
9. [Observability and Filter Consistency](#9-observability-and-filter-consistency)
10. [Implementation Details](#10-implementation-details)

---

## 1. Introduction

Extended Kalman Filter SLAM (EKF-SLAM) is a probabilistic approach to simultaneously estimate the robot's pose and build a map of the environment. The key idea is to maintain a **joint probability distribution** over the robot pose and all landmark positions, represented as a Gaussian with mean $\mathbf{x}$ and covariance $\mathbf{P}$.

### 1.0 Historical Context

The foundation of EKF-SLAM was established by **Smith & Cheeseman (1987)**, who first introduced the concept of representing spatial uncertainty using covariance matrices and recognized that landmarks observed from uncertain vehicle locations inherit **correlated errors** through the common vehicle uncertainty. This insight—that correlation structure is fundamental to consistent SLAM estimation—remains the cornerstone of all subsequent SLAM research.

**Dissanayake et al. (2001)** provided the first rigorous **convergence proof** for SLAM, demonstrating that the estimated map converges monotonically to a relative map with zero uncertainty. Their three fundamental theorems established that:
1. Any submatrix of the map covariance decreases monotonically
2. Landmark estimates become fully correlated in the limit
3. The lower bound on covariance is determined solely by initial vehicle uncertainty

**Durrant-Whyte & Bailey (2006)** presented a comprehensive tutorial establishing the Bayesian filtering formulation and recursive solution structure that defines modern SLAM algorithms. They emphasized that landmark correlations are the **critical component**—minimizing or ignoring these correlations leads to filter inconsistency.

### 1.1 Why EKF-SLAM?

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

The process noise models the uncertainty introduced by robot motion between time steps. This is critical for maintaining filter consistency and preventing overconfidence in the pose estimate.

### 3.5.1 Odometry Error Model

**Physical sources of odometry error** (Thrun et al., 2005, Section 5.4):

1. **Wheel slip**: Wheels slip proportionally to distance traveled
2. **Encoder quantization**: Discrete encoder ticks accumulate error over distance
3. **Ground contact variation**: Uneven terrain, carpet compression
4. **Kinematic model errors**: Wheelbase uncertainty, wheel radius variation

**Key observation**: Odometry error is **proportional to distance traveled**.

**Experimental validation** (Martinelli et al., 2007):

From odometry calibration experiments on real robots:

$$
\sigma_{\text{error}}(d) = k_1 \cdot d + k_0
$$

where:
- $k_1$: Distance-proportional coefficient (typical: 0.01-0.05)
- $k_0$: Constant offset (sensor noise floor)
- $d$: Distance traveled

This empirically confirms that **uncertainty grows with motion**.

### 3.5.2 Two-Component Process Noise Model

The control noise covariance combines **motion-dependent** and **minimum floor** components:

$$
\mathbf{Q}_t = \begin{bmatrix}
\sigma_d^2 \cdot |\delta_d|^2 + \sigma_{\min,d}^2 & 0 \\
0 & \sigma_\theta^2 \cdot |\delta_\theta|^2 + \sigma_{\min,\theta}^2
\end{bmatrix}
$$

**Component 1: Motion-Dependent Noise** $\sigma_d^2 \cdot |\delta_d|^2$

- Represents **distance-proportional** uncertainty
- $\sigma_d$: Odometry noise coefficient (typical: 0.02-0.10 for wheeled robots)
- Squared distance: variance grows quadratically with motion
- Physical basis: cumulative wheel slip, encoder drift

**Component 2: Minimum Noise Floor** $\sigma_{\min,d}^2$

- Represents **sensor noise** independent of motion
- Prevents singular covariance when robot is stationary ($\delta_d = 0$)
- Accounts for:
  - Encoder jitter
  - IMU bias drift (if using IMU-augmented odometry)
  - Numerical precision limits
- Typical value: $\sigma_{\min,d} = 0.001$-$0.01$ m

**Combined Model** (Thrun et al., 2005):

$$
\mathbf{Q}_t = \mathbf{M}(\delta_d, \delta_\theta) \begin{bmatrix}
\alpha_1 & 0 \\
0 & \alpha_2
\end{bmatrix} \mathbf{M}(\delta_d, \delta_\theta)^T + \mathbf{Q}_{\min}
$$

where $\mathbf{M}(\delta_d, \delta_\theta)$ is motion-dependent and $\mathbf{Q}_{\min}$ is the noise floor.

### 3.5.3 Motion-Scaled vs. Constant Noise

**Standard Approach (Motion-Scaled)** ✅ **Used in this thesis**

For **general mobile robot applications**, motion-scaled noise is essential:

$$
\mathbf{Q}_t = f(\delta_d, \delta_\theta) \quad \text{(depends on motion)}
$$

**When is constant noise acceptable?**

For **high-frequency systems** with guaranteed small motions:

$$
\delta_d < \sqrt{\frac{\sigma_{\min,d}^2}{\sigma_d^2}} \quad \Rightarrow \quad \mathbf{Q}_t \approx \mathbf{Q}_{\min}
$$

**Example:**
- If $\sigma_d = 0.05$ and $\sigma_{\min,d} = 0.01$ m
- Threshold: $\delta_d < 0.2$ m
- For updates with $\delta_d < 0.1$ m, motion term contributes < 25% of total noise

**Comparison** (Bar-Shalom et al., 2001):

| Motion Magnitude | Motion-Scaled $\mathbf{Q}$ | Constant $\mathbf{Q}$ | Risk if Using Constant |
|------------------|--------------------------|---------------------|----------------------|
| Small ($< 0.1$ m) | Correct | Acceptable approximation | Low (< 10% error) |
| Moderate ($0.1$-$0.5$ m) | Correct | Underestimates uncertainty | Medium (filter overconfident) |
| Large ($> 0.5$ m) | Correct | Severely underestimates | High (**inconsistency**) |

**For this thesis:**
- Operating regime: Small to moderate motion ($\delta_d \approx 0.05$-$0.3$ m per update)
- **Motion-scaled model is appropriate** and follows standard robotics practice

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

### 4.9 Hybrid Update Strategy: Combining ICP and Landmark Observations

This thesis implements a **hybrid SLAM architecture** that combines two complementary update mechanisms:

1. **Landmark-based updates** (Section 4.1-4.8): Sparse, feature-based corrections
2. **ICP-based updates** (Sections 4.10-4.12): Dense, point cloud-based corrections

**Complementary Strengths:**

| Aspect | Landmark Updates | ICP Updates |
|--------|-----------------|-------------|
| **Frequency** | Every scan (~10 Hz) | Every scan (when converged) |
| **Information** | Discrete features (walls, corners) | Dense point correspondences |
| **Robustness** | Depends on feature quality | Works in all environments |
| **Accuracy** | Medium (discrete observations) | High (continuous alignment) |
| **Long-term** | Excellent (loop closure) | Prone to drift |
| **Computational** | $O(k)$ for $k$ landmarks | $O(n \log n)$ for $n$ points |

**Integration Strategy:**

Both types of measurements are incorporated as **EKF updates** using the standard Kalman filter framework:

1. **Landmark observation**: $\mathbf{z}_{\text{landmark}} = h_j(\mathbf{x}_r, \mathbf{m}_j) + \mathbf{v}_{\text{landmark}}$
2. **ICP pose measurement**: $\mathbf{z}_{\text{ICP}} = \mathbf{x}_r + \Delta\mathbf{x}_{\text{ICP}} + \mathbf{v}_{\text{ICP}}$

Both use the same Kalman gain framework but with different observation models and measurement covariances.

**Execution Order per Scan:**

```
1. Odometry prediction (Section 3)
2. Landmark updates (Section 4.1-4.8) — sparse corrections
3. ICP update (Section 4.10) — dense correction
4. Landmark pruning (Section 7.2)
```

This dual-correction strategy provides both **short-term accuracy** (ICP) and **long-term consistency** (landmarks).

---

### 4.10 ICP-Based Pose Measurement Update

**Motivation:** ICP (Iterative Closest Point) aligns the current LiDAR scan to the accumulated point cloud, providing a direct estimate of the robot's pose. This serves as an **absolute pose measurement** in the EKF framework.

#### 4.10.1 ICP as a Pose Sensor

ICP solves the alignment problem:

$$
\mathbf{T}^* = \arg\min_{\mathbf{T}} \sum_{i=1}^N \| \mathbf{p}_{\text{target}}^i - \mathbf{T} \mathbf{p}_{\text{source}}^i \|^2
$$

Where:
- $\mathbf{T} \in SE(3)$: Rigid transformation (rotation + translation)
- $\mathbf{p}_{\text{source}}$: Points in current scan
- $\mathbf{p}_{\text{target}}$: Points in accumulated map
- $N$: Number of point correspondences

**2D Pose Extraction:**

From the optimal transformation $\mathbf{T}^*$, we extract the pose correction:

$$
\Delta\mathbf{x}_{\text{ICP}} = \begin{bmatrix}
\Delta x \\
\Delta y \\
\Delta \theta
\end{bmatrix} = \begin{bmatrix}
\mathbf{T}^*_{0,3} \\
\mathbf{T}^*_{1,3} \\
\text{atan2}(\mathbf{T}^*_{1,0}, \mathbf{T}^*_{0,0})
\end{bmatrix}
$$

#### 4.10.2 ICP Observation Model

The ICP result provides a **direct pose measurement**:

$$
\mathbf{z}_{\text{ICP}} = \mathbf{x}_r + \Delta\mathbf{x}_{\text{ICP}} + \mathbf{v}_{\text{ICP}}
$$

Where:
- $\mathbf{x}_r$: Current robot pose estimate (from prediction)
- $\Delta\mathbf{x}_{\text{ICP}}$: Pose correction from ICP alignment
- $\mathbf{v}_{\text{ICP}} \sim \mathcal{N}(\mathbf{0}, \mathbf{R}_{\text{ICP}})$: Measurement noise

**Observation Jacobian:**

Since ICP directly observes the robot pose, the Jacobian is trivial:

$$
\mathbf{H}_{\text{ICP}} = \begin{bmatrix}
\mathbf{I}_{3 \times 3} & \mathbf{0}_{3 \times 2N}
\end{bmatrix}
$$

**Interpretation:** ICP only observes the robot pose, not landmarks.

#### 4.10.3 EKF Update with ICP Measurement

**Predicted Observation:**

$$
\hat{\mathbf{z}}_{\text{ICP}} = \bar{\mathbf{x}}_r
$$

**Innovation:**

$$
\boldsymbol{\nu}_{\text{ICP}} = \mathbf{z}_{\text{ICP}} - \hat{\mathbf{z}}_{\text{ICP}} = \Delta\mathbf{x}_{\text{ICP}}
$$

With angle normalization:
$$
\nu_\theta = \text{atan2}(\sin(\nu_\theta), \cos(\nu_\theta))
$$

**Innovation Covariance:**

$$
\mathbf{S}_{\text{ICP}} = \mathbf{H}_{\text{ICP}} \bar{\mathbf{P}} \mathbf{H}_{\text{ICP}}^T + \mathbf{R}_{\text{ICP}}
$$

Since $\mathbf{H}_{\text{ICP}}$ is $[{\bf I}_{3\times3} \mid {\bf 0}]$:

$$
\mathbf{S}_{\text{ICP}} = \bar{\mathbf{P}}_{rr} + \mathbf{R}_{\text{ICP}}
$$

**Kalman Gain:**

$$
\mathbf{K}_{\text{ICP}} = \bar{\mathbf{P}} \mathbf{H}_{\text{ICP}}^T \mathbf{S}_{\text{ICP}}^{-1} = \begin{bmatrix}
\bar{\mathbf{P}}_{rr} \mathbf{S}_{\text{ICP}}^{-1} \\
\bar{\mathbf{P}}_{m_1 r} \mathbf{S}_{\text{ICP}}^{-1} \\
\bar{\mathbf{P}}_{m_2 r} \mathbf{S}_{\text{ICP}}^{-1} \\
\vdots
\end{bmatrix}
$$

**State Update:**

$$
\mathbf{x} = \bar{\mathbf{x}} + \mathbf{K}_{\text{ICP}} \boldsymbol{\nu}_{\text{ICP}}
$$

**Critical Insight:** Even though ICP only measures the robot pose, the update affects **all landmarks** through the cross-covariance terms $\bar{\mathbf{P}}_{m_i r}$. This is the power of full covariance tracking!

**Covariance Update (Joseph Form):**

$$
\mathbf{P} = (\mathbf{I} - \mathbf{K}_{\text{ICP}} \mathbf{H}_{\text{ICP}}) \bar{\mathbf{P}} (\mathbf{I} - \mathbf{K}_{\text{ICP}} \mathbf{H}_{\text{ICP}})^T + \mathbf{K}_{\text{ICP}} \mathbf{R}_{\text{ICP}} \mathbf{K}_{\text{ICP}}^T
$$

---

### 4.11 Hessian-Based Measurement Uncertainty for ICP

**Problem:** How do we determine the measurement covariance $\mathbf{R}_{\text{ICP}}$?

**Solution:** Use the **Hessian (Information Matrix)** from the ICP optimization to estimate the geometry-dependent uncertainty.

#### 4.11.1 Theoretical Foundation

**ICP Cost Function:**

ICP minimizes the sum of squared distances between corresponding points:

$$
E(\mathbf{x}) = \sum_{i=1}^N \| \mathbf{r}_i(\mathbf{x}) \|^2
$$

Where:
- $\mathbf{x} = [x, y, \theta]^T$: Pose parameters
- $\mathbf{r}_i(\mathbf{x}) = \mathbf{p}_{\text{target}}^i - (\mathbf{R}(\theta) \mathbf{p}_{\text{source}}^i + \mathbf{t})$: Residual for point $i$
- $\mathbf{R}(\theta)$: 2D rotation matrix
- $\mathbf{t} = [x, y]^T$: Translation vector

**Gauss-Newton Approximation:**

At the optimum $\mathbf{x}^*$, the cost function can be approximated by a quadratic:

$$
E(\mathbf{x}) \approx E(\mathbf{x}^*) + \frac{1}{2} (\mathbf{x} - \mathbf{x}^*)^T \mathbf{A} (\mathbf{x} - \mathbf{x}^*)
$$

Where $\mathbf{A}$ is the **Hessian matrix**:

$$
\mathbf{A} = \frac{\partial^2 E}{\partial \mathbf{x}^2} \bigg|_{\mathbf{x}^*}
$$

#### 4.11.2 Hessian Computation via Jacobian

For least-squares problems, the Hessian can be approximated using the Jacobian:

$$
\mathbf{A} \approx \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i
$$

Where $\mathbf{J}_i$ is the Jacobian of the residual:

$$
\mathbf{J}_i = \frac{\partial \mathbf{r}_i}{\partial \mathbf{x}} \in \mathbb{R}^{2 \times 3}
$$

**Residual for Point $i$:**

$$
\mathbf{r}_i = \mathbf{p}_{\text{target}}^i - \left( \mathbf{R}(\theta) \mathbf{p}_{\text{source}}^i + \begin{bmatrix} x \\ y \end{bmatrix} \right)
$$

**Rotation Matrix Derivative:**

$$
\frac{\partial}{\partial \theta} \mathbf{R}(\theta) = \frac{\partial}{\partial \theta} \begin{bmatrix}
\cos\theta & -\sin\theta \\
\sin\theta & \cos\theta
\end{bmatrix} = \begin{bmatrix}
-\sin\theta & -\cos\theta \\
\cos\theta & -\sin\theta
\end{bmatrix}
$$

**Jacobian Components:**

$$
\frac{\partial \mathbf{r}_i}{\partial x} = -\begin{bmatrix} 1 \\ 0 \end{bmatrix}, \quad
\frac{\partial \mathbf{r}_i}{\partial y} = -\begin{bmatrix} 0 \\ 1 \end{bmatrix}
$$

$$
\frac{\partial \mathbf{r}_i}{\partial \theta} = -\frac{\partial \mathbf{R}(\theta)}{\partial \theta} \mathbf{p}_{\text{source}}^i = -\begin{bmatrix}
-\sin\theta & -\cos\theta \\
\cos\theta & -\sin\theta
\end{bmatrix} \begin{bmatrix}
p_x^i \\
p_y^i
\end{bmatrix}
$$

**Complete Jacobian:**

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & \sin\theta \cdot p_x^i + \cos\theta \cdot p_y^i \\
0 & -1 & -\cos\theta \cdot p_x^i + \sin\theta \cdot p_y^i
\end{bmatrix}
$$

**Hessian Accumulation:**

$$
\mathbf{A} = \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i
$$

Expanded:

$$
\mathbf{A} = \sum_{i=1}^N \begin{bmatrix}
1 & 0 & -\frac{\partial r_{ix}}{\partial \theta} \\
0 & 1 & -\frac{\partial r_{iy}}{\partial \theta} \\
-\frac{\partial r_{ix}}{\partial \theta} & -\frac{\partial r_{iy}}{\partial \theta} & \left(\frac{\partial r_{ix}}{\partial \theta}\right)^2 + \left(\frac{\partial r_{iy}}{\partial \theta}\right)^2
\end{bmatrix}
$$

#### 4.11.3 From Hessian to Covariance

**Fisher Information Matrix:**

The Hessian $\mathbf{A}$ is the **Information Matrix** — it quantifies how much information the data provides about the parameters.

**Cramér-Rao Bound:**

The covariance of the parameter estimate is bounded by:

$$
\mathbf{P} \geq (\mathbf{I}(\mathbf{x}))^{-1}
$$

Where $\mathbf{I}(\mathbf{x})$ is the Fisher Information Matrix.

**For ICP with Gaussian noise:**

$$
\mathbf{I}(\mathbf{x}) = \frac{1}{\sigma^2} \mathbf{A}
$$

Therefore, the **measurement covariance** is:

$$
\mathbf{R}_{\text{ICP}} = \sigma^2 \mathbf{A}^{-1}
$$

Where:
- $\sigma^2$: **LiDAR noise variance** (sensor specification)
- $\mathbf{A}^{-1}$: **Geometric uncertainty** (environment-dependent)

#### 4.11.4 Sensor Noise Parameter

**LiDAR Noise from Manufacturer Specifications:**

For TurtleBot3 LDS-01 (HLS LFCD LDS):
- **Accuracy**: ±10 mm
- **Standard deviation**: $\sigma = 0.01$ m
- **Variance**: $\sigma^2 = 0.0001$ m²

This is used as the **fixed noise parameter** in the covariance formula.

**Why Fixed Noise?**

Using a fixed $\sigma^2$ based on sensor specs avoids the **"optimism problem"**:

**Residual-Based Estimation (WRONG):**
$$
\hat{\sigma}^2 = \frac{1}{N-3} \sum_{i=1}^N \|\mathbf{r}_i\|^2
$$

**Problem:** As $N$ increases (more scan points), $\hat{\sigma}^2$ decreases, making the filter **overconfident**.

**Fixed Sensor Noise (CORRECT):**
$$
\sigma^2 = \text{const} = 0.0001 \quad \text{(from sensor datasheet)}
$$

**Result:** Uncertainty properly reflects geometry via $\mathbf{A}^{-1}$, not point density.

#### 4.11.5 Geometric Interpretation of $\mathbf{A}^{-1}$

The Hessian $\mathbf{A}$ captures the **geometry of the environment**:

**High-Feature Environment (e.g., corner):**
- Many points with diverse orientations
- Large $\mathbf{A}$ (high information)
- Small $\mathbf{A}^{-1}$ (low geometric uncertainty)
- **Result**: $\mathbf{R}_{\text{ICP}} = \sigma^2 \mathbf{A}^{-1}$ is small → EKF trusts ICP

**Low-Feature Environment (e.g., corridor):**
- Points aligned along one direction
- Small/singular $\mathbf{A}$ (low information)
- Large $\mathbf{A}^{-1}$ (high geometric uncertainty)
- **Result**: $\mathbf{R}_{\text{ICP}} = \sigma^2 \mathbf{A}^{-1}$ is large → EKF relies on odometry

**Rank Deficiency:**

In degenerate cases (e.g., single wall), $\mathbf{A}$ may be singular. Use **pseudo-inverse**:

$$
\mathbf{R}_{\text{ICP}} = \sigma^2 \mathbf{A}^{\dagger}
$$

Where $\mathbf{A}^{\dagger} = (\mathbf{A} + \epsilon \mathbf{I})^{-1}$ or SVD-based pseudo-inverse.

---

### 4.12 Implementation of Hessian-Based ICP Covariance

**Algorithm:**

```python
def compute_icp_covariance(source_points, target_points, transform, lidar_sigma=0.01):
    """
    Compute Hessian-based covariance for ICP alignment.

    Args:
        source_points: Nx2 array of scan points
        target_points: Mx2 array of map points
        transform: 4x4 transformation matrix from ICP
        lidar_sigma: LiDAR noise std (default: 0.01m for LDS-01)

    Returns:
        R_icp: 3x3 covariance matrix for (x, y, theta)
    """
    # Extract 2D pose from transform
    dx = transform[0, 3]
    dy = transform[1, 3]
    dtheta = np.arctan2(transform[1, 0], transform[0, 0])

    # Rotation matrix and derivative
    c = np.cos(dtheta)
    s = np.sin(dtheta)
    R = np.array([[c, -s], [s, c]])
    dR_dtheta = np.array([[-s, -c], [c, -s]])

    # Find point correspondences
    tree = KDTree(target_points)
    transformed_source = (R @ source_points.T).T + np.array([dx, dy])
    distances, indices = tree.query(transformed_source)

    # Filter inliers (distance threshold)
    inlier_mask = distances < 0.1  # 10cm threshold
    src_inliers = source_points[inlier_mask]
    tgt_inliers = target_points[indices[inlier_mask]]

    if len(src_inliers) < 6:
        return None  # Insufficient correspondences

    # Compute Hessian: A = sum(J^T @ J)
    A = np.zeros((3, 3))

    for p_s in src_inliers:
        # Jacobian of transformed point wrt pose
        dp_dtheta = dR_dtheta @ p_s

        # Jacobian of residual wrt pose (2x3)
        J = np.array([
            [-1.0, 0.0, -dp_dtheta[0]],
            [0.0, -1.0, -dp_dtheta[1]]
        ])

        # Accumulate Hessian
        A += J.T @ J

    # Covariance: P = sigma^2 * A^(-1)
    sigma2 = lidar_sigma ** 2  # Fixed sensor noise (0.0001 for LDS-01)

    try:
        A_inv = np.linalg.inv(A)
    except np.linalg.LinAlgError:
        # Singular matrix (degenerate geometry) - use pseudo-inverse
        A_inv = np.linalg.pinv(A)

    R_icp = sigma2 * A_inv

    return R_icp
```

**Usage in EKF Update:**

```python
# After ICP alignment
points_corrected, pose_correction = scan_to_map_icp(scan, accumulated_map)

if pose_correction is not None:
    # Extract pose correction
    dx = pose_correction['dx']
    dy = pose_correction['dy']
    dtheta = pose_correction['dtheta']

    # Hessian-based covariance (geometry-aware)
    R_icp = pose_correction['covariance']  # 3x3 matrix

    # Compute corrected pose (measurement)
    z_icp = np.array([
        current_pose['x'] + dx,
        current_pose['y'] + dy,
        current_pose['theta'] + dtheta
    ])

    # EKF update with ICP measurement
    ekf.update(z_icp[0], z_icp[1], z_icp[2],
               measurement_covariance=R_icp,
               measurement_type='icp')
```

**Key Points:**

1. **Geometry Adaptation**: $\mathbf{R}_{\text{ICP}}$ automatically adapts to environment
2. **Sensor Calibration**: $\sigma^2 = 0.0001$ from LDS-01 datasheet
3. **Rank Handling**: Pseudo-inverse for degenerate configurations
4. **EKF Integration**: Standard Kalman update framework

---

### 4.13 Unified Hessian-Based Uncertainty Framework

This thesis implements a **mathematically consistent** uncertainty quantification framework across all measurement types using the Hessian (Fisher Information Matrix) with **fixed LiDAR noise**.

#### 4.13.1 Unified Measurement Covariance Formula

**All measurement covariances follow the same principle:**

$$
\mathbf{R} = \sigma^2 \mathbf{A}^{-1}
$$

Where:
- $\sigma^2 = 0.0001$ m²: **Fixed LiDAR noise** (TurtleBot3 LDS-01: ±10mm)
- $\mathbf{A}$: **Hessian (Information Matrix)** from measurement geometry

**Key Insight:** Separating sensor noise ($\sigma^2$) from geometric information ($\mathbf{A}^{-1}$) ensures:
1. **No optimism**: More measurements don't artificially reduce uncertainty
2. **Geometry-aware**: Uncertainty reflects environment structure
3. **Physically grounded**: Based on sensor specifications, not data fitting

#### 4.13.2 Wall Landmark Covariance

**Measurement Model:**

Wall in Hessian form: $\rho = x \cos(\alpha) + y \sin(\alpha)$

**Residual for Point $i$:**

$$
r_i = (x_i \cos\alpha + y_i \sin\alpha) - \rho
$$

**Jacobian (1×2):**

$$
\mathbf{J}_i = \begin{bmatrix}
\frac{\partial r_i}{\partial \rho} & \frac{\partial r_i}{\partial \alpha}
\end{bmatrix} = \begin{bmatrix}
-1 & -x_i \sin\alpha + y_i \cos\alpha
\end{bmatrix}
$$

**Hessian (2×2):**

$$
\mathbf{A}_{\text{wall}} = \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i = \begin{bmatrix}
N & -\sum_i(x_i \sin\alpha - y_i \cos\alpha) \\
-\sum_i(x_i \sin\alpha - y_i \cos\alpha) & \sum_i(x_i \sin\alpha - y_i \cos\alpha)^2
\end{bmatrix}
$$

**Covariance:**

$$
\mathbf{R}_{\text{wall}} = \sigma^2 \mathbf{A}_{\text{wall}}^{-1} \in \mathbb{R}^{2 \times 2} \quad \text{for } (\rho, \alpha)
$$

**Geometric Interpretation:**
- Long wall with many points: Large $\mathbf{A}$ → Small covariance (high certainty)
- Short wall or few points: Small $\mathbf{A}$ → Large covariance (low certainty)

#### 4.13.3 Corner Landmark Covariance

**Measurement Model:**

Corner at position $\mathbf{c} = [x_c, y_c]^T$ estimated from $N$ neighboring points.

**Residual for Point $i$:**

$$
\mathbf{r}_i = \mathbf{p}_i - \mathbf{c} \in \mathbb{R}^2
$$

**Jacobian (2×2):**

$$
\mathbf{J}_i = \frac{\partial \mathbf{r}_i}{\partial \mathbf{c}} = -\mathbf{I}_{2 \times 2}
$$

**Hessian (Isotropic Case):**

$$
\mathbf{A}_{\text{corner,iso}} = \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i = \sum_{i=1}^N \mathbf{I} = N \mathbf{I}
$$

**Hessian (Anisotropic Case):**

If point scatter is directional (e.g., along two walls), use PCA eigenstructure:

$$
\mathbf{A}_{\text{corner}} = N \mathbf{V} \boldsymbol{\Lambda}^{-1} \mathbf{V}^T
$$

Where:
- $\mathbf{V}$: Eigenvectors from PCA (principal directions)
- $\boldsymbol{\Lambda} = \text{diag}(\lambda_1, \lambda_2)$: Eigenvalues (point variance along axes)
- $\boldsymbol{\Lambda}^{-1}$: Information = inverse variance

**Covariance:**

$$
\mathbf{R}_{\text{corner}} = \sigma^2 \mathbf{A}_{\text{corner}}^{-1} \in \mathbb{R}^{2 \times 2} \quad \text{for } (x, y)
$$

**Geometric Interpretation:**
- More neighboring points: Larger $N$ → smaller covariance
- Points tightly clustered: Small $\lambda$ → large information → small covariance
- Points scattered: Large $\lambda$ → small information → large covariance

#### 4.13.4 ICP Pose Covariance (Scan-to-Map)

**Measurement Model:**

ICP estimates pose correction $\Delta\mathbf{x} = [\Delta x, \Delta y, \Delta\theta]^T$.

**Residual for Correspondence $i$:**

$$
\mathbf{r}_i = \mathbf{p}_{\text{target}}^i - (\mathbf{R}(\theta) \mathbf{p}_{\text{source}}^i + \mathbf{t})
$$

**Jacobian (2×3):**

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 & \sin\theta \cdot p_x^i + \cos\theta \cdot p_y^i \\
0 & -1 & -\cos\theta \cdot p_x^i + \sin\theta \cdot p_y^i
\end{bmatrix}
$$

**Hessian (3×3):**

$$
\mathbf{A}_{\text{ICP}} = \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i
$$

**Covariance:**

$$
\mathbf{R}_{\text{ICP}} = \sigma^2 \mathbf{A}_{\text{ICP}}^{-1} \in \mathbb{R}^{3 \times 3} \quad \text{for } (x, y, \theta)
$$

**Geometric Interpretation:**
- Corner environment: Large $\mathbf{A}$ (all directions constrained) → small covariance
- Corridor environment: Small/singular $\mathbf{A}$ (one direction unconstrained) → large covariance in that direction

#### 4.13.5 Comparison with Traditional Approaches

| Approach | Formula | Pros | Cons |
|----------|---------|------|------|
| **Fixed Covariance** | $\mathbf{R} = \mathbf{R}_0$ (constant) | Simple | Ignores geometry and point count |
| **Residual-Based** | $\mathbf{R} = \hat{\sigma}^2 \mathbf{A}^{-1}$ | Adaptive | **Optimism problem**: More points → smaller $\hat{\sigma}^2$ → overconfident |
| **Sample Covariance** | $\mathbf{R} = \frac{1}{N-1}\sum_i(\mathbf{x}_i - \bar{\mathbf{x}})(\mathbf{x}_i - \bar{\mathbf{x}})^T$ | Empirical | Not measurement model-based; biased for small $N$ |
| **Hessian + Fixed Noise** ✓ | $\mathbf{R} = \sigma^2 \mathbf{A}^{-1}$ | **Geometry-aware, physically calibrated, mathematically rigorous** | Requires sensor noise specification |

#### 4.13.6 Implementation Summary

| Measurement Type | Dimension | Hessian Source | Implementation |
|------------------|-----------|----------------|----------------|
| **Wall (Hessian form)** | 2D: $(\rho, \alpha)$ | Point-to-line residuals | `landmark_features.py:353-406` |
| **Corner (Cartesian)** | 2D: $(x, y)$ | Point scatter with PCA | `landmark_features.py:408-485` |
| **Scan-to-Map ICP** | 3D: $(x, y, \theta)$ | Point-to-point correspondences | `mapping_utils.py:68-112` |
| **Submap ICP** | 3D: $(x, y, \theta)$ | Point-to-point correspondences | `submap_stitcher.py:88-180` |

**All use**: $\sigma = 0.01$ m (LDS-01 specification) → $\sigma^2 = 0.0001$ m²

#### 4.13.7 Mathematical Correctness Verification

**Cramér-Rao Bound:**

The Hessian-based covariance satisfies the Cramér-Rao lower bound:

$$
\mathbf{R} = \sigma^2 \mathbf{A}^{-1} = \sigma^2 (\mathbf{I}(\boldsymbol{\theta}))^{-1}
$$

Where $\mathbf{I}(\boldsymbol{\theta}) = \frac{1}{\sigma^2}\mathbf{A}$ is the Fisher Information Matrix.

**Consistency:**

For a Gaussian measurement model $\mathbf{z} = h(\boldsymbol{\theta}) + \boldsymbol{\epsilon}$ with $\boldsymbol{\epsilon} \sim \mathcal{N}(0, \sigma^2 \mathbf{I})$:

$$
\mathbf{I}(\boldsymbol{\theta}) = \frac{1}{\sigma^2} \mathbb{E}\left[\left(\frac{\partial h}{\partial \boldsymbol{\theta}}\right)^T \left(\frac{\partial h}{\partial \boldsymbol{\theta}}\right)\right] = \frac{1}{\sigma^2} \sum_i \mathbf{J}_i^T \mathbf{J}_i
$$

This is exactly what we compute!

**Key Properties:**

1. ✓ **Unbiased**: Covariance correctly captures measurement uncertainty
2. ✓ **Consistent**: Matches Cramér-Rao bound (optimal estimator)
3. ✓ **Geometry-aware**: $\mathbf{A}^{-1}$ adapts to environment structure
4. ✓ **Calibrated**: $\sigma^2$ from sensor specifications
5. ✓ **No optimism**: Fixed $\sigma^2$ prevents artificial confidence growth

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

## 8. Convergence Theory and Correlation Growth

### 8.1 Fundamental Theorems (Dissanayake et al., 2001)

The convergence properties of SLAM are governed by three fundamental theorems:

**Theorem 1: Monotonic Covariance Reduction**

The determinant of any submatrix of the map covariance matrix decreases monotonically as observations are incorporated:

$$
|\mathbf{P}_{k+1}| \leq |\mathbf{P}_k|
$$

**Interpretation:** Uncertainty never increases with more observations.

**Theorem 2: Full Correlation in the Limit**

In the limit, all landmark estimates become fully correlated:

$$
\lim_{k \to \infty} \text{corr}(\mathbf{m}_i, \mathbf{m}_j) = 1 \quad \forall i, j
$$

**Theorem 3: Bounded Absolute Uncertainty**

The absolute location uncertainty of landmarks is bounded by the initial vehicle uncertainty:

$$
\mathbf{P}_{\infty} \geq \mathbf{P}_0^v
$$

where $\mathbf{P}_0^v$ is the initial robot pose uncertainty.

### 8.2 Correlation Growth Mechanism

As the robot observes landmarks from different poses, the correlation between landmarks grows due to the **common vehicle uncertainty**:

1. **Initial observation**: Landmark $\mathbf{m}_i$ inherits robot uncertainty $\mathbf{P}_{rr}$
2. **Robot moves**: New uncertainty added through motion
3. **Second observation**: Landmark $\mathbf{m}_j$ inherits the **same** robot uncertainty
4. **Result**: $\mathbf{m}_i$ and $\mathbf{m}_j$ are correlated through common origin uncertainty

**Mathematical Expression:**

$$
\text{Cov}(\mathbf{m}_i, \mathbf{m}_j) = \mathbf{H}_i \mathbf{P}_{rr} \mathbf{H}_j^T
$$

### 8.3 Practical Implications

1. **Map consistency**: Relative landmark positions converge to zero uncertainty
2. **Absolute drift**: Global map position remains uncertain (unobservable)
3. **Loop closure benefit**: Re-observing landmarks drastically reduces all correlations
4. **Computational cost**: Full covariance $O(n^2)$ storage, $O(n^3)$ update

---

## 9. Observability and Filter Consistency

### 9.1 The Consistency Problem (Huang et al., 2010)

**Critical Discovery**: Standard EKF-SLAM implementations suffer from **inconsistency**—the filter underestimates its true uncertainty, leading to overconfidence and potential divergence.

**Root Cause**: Observability properties mismatch between the true nonlinear system and the linearized EKF system.

### 9.2 Unobservable Directions in SLAM

The nonlinear SLAM system has **three unobservable degrees of freedom**:

1. **Global X position**: Cannot determine absolute x-coordinate in world
2. **Global Y position**: Cannot determine absolute y-coordinate in world
3. **Global orientation**: Cannot determine absolute heading in world

**Why?** Observations only provide relative measurements—we can build a consistent map, but not know its absolute pose in the world.

**Nullspace of Observability Matrix** (true system):

$$
\mathcal{N}(\mathbf{M}) = \text{span}\left\{\begin{bmatrix} \mathbf{1}_2 \\ 0 \\ \mathbf{1}_2 \\ \vdots \end{bmatrix}, \begin{bmatrix} \mathbf{0}_2 \\ 1 \\ \mathbf{0}_2 \\ \vdots \end{bmatrix}\right\}
$$

**Dimension**: 3 (global x, y, θ)

### 9.3 EKF Linearization Error

**Problem**: Standard EKF uses **current state estimates** for linearization:

$$
\mathbf{F}_k = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k|k}}
$$

This causes the linearized system to have only **2 unobservable directions** (missing global orientation component), creating **spurious information gain** in the orientation.

**Nullspace of EKF Observability Matrix** (linearized):

$$
\mathcal{N}(\mathbf{M}_{\text{EKF}}) = \text{span}\left\{\begin{bmatrix} \mathbf{I}_2 \\ \mathbf{0}_{1 \times 2} \\ \mathbf{I}_2 \\ \vdots \end{bmatrix}\right\}
$$

**Dimension**: 2 (only global x, y) — **orientation artificially observable!**

### 9.4 First Estimates Jacobian (FEJ) Solution

**Fix**: Use **first-ever estimates** for Jacobian computation instead of current estimates:

**Modified State Jacobian:**
$$
\bar{\mathbf{F}}_k = \frac{\partial f}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{k|k-1}} \quad \text{(use prior, not posterior)}
$$

**Modified Measurement Jacobian:**
$$
\bar{\mathbf{H}}_k = \frac{\partial h}{\partial \mathbf{x}}\bigg|_{\hat{\mathbf{x}}_{\text{first}}} \quad \text{(use first-ever landmark estimate)}
$$

**Result**: Restores correct 3D unobservable subspace → improved consistency

**Experimental Validation** (Huang et al., 2010, Tables 1-3):
- Standard EKF: 95% confidence bounds violated ~50% of time (inconsistent)
- FEJ-EKF: 95% confidence bounds respected ~95% of time (consistent)

### 9.5 Implementation Consideration

For this thesis implementation, we use **standard EKF** for simplicity, accepting minor inconsistency. For production systems or long-duration missions, **FEJ-EKF** or other consistency-preserving methods are recommended.

---

## 10. Implementation Details

### 10.1 Hybrid Algorithm Summary

```
Initialize:
  x ← [x_r, y_r, θ_r]  (robot pose only)
  P ← I * 0.01  (small initial uncertainty)
  landmarks ← {}  (empty map)
  accumulated_scan_points ← []  (for ICP)

For each timestep t:

  // ============================================================
  // 1. PREDICT (Motion Model - Section 3)
  // ============================================================
  Δd, Δθ ← get_odometry_delta()
  x_r ← motion_model(x_r, Δd, Δθ)
  F ← compute_motion_jacobian(x_r, Δd, Δθ)
  G ← compute_control_jacobian(x_r, Δd, Δθ)
  Q ← compute_process_noise(Δd, Δθ)  // Motion-scaled
  P ← F * P * F^T + G * Q * G^T

  // ============================================================
  // 2. LANDMARK-BASED UPDATES (Sparse Corrections - Section 4.1-4.8)
  // ============================================================

  // 2a. Feature Extraction
  features ← extract_features(scan)  // Walls and corners

  // 2b. Data Association
  For each feature z:
    matched_id ← find_matching_landmark(z, x, P)

    If matched_id exists:
      // UPDATE (Landmark Observation)
      ẑ ← predict_observation(x_r, landmarks[matched_id])
      ν ← z - ẑ  (innovation)
      H ← compute_observation_jacobian(x_r, landmarks[matched_id])
      S ← H * P * H^T + R_landmark  (innovation covariance)

      // Mahalanobis gating
      d_mahal ← sqrt(ν^T * inv(S) * ν)
      If d_mahal < threshold:
        K ← P * H^T * inv(S)  (Kalman gain)
        x ← x + K * ν  (state update - all landmarks affected!)
        P ← (I - K*H) * P * (I - K*H)^T + K*R_landmark*K^T  (Joseph form)

    Else:
      // INITIALIZE (New Landmark)
      m_new ← inverse_observation_model(x_r, z)
      x ← [x; m_new]  (augment state)
      P ← augment_covariance(P, x_r, z, R_landmark)

  // ============================================================
  // 3. ICP-BASED UPDATE (Dense Correction - Section 4.10-4.12)
  // ============================================================

  // 3a. Transform current scan to submap frame
  scan_points ← transform_to_submap_frame(scan, x_r, submap_start_pose)
  accumulated_scan_points.append(scan_points)

  // 3b. ICP alignment (when sufficient points accumulated)
  If len(accumulated_scan_points) >= min_points:
    // Run ICP
    T_icp, fitness ← run_icp(scan_points, accumulated_scan_points)

    If fitness > threshold:
      // Extract pose correction
      Δx_icp ← extract_pose_from_transform(T_icp)

      // Compute Hessian-based covariance (Section 4.11)
      A ← compute_hessian(scan_points, accumulated_scan_points, T_icp)
      σ² ← lidar_noise_variance  // 0.0001 for LDS-01
      R_icp ← σ² * inv(A)  // Geometry-aware uncertainty

      // ICP measurement
      z_icp ← x_r + Δx_icp

      // EKF update with ICP measurement
      ν_icp ← z_icp - x_r
      H_icp ← [I_3x3 | 0]  // Only observes robot pose
      S_icp ← P_rr + R_icp
      K_icp ← P * H_icp^T * inv(S_icp)
      x ← x + K_icp * ν_icp  (robot AND landmarks updated!)
      P ← (I - K_icp*H_icp) * P * (I - K_icp*H_icp)^T + K_icp*R_icp*K_icp^T

  // ============================================================
  // 4. LANDMARK MANAGEMENT (Section 7.2)
  // ============================================================
  remove_old_landmarks()  // Prune unseen landmarks

  // ============================================================
  // 5. COVARIANCE CONDITIONING (Section 7.1)
  // ============================================================
  P ← condition_covariance(P)  // Enforce eigenvalue bounds
```

### 10.2 Code Mapping

| Algorithm Step | Implementation File | Function |
|---------------|-------------------|----------|
| **Motion Prediction** | `ekf_slam.py` | `predict_with_relative_motion()` |
| **Landmark Extraction** | `landmark_features.py` | `extract_features()` |
| **Data Association** | `data_association.py` | `associate_landmarks()` |
| **Landmark Update** | `ekf_slam.py` | `update_landmark_observation()` |
| **Landmark Initialization** | `ekf_slam.py` | `add_landmark()` |
| **ICP Alignment** | `mapping_utils.py` | `scan_to_map_icp()` |
| **ICP Covariance** | `mapping_utils.py` | (inline, lines 87-112) |
| **ICP Update** | `ekf_slam.py` | `update()` |
| **Submap ICP** | `submap_stitcher.py` | `align_submap_with_icp()` |
| **Submap Covariance** | `submap_stitcher.py` | `_compute_icp_covariance()` |
| **Landmark Pruning** | `ekf_slam.py` | `prune_landmarks()` |
| **Covariance Conditioning** | `ekf_slam.py` | `_condition_covariance()` |

### 10.3 Numerical Considerations

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

### Foundational Papers

1. **Smith, R. C., & Cheeseman, P. (1987).** "On the Representation and Estimation of Spatial Uncertainty." *The International Journal of Robotics Research*, 5(4), 56-68.
   - Introduced covariance-based uncertainty representation and correlation structure in spatial estimation

2. **Smith, R., Self, M., & Cheeseman, P. (1990).** "Estimating Uncertain Spatial Relationships in Robotics." *Autonomous Robot Vehicles*, pp. 167-193.
   - Expanded spatial uncertainty framework to robotic mapping applications

3. **Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001).** "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem." *IEEE Transactions on Robotics and Automation*, 17(3), 229-241.
   - First rigorous convergence proof for SLAM; established three fundamental theorems

4. **Durrant-Whyte, H., & Bailey, T. (2006).** "Simultaneous Localization and Mapping: Part I." *IEEE Robotics & Automation Magazine*, 13(2), 99-110.
   - Comprehensive tutorial establishing Bayesian filtering formulation for SLAM

5. **Bailey, T., & Durrant-Whyte, H. (2006).** "Simultaneous Localization and Mapping (SLAM): Part II." *IEEE Robotics & Automation Magazine*, 13(3), 108-117.
   - Continuation covering computational complexity, data association, and loop closure

### Observability and Consistency

6. **Huang, S., & Dissanayake, G. (2007).** "Convergence and Consistency Analysis for Extended Kalman Filter Based SLAM." *IEEE Transactions on Robotics*, 23(5), 1036-1049.
   - Analyzed consistency properties and convergence of EKF-SLAM

7. **Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010).** "Observability-based Rules for Designing Consistent EKF SLAM Estimators." *The International Journal of Robotics Research*, 29(5), 502-528.
   - Discovered observability mismatch causing EKF inconsistency; proposed FEJ-EKF solution

### Process Noise and Motion Models

8. **Martinelli, A., Tomatis, N., & Siegwart, R. (2007).** "Simultaneous Localization and Odometry Self-Calibration for Mobile Robot." *Autonomous Robots*, 22(1), 75-85.
   - Experimental validation of distance-proportional odometry errors
   - Odometry calibration showing $\sigma_{\text{error}}(d) = k_1 \cdot d + k_0$
   - Empirical coefficients for differential drive robots

9. **Roy, N., & Thrun, S. (1999).** "Coastal Navigation with Mobile Robots." *Advances in Neural Information Processing Systems*, 12, 1043-1049.
   - Motion-scaled uncertainty in practical navigation systems
   - Experimental validation of motion-proportional process noise

10. **Borenstein, J., & Feng, L. (1996).** "Measurement and Correction of Systematic Odometry Errors in Mobile Robots." *IEEE Transactions on Robotics and Automation*, 12(6), 869-880.
    - Comprehensive analysis of odometry error sources
    - Systematic vs. random errors in wheel encoders
    - Calibration methods for reducing motion-dependent errors

11. **Bailey, T., Nieto, J., Guivant, J., Stevens, M., & Nebot, E. (2006).** "Consistency of the EKF-SLAM Algorithm." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 3562-3568.
    - NEES testing for filter consistency validation
    - Relationship between process noise tuning and consistency
    - Experimental methodology for parameter selection

### Textbooks and Tutorials

12. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.
    - Chapter 3: Recursive State Estimation (Kalman Filter derivation)
    - Chapter 5: Robot Motion (Section 5.4: Odometry motion model with distance-dependent noise)
    - Chapter 10: Extended Kalman Filter SLAM

13. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation*. Wiley.
    - Chapter 5: Information and covariance forms of the Kalman filter
    - Chapter 6: Process noise modeling for maneuvering targets
    - Chapter 11: Performance evaluation and consistency testing

14. **Barfoot, T. D. (2017).** *State Estimation for Robotics*. Cambridge University Press.
    - Chapter 8: Batch and Recursive Estimation
    - Chapter 11: Continuous-Time Estimation

15. **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011).** *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.
    - Chapter 5: Mobile Robot Localization
    - Section 5.2: Odometry error modeling

### ICP and Hessian-Based Covariance

16. **Besl, P. J., & McKay, N. D. (1992).** "A Method for Registration of 3-D Shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239-256.
    - Original ICP algorithm for point cloud registration
    - Closest point iteration for rigid body transformation
    - Foundational work for all subsequent ICP variants

17. **Censi, A. (2007).** "An Accurate Closed-Form Estimate of ICP's Covariance." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3167-3172.
    - **Hessian-based covariance estimation for ICP** (directly used in this thesis)
    - Closed-form solution using Fisher Information Matrix: $\mathbf{R} = \sigma^2 \mathbf{A}^{-1}$
    - Mathematical foundation for geometry-dependent uncertainty

18. **Censi, A. (2008).** "An ICP Variant Using a Point-to-Line Metric." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 19-25.
    - Point-to-line ICP for 2D laser scans
    - Improved convergence for structured environments
    - Extended covariance formulation for different ICP variants

19. **Grisetti, G., Stachniss, C., & Burgard, W. (2007).** "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters." *IEEE Transactions on Robotics*, 23(1), 34-46.
    - Scan matching with covariance estimation
    - Integration with particle filter SLAM
    - Practical implementation for mobile robots

20. **Prakhya, S. M., Liu, B., & Lin, W. (2015).** "A Closed-Form Estimate of 3D ICP Covariance." *Proceedings of 14th IAPR International Conference on Machine Vision Applications (MVA)*, pp. 526-529.
    - Extension of Censi's work to 3D
    - Computational efficiency improvements
    - Validation on real datasets

### Hybrid SLAM Approaches

21. **Konolige, K., & Chou, K. (1999).** "Markov Localization using Correlation." *Proceedings of International Joint Conference on Artificial Intelligence (IJCAI)*, pp. 1154-1159.
    - Combining scan matching with feature-based localization
    - Early hybrid approach for mobile robots

22. **Hähnel, D., Burgard, W., Fox, D., & Thrun, S. (2003).** "An Efficient FastSLAM Algorithm for Generating Maps of Large-Scale Cyclic Environments from Raw Laser Range Measurements." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 206-211.
    - Combining scan matching with landmark detection
    - Hybrid dense-sparse mapping strategy

23. **Barfoot, T. D., & Furgale, P. T. (2014).** "Associating Uncertainty with Three-Dimensional Poses for use in Estimation Problems." *IEEE Transactions on Robotics*, 30(3), 679-693.
    - Lie group representation for pose uncertainty
    - Covariance propagation for SE(3) transformations
    - Theoretical foundations for registration uncertainty

---

**Next:** `02_landmark_features.md` — Feature extraction algorithms and Hessian normal form geometry
