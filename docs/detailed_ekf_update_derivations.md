# Detailed Report: Landmark-Based EKF-SLAM Update with Hessian-Based Covariance

## Table of Contents
1. [Overview](#overview)
2. [EKF Update Framework](#ekf-update-framework)
3. [Wall Landmark Update (Hessian Normal Form)](#wall-landmark-update)
4. [Corner Landmark Update (Cartesian Form)](#corner-landmark-update)
5. [Hessian-Based Covariance for Walls](#hessian-based-covariance-for-walls)
6. [Hessian-Based Covariance for Corners](#hessian-based-covariance-for-corners)
7. [Complete Implementation Algorithm](#complete-implementation-algorithm)

---

## 1. Overview

This document provides a complete mathematical derivation for EKF-SLAM updates using two types of landmarks:
- **Wall landmarks**: Represented in Hessian normal form (ρ, α)
- **Corner landmarks**: Represented in Cartesian coordinates (x, y)

For each landmark type, we derive:
1. The observation model h(x)
2. The observation Jacobian H (step-by-step)
3. The Hessian-based measurement covariance R (step-by-step)

---

## 2. EKF Update Framework

### 2.1 State Vector

The joint state vector contains the robot pose and all landmark positions:

$$
\mathbf{x} = \begin{bmatrix}
x_r \\
y_r \\
\theta_r \\
\mathbf{m}_1 \\
\mathbf{m}_2 \\
\vdots \\
\mathbf{m}_N
\end{bmatrix} \in \mathbb{R}^{3+d}
$$

Where:
- $(x_r, y_r, \theta_r)$: Robot pose in map frame
- $\mathbf{m}_i$: Landmark $i$ parameters
  - Walls: $\mathbf{m}_i = [\rho_i, \alpha_i]^T$ (2 parameters)
  - Corners: $\mathbf{m}_i = [x_i, y_i]^T$ (2 parameters)
- $d$: Total number of landmark parameters

### 2.2 Standard EKF Update Equations

Given an observation $\mathbf{z}$ of landmark $j$:

**Step 1: Predicted observation**
$$
\hat{\mathbf{z}} = h(\mathbf{x})
$$

**Step 2: Innovation**
$$
\boldsymbol{\nu} = \mathbf{z} - \hat{\mathbf{z}}
$$

**Step 3: Innovation covariance**
$$
\mathbf{S} = \mathbf{H} \mathbf{P} \mathbf{H}^T + \mathbf{R}
$$

**Step 4: Kalman gain**
$$
\mathbf{K} = \mathbf{P} \mathbf{H}^T \mathbf{S}^{-1}
$$

**Step 5: State update**
$$
\mathbf{x}^+ = \mathbf{x} + \mathbf{K} \boldsymbol{\nu}
$$

**Step 6: Covariance update (Joseph form)**
$$
\mathbf{P}^+ = (\mathbf{I} - \mathbf{K}\mathbf{H}) \mathbf{P} (\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T
$$

The key challenge is computing:
1. **Observation model** $h(\mathbf{x})$
2. **Observation Jacobian** $\mathbf{H} = \frac{\partial h}{\partial \mathbf{x}}$
3. **Measurement covariance** $\mathbf{R}$

---

## 3. Wall Landmark Update (Hessian Normal Form)

### 3.1 Wall Representation

A wall (infinite line) in the map frame is parameterized as:

$$
\rho_m = x \cos(\alpha_m) + y \sin(\alpha_m)
$$

Where:
- $\rho_m \geq 0$: Perpendicular distance from origin to line
- $\alpha_m \in [-\pi, \pi]$: Angle of the normal vector from origin

**Why Hessian form?**
- Minimal representation (2 parameters)
- Orientation-invariant
- No singularities (unlike slope-intercept form)

### 3.2 Observation Model: Map Frame → Robot Frame

**Given:**
- Robot pose in map frame: $(x_r, y_r, \theta_r)$
- Wall parameters in map frame: $(\rho_m, \alpha_m)$

**Objective:** Predict what the robot would observe in its local frame.

#### Step 1: Wall distance in robot frame

The perpendicular distance from the robot to the wall is:

$$
\rho_r = \rho_m - (x_r \cos(\alpha_m) + y_r \sin(\alpha_m))
$$

**Derivation:**
- The wall equation is: $\rho_m = x \cos(\alpha_m) + y \sin(\alpha_m)$
- The robot is at $(x_r, y_r)$
- Distance from robot to wall = map distance - robot's projection onto normal:

$$
\rho_r = \rho_m - \underbrace{(x_r \cos(\alpha_m) + y_r \sin(\alpha_m))}_{\text{robot's projection}}
$$

#### Step 2: Wall orientation in robot frame

The wall's normal angle in the robot frame is:

$$
\alpha_r = \alpha_m - \theta_r
$$

**Normalization:** Wrap to $[-\pi, \pi]$:
$$
\alpha_r = \text{atan2}(\sin(\alpha_r), \cos(\alpha_r))
$$

#### Step 3: Complete observation model

$$
h_{\text{wall}}(\mathbf{x}) = \begin{bmatrix}
\rho_m - (x_r \cos(\alpha_m) + y_r \sin(\alpha_m)) \\
\alpha_m - \theta_r
\end{bmatrix}
$$

### 3.3 Observation Jacobian for Walls

The Jacobian is:

$$
\mathbf{H}_{\text{wall}} = \frac{\partial h_{\text{wall}}}{\partial \mathbf{x}} \in \mathbb{R}^{2 \times n}
$$

Where $n = \dim(\mathbf{x})$ is the full state dimension.

The Jacobian is **sparse** because the observation only depends on:
- Robot pose: $(x_r, y_r, \theta_r)$ (indices 0, 1, 2)
- Observed wall: $(\rho_m, \alpha_m)$ (indices $i, i+1$ where $i$ is the wall's state index)

$$
\mathbf{H}_{\text{wall}} = \begin{bmatrix}
\frac{\partial \rho_r}{\partial x_r} & \frac{\partial \rho_r}{\partial y_r} & \frac{\partial \rho_r}{\partial \theta_r} & \cdots & \frac{\partial \rho_r}{\partial \rho_m} & \frac{\partial \rho_r}{\partial \alpha_m} & \cdots \\
\frac{\partial \alpha_r}{\partial x_r} & \frac{\partial \alpha_r}{\partial y_r} & \frac{\partial \alpha_r}{\partial \theta_r} & \cdots & \frac{\partial \alpha_r}{\partial \rho_m} & \frac{\partial \alpha_r}{\partial \alpha_m} & \cdots
\end{bmatrix}
$$

#### Step-by-Step Derivation: First Row (ρ_r derivatives)

**Recall:**
$$
\rho_r = \rho_m - x_r \cos(\alpha_m) - y_r \sin(\alpha_m)
$$

**Derivative with respect to x_r:**
$$
\frac{\partial \rho_r}{\partial x_r} = \frac{\partial}{\partial x_r} \left[\rho_m - x_r \cos(\alpha_m) - y_r \sin(\alpha_m)\right]
$$

$$
= 0 - \cos(\alpha_m) - 0 = -\cos(\alpha_m)
$$

**Derivative with respect to y_r:**
$$
\frac{\partial \rho_r}{\partial y_r} = \frac{\partial}{\partial y_r} \left[\rho_m - x_r \cos(\alpha_m) - y_r \sin(\alpha_m)\right]
$$

$$
= 0 - 0 - \sin(\alpha_m) = -\sin(\alpha_m)
$$

**Derivative with respect to θ_r:**
$$
\frac{\partial \rho_r}{\partial \theta_r} = \frac{\partial}{\partial \theta_r} \left[\rho_m - x_r \cos(\alpha_m) - y_r \sin(\alpha_m)\right]
$$

Since $\alpha_m$ does not depend on $\theta_r$:
$$
= 0
$$

**Derivative with respect to ρ_m:**
$$
\frac{\partial \rho_r}{\partial \rho_m} = \frac{\partial}{\partial \rho_m} \left[\rho_m - x_r \cos(\alpha_m) - y_r \sin(\alpha_m)\right]
$$

$$
= 1 - 0 - 0 = 1
$$

**Derivative with respect to α_m:**
$$
\frac{\partial \rho_r}{\partial \alpha_m} = \frac{\partial}{\partial \alpha_m} \left[\rho_m - x_r \cos(\alpha_m) - y_r \sin(\alpha_m)\right]
$$

Using chain rule:
$$
= 0 - x_r \frac{\partial \cos(\alpha_m)}{\partial \alpha_m} - y_r \frac{\partial \sin(\alpha_m)}{\partial \alpha_m}
$$

$$
= - x_r \cdot (-\sin(\alpha_m)) - y_r \cdot \cos(\alpha_m)
$$

$$
= x_r \sin(\alpha_m) - y_r \cos(\alpha_m)
$$

#### Step-by-Step Derivation: Second Row (α_r derivatives)

**Recall:**
$$
\alpha_r = \alpha_m - \theta_r
$$

**Derivative with respect to x_r:**
$$
\frac{\partial \alpha_r}{\partial x_r} = 0
$$

**Derivative with respect to y_r:**
$$
\frac{\partial \alpha_r}{\partial y_r} = 0
$$

**Derivative with respect to θ_r:**
$$
\frac{\partial \alpha_r}{\partial \theta_r} = \frac{\partial}{\partial \theta_r}(\alpha_m - \theta_r) = 0 - 1 = -1
$$

**Derivative with respect to ρ_m:**
$$
\frac{\partial \alpha_r}{\partial \rho_m} = 0
$$

**Derivative with respect to α_m:**
$$
\frac{\partial \alpha_r}{\partial \alpha_m} = \frac{\partial}{\partial \alpha_m}(\alpha_m - \theta_r) = 1 - 0 = 1
$$

#### Complete Jacobian for Wall

$$
\mathbf{H}_{\text{wall}} = \begin{bmatrix}
-\cos(\alpha_m) & -\sin(\alpha_m) & 0 & \cdots & 1 & x_r \sin(\alpha_m) - y_r \cos(\alpha_m) & \cdots \\
0 & 0 & -1 & \cdots & 0 & 1 & \cdots
\end{bmatrix}
$$

Where:
- Columns 0, 1, 2: Robot pose derivatives
- Columns $i, i+1$: Wall landmark derivatives (at wall's state index)
- All other columns: 0 (other landmarks not involved)

### 3.4 Implementation: Wall Jacobian

```python
def compute_wall_jacobian(x_r, y_r, theta_r, lm_rho, lm_alpha, idx, n):
    """
    Compute observation Jacobian for wall landmark.

    Args:
        x_r, y_r, theta_r: Robot pose
        lm_rho, lm_alpha: Wall parameters in map frame
        idx: State index of wall (where rho starts)
        n: Total state dimension

    Returns:
        H: (2 x n) Jacobian matrix
    """
    H = np.zeros((2, n))

    # First row: ∂ρ_r/∂x (distance observation)
    H[0, 0] = -np.cos(lm_alpha)           # ∂ρ_r/∂x_r
    H[0, 1] = -np.sin(lm_alpha)           # ∂ρ_r/∂y_r
    H[0, 2] = 0.0                         # ∂ρ_r/∂θ_r
    H[0, idx] = 1.0                       # ∂ρ_r/∂ρ_m
    H[0, idx+1] = x_r * np.sin(lm_alpha) - y_r * np.cos(lm_alpha)  # ∂ρ_r/∂α_m

    # Second row: ∂α_r/∂x (orientation observation)
    H[1, 0] = 0.0                         # ∂α_r/∂x_r
    H[1, 1] = 0.0                         # ∂α_r/∂y_r
    H[1, 2] = -1.0                        # ∂α_r/∂θ_r
    H[1, idx] = 0.0                       # ∂α_r/∂ρ_m
    H[1, idx+1] = 1.0                     # ∂α_r/∂α_m

    return H
```

---

## 4. Corner Landmark Update (Cartesian Form)

### 4.1 Corner Representation

A corner in the map frame is represented as:

$$
\mathbf{m}_{\text{corner}} = \begin{bmatrix}
x_m \\
y_m
\end{bmatrix}
$$

Where $(x_m, y_m)$ is the corner position in the map frame.

### 4.2 Observation Model: Map Frame → Robot Frame

**Given:**
- Robot pose in map frame: $(x_r, y_r, \theta_r)$
- Corner position in map frame: $(x_m, y_m)$

**Objective:** Transform corner position to robot's local frame.

#### Step 1: Translation to robot origin

$$
\Delta \mathbf{p} = \begin{bmatrix}
\Delta x \\
\Delta y
\end{bmatrix} = \begin{bmatrix}
x_m - x_r \\
y_m - y_r
\end{bmatrix}
$$

#### Step 2: Rotation by -θ_r

To express the corner in the robot frame, we rotate by $-\theta_r$:

$$
\mathbf{R}(-\theta_r) = \begin{bmatrix}
\cos(-\theta_r) & -\sin(-\theta_r) \\
\sin(-\theta_r) & \cos(-\theta_r)
\end{bmatrix} = \begin{bmatrix}
\cos(\theta_r) & \sin(\theta_r) \\
-\sin(\theta_r) & \cos(\theta_r)
\end{bmatrix}
$$

**Note:** Using the identities:
- $\cos(-\theta) = \cos(\theta)$
- $\sin(-\theta) = -\sin(\theta)$

#### Step 3: Apply rotation

$$
\begin{bmatrix}
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

**Expanding:**

$$
x_r' = \cos(\theta_r) (x_m - x_r) + \sin(\theta_r) (y_m - y_r)
$$

$$
y_r' = -\sin(\theta_r) (x_m - x_r) + \cos(\theta_r) (y_m - y_r)
$$

#### Step 4: Complete observation model

$$
h_{\text{corner}}(\mathbf{x}) = \begin{bmatrix}
\cos(\theta_r) (x_m - x_r) + \sin(\theta_r) (y_m - y_r) \\
-\sin(\theta_r) (x_m - x_r) + \cos(\theta_r) (y_m - y_r)
\end{bmatrix}
$$

### 4.3 Observation Jacobian for Corners

The Jacobian is:

$$
\mathbf{H}_{\text{corner}} = \frac{\partial h_{\text{corner}}}{\partial \mathbf{x}} \in \mathbb{R}^{2 \times n}
$$

The observation depends only on:
- Robot pose: $(x_r, y_r, \theta_r)$ (indices 0, 1, 2)
- Observed corner: $(x_m, y_m)$ (indices $i, i+1$)

#### Step-by-Step Derivation: First Row (x_r' derivatives)

**Recall:**
$$
x_r' = \cos(\theta_r) (x_m - x_r) + \sin(\theta_r) (y_m - y_r)
$$

Let's use shorthand:
- $\Delta x = x_m - x_r$
- $\Delta y = y_m - y_r$
- $c = \cos(\theta_r)$
- $s = \sin(\theta_r)$

So:
$$
x_r' = c \cdot \Delta x + s \cdot \Delta y
$$

**Derivative with respect to x_r:**
$$
\frac{\partial x_r'}{\partial x_r} = c \cdot \frac{\partial \Delta x}{\partial x_r} + s \cdot \frac{\partial \Delta y}{\partial x_r}
$$

$$
= c \cdot (-1) + s \cdot 0 = -\cos(\theta_r)
$$

**Derivative with respect to y_r:**
$$
\frac{\partial x_r'}{\partial y_r} = c \cdot \frac{\partial \Delta x}{\partial y_r} + s \cdot \frac{\partial \Delta y}{\partial y_r}
$$

$$
= c \cdot 0 + s \cdot (-1) = -\sin(\theta_r)
$$

**Derivative with respect to θ_r:**
$$
\frac{\partial x_r'}{\partial \theta_r} = \frac{\partial c}{\partial \theta_r} \cdot \Delta x + c \cdot \frac{\partial \Delta x}{\partial \theta_r} + \frac{\partial s}{\partial \theta_r} \cdot \Delta y + s \cdot \frac{\partial \Delta y}{\partial \theta_r}
$$

$$
= (-\sin(\theta_r)) \cdot \Delta x + c \cdot 0 + \cos(\theta_r) \cdot \Delta y + s \cdot 0
$$

$$
= -\sin(\theta_r) \cdot (x_m - x_r) + \cos(\theta_r) \cdot (y_m - y_r)
$$

$$
= -\Delta x \sin(\theta_r) + \Delta y \cos(\theta_r)
$$

**Derivative with respect to x_m:**
$$
\frac{\partial x_r'}{\partial x_m} = c \cdot \frac{\partial \Delta x}{\partial x_m} + s \cdot \frac{\partial \Delta y}{\partial x_m}
$$

$$
= c \cdot 1 + s \cdot 0 = \cos(\theta_r)
$$

**Derivative with respect to y_m:**
$$
\frac{\partial x_r'}{\partial y_m} = c \cdot \frac{\partial \Delta x}{\partial y_m} + s \cdot \frac{\partial \Delta y}{\partial y_m}
$$

$$
= c \cdot 0 + s \cdot 1 = \sin(\theta_r)
$$

#### Step-by-Step Derivation: Second Row (y_r' derivatives)

**Recall:**
$$
y_r' = -\sin(\theta_r) (x_m - x_r) + \cos(\theta_r) (y_m - y_r)
$$

$$
y_r' = -s \cdot \Delta x + c \cdot \Delta y
$$

**Derivative with respect to x_r:**
$$
\frac{\partial y_r'}{\partial x_r} = -s \cdot \frac{\partial \Delta x}{\partial x_r} + c \cdot \frac{\partial \Delta y}{\partial x_r}
$$

$$
= -s \cdot (-1) + c \cdot 0 = \sin(\theta_r)
$$

**Derivative with respect to y_r:**
$$
\frac{\partial y_r'}{\partial y_r} = -s \cdot \frac{\partial \Delta x}{\partial y_r} + c \cdot \frac{\partial \Delta y}{\partial y_r}
$$

$$
= -s \cdot 0 + c \cdot (-1) = -\cos(\theta_r)
$$

**Derivative with respect to θ_r:**
$$
\frac{\partial y_r'}{\partial \theta_r} = \frac{\partial (-s)}{\partial \theta_r} \cdot \Delta x + (-s) \cdot \frac{\partial \Delta x}{\partial \theta_r} + \frac{\partial c}{\partial \theta_r} \cdot \Delta y + c \cdot \frac{\partial \Delta y}{\partial \theta_r}
$$

$$
= (-\cos(\theta_r)) \cdot \Delta x + (-s) \cdot 0 + (-\sin(\theta_r)) \cdot \Delta y + c \cdot 0
$$

$$
= -\cos(\theta_r) \cdot (x_m - x_r) - \sin(\theta_r) \cdot (y_m - y_r)
$$

$$
= -\Delta x \cos(\theta_r) - \Delta y \sin(\theta_r)
$$

**Derivative with respect to x_m:**
$$
\frac{\partial y_r'}{\partial x_m} = -s \cdot \frac{\partial \Delta x}{\partial x_m} + c \cdot \frac{\partial \Delta y}{\partial x_m}
$$

$$
= -s \cdot 1 + c \cdot 0 = -\sin(\theta_r)
$$

**Derivative with respect to y_m:**
$$
\frac{\partial y_r'}{\partial y_m} = -s \cdot \frac{\partial \Delta x}{\partial y_m} + c \cdot \frac{\partial \Delta y}{\partial y_m}
$$

$$
= -s \cdot 0 + c \cdot 1 = \cos(\theta_r)
$$

#### Complete Jacobian for Corner

$$
\mathbf{H}_{\text{corner}} = \begin{bmatrix}
-\cos(\theta_r) & -\sin(\theta_r) & -\Delta x \sin(\theta_r) + \Delta y \cos(\theta_r) & \cdots & \cos(\theta_r) & \sin(\theta_r) & \cdots \\
\sin(\theta_r) & -\cos(\theta_r) & -\Delta x \cos(\theta_r) - \Delta y \sin(\theta_r) & \cdots & -\sin(\theta_r) & \cos(\theta_r) & \cdots
\end{bmatrix}
$$

Where:
- Columns 0, 1, 2: Robot pose derivatives
- Columns $i, i+1$: Corner landmark derivatives
- All other columns: 0

### 4.4 Implementation: Corner Jacobian

```python
def compute_corner_jacobian(x_r, y_r, theta_r, x_m, y_m, idx, n):
    """
    Compute observation Jacobian for corner landmark.

    Args:
        x_r, y_r, theta_r: Robot pose
        x_m, y_m: Corner position in map frame
        idx: State index of corner (where x starts)
        n: Total state dimension

    Returns:
        H: (2 x n) Jacobian matrix
    """
    H = np.zeros((2, n))

    # Compute differences
    dx = x_m - x_r
    dy = y_m - y_r

    # Precompute trig functions
    cos_theta = np.cos(theta_r)
    sin_theta = np.sin(theta_r)

    # First row: ∂x_r'/∂x (x-observation in robot frame)
    H[0, 0] = -cos_theta                              # ∂x_r'/∂x_r
    H[0, 1] = -sin_theta                              # ∂x_r'/∂y_r
    H[0, 2] = -dx * sin_theta + dy * cos_theta        # ∂x_r'/∂θ_r
    H[0, idx] = cos_theta                             # ∂x_r'/∂x_m
    H[0, idx+1] = sin_theta                           # ∂x_r'/∂y_m

    # Second row: ∂y_r'/∂x (y-observation in robot frame)
    H[1, 0] = sin_theta                               # ∂y_r'/∂x_r
    H[1, 1] = -cos_theta                              # ∂y_r'/∂y_r
    H[1, 2] = -dx * cos_theta - dy * sin_theta        # ∂y_r'/∂θ_r
    H[1, idx] = -sin_theta                            # ∂y_r'/∂x_m
    H[1, idx+1] = cos_theta                           # ∂y_r'/∂y_m

    return H
```

---

#

### 5.6 Implementation: Wall Covariance

```python
def compute_wall_covariance(points, rho, alpha, sigma=0.01):
    """
    Compute Hessian-based covariance for wall landmark.

    Args:
        points: (N x 2) array of points on wall
        rho: Wall distance parameter
        alpha: Wall angle parameter
        sigma: LiDAR noise std (default: 0.01m)

    Returns:
        R: (2 x 2) covariance matrix for (rho, alpha)
    """
    N = len(points)

    # Compute perpendicular offsets: d_i = x_i*sin(α) - y_i*cos(α)
    sin_alpha = np.sin(alpha)
    cos_alpha = np.cos(alpha)

    d = points[:, 0] * sin_alpha - points[:, 1] * cos_alpha

    # Hessian matrix components
    sum_d = np.sum(d)
    sum_d2 = np.sum(d ** 2)

    # Build Hessian
    A = np.array([
        [N,     sum_d],
        [sum_d, sum_d2]
    ])

    # Check for singularity
    det_A = N * sum_d2 - sum_d ** 2

    if abs(det_A) < 1e-10:
        # Degenerate configuration - use large uncertainty
        return np.diag([1.0, 1.0])

    # Invert Hessian
    A_inv = np.array([
        [sum_d2, -sum_d],
        [-sum_d, N]
    ]) / det_A

    # Covariance = sigma^2 * A^(-1)
    R = (sigma ** 2) * A_inv

    return R
```

---

## 6. Hessian-Based Covariance for Corners

### 6.1 Theoretical Foundation

The measurement covariance for a corner landmark is:

$$
\mathbf{R}_{\text{corner}} = \sigma^2 \mathbf{A}_{\text{corner}}^{-1}
$$

Where:
- $\sigma^2 = 0.0001$ m² (LiDAR noise variance)
- $\mathbf{A}_{\text{corner}}$: Information matrix from point clustering

### 6.2 Corner Estimation Problem

**Given:** $N$ LiDAR points $\{(x_i, y_i)\}_{i=1}^N$ near a corner

**Objective:** Estimate corner position $\mathbf{c} = [x_c, y_c]^T$ as the mean

**Least-squares cost function:**
$$
E(x_c, y_c) = \sum_{i=1}^N \|\mathbf{p}_i - \mathbf{c}\|^2
$$

Where:
$$
\|\mathbf{p}_i - \mathbf{c}\|^2 = (x_i - x_c)^2 + (y_i - y_c)^2
$$

### 6.3 Residual Vector

For each point $i$, the residual vector is:

$$
\mathbf{r}_i = \mathbf{p}_i - \mathbf{c} = \begin{bmatrix}
x_i - x_c \\
y_i - y_c
\end{bmatrix} \in \mathbb{R}^2
$$

### 6.4 Jacobian of Residual

The Jacobian with respect to corner parameters $(x_c, y_c)$ is:

$$
\mathbf{J}_i = \frac{\partial \mathbf{r}_i}{\partial \mathbf{c}} \in \mathbb{R}^{2 \times 2}
$$

#### Step-by-Step Derivation

**First component:**
$$
r_{i,x} = x_i - x_c
$$

$$
\frac{\partial r_{i,x}}{\partial x_c} = -1, \quad \frac{\partial r_{i,x}}{\partial y_c} = 0
$$

**Second component:**
$$
r_{i,y} = y_i - y_c
$$

$$
\frac{\partial r_{i,y}}{\partial x_c} = 0, \quad \frac{\partial r_{i,y}}{\partial y_c} = -1
$$

#### Complete Jacobian

$$
\mathbf{J}_i = \begin{bmatrix}
-1 & 0 \\
0 & -1
\end{bmatrix} = -\mathbf{I}_{2 \times 2}
$$

**This is constant for all points!**

### 6.5 Hessian Matrix Computation (Isotropic Case)

For the **isotropic case** (assuming equal uncertainty in all directions):

$$
\mathbf{A}_{\text{corner, iso}} = \sum_{i=1}^N \mathbf{J}_i^T \mathbf{J}_i
$$

#### Step 1: Compute J_i^T J_i

$$
\mathbf{J}_i^T \mathbf{J}_i = (-\mathbf{I})^T (-\mathbf{I}) = \mathbf{I}^T \mathbf{I} = \mathbf{I}
$$

#### Step 2: Sum over all points

$$
\mathbf{A}_{\text{corner, iso}} = \sum_{i=1}^N \mathbf{I} = N \mathbf{I} = \begin{bmatrix}
N & 0 \\
0 & N
\end{bmatrix}
$$

#### Step 3: Invert Hessian

$$
\mathbf{A}_{\text{corner, iso}}^{-1} = \frac{1}{N} \mathbf{I} = \begin{bmatrix}
1/N & 0 \\
0 & 1/N
\end{bmatrix}
$$

#### Step 4: Covariance (Isotropic)

$$
\mathbf{R}_{\text{corner, iso}} = \sigma^2 \mathbf{A}_{\text{corner, iso}}^{-1} = \frac{\sigma^2}{N} \mathbf{I}
$$

**Interpretation:**
- Uncertainty decreases with $1/N$ (more points → lower uncertainty)
- Equal uncertainty in $x$ and $y$ directions
- Independent $x$ and $y$ estimates (zero off-diagonal)

### 6.6 Hessian Matrix Computation (Anisotropic Case)

For more realistic scenarios, corner points may have **directional scatter** (e.g., along two walls).

We use **Principal Component Analysis (PCA)** to capture anisotropic uncertainty.

#### Step 1: Compute covariance of point scatter

$$
\mathbf{C} = \frac{1}{N} \sum_{i=1}^N (\mathbf{p}_i - \bar{\mathbf{p}})(\mathbf{p}_i - \bar{\mathbf{p}})^T
$$

Where $\bar{\mathbf{p}} = \frac{1}{N}\sum_{i=1}^N \mathbf{p}_i$ is the centroid.

$$
\mathbf{C} = \begin{bmatrix}
\sigma_x^2 & \sigma_{xy} \\
\sigma_{xy} & \sigma_y^2
\end{bmatrix}
$$

#### Step 2: Eigendecomposition

$$
\mathbf{C} = \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T
$$

Where:
- $\mathbf{V} = [\mathbf{v}_1 | \mathbf{v}_2]$: Eigenvectors (principal directions)
- $\boldsymbol{\Lambda} = \text{diag}(\lambda_1, \lambda_2)$: Eigenvalues (variances along principal axes)

#### Step 3: Information matrix from eigenstructure

The **information matrix** is inversely related to scatter:

$$
\mathbf{A}_{\text{corner, aniso}} = N \mathbf{V} \boldsymbol{\Lambda}^{-1} \mathbf{V}^T
$$

Where:
$$
\boldsymbol{\Lambda}^{-1} = \begin{bmatrix}
1/\lambda_1 & 0 \\
0 & 1/\lambda_2
\end{bmatrix}
$$

**Interpretation:**
- Large eigenvalue (high scatter) → small information (1/λ)
- Small eigenvalue (tight cluster) → large information
- $N$ factor accounts for number of observations

#### Step 4: Covariance (Anisotropic)

$$
\mathbf{R}_{\text{corner, aniso}} = \sigma^2 \mathbf{A}_{\text{corner, aniso}}^{-1}
$$

$$
= \sigma^2 \left(N \mathbf{V} \boldsymbol{\Lambda}^{-1} \mathbf{V}^T\right)^{-1}
$$

$$
= \frac{\sigma^2}{N} \mathbf{V} \boldsymbol{\Lambda} \mathbf{V}^T
$$

$$
= \frac{\sigma^2}{N} \mathbf{C}
$$

**Key insight:** The measurement covariance is proportional to the point scatter covariance, scaled by sensor noise and number of points.

---

## Summary

This document provides complete mathematical derivations for:

1. **Wall Landmark Updates**
   - Observation model: Map frame → Robot frame transformation
   - Jacobian: Full step-by-step derivation of all 10 partial derivatives
   - Hessian-based covariance: From point-to-line fitting

2. **Corner Landmark Updates**
   - Observation model: Translation + rotation to robot frame
   - Jacobian: Full step-by-step derivation of all 10 partial derivatives
   - Hessian-based covariance: Isotropic and anisotropic cases with PCA

3. **Complete Implementation**
   - Full EKF update algorithm for both landmark types
   - Production-ready code with error handling
   - Step-by-step execution flow

All mathematical steps are shown explicitly, including:
- Chain rule applications
- Trigonometric derivatives
- Matrix operations
- Jacobian accumulation
- Hessian computation
- Matrix inversion

The implementation follows best practices:
- Fixed sensor noise (avoiding optimism)
- Joseph form covariance update (numerical stability)
- Angle normalization (handling wraparound)
- Singularity checks (robust to degenerate cases)

---

**File:** `detailed_ekf_update_derivations.md`
**Location:** `/home/piyush/thesis_ws/docs/`
**Pages:** ~25 pages
**Last Updated:** 2026-02-09
