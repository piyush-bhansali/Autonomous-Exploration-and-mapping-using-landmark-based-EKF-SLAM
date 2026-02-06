# Statistical SLAM: Complete Mathematical Formulation

**Author**: Technical Documentation for Landmark-Based EKF-SLAM
**Date**: February 2026
**Purpose**: Comprehensive mathematical explanation of statistical mapping with landmarks

---

## Table of Contents

1. [Introduction and Motivation](#1-introduction-and-motivation)
2. [State Representation](#2-state-representation)
3. [Uncertainty Representation](#3-uncertainty-representation)
4. [Sensor Noise Modeling](#4-sensor-noise-modeling)
5. [Coordinate Transformation and Error Propagation](#5-coordinate-transformation-and-error-propagation)
6. [EKF Prediction Step](#6-ekf-prediction-step)
7. [EKF Update Step](#7-ekf-update-step)
8. [Landmark Initialization](#8-landmark-initialization)
9. [Landmark Re-observation](#9-landmark-re-observation)
10. [Data Association](#10-data-association)
11. [Why Robot and Landmarks Are Interconnected](#11-why-robot-and-landmarks-are-interconnected)
12. [Numerical Example](#12-numerical-example)
13. [Implementation Notes](#13-implementation-notes)
14. [Mixed Representation: Lines and Corners](#14-mixed-representation-lines-and-corners)

---

## 1. Introduction and Motivation

### 1.1 The Fundamental Problem

In robotics, we face two simultaneous uncertainties:
- **Where is the robot?** (Localization)
- **Where are the landmarks?** (Mapping)

These uncertainties are **coupled**:
- If we don't know where the robot is, we can't accurately place landmarks
- If we don't know where landmarks are, we can't accurately localize the robot

### 1.2 Why Statistical Modeling?

Every sensor measurement has noise. Instead of treating measurements as exact values, we model them as **probability distributions** (typically Gaussian).

**Example**: A LiDAR measures a landmark at range r = 3.00m. But due to:
- Range error: σ_r = 2cm
- Angular error: σ_θ = 0.5°

The true position could be anywhere in an **uncertainty ellipse** around the measured position.

### 1.3 The Key Insight

When we observe a landmark multiple times:
1. **Landmark uncertainty decreases** (more measurements = better estimate)
2. **Robot uncertainty decreases** (landmarks act as reference points)
3. These happen **simultaneously** because they share correlations

---

## 2. State Representation

### 2.1 State Vector

In landmark-based SLAM, the state vector contains:
1. Robot pose (3 DOF in 2D)
2. Landmark parameters (2 DOF each)

```
x = [x_r, y_r, θ_r, p₁¹, p₁², p₂¹, p₂², ..., pₙ¹, pₙ²]ᵀ
```

Where:
- `x_r, y_r, θ_r` = robot position and orientation in global frame
- `pᵢ¹, pᵢ²` = parameters of landmark i in global frame

**Dimension**: n_state = 3 + 2×n_landmarks

### 2.2 Mixed Representation for Different Landmark Types

Different landmark types use different optimal parameterizations:

**For Line Landmarks** (walls):
- Parameters: `(ρ, α)` in **Hessian Normal Form**
  - `ρ` = perpendicular distance from origin to line
  - `α` = angle of normal vector (perpendicular to line)
- **View-invariant**: Same (ρ, α) regardless of observation distance
- Example: A wall 3m from origin with normal pointing east → ρ=3.0, α=0°

**For Corner Landmarks** (point features):
- Parameters: `(x, y)` in **Cartesian coordinates**
  - `x, y` = position in global frame
- **Natural representation** for point features
- Example: A corner at (5.0, 2.3) in map frame

**State vector example** with 2 lines and 2 corners:
```
x = [x_r, y_r, θ_r, ρ₁, α₁, ρ₂, α₂, x₃, y₃, x₄, y₄]ᵀ
     └─robot─┘  └line1┘ └line2┘ └corner3┘ └corner4┘
```

### 2.3 Why Hessian Normal Form for Lines?

**The Problem with Centroid Representation**:
Consider a wall observed from different distances:

```
View from far (5m):    View from close (2m):
Robot sees 3m section  Robot sees 1m section
Centroid at (5.0, 1.5) Centroid at (5.3, 1.2)
                       ↓
Same wall, different centroids → wrongly treated as different landmarks!
```

**Solution: Hessian Normal Form**

The Hessian parameters (ρ, α) are **view-invariant**:

```
Line equation: x·cos(α) + y·sin(α) = ρ

For a wall at 3m perpendicular distance with normal pointing east:
  ρ = 3.0m
  α = 0° (0 radians)

Observed from ANY position → same (ρ, α)!
```

**Mathematical Definition**:
- **ρ**: Signed perpendicular distance from origin to line
  - `ρ = n · p` where n is unit normal, p is any point on line
- **α**: Angle of normal vector
  - `α = atan2(n_y, n_x)` where n = [n_x, n_y] is unit normal

**Geometric Interpretation**:
```
        ↑ y
        │
        │     Wall
        │     ║
        │     ║
    ────┼─────╫────→ x
        │   ↗ ║
        │  ρ  ║
        │ ↙   ║
       O│α    ║

O = origin
ρ = perpendicular distance from O to wall
α = angle of normal vector (arrow perpendicular to wall)
```

### 2.4 Why Global Frame?

Landmarks are stored in the **global (map) frame** because:
- They are static (don't move with the robot)
- Once initialized, their global parameters improve over time
- Makes data association easier (consistent reference frame)
- For lines: (ρ, α) is measured from global origin
- For corners: (x, y) is in global coordinates

---

## 3. Uncertainty Representation

### 3.1 Covariance Matrix

Uncertainty is represented by a covariance matrix **P**:

```
P = E[(x - x̂)(x - x̂)ᵀ]
```

**Dimension**: (n_state × n_state)

### 3.2 Structure of P

```
P = ┌─────────────────────────────────────┐
    │  P_rr  │  P_rl₁  │  P_rl₂  │  ...   │  Robot-Robot
    ├────────┼─────────┼─────────┼────────┤  Robot-Landmark₁
    │  P_l₁r │  P_l₁l₁ │  P_l₁l₂ │  ...   │  Landmark₁-Landmark₁
    ├────────┼─────────┼─────────┼────────┤  etc.
    │  P_l₂r │  P_l₂l₁ │  P_l₂l₂ │  ...   │
    └─────────────────────────────────────┘
```

**Key blocks**:

1. **P_rr (3×3)**: Robot pose uncertainty
   ```
   P_rr = ┌──────────────────┐
          │  σ²_x   σ_xy  σ_xθ │
          │  σ_xy   σ²_y  σ_yθ │
          │  σ_xθ   σ_yθ  σ²_θ │
          └──────────────────┘
   ```

2. **P_lᵢlᵢ (2×2)**: Landmark i parameter uncertainty
   ```
   For LINE landmarks (ρ, α):
   P_lᵢlᵢ = ┌──────────┐
            │  σ²_ρ   σ_ρα  │
            │  σ_ρα   σ²_α  │
            └──────────┘

   For CORNER landmarks (x, y):
   P_lᵢlᵢ = ┌──────────┐
            │  σ²_x   σ_xy  │
            │  σ_xy   σ²_y  │
            └──────────┘
   ```

3. **P_rlᵢ (3×2)**: Robot-Landmark i **cross-correlation**
   - This is the coupling between robot and landmark uncertainty
   - **NON-ZERO** when landmark was observed by robot
   - Allows mutual refinement during updates
   - For lines: couples (x_r, y_r, θ_r) with (ρ, α)
   - For corners: couples (x_r, y_r, θ_r) with (x, y)

4. **P_lᵢlⱼ (2×2)**: Landmark i - Landmark j cross-correlation
   - Non-zero because landmarks share robot pose uncertainty
   - Weak coupling (smaller magnitudes than P_rlᵢ)
   - Can couple different representations (e.g., line ρ with corner x)

### 3.3 Physical Interpretation

**Diagonal elements** (variances):
- `P[0,0] = σ²_x`: Uncertainty in robot x-position
- `P[1,1] = σ²_y`: Uncertainty in robot y-position
- `P[2,2] = σ²_θ`: Uncertainty in robot orientation
- For line landmarks: `σ²_ρ` (distance uncertainty), `σ²_α` (angular uncertainty)
- For corner landmarks: `σ²_x` (x uncertainty), `σ²_y` (y uncertainty)

**Off-diagonal elements** (covariances):
- Positive: Variables tend to increase together
- Negative: Variables tend to oppose each other
- Zero: Variables are independent (uncorrelated)

**Example**: If `P[0,2] > 0` (positive x-θ covariance), then:
- Overestimating x → likely overestimated θ
- Correcting one → automatically adjusts the other

---

## 4. Sensor Noise Modeling

### 4.1 LiDAR Sensor Model

A LiDAR measures landmarks in **polar coordinates**:
- **Range**: r (distance from robot to landmark)
- **Bearing**: θ (angle from robot's forward axis)

### 4.2 Sensor Noise Parameters

Real LiDARs have two independent noise sources:

1. **Range error**: σ_r (typically 1-3 cm)
   - Gaussian noise: `r_measured ~ N(r_true, σ²_r)`

2. **Angular error**: σ_θ (typically 0.1-1.0°)
   - Gaussian noise: `θ_measured ~ N(θ_true, σ²_θ)`

**Polar covariance matrix**:
```
C_polar = ┌──────────┐
          │  σ²_r    0  │  (diagonal because r and θ are independent)
          │   0    σ²_θ │
          └──────────┘
```

### 4.3 Why Not Directly Use Polar Coordinates?

**Problem**: SLAM state is in Cartesian coordinates (x, y), but sensors measure in polar (r, θ).

**Solution**: Transform measurements to Cartesian, then propagate uncertainty through the transformation.

---

## 5. Coordinate Transformation and Error Propagation

### 5.1 Polar to Cartesian Transformation

**Forward transformation**:
```
x = r cos(θ)
y = r sin(θ)
```

**Question**: If we know `C_polar` (uncertainty in r, θ), what is `C_cartesian` (uncertainty in x, y)?

### 5.2 First-Order Error Propagation (Jacobian Method)

For a nonlinear transformation `y = f(x)`, uncertainty propagates as:

```
C_y ≈ J · C_x · Jᵀ
```

Where **J** is the **Jacobian matrix**:
```
J = ∂f/∂x
```

### 5.3 Deriving the Jacobian for Polar → Cartesian

**Functions**:
```
x = r cos(θ)
y = r sin(θ)
```

**Partial derivatives**:
```
∂x/∂r = cos(θ)
∂x/∂θ = -r sin(θ)
∂y/∂r = sin(θ)
∂y/∂θ = r cos(θ)
```

**Jacobian matrix**:
```
J = ┌─────────────────┐
    │  cos(θ)  -r·sin(θ) │
    │  sin(θ)   r·cos(θ) │
    └─────────────────┘
```

### 5.4 Complete Covariance Transformation

```
C_cartesian = J · C_polar · Jᵀ
```

**Expanding**:
```
C_xy = ┌─────────────────┐   ┌──────────┐   ┌─────────────────┐ᵀ
       │  cos(θ)  -r·sin(θ) │ · │  σ²_r    0  │ · │  cos(θ)  -r·sin(θ) │
       │  sin(θ)   r·cos(θ) │   │   0    σ²_θ │   │  sin(θ)   r·cos(θ) │
       └─────────────────┘   └──────────┘   └─────────────────┘
```

**Result** (after matrix multiplication):
```
C_xy = ┌───────────────────────────────────────────────────────┐
       │  σ²_r·cos²(θ) + r²·σ²_θ·sin²(θ)    (σ²_r - r²·σ²_θ)·sin(θ)·cos(θ) │
       │  (σ²_r - r²·σ²_θ)·sin(θ)·cos(θ)    σ²_r·sin²(θ) + r²·σ²_θ·cos²(θ) │
       └───────────────────────────────────────────────────────┘
```

### 5.5 Physical Interpretation

**Variance in x-direction**:
```
σ²_x = σ²_r·cos²(θ) + r²·σ²_θ·sin²(θ)
       └──────────┘   └──────────────┘
       range error    angular error scaled by range
```

**Key insight**:
- At short range (r small): range error dominates
- At long range (r large): angular error dominates (×r² scaling!)
- Orientation matters: uncertainty ellipse rotates with bearing angle

**Example**:
- σ_r = 2cm, σ_θ = 0.5° = 0.0087 rad
- At r = 1m: σ_x ≈ 2cm (range error dominates)
- At r = 5m: σ_x ≈ 4.4cm (angular error significant: 5m × 0.0087 rad ≈ 4.3cm)

### 5.6 Uncertainty Ellipse

The covariance matrix `C_xy` defines an **uncertainty ellipse**:

**Eigenvalue decomposition**:
```
C_xy = V · Λ · Vᵀ
```

Where:
- **Λ** = diagonal matrix of eigenvalues (λ₁, λ₂)
- **V** = eigenvectors (ellipse axes directions)

**Ellipse parameters**:
- Major axis length: `2√λ₁` (95% confidence: `2√(5.99·λ₁)`)
- Minor axis length: `2√λ₂`
- Rotation angle: `atan2(V[1,0], V[0,0])`

---

## 6. EKF Prediction Step

### 6.1 Robot Motion Model

The robot moves based on odometry measurements:
- **Δd**: Distance traveled (from wheel encoders)
- **Δθ**: Rotation angle (from wheel encoders or IMU)

**Motion model** (2D kinematic):
```
x_{k+1} = x_k + Δd · cos(θ_k + Δθ/2)
y_{k+1} = y_k + Δd · sin(θ_k + Δθ/2)
θ_{k+1} = θ_k + Δθ
```

**Note**: We use `θ + Δθ/2` (mid-point integration) for better accuracy when the robot turns while moving.

### 6.2 State Prediction

**Landmarks don't move**, so only robot pose changes:

```
x⁻ = f(x, u)

where:
┌──────┐   ┌─────────────────────────────────┐
│  x_r │   │  x_r + Δd·cos(θ_r + Δθ/2)       │
│  y_r │   │  y_r + Δd·sin(θ_r + Δθ/2)       │
│  θ_r │ = │  θ_r + Δθ                        │
│  l₁ˣ │   │  l₁ˣ  (unchanged)                │
│  l₁ʸ │   │  l₁ʸ  (unchanged)                │
│  ... │   │  ...                             │
└──────┘   └─────────────────────────────────┘
```

### 6.3 Jacobian of Motion Model

**State transition Jacobian** F_x = ∂f/∂x:

```
F_x = ┌────────────────────────────────────────┐
      │  1    0   -Δd·sin(θ + Δθ/2)  │  0  0  ... │  3×3 robot block
      │  0    1    Δd·cos(θ + Δθ/2)  │  0  0  ... │  (affects only robot)
      │  0    0            1           │  0  0  ... │
      ├─────────────────────────────────────────┤
      │  0    0            0           │  I₂   ... │  Identity for landmarks
      │  .    .            .           │   .   ... │  (they don't move)
      └────────────────────────────────────────┘
```

**Control Jacobian** F_u = ∂f/∂u (with respect to motion):

```
F_u = ┌──────────────────────────────────┐
      │  cos(θ + Δθ/2)  -Δd·sin(θ + Δθ/2)/2 │
      │  sin(θ + Δθ/2)   Δd·cos(θ + Δθ/2)/2 │
      │       0                   1           │
      │       0                   0           │  (rest are zeros)
      │      ...                 ...          │
      └──────────────────────────────────────┘
```

### 6.4 Motion Noise Model

Odometry has errors that scale with motion:

```
Q = ┌─────────────────────────────────┐
    │  k_d·|Δd| + ε       0            │  Distance noise
    │      0         k_θ·|Δθ| + ε      │  Angular noise
    └─────────────────────────────────┘
```

Where:
- k_d ≈ 0.01: distance noise coefficient (1% of motion)
- k_θ ≈ 0.005: angular noise coefficient
- ε = small constant to avoid zero noise during zero motion

**Example**:
- Move Δd = 1m, rotate Δθ = 30° = 0.52 rad
- Distance noise: 0.01 × 1 = 0.01 m²
- Angular noise: 0.005 × 0.52 = 0.0026 rad²

### 6.5 Covariance Prediction

```
P⁻ = F_x · P · F_xᵀ + F_u · Q · F_uᵀ
```

**Intuition**:
1. `F_x · P · F_xᵀ`: How existing uncertainty propagates through motion
2. `F_u · Q · F_uᵀ`: Additional uncertainty from motion noise

**Key observation**:
- `P⁻[0:3, 0:3]` (robot uncertainty) **INCREASES** (motion adds uncertainty)
- `P⁻[3:, 3:]` (landmark uncertainty) **UNCHANGED** (landmarks stationary)
- `P⁻[0:3, 3:]` (robot-landmark correlation) **INCREASES** (robot moved, relative uncertainty grows)

### 6.6 Why Uncertainty Grows During Prediction

Think of a robot moving with imperfect odometry:
- After 1m: σ_x ≈ 3cm
- After 10m: σ_x ≈ 30cm
- After 100m: σ_x ≈ 3m

**Without corrections**, uncertainty grows **unbounded** with traveled distance.

**Solution**: Use landmark observations to constrain uncertainty (EKF update step).

---

## 7. EKF Update Step

### 7.1 Measurement Model

When the robot observes a landmark, it measures the landmark's position in the **robot frame**:

**Observation equation**:
```
z = h(x) + v

where v ~ N(0, R)  (measurement noise)
```

### 7.2 Deriving h(x): Global → Robot Frame Transformation

The measurement model depends on the landmark type being observed.

#### 7.2.1 Case 1: Observing a Line Landmark

**Given**:
- Robot pose: `(x_r, y_r, θ_r)` in global frame
- Line parameters: `(ρ_global, α_global)` in global frame

**Want**: Observation `(ρ_robot, α_robot)` in robot frame

**Transformation**:
```
α_robot = α_global - θ_r
ρ_robot = ρ_global - (x_r·cos(α_global) + y_r·sin(α_global))
```

**Angle normalization**:
```
α_robot = atan2(sin(α_robot), cos(α_robot))  ∈ [-π, π]
```

**Geometric interpretation**:
- The line's normal angle rotates by -θ_r (inverse robot rotation)
- The perpendicular distance adjusts by projecting robot position onto line normal

#### 7.2.2 Case 2: Observing a Corner Landmark

**Given**:
- Robot pose: `(x_r, y_r, θ_r)` in global frame
- Corner position: `(x_global, y_global)` in global frame

**Want**: Observation `(x_robot, y_robot)` in robot frame

**Transformation**:
```
┌────────┐   ┌─────────────────────────────┐   ┌────────────┐
│x_robot │ = │  cos(-θ_r)   -sin(-θ_r)     │ · │x_g - x_r   │
│y_robot │   │  sin(-θ_r)    cos(-θ_r)     │   │y_g - y_r   │
└────────┘   └─────────────────────────────┘   └────────────┘
```

**Expanded**:
```
Let: dx = x_global - x_r
     dy = y_global - y_r
     cos_θ = cos(-θ_r) = cos(θ_r)
     sin_θ = sin(-θ_r) = -sin(θ_r)

Then: x_robot = cos_θ · dx - sin_θ · dy
      y_robot = sin_θ · dx + cos_θ · dy
```

### 7.3 Measurement Jacobian H = ∂h/∂x

The Jacobian depends on landmark type.

#### 7.3.1 Jacobian for Line Landmarks

**With respect to robot pose (x_r, y_r, θ_r)**:

```
Let: cos_α = cos(α_global)
     sin_α = sin(α_global)

∂ρ_robot/∂x_r = -cos_α
∂ρ_robot/∂y_r = -sin_α
∂ρ_robot/∂θ_r = 0

∂α_robot/∂x_r = 0
∂α_robot/∂y_r = 0
∂α_robot/∂θ_r = -1
```

**With respect to observed line landmark (ρ_global, α_global)**:

```
∂ρ_robot/∂ρ_global = 1
∂ρ_robot/∂α_global = x_r·sin_α - y_r·cos_α

∂α_robot/∂ρ_global = 0
∂α_robot/∂α_global = 1
```

**Jacobian matrix for line**:
```
H_line = ┌──────────────────────────────────┐
         │ -cos_α  -sin_α   0  │ ...│  1    x_r·sin_α - y_r·cos_α │...│
         │   0       0     -1  │ ...│  0             1             │...│
         └──────────────────────────────────┘
          └──── robot pose ────┘     └─── line params (ρ, α) ────┘
```

#### 7.3.2 Jacobian for Corner Landmarks

**With respect to robot pose (x_r, y_r, θ_r)**:

```
∂x_robot/∂x_r = -cos(θ_r)
∂x_robot/∂y_r = -sin(θ_r)
∂x_robot/∂θ_r = -sin(θ_r)·dx - cos(θ_r)·dy

∂y_robot/∂x_r = sin(θ_r)
∂y_robot/∂y_r = -cos(θ_r)
∂y_robot/∂θ_r = cos(θ_r)·dx - sin(θ_r)·dy
```

**With respect to observed corner landmark (x_global, y_global)**:

```
∂x_robot/∂x_global = cos(θ_r)
∂x_robot/∂y_global = sin(θ_r)

∂y_robot/∂x_global = -sin(θ_r)
∂y_robot/∂y_global = cos(θ_r)
```

**Jacobian matrix for corner**:
```
H_corner = ┌────────────────────────────────────────────────┐
           │-cos_θ -sin_θ  -sin_θ·dx - cos_θ·dy │...│ cos_θ  sin_θ │...│
           │ sin_θ -cos_θ   cos_θ·dx - sin_θ·dy │...│-sin_θ  cos_θ │...│
           └────────────────────────────────────────────────┘
            └──────── robot pose ───────────┘     └─corner (x,y)─┘
```

**With respect to other landmarks**: All zeros (other landmarks don't affect this observation)

**Full Jacobian** (2 × n_state):

```
H = ┌────────────────────────────────────────────────────────┐
    │  ∂h/∂x_r  ∂h/∂y_r  ∂h/∂θ_r │ 0 0 │ ... │ ∂h/∂l_ix  ∂h/∂l_iy │ 0 0 │ ... │
    └────────────────────────────────────────────────────────┘
     └─────── robot pose ───────┘       └── landmark i ──┘
```

**Numerical example**: If observing landmark #3 in a system with 3 robot states + 6 landmarks (state size = 3 + 2×6 = 15):

```
H = ┌────────────────────────────────────────────────────────────────┐
    │ -cos(θ) -sin(θ)  -sin(θ)dx - cos(θ)dy │ 0 0 │ 0 0 │ cos(θ)  sin(θ) │ 0 0 │ 0 0 │
    │  sin(θ) -cos(θ)   cos(θ)dx - sin(θ)dy │ 0 0 │ 0 0 │-sin(θ)  cos(θ) │ 0 0 │ 0 0 │
    └────────────────────────────────────────────────────────────────┘
       robot (3)                               lm1   lm2      lm3        lm4   lm5
```

### 7.4 Innovation (Measurement Residual)

```
y = z_measured - z_predicted
  = z_measured - h(x⁻)
```

**Example**:
- Predicted observation: `z_pred = [1.50, 0.30]` (based on current state estimate)
- Actual measurement: `z_meas = [1.48, 0.35]`
- Innovation: `y = [1.48 - 1.50, 0.35 - 0.30] = [-0.02, 0.05]`

**Interpretation**: The landmark is 2cm closer and 5cm to the right compared to prediction.

### 7.5 Innovation Covariance

```
S = H · P⁻ · Hᵀ + R
```

Where:
- `H · P⁻ · Hᵀ`: Uncertainty from state estimate (robot + landmark uncertainty)
- `R`: Measurement noise (from sensor)

**Dimension**: 2×2 (for 2D observation)

**Physical meaning**:
- S represents the **total uncertainty** in the innovation
- Combines "how uncertain are we about where the landmark should be?" with "how noisy is the sensor?"

### 7.6 Kalman Gain

```
K = P⁻ · Hᵀ · S⁻¹
```

**Dimension**: n_state × 2

**Interpretation**: K determines how much to trust the measurement vs. the prediction

**Extreme cases**:
1. **R → 0** (perfect sensor): K → large, trust measurement fully
2. **R → ∞** (terrible sensor): K → 0, ignore measurement
3. **P → 0** (perfect prior): K → 0, trust prediction
4. **P → ∞** (no prior): K → large, trust measurement

**Key insight**: Each state variable has its own gain in K!
- `K[0:3, :]`: How much robot pose should be corrected
- `K[3+2i:3+2i+2, :]`: How much landmark i should be corrected

### 7.7 State Update

```
x⁺ = x⁻ + K · y
```

**Expanded**:
```
┌──────┐   ┌──────┐   ┌─────────────┐   ┌─────┐
│  x_r │   │  x_r⁻ │   │  K[0:3, :]   │   │  y_x │
│  y_r │   │  y_r⁻ │   │              │   │  y_y │
│  θ_r │   │  θ_r⁻ │   │              │   └─────┘
│  l₁ˣ │ = │  l₁ˣ⁻ │ + │  K[3:5, :]   │ · [y_x, y_y]
│  l₁ʸ │   │  l₁ʸ⁻ │   │              │
│  l₂ˣ │   │  l₂ˣ⁻ │   │  K[5:7, :]   │
│  l₂ʸ │   │  l₂ʸ⁻ │   │              │
│  ... │   │  ...  │   │     ...      │
└──────┘   └──────┘   └─────────────┘
```

**What gets updated**:
1. **Robot pose** (x_r, y_r, θ_r): Corrected based on where landmark "should be" vs. where it was observed
2. **Observed landmark** (l_ix, l_iy): Position refined based on new observation
3. **Other landmarks**: Slightly adjusted due to correlations (robot pose affects all landmarks)

### 7.8 Covariance Update (Joseph Form)

```
P⁺ = (I - K·H) · P⁻ · (I - K·H)ᵀ + K · R · Kᵀ
```

**Why Joseph form?** Numerical stability (ensures P remains positive semi-definite despite rounding errors)

**Simplified form** (less stable but more intuitive):
```
P⁺ = (I - K·H) · P⁻
```

**Key result**:
- Uncertainty **DECREASES** for robot pose
- Uncertainty **DECREASES** for observed landmark
- Uncertainty **DECREASES** (slightly) for other landmarks through correlations

### 7.9 Numerical Example of Update

**Setup**:
- 1 robot + 1 landmark
- State: `x = [x_r, y_r, θ_r, l_x, l_y]ᵀ`

**Before update**:
```
Robot uncertainty: σ_x = 0.10m, σ_y = 0.10m, σ_θ = 5°
Landmark uncertainty: σ_lx = 0.20m, σ_ly = 0.20m
```

**Observation**:
- Sensor measures landmark at `z = [2.00m, 0.50m]` in robot frame
- Measurement noise: R = diag([0.05², 0.05²])

**Innovation**: Suppose prediction was off by 10cm:
```
y = [0.10, 0.05]  (landmark is 10cm further, 5cm left)
```

**After update**:
```
Robot uncertainty: σ_x = 0.07m, σ_y = 0.08m, σ_θ = 4°
Landmark uncertainty: σ_lx = 0.12m, σ_ly = 0.13m
```

**Result**: Both robot and landmark became more certain!

---

## 8. Landmark Initialization

### 8.1 The Problem

When we first observe a new landmark:
- We have robot pose: `(x_r, y_r, θ_r)` with uncertainty `P_robot`
- We measure landmark parameters in robot frame with uncertainty `R_obs`
- We need to add it to the state vector in global frame with proper uncertainty

**Question**: How do we transform parameters and propagate uncertainty?

**Answer depends on landmark type**:
- **Lines**: Transform (ρ_robot, α_robot) → (ρ_global, α_global)
- **Corners**: Transform (x_robot, y_robot) → (x_global, y_global)

### 8.2 Case 1: Line Landmark Initialization (Hessian Form)

**Observation in robot frame**:
- Measured parameters: `z = [ρ_robot, α_robot]` with covariance `R_line`

**Transformation to global frame**:
```
α_global = α_robot + θ_r
ρ_global = ρ_robot + x_r·cos(α_global) + y_r·sin(α_global)
```

**Angle normalization**:
```
α_global = atan2(sin(α_global), cos(α_global))  ∈ [-π, π]
```

**Jacobian with respect to robot pose**:
```
J_robot = ∂(ρ_g, α_g)/∂(x_r, y_r, θ_r)

        = ┌──────────────────────────────────────────────┐
          │  cos(α_g)   sin(α_g)    0                     │
          │  0          0           1                     │
          └──────────────────────────────────────────────┘
```

**Jacobian with respect to observation**:
```
J_obs = ∂(ρ_g, α_g)/∂(ρ_r, α_r)

      = ┌─────────────────────────────────────────────┐
        │  1   x_r·(-sin(α_g)) + y_r·cos(α_g)        │
        │  0   1                                      │
        └─────────────────────────────────────────────┘
```

**Initial line landmark covariance**:
```
C_line = J_robot · P_robot · J_robotᵀ + J_obs · R_line · J_obsᵀ
```

**Physical interpretation**:
- `σ²_ρ` inherits robot position uncertainty projected onto normal direction
- `σ²_α` inherits robot orientation uncertainty directly (α_g = α_r + θ_r)

### 8.3 Case 2: Corner Landmark Initialization (Cartesian)

**Observation in robot frame**:
- Measured position: `z = [x_robot, y_robot]` with covariance `R_corner`

**Transformation to global frame**:
```
x_global = x_r + cos(θ_r)·x_robot - sin(θ_r)·y_robot
y_global = y_r + sin(θ_r)·x_robot + cos(θ_r)·y_robot
```

Or in matrix form:
```
┌─────────┐   ┌────┐   ┌──────────────────────┐   ┌────────┐
│ x_global│ = │ x_r│ + │  cos(θ_r)  -sin(θ_r) │ · │x_robot │
│ y_global│   │ y_r│   │  sin(θ_r)   cos(θ_r) │   │y_robot │
└─────────┘   └────┘   └──────────────────────┘   └────────┘
```

**Jacobian with respect to robot pose**:
```
J_robot = ∂(x_g, y_g)/∂(x_r, y_r, θ_r)

        = ┌──────────────────────────────────────────────┐
          │  1    0   -sin(θ_r)·x_robot - cos(θ_r)·y_robot│
          │  0    1    cos(θ_r)·x_robot - sin(θ_r)·y_robot│
          └──────────────────────────────────────────────┘
```

**Jacobian with respect to observation**:
```
J_obs = ∂(x_g, y_g)/∂(x_r, y_r)

      = ┌──────────────────────┐
        │  cos(θ_r)  -sin(θ_r)  │
        │  sin(θ_r)   cos(θ_r)  │
        └──────────────────────┘
```

**Initial corner landmark covariance**:
```
C_corner = J_robot · P_robot · J_robotᵀ + J_obs · R_corner · J_obsᵀ
```

**Physical interpretation**:
- Corner position rotates with robot orientation uncertainty
- Uncertainty ellipse oriented according to measurement geometry

### 8.4 Unified Uncertainty Propagation Formula

Both cases follow the same general form:
```
C_landmark = J_robot · P_robot · J_robotᵀ + J_obs · R_obs · J_obsᵀ
```

Where the Jacobians differ based on the transformation used.

### 8.4 Augmenting the State Vector

**Before**: State has n states
```
x = [x_r, y_r, θ_r, l₁ˣ, l₁ʸ, ..., lₙˣ, lₙʸ]ᵀ
P is (n × n)
```

**After**: State has n+2 states
```
x_new = [x_r, y_r, θ_r, l₁ˣ, l₁ʸ, ..., lₙˣ, lₙʸ, l_{n+1}ˣ, l_{n+1}ʸ]ᵀ
P_new is ((n+2) × (n+2))
```

**Covariance augmentation**:
```
P_new = ┌───────────────────────────┐
        │      P_old     │  P_cross  │
        ├────────────────┼───────────┤
        │   P_crossᵀ     │  C_lm     │
        └───────────────────────────┘
```

Where:
- `P_old`: Existing covariance (unchanged)
- `C_lm`: New landmark covariance (computed above)
- `P_cross`: Cross-correlation between new landmark and existing states

**Cross-correlation**:
```
P_cross = J_robot · P_robot
```

**This is critical!** The new landmark is **correlated** with:
1. Robot pose (strongly correlated)
2. Existing landmarks (weakly correlated through robot)

### 8.5 Why Cross-Correlation Matters

Suppose:
- Robot is uncertain in x-direction: σ_x = 0.10m
- We observe a landmark at [2m, 0m] in robot frame

The landmark will have:
- Uncertainty in global x: includes the 0.10m robot uncertainty
- **Positive correlation** with robot x: if robot is actually 0.10m to the right, landmark is too!

Later, if we observe this landmark again from a different pose:
- The measurement constrains **both** robot position and landmark position
- Due to correlation, correcting one automatically improves the other

---

## 9. Landmark Re-observation

### 9.1 What Happens When We Re-observe?

When observing an **existing** landmark:

**Input**:
- Current state: `x = [x_r, y_r, θ_r, ..., l_ix, l_iy, ...]ᵀ`
- Current covariance: `P`
- Measurement: `z = [z_x, z_y]` with noise `R`

**The Measurement Jacobian H has non-zero entries for**:
1. Robot pose [x_r, y_r, θ_r]
2. Observed landmark [l_ix, l_iy]

**This is the key!** The observation **couples** robot and landmark.

### 9.2 Why Uncertainty Decreases

**Innovation covariance**:
```
S = H · P · Hᵀ + R
```

Since H couples robot and landmark, S depends on:
- `P_rr`: Robot uncertainty
- `P_ll`: Landmark uncertainty
- `P_rl`: Robot-landmark correlation

**Kalman gain**:
```
K = P · Hᵀ · S⁻¹
```

K distributes the correction across:
- Robot pose (K[0:3, :])
- Observed landmark (K[idx:idx+2, :])
- Other landmarks (K[other, :]) through correlations

### 9.3 Intuitive Explanation

**Scenario**: Robot observes a landmark it saw before

1. **Prediction says**: "Based on odometry, robot is at [5.0, 2.0] and landmark is at [7.0, 2.5]"
2. **Measurement says**: "I see the landmark at [1.95, 0.48] in robot frame"
3. **Transform to global**: "If robot is at [5.0, 2.0] and I measure [1.95, 0.48], landmark should be at [6.95, 2.48]"
4. **Innovation**: Landmark is 5cm closer than predicted
5. **EKF reasoning**:
   - "Either robot moved less than odometry said (robot is at [4.97, 2.0])"
   - "OR landmark was closer than I thought (landmark is at [6.95, 2.48])"
   - "OR combination of both"
6. **Kalman gain decides** the split based on relative uncertainties
7. **Update**: Correct **both** robot position and landmark position

### 9.4 Mathematical Proof of Uncertainty Reduction

**Lemma**: For EKF update with H ≠ 0 and R finite:
```
trace(P⁺) < trace(P⁻)
```

**Proof sketch**:
```
P⁺ = (I - K·H) · P⁻ · (I - K·H)ᵀ + K · R · Kᵀ

The term (I - K·H) acts as a "shrinking" operator on P⁻ when K is chosen optimally.

Specifically, if K = P⁻·Hᵀ·S⁻¹ (optimal Kalman gain), then:

P⁺ = P⁻ - P⁻·Hᵀ·S⁻¹·H·P⁻

The second term is positive semi-definite (by construction of S), so:

P⁺ ≤ P⁻  (in matrix sense)

Therefore: trace(P⁺) ≤ trace(P⁻)
```

**Consequence**: Total uncertainty (sum of all variances) never increases from a measurement.

### 9.5 How Much Uncertainty Decreases

The reduction depends on:

1. **Observation quality** (R):
   - Low noise → large reduction
   - High noise → small reduction

2. **Observation geometry**:
   - Perpendicular observations better than tangential
   - Multiple viewpoints reduce uncertainty faster

3. **Number of observations**:
   - First observation: large reduction
   - Subsequent observations: diminishing returns (but never zero!)

**Example**: Observing a landmark N times (independent observations):

```
σ²_final ≈ σ²_initial / √N
```

This is the **Fisher information accumulation** principle.

### 9.6 Effect on Other Landmarks

When updating with landmark #i:

**Direct effect**:
- Robot uncertainty: **decreases** (landmark acts as reference)
- Landmark #i uncertainty: **decreases** (refined estimate)

**Indirect effect** (through correlations):
- Landmark #j (j ≠ i) uncertainty: **slightly decreases**

**Why?** Because:
1. Robot pose is corrected
2. All landmarks are correlated with robot pose (through P_rl)
3. Improving robot pose → improves estimates of all landmarks

**Magnitude**: Indirect effect is typically 10-100× smaller than direct effect.

---

## 10. Data Association

### 10.1 The Data Association Problem

**Question**: Given a new observation, which landmark does it correspond to?

**Challenges**:
1. Multiple landmarks of same type (e.g., many corners)
2. Sensor noise makes observations ambiguous
3. Wrong association → corrupted map (catastrophic failure!)

### 10.2 Mahalanobis Distance

Instead of Euclidean distance, use **Mahalanobis distance** which accounts for uncertainty:

```
d²_M = yᵀ · S⁻¹ · y
```

Where:
- `y = z_measured - z_predicted` (innovation)
- `S = H·P·Hᵀ + R` (innovation covariance)

**Physical interpretation**:
- Measures distance in units of "standard deviations"
- Automatically adapts to uncertainty ellipse shape and orientation

**Example**:

Suppose:
- Innovation: `y = [0.10, 0.05]` (10cm in x, 5cm in y)

Euclidean distance:
```
d_E = √(0.10² + 0.05²) = 0.112m
```

With uncertainty ellipse:
```
S = ┌─────────────┐
    │  0.04   0.01 │  (σ_x = 20cm, σ_y = 10cm, correlated)
    │  0.01   0.01 │
    └─────────────┘

S⁻¹ = ┌──────────────┐
      │  33.3  -33.3  │
      │ -33.3  133.3  │
      └──────────────┘

d²_M = [0.10, 0.05] · S⁻¹ · [0.10, 0.05]ᵀ = 1.67

d_M = √1.67 = 1.29  (standard deviations)
```

**Interpretation**: The innovation is 1.29 standard deviations away (well within expected range).

### 10.3 Chi-Squared Gating

Under the null hypothesis (observation matches landmark), d²_M follows a **chi-squared distribution** with 2 degrees of freedom (2D observation).

**Gating threshold** (95% confidence):
```
d²_M ≤ χ²_{2, 0.05} = 5.99
```

**Decision rule**:
- `d²_M ≤ 5.99`: **Accept** (observation likely matches landmark)
- `d²_M > 5.99`: **Reject** (observation doesn't match, probably different landmark)

**Other confidence levels**:
- 99%: threshold = 9.21
- 90%: threshold = 4.61
- 68%: threshold = 2.28

### 10.4 Multi-Hypothesis Matching

For each observation:

1. Compute Mahalanobis distance to **all** landmarks of same type
2. Keep only matches with `d²_M < threshold`
3. If multiple matches: pick **closest** (smallest d_M)
4. If no matches: create **new landmark**

**Algorithm**:
```python
for observation in observations:
    best_match = None
    best_distance = infinity

    for landmark in landmarks:
        if observation.type != landmark.type:
            continue  # Type mismatch

        y = observation.position - predict_observation(landmark)
        S = H @ P @ H.T + R
        d_squared = y.T @ inv(S) @ y

        if d_squared < 5.99:  # Chi-squared gate
            d = sqrt(d_squared)
            if d < best_distance:
                best_distance = d
                best_match = landmark

    if best_match:
        update_ekf(best_match, observation)
    else:
        add_new_landmark(observation)
```

### 10.5 Why Use Feature Covariance in Data Association?

**Problem with fixed R**: Treats all features equally

**Solution**: Use feature-specific covariance R_feature

**Example**:

**Feature 1** (line with 50 points):
```
R₁ = ┌─────────────┐
     │  0.001   0   │  (σ = 3cm, high quality)
     │   0    0.001 │
     └─────────────┘

→ Tight gating threshold
→ Less likely to match incorrectly
```

**Feature 2** (corner from 1 point):
```
R₂ = ┌─────────────┐
     │  0.01   0    │  (σ = 10cm, lower quality)
     │   0    0.01  │
     └─────────────┘

→ Looser gating threshold
→ More lenient matching (accounts for uncertainty)
```

**Result**: High-quality features are matched strictly, low-quality features are matched leniently. This is statistically optimal!

---

## 11. Why Robot and Landmarks Are Interconnected

### 11.1 The Coupling Mechanism

**Three sources of coupling**:

1. **Initialization**: When landmark is first added
   ```
   C_landmark = J_robot · P_robot · J_robotᵀ + J_obs · R · J_obsᵀ
               └─────────────────────────┘
                 inherits robot uncertainty

   P_cross = J_robot · P_robot
            └─────────────────┘
             creates correlation
   ```

2. **Re-observation**: Measurement Jacobian couples them
   ```
   H = [∂h/∂robot | ... | ∂h/∂landmark_i | ...]
        └─ non-zero ─┘      └─ non-zero ──┘
   ```

3. **Covariance update**: EKF update modifies entire covariance matrix
   ```
   P⁺ = (I - K·H) · P⁻ · (I - K·H)ᵀ
        └────────────────────────┘
         affects all elements
   ```

### 11.2 Information Flow During Update

**Observation of landmark #i**:

```
          ┌──────────────────────────────────┐
          │        EKF Update                │
          └──────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
         ▼                               ▼
   ┌──────────┐                   ┌────────────┐
   │  Robot   │◄──────────────────┤ Landmark i │
   │   Pose   │                   │  Position  │
   └──────────┘                   └────────────┘
         │                               │
         │ correlations                  │
         ├──────────────┬────────────────┤
         ▼              ▼                ▼
   ┌────────┐    ┌────────┐      ┌────────┐
   │  Lm 1  │    │  Lm 2  │ ...  │  Lm n  │
   └────────┘    └────────┘      └────────┘
```

**Information propagates**:
1. **Direct**: Observation → Robot & Landmark i
2. **Indirect**: Through correlations → All other landmarks

### 11.3 The SLAM Loop

**Positive feedback loop**:

```
Better robot     →    Better landmark
localization          positions
      ↑                      ↓
      └──────────────────────┘
      Landmarks constrain     Landmarks observed
      robot position          from better-known
                              robot poses
```

This is why SLAM **converges**: Each observation improves everything slightly, leading to consistent map and trajectory.

### 11.4 Mathematical Proof: Observing Landmark Reduces Robot Uncertainty

**Claim**: Observing a landmark reduces robot pose uncertainty.

**Proof**:

Let's partition the state as:
```
x = [x_r, x_l]ᵀ  (robot and landmarks)

P = ┌───────────┐
    │  P_rr  P_rl │
    │  P_lr  P_ll │
    └───────────┘
```

When observing landmark i, the Jacobian is:
```
H = [H_r | 0 ... 0 | H_l | 0 ... 0]
     └─robot─┘      └lm i─┘
```

The Kalman gain has form:
```
K = P · Hᵀ · S⁻¹

K_r = (P_rr·H_rᵀ + P_rl·H_lᵀ) · S⁻¹
     └──────────────────────────┘
      depends on robot-landmark correlation
```

The covariance update:
```
P_rr⁺ = P_rr⁻ - K_r·S·K_rᵀ
                └──────────┘
                 positive semi-definite

Therefore: P_rr⁺ ≤ P_rr⁻
```

**Key insight**: The term `P_rl·H_lᵀ` is what allows landmark observations to correct robot pose!

If `P_rl = 0` (no correlation), landmark observations wouldn't help robot localization.

### 11.5 Correlation Over Time

**Initially** (just after landmark initialization):
- Strong correlation between robot and landmark: `|P_rl|` large
- Observing landmark greatly helps robot localization

**After many observations**:
- Landmark well-localized: `P_ll` small
- Correlation still non-zero but less dominant
- Landmark acts as reliable "anchor" for robot

**Long-term**:
- Multiple well-localized landmarks create a "network" of constraints
- Robot localization bounded by landmark network
- Drift eliminated (unlike pure odometry)

---

## 12. Numerical Example

### 12.1 Setup

**Scenario**: Robot explores, sees one landmark multiple times

**Initial state**:
```
Robot: x_r = 0, y_r = 0, θ_r = 0
       σ_x = 0.1m, σ_y = 0.1m, σ_θ = 5° = 0.087 rad

State: x = [0, 0, 0]ᵀ
P = diag([0.01, 0.01, 0.0076])  (variances)
```

**Sensor**:
```
σ_r = 0.02m (2cm range error)
σ_θ = 0.5° = 0.0087 rad (angular error)
```

---

### 12.2 Step 1: First Observation (Landmark Initialization)

**Observation**: Landmark seen at `z = [2.00m, 0.50m]` in robot frame

**Transform to global**:
```
l_x = x_r + cos(θ_r)·z_x - sin(θ_r)·z_y
    = 0 + cos(0)·2.00 - sin(0)·0.50
    = 2.00m

l_y = y_r + sin(θ_r)·z_x + cos(θ_r)·z_y
    = 0 + sin(0)·2.00 + cos(0)·0.50
    = 0.50m
```

**Observation covariance** (polar → Cartesian):
```
r = √(2.00² + 0.50²) = 2.06m
θ = atan2(0.50, 2.00) = 0.245 rad = 14°

J = ┌──────────────────────────┐
    │  cos(14°)  -2.06·sin(14°) │  = ┌─────────────┐
    │  sin(14°)   2.06·cos(14°) │    │  0.97  -0.50 │
    └──────────────────────────┘    │  0.24   2.00 │
                                    └─────────────┘

C_polar = ┌───────────────────┐
          │  0.02²      0      │
          │    0     0.0087²   │
          └───────────────────┘

R_obs = J · C_polar · Jᵀ = ┌─────────────┐
                            │  0.00059  0.00007 │  (≈ 2.4cm × 2.0cm ellipse)
                            │  0.00007  0.00035 │
                            └─────────────┘
```

**Robot pose contribution**:
```
J_robot = ┌──────────────────────────────┐
          │  1   0   -sin(0)·2.0 - cos(0)·0.5 │  = ┌──────────┐
          │  0   1    cos(0)·2.0 - sin(0)·0.5 │    │  1  0  -0.5 │
          └──────────────────────────────────┘    │  0  1   2.0 │
                                                  └──────────┘

C_from_robot = J_robot · P_robot · J_robotᵀ
             = ┌──────────┐   ┌──────────────┐   ┌──────────┐
               │  1  0  -0.5 │ · │ 0.01   0     0    │ · │  1   0 │
               │  0  1   2.0 │   │  0    0.01   0    │   │  0   1 │
               └──────────┘   │  0     0    0.0076│   │-0.5  2.0│
                              └──────────────┘   └──────────┘

             = ┌─────────────┐
               │  0.0119  -0.0152 │
               │ -0.0152   0.0404 │
               └─────────────┘
```

**Total landmark uncertainty**:
```
C_landmark = C_from_robot + R_obs
           = ┌─────────────┐   ┌─────────────┐
             │  0.0119  -0.0152 │ + │  0.00059  0.00007 │
             │ -0.0152   0.0404 │   │  0.00007  0.00035 │
             └─────────────┘   └─────────────┘

           = ┌─────────────┐
             │  0.0125  -0.0151 │  → σ_lx ≈ 11.2cm, σ_ly ≈ 20.1cm
             │ -0.0151   0.0408 │
             └─────────────┘
```

**Augmented state**:
```
x = [0, 0, 0, 2.00, 0.50]ᵀ
    └robot─┘ └landmark─┘

P = ┌─────────────────────────────────────┐
    │  0.01   0      0      0.01    0      │  x_r
    │   0    0.01    0       0     0.01    │  y_r
    │   0     0    0.0076  -0.0038  0.0152 │  θ_r
    │  0.01   0   -0.0038   0.0125 -0.0151 │  l_x
    │   0    0.01  0.0152  -0.0151  0.0408 │  l_y
    └─────────────────────────────────────┘
```

**Note the cross-correlations**:
- `P[3,0] = 0.01`: Landmark x correlated with robot x
- `P[4,2] = 0.0152`: Landmark y correlated with robot θ

---

### 12.3 Step 2: Robot Moves

**Motion**: Robot drives Δd = 1.0m forward (in x-direction)

**Prediction**:
```
x_r⁺ = 0 + 1.0·cos(0) = 1.0m
y_r⁺ = 0 + 1.0·sin(0) = 0.0m
θ_r⁺ = 0rad
l_x⁺ = 2.00m  (landmark doesn't move)
l_y⁺ = 0.50m
```

**Motion noise**:
```
Q = ┌─────────────┐
    │  0.01·1.0   0  │  = ┌──────────┐
    │      0      0  │    │  0.01  0 │
    └─────────────┘    │   0   0 │
                       └──────────┘
```

**Covariance prediction** (simplified, showing only robot block):
```
P_rr = F_x · P_rr · F_xᵀ + F_u · Q · F_uᵀ

Robot uncertainty increases:
Before: σ_x = 0.10m
After:  σ_x = √(0.01 + 0.01) = 0.14m  (worse!)

Landmark uncertainty: unchanged
Robot-landmark correlation: grows (robot moved, relative uncertainty higher)
```

**State after prediction**:
```
x⁻ = [1.0, 0.0, 0.0, 2.00, 0.50]ᵀ

P⁻ slightly larger (robot uncertainty increased)
```

---

### 12.4 Step 3: Re-observe Landmark

**New observation**: Landmark now seen at `z = [1.00m, 0.48m]` in robot frame

**Expected observation** (based on prediction):
```
Expected: Transform landmark [2.00, 0.50] from global to robot frame:
          z_pred = [2.00 - 1.0, 0.50 - 0.0] = [1.00, 0.50]
```

**Innovation**:
```
y = z_measured - z_pred
  = [1.00, 0.48] - [1.00, 0.50]
  = [0.00, -0.02]
```

**Interpretation**: Landmark is 2cm lower than expected (in robot frame)

**Measurement Jacobian**:
```
H = [H_robot | H_landmark]

H_robot = ┌───────────────────────────┐
          │  -cos(0)  -sin(0)  -term_θ │  = ┌──────────────┐
          │   sin(0)  -cos(0)   term_θ │    │ -1   0  -0.5 │
          └───────────────────────────┘    │  0  -1   1.0 │
                                          └──────────────┘

H_landmark = ┌────────────┐
             │  cos(0)  sin(0) │  = ┌──────┐
             │ -sin(0)  cos(0) │    │  1  0 │
             └────────────┘    │  0  1 │
                               └──────┘

Full H = ┌──────────────────────────┐
         │ -1  0 -0.5 │  1  0 │
         │  0 -1  1.0 │  0  1 │
         └──────────────────────────┘
```

**Innovation covariance**:
```
S = H · P⁻ · Hᵀ + R

(Computation omitted for brevity)

S ≈ ┌─────────────┐
    │  0.033   0.002 │  (5.7cm × 5.3cm uncertainty)
    │  0.002   0.028 │
    └─────────────┘
```

**Kalman gain**:
```
K = P⁻ · Hᵀ · S⁻¹

K ≈ ┌──────────┐
    │  0.42  0.01 │  x_r: moderately affected
    │  0.02  0.36 │  y_r: moderately affected
    │ -0.18  0.53 │  θ_r: significantly affected (2cm error at 1m → ~1° error)
    │  0.58  0.01 │  l_x: significantly affected
    │  0.02  0.64 │  l_y: most affected (direct measurement in y)
    └──────────┘
```

**State update**:
```
x⁺ = x⁻ + K · y
   = [1.0, 0.0, 0.0, 2.00, 0.50]ᵀ + K · [0.00, -0.02]ᵀ

x⁺ = [1.0 + 0.42·0 + 0.01·(-0.02),      → 1.000m  (negligible change)
      0.0 + 0.02·0 + 0.36·(-0.02),      → -0.007m (7mm adjustment)
      0.0 - 0.18·0 + 0.53·(-0.02),      → -0.011rad (0.6° adjustment)
      2.00 + 0.58·0 + 0.01·(-0.02),     → 2.000m  (negligible change)
      0.50 + 0.02·0 + 0.64·(-0.02)]     → 0.487m  (13mm adjustment)
```

**Covariance update**:
```
P⁺ = (I - K·H) · P⁻ · (I - K·H)ᵀ + K · R · Kᵀ

Result (variances only):
Before update:  σ_x = 0.14m, σ_y = 0.14m, σ_θ = 0.087rad, σ_lx = 0.11m, σ_ly = 0.20m
After update:   σ_x = 0.11m, σ_y = 0.10m, σ_θ = 0.065rad, σ_lx = 0.08m, σ_ly = 0.14m
```

**Uncertainty reduced for**:
- ✅ Robot x: 14cm → 11cm (21% reduction)
- ✅ Robot y: 14cm → 10cm (29% reduction)
- ✅ Robot θ: 5.0° → 3.7° (25% reduction)
- ✅ Landmark x: 11cm → 8cm (27% reduction)
- ✅ Landmark y: 20cm → 14cm (30% reduction)

---

### 12.5 Summary of Example

| Event | Robot σ_x | Robot σ_y | Robot σ_θ | Landmark σ_x | Landmark σ_y |
|-------|-----------|-----------|-----------|--------------|--------------|
| **Initial** | 10cm | 10cm | 5.0° | - | - |
| **Add landmark** | 10cm | 10cm | 5.0° | 11cm | 20cm |
| **Move 1m** | 14cm ⬆ | 14cm ⬆ | 5.0° | 11cm | 20cm |
| **Observe again** | 11cm ⬇ | 10cm ⬇ | 3.7° ⬇ | 8cm ⬇ | 14cm ⬇ |

**Key observations**:
1. Motion increases robot uncertainty
2. Landmark observation reduces **both** robot and landmark uncertainty
3. Both are updated even though we only measured landmark position
4. Uncertainty reduction is proportional to observation quality and geometry

---

## 13. Implementation Notes

### 13.1 Numerical Stability

**Issues**:
1. Covariance matrices can become non-positive-definite due to rounding errors
2. Matrix inversions can fail if S is nearly singular

**Solutions**:

**Joseph form covariance update**:
```python
I = np.eye(n)
I_KH = I - K @ H
P = I_KH @ P @ I_KH.T + K @ R @ K.T  # More stable than (I-KH)@P
```

**Symmetry enforcement**:
```python
P = (P + P.T) / 2  # Ensure perfect symmetry
```

**Regularization**:
```python
R = R + np.eye(2) * 1e-6  # Small diagonal term prevents singularity
```

**Check positive definiteness**:
```python
eigenvalues = np.linalg.eigvals(P)
assert np.all(eigenvalues > 0), "Covariance not positive definite!"
```

### 13.2 Computational Complexity

**Prediction**: O(n²) where n = state size
- Dominant operation: `P = F @ P @ F.T`

**Update**: O(n³) for general case
- Dominant operation: Matrix inversion `inv(S)`
- For 2D observations: S is 2×2, so O(n²) dominates instead

**Landmark augmentation**: O(n²)
- Creating larger covariance matrix

**Scaling**:
- 10 landmarks: n = 3 + 2×10 = 23 states → manageable
- 100 landmarks: n = 203 states → 10000× slower!

**Solution for large maps**: Sliding window SLAM (implemented in your code)
- Keep only M most recent landmarks (e.g., M = 10)
- Older landmarks removed (marginalized out)
- Complexity: O(M²) regardless of map size

### 13.3 Coordinate System Conventions

**Frames used**:
1. **Global (map) frame**: World-fixed, landmarks stored here
2. **Robot (body) frame**: Robot-centric, measurements made here
3. **Sensor frame**: Sensor-centric (often same as robot frame)

**Transformation notation**:
- `T_G_R`: Transform from Robot frame to Global frame
- `T_R_G`: Transform from Global frame to Robot frame = T_G_R^(-1)

**Rotation direction convention**:
- Positive θ: Counter-clockwise (right-hand rule)
- Rotation matrix: cos θ in diagonal, sin θ in off-diagonal

### 13.4 Degenerate Cases

**Problem 1**: All landmarks collinear
- **Effect**: Uncertainty in perpendicular direction unbounded
- **Solution**: Require landmarks with diverse geometry

**Problem 2**: Robot doesn't rotate
- **Effect**: Rotation uncertainty grows unbounded
- **Solution**: Add IMU or enforce motion diversity

- **Effect**: Global uncertainty grows (local consistency maintained)
- **Solution**: Revisit areas, use global sensors (GPS, magnetic compass)

### 13.5 Tuning Parameters

**Sensor noise** (σ_r, σ_θ):
- Too small: Overconfident, rejects correct observations
- Too large: Underconfident, slow convergence
- **How to tune**: Measure sensor statistics empirically

**Motion noise** (Q):
- Too small: Diverges (filter overconfident in odometry)
- Too large: Oscillates (doesn't trust odometry enough)
- **How to tune**: Match Q to actual odometry error

**Data association threshold** (chi-squared):
- Too small: Misses correct matches
- Too large: Accepts wrong matches (catastrophic!)
- **Recommended**: 95% confidence (χ²_{2} = 5.99)

**Initial uncertainty** (P₀):
- Too small: Filter doesn't adapt
- Too large: Initial convergence slow but safer
- **Recommended**: Optimistic but realistic (e.g., σ_x = 10cm, σ_θ = 5°)

---

## 14. Mixed Representation: Lines and Corners

### 14.1 Why Different Representations?

Different landmark types have different optimal parameterizations:

**Lines (Walls)** → **Hessian Normal Form (ρ, α)**:
- **View-invariant**: Same parameters regardless of observation distance
- **Stable**: No ambiguity about which part of the wall was observed
- **Natural**: Perpendicular distance and orientation are meaningful

**Corners (Point Features)** → **Cartesian Coordinates (x, y)**:
- **Direct**: Position is the natural representation
- **Simple**: Straightforward transformation equations
- **Intuitive**: Matches human understanding of point locations

### 14.2 The Centroid Problem for Lines

Using Cartesian coordinates (x, y) for lines leads to **data association failure**:

```
Scenario: Robot observes a wall from different distances

Far observation (5m away):          Close observation (2m away):
  Visible wall section: 3m            Visible wall section: 1m
  Centroid: (5.0, 1.5)               Centroid: (5.3, 1.2)

  ↓ Data association sees:           ↓
  Different centroids → Treated as DIFFERENT landmarks!

  Result: Same wall appears multiple times in map (WRONG!)
```

**Root cause**: Centroid changes with viewpoint, even though it's the same wall.

### 14.3 Hessian Solution

With Hessian Normal Form (ρ, α), the **same wall always has the same parameters**:

```
Line equation: x·cos(α) + y·sin(α) = ρ

Wall perpendicular distance: ρ = 3.0m
Wall normal angle: α = 0° (pointing east)

From ANY robot position:
  - Far (5m): Measure ρ_robot, α_robot → Transform to (3.0, 0°)
  - Close (2m): Measure ρ_robot, α_robot → Transform to (3.0, 0°)

  ↓ Data association sees:
  Same (ρ, α) → Correctly recognized as SAME landmark!
```

### 14.4 Mathematical Benefits

**Compact State Space**:
- Each landmark: 2 parameters (regardless of type)
- Covariance matrix structure unchanged
- EKF equations remain 2D for both types

**Different Jacobians**:
- Lines: Couple (ρ, α) with robot (x, y, θ)
- Corners: Couple (x, y) with robot (x, y, θ)
- Both follow same uncertainty propagation rules

**Mixed Correlations**:
- Line ρ can correlate with corner x
- All landmarks share robot pose uncertainty
- Observing one landmark improves estimates of others

### 14.5 Implementation Considerations

**Feature Extraction**:
```python
if feature_type == 'line':
    # Compute Hessian parameters
    normal = perpendicular_to_line_direction
    rho = dot(normal, centroid)
    alpha = atan2(normal_y, normal_x)

    # Compute Hessian covariance
    C_hessian = compute_hessian_covariance(centroid_cov, normal)

elif feature_type == 'corner':
    # Use Cartesian position directly
    position = [x, y]
    C_cartesian = point_covariance
```

**Landmark Initialization**:
```python
if feature_type == 'line':
    # Transform (ρ_robot, α_robot) → (ρ_global, α_global)
    α_global = α_robot + θ_robot
    ρ_global = ρ_robot + x_robot·cos(α_global) + y_robot·sin(α_global)

    J_robot_line = [[cos_α, sin_α, 0],
                    [0,      0,      1]]

elif feature_type == 'corner':
    # Transform (x_robot, y_robot) → (x_global, y_global)
    x_global = x_robot + cos(θ)·dx - sin(θ)·dy
    y_global = y_robot + sin(θ)·dx + cos(θ)·dy

    J_robot_corner = [[1, 0, -sin(θ)·dx - cos(θ)·dy],
                      [0, 1,  cos(θ)·dx - sin(θ)·dy]]
```

**Measurement Update**:
```python
if feature_type == 'line':
    # Predict observation in robot frame
    ρ_pred = ρ_global - (x_r·cos(α_g) + y_r·sin(α_g))
    α_pred = α_global - θ_r

    H_line = [[-cos_α, -sin_α, 0, ..., 1, x_r·sin_α - y_r·cos_α, ...],
              [0,       0,      -1, ..., 0, 1,                    ...]]

elif feature_type == 'corner':
    # Predict observation in robot frame
    x_pred = cos_θ·dx - sin_θ·dy
    y_pred = sin_θ·dx + cos_θ·dy

    H_corner = [[-cos_θ, -sin_θ, -sin_θ·dx - cos_θ·dy, ..., cos_θ, sin_θ, ...],
                [sin_θ,  -cos_θ,  cos_θ·dx - sin_θ·dy, ..., -sin_θ, cos_θ, ...]]
```

### 14.6 Summary: Why Mixed Representation Works

**Key Advantages**:

1. **View-invariance for lines**: Walls recognized from any distance/angle
2. **Simplicity for corners**: Natural Cartesian representation
3. **Unified framework**: Both types fit into same EKF structure
4. **Optimal parameterization**: Each feature uses its best representation
5. **Data association robustness**: Lines don't duplicate when re-observed

**Theoretical Guarantee**:
The EKF framework is **representation-agnostic**. As long as:
- Transformations are differentiable (Jacobians exist)
- Uncertainty propagates correctly (J·C·Jᵀ)
- Innovations computed consistently

The system maintains statistical correctness regardless of parameterization choice.

**Practical Impact**:
- **Before** (all Cartesian): Walls appeared multiple times, map inconsistent
- **After** (mixed representation): Walls recognized reliably, map stable

---

## Conclusion

Statistical SLAM treats every quantity as a probability distribution rather than a fixed value. This enables:

1. **Optimal fusion** of noisy measurements using Kalman filtering
2. **Uncertainty quantification** for every estimated quantity
3. **Coupled refinement** of robot and landmark positions
4. **Principled data association** using statistical distance metrics

The key mathematical insight is that **robot and landmarks are correlated** through the covariance matrix. This correlation allows:
- Landmarks to constrain robot motion (reduce drift)
- Robot observations to refine landmark positions
- Information to propagate across the entire map

**Mixed Representation Enhancement**: By using optimal parameterizations for each landmark type (Hessian Normal Form for lines, Cartesian for corners), we achieve:
- **View-invariant** line representation
- **Robust** data association
- **Stable** map consistency
- **Flexibility** to handle diverse features

By properly modeling sensor noise, propagating uncertainty through nonlinear transformations, maintaining cross-correlations, and choosing appropriate parameterizations, we achieve a statistically consistent, production-grade SLAM system.

---

**End of Document**
