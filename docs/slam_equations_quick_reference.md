# EKF-SLAM Equations Quick Reference

**Quick lookup for all key equations in statistical landmark-based SLAM**

---

## State Representation

```
State vector:
x = [x_r, y_r, θ_r, l₁ˣ, l₁ʸ, l₂ˣ, l₂ʸ, ..., lₙˣ, lₙʸ]ᵀ
    └─ robot ─┘  └────── n landmarks ──────────┘

Dimension: n_state = 3 + 2×n_landmarks

Covariance matrix:
P ∈ ℝⁿˣⁿ  (symmetric, positive semi-definite)
```

---

## Sensor Noise Model

```
Polar covariance:
C_polar = ┌──────────┐
          │  σ²_r    0  │
          │   0    σ²_θ │
          └──────────┘

Typical values:
σ_r = 0.02m (2cm range error)
σ_θ = 0.5° = 0.0087 rad (angular error)
```

---

## Error Propagation (Polar → Cartesian)

```
Transformation:
x = r·cos(θ)
y = r·sin(θ)

Jacobian:
J = ┌─────────────────┐
    │  cos(θ)  -r·sin(θ) │
    │  sin(θ)   r·cos(θ) │
    └─────────────────┘

Covariance transformation:
C_cartesian = J · C_polar · Jᵀ

Result:
C_xy = ┌───────────────────────────────────────────────────┐
       │  σ²_r·cos²θ + r²σ²_θ·sin²θ      (σ²_r - r²σ²_θ)sinθcosθ │
       │  (σ²_r - r²σ²_θ)sinθcosθ      σ²_r·sin²θ + r²σ²_θ·cos²θ │
       └───────────────────────────────────────────────────┘
```

---

## EKF Prediction

### Motion Model

```
State prediction:
x_r⁺ = x_r + Δd·cos(θ_r + Δθ/2)
y_r⁺ = y_r + Δd·sin(θ_r + Δθ/2)
θ_r⁺ = θ_r + Δθ
lᵢ⁺ = lᵢ  (landmarks don't move)

State transition Jacobian:
F_x = ┌────────────────────────────────┐
      │  1  0  -Δd·sin(θ + Δθ/2) │ 0 ... │
      │  0  1   Δd·cos(θ + Δθ/2) │ 0 ... │
      │  0  0          1           │ 0 ... │
      ├──────────────────────────────────┤
      │  0  0          0           │  I  . │
      └────────────────────────────────┘

Control Jacobian:
F_u = ┌──────────────────────────┐
      │  cos(θ+Δθ/2)  -Δd·sin(θ+Δθ/2)/2 │
      │  sin(θ+Δθ/2)   Δd·cos(θ+Δθ/2)/2 │
      │       0               1           │
      │       0               0           │
      │      ...             ...          │
      └──────────────────────────┘

Motion noise:
Q = ┌─────────────────────────────┐
    │  k_d·|Δd| + ε       0        │
    │       0        k_θ·|Δθ| + ε  │
    └─────────────────────────────┘

Typical: k_d = 0.01, k_θ = 0.005, ε = 0.001
```

### Covariance Prediction

```
P⁻ = F_x · P · F_xᵀ + F_u · Q · F_uᵀ

Result: Uncertainty increases (motion adds noise)
```

---

## EKF Update

### Measurement Model

```
Observation equation (global → robot frame):
z_x = cos(θ_r)·(l_x - x_r) + sin(θ_r)·(l_y - y_r)
z_y = -sin(θ_r)·(l_x - x_r) + cos(θ_r)·(l_y - y_r)

Or using: dx = l_x - x_r, dy = l_y - y_r
          cos_θ = cos(-θ_r), sin_θ = sin(-θ_r)

z_x = cos_θ·dx - sin_θ·dy
z_y = sin_θ·dx + cos_θ·dy
```

### Measurement Jacobian

```
H = [H_robot | 0 ... 0 | H_landmark | 0 ... 0]
     └─ 3 ─┘           └──── 2 ────┘

H_robot = ┌─────────────────────────────────────┐
          │ -cos(θ)  sin(θ)  -dx·sin(θ) - dy·cos(θ) │
          │ -sin(θ) -cos(θ)   dx·cos(θ) - dy·sin(θ) │
          └─────────────────────────────────────┘

H_landmark = ┌──────────────┐
             │  cos(θ) -sin(θ) │
             │  sin(θ)  cos(θ) │
             └──────────────┘

Full dimension: H ∈ ℝ²ˣⁿ (2D observation, n-dimensional state)
```

### Innovation

```
y = z_measured - h(x⁻)

Innovation covariance:
S = H · P⁻ · Hᵀ + R

where R = observation noise covariance (2×2)
```

### Kalman Gain

```
K = P⁻ · Hᵀ · S⁻¹

Dimension: K ∈ ℝⁿˣ² (n states, 2D measurement)
```

### State Update

```
x⁺ = x⁻ + K · y

Updates:
- Robot pose: x[0:3]
- Observed landmark: x[idx:idx+2]
- Other landmarks: x[others] (through correlations)
```

### Covariance Update

```
Joseph form (numerically stable):
P⁺ = (I - K·H) · P⁻ · (I - K·H)ᵀ + K · R · Kᵀ

Simplified form:
P⁺ = (I - K·H) · P⁻

Result: Uncertainty decreases (measurement reduces uncertainty)
```

---

## Landmark Initialization

### Transform to Global Frame

```
l_x = x_r + cos(θ_r)·z_x - sin(θ_r)·z_y
l_y = y_r + sin(θ_r)·z_x + cos(θ_r)·z_y
```

### Jacobians

```
J_robot = ∂l/∂(x_r, y_r, θ_r)
        = ┌──────────────────────────────────┐
          │  1  0  -sin(θ_r)·z_x - cos(θ_r)·z_y │
          │  0  1   cos(θ_r)·z_x - sin(θ_r)·z_y │
          └──────────────────────────────────┘

J_obs = ∂l/∂(z_x, z_y)
      = ┌────────────────────┐
        │  cos(θ_r)  -sin(θ_r) │
        │  sin(θ_r)   cos(θ_r) │
        └────────────────────┘
```

### Initial Landmark Covariance

```
C_landmark = J_robot · P_robot · J_robotᵀ + J_obs · R_obs · J_obsᵀ
             └────────────────────────────┘   └───────────────┘
              from robot uncertainty         from sensor noise
```

### Cross-Correlation

```
P_cross = J_robot · P_robot

This creates correlation between new landmark and robot!
```

### State Augmentation

```
Old state: x_old ∈ ℝⁿ, P_old ∈ ℝⁿˣⁿ

New state: x_new = [x_old, l_x, l_y]ᵀ ∈ ℝ⁽ⁿ⁺²⁾

New covariance:
P_new = ┌──────────────────┐
        │  P_old   P_cross  │
        │ P_crossᵀ C_landmark│
        └──────────────────┘
```

---

## Data Association

### Mahalanobis Distance

```
For each observation-landmark pair:

1. Compute innovation:
   y = z_obs - h(x, landmark)

2. Compute innovation covariance:
   S = H · P · Hᵀ + R

3. Compute Mahalanobis distance:
   d²_M = yᵀ · S⁻¹ · y

4. Chi-squared gating (95% confidence, 2 DOF):
   if d²_M ≤ 5.99:
       Accept match
   else:
       Reject match

5. If multiple matches: pick smallest d_M
   If no matches: initialize new landmark
```

### Why Not Euclidean Distance?

```
Euclidean: d_E = ||y||

Problem: Doesn't account for:
- Uncertainty magnitude
- Uncertainty direction (ellipse orientation)
- State correlations

Mahalanobis: Adapts to uncertainty ellipse automatically
```

---

## Key Properties

### Uncertainty Evolution

```
Prediction:  P⁻ ≥ P  (uncertainty grows)
Update:      P⁺ ≤ P⁻ (uncertainty shrinks)

Mathematically:
trace(P_prediction) > trace(P)
trace(P_update) < trace(P_prediction)
```

### Information Accumulation

```
After N independent observations of same landmark:

σ²_final ≈ σ²_initial / N  (variance reduces)
σ_final ≈ σ_initial / √N   (std dev reduces)

This is Fisher Information accumulation principle.
```

### Correlation Structure

```
Correlation coefficient between variables i and j:

ρ_ij = P[i,j] / (σ_i · σ_j)

where: -1 ≤ ρ_ij ≤ 1

ρ = +1: Perfect positive correlation
ρ =  0: No correlation (independent)
ρ = -1: Perfect negative correlation
```

---

## Common Numerical Values

### Sensor Noise (LiDAR)

```
Range error:   σ_r = 0.01 - 0.03m  (1-3cm)
Angular error: σ_θ = 0.1 - 1.0°    (0.002 - 0.017 rad)
```

### Motion Noise (Wheel Odometry)

```
Distance: 1-3% of distance traveled
Angle:    0.5-1% of rotation angle

Q = ┌─────────────────────────┐
    │  0.01·|Δd|       0        │
    │      0       0.005·|Δθ|   │
    └─────────────────────────┘
```

### Initial Uncertainty

```
Position: σ_x = σ_y = 0.1m  (10cm)
Heading:  σ_θ = 5° = 0.087 rad
```

### Data Association Thresholds

```
Chi-squared (2 DOF):
90% confidence: χ² = 4.61
95% confidence: χ² = 5.99 (recommended)
99% confidence: χ² = 9.21
```

---

## Computational Complexity

```
Operation              | Complexity | Notes
-----------------------|------------|------------------
Prediction             | O(n²)      | Matrix multiply
Update                 | O(n²)      | For 2D obs (S is 2×2)
Landmark init          | O(n²)      | State augmentation
Data association       | O(m·n)     | m observations, n landmarks

where n = state size = 3 + 2·(# landmarks)
```

---

## Coordinate Frames

```
Global (Map) Frame:
- World-fixed
- Landmarks stored here
- Symbol: G or {W}

Robot (Body) Frame:
- Robot-centric
- Measurements made here
- Symbol: R or {B}

Transformations:
T_G_R: Robot frame → Global frame (forward)
T_R_G: Global frame → Robot frame (inverse)

Rotation matrix (2D):
R(θ) = ┌──────────────┐
       │  cos(θ) -sin(θ) │
       │  sin(θ)  cos(θ) │
       └──────────────┘
```

---

## Implementation Checklist

### Initialization
- [ ] Initialize state: x = [x_r, y_r, θ_r]ᵀ
- [ ] Initialize covariance: P = diag([σ²_x, σ²_y, σ²_θ])
- [ ] Set sensor noise: σ_r, σ_θ
- [ ] Set motion noise: k_d, k_θ

### Prediction Step
- [ ] Compute predicted state: x⁻ = f(x, u)
- [ ] Compute Jacobians: F_x, F_u
- [ ] Scale motion noise: Q = Q(Δd, Δθ)
- [ ] Update covariance: P⁻ = F_x·P·F_xᵀ + F_u·Q·F_uᵀ
- [ ] Enforce symmetry: P = (P + Pᵀ)/2

### Feature Extraction
- [ ] Convert scan to Cartesian with covariances
- [ ] Extract lines/corners
- [ ] Compute feature covariances
- [ ] Return features with position + covariance

### Data Association
- [ ] For each observation:
  - [ ] Compute innovation with each landmark
  - [ ] Compute Mahalanobis distance
  - [ ] Apply chi-squared gating
  - [ ] Pick best match
- [ ] Separate matched/unmatched features

### Update Step (Matched)
- [ ] Compute measurement Jacobian: H
- [ ] Compute innovation: y = z - h(x)
- [ ] Get observation noise: R (from feature covariance)
- [ ] Compute innovation covariance: S = H·P·Hᵀ + R
- [ ] Compute Kalman gain: K = P·Hᵀ·S⁻¹
- [ ] Update state: x⁺ = x⁻ + K·y
- [ ] Update covariance: P⁺ = (I-KH)·P·(I-KH)ᵀ + K·R·Kᵀ
- [ ] Normalize angles: θ ∈ [-π, π]

### Landmark Init (Unmatched)
- [ ] Transform to global frame: l = T_G_R(x_r, z)
- [ ] Compute Jacobians: J_robot, J_obs
- [ ] Compute landmark covariance: C_lm
- [ ] Compute cross-correlation: P_cross
- [ ] Augment state: x_new = [x, l]
- [ ] Augment covariance: P_new

### Landmark Management
- [ ] Prune unobserved landmarks (e.g., 10 scans)
- [ ] Limit max landmarks (sliding window, e.g., 10)
- [ ] Remove oldest if at capacity

---

## Debugging Tips

### Divergence (Filter Explodes)
**Symptom**: Uncertainty grows unbounded
**Causes**:
- Motion noise Q too small
- Sensor noise R too small
- Numerical instability in covariance update

**Fix**:
- Increase Q and/or R
- Use Joseph form for covariance update
- Check P is positive definite after each step

### Over-Confidence (Filter Ignores Measurements)
**Symptom**: State doesn't update, rejects all observations
**Causes**:
- Motion noise Q too large
- Sensor noise R too large
- Wrong Jacobians

**Fix**:
- Decrease Q and/or R
- Verify Jacobian calculations
- Check innovation covariance S is reasonable

### Wrong Data Association
**Symptom**: Map is corrupted, multiple landmarks at same location
**Causes**:
- Chi-squared threshold too loose
- Features not distinctive enough
- Insufficient geometric diversity

**Fix**:
- Tighten threshold (e.g., 5.99 → 4.61)
- Improve feature quality checks
- Require minimum feature distinctiveness

### Covariance Not Positive Definite
**Symptom**: Eigenvalues become negative
**Causes**:
- Numerical rounding errors
- Non-symmetric covariance matrix
- Bad Jacobian causing ill-conditioned update

**Fix**:
- Enforce symmetry: P = (P + Pᵀ)/2
- Add regularization: P += ε·I
- Use Joseph form update
- Check Jacobian correctness

---

**End of Quick Reference**
