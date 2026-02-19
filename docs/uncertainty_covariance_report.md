# Uncertainty Quantification in Feature-Based EKF-SLAM
## Wall, Corner, and SVD Alignment Covariance — Theory, Derivation, and Implementation

---

## Introduction: Uncertainty in Landmarks and Submap Alignment

This report treats uncertainty as a first-class output of the mapping stack, not an afterthought. In feature-based EKF-SLAM, every landmark estimate (walls and corners) and every submap alignment correction (feature-based SVD pose update) is only as reliable as the geometry and data that produced it. A short, clean wall segment seen at a shallow angle carries high uncertainty in orientation; a long, well-populated wall yields tight angular estimates but still has range uncertainty. Corners inherit and combine the uncertainty of their two parent walls. SVD alignment updates behave similarly: they can be highly confident when the shared-wall geometry is well conditioned, and weak when the scene is symmetric or dominated by a single direction.

The EKF uses these covariances to decide how strongly to trust each observation. Over-confident covariances cause inconsistency and filter divergence; over-conservative covariances waste information and slow convergence. For that reason this system computes covariances directly from the data geometry (via Fisher information / Gauss–Newton Hessians and first-order propagation) rather than using a fixed noise matrix. The rest of this document derives those expressions and explains how they are regularised for numerical stability before being injected into the EKF.

## Table of Contents

1. [Foundations: Why Covariance Matters](#1-foundations-why-covariance-matters)
2. [Mathematical Background](#2-mathematical-background)
   - 2.1 The Cramér–Rao Lower Bound
   - 2.1.1 Connecting CRLB, FIM, and Gauss–Newton Hessian
   - 2.2 First-Order Error Propagation
   - 2.3 Landmark Detection Uncertainty
   - 2.4 Uncertainty in Submap Alignment
   - 2.5 Positive Definiteness and Numerical Conditioning
3. [Wall Landmark Covariance](#3-wall-landmark-covariance)
   - 3.1 Hessian Normal Form
   - 3.2 Perpendicular-Distance Measurement Model
   - 3.3 Fisher Information Matrix for a Wall
   - 3.4 CRLB Covariance
   - 3.5 Numerical Regularisation
   - 3.6 Geometric Interpretation
4. [Corner Landmark Covariance](#4-corner-landmark-covariance)
   - 4.1 Corner as the Intersection of Two Walls
   - 4.2 First-Order Jacobian
   - 4.3 Error Propagation
   - 4.4 Geometric Interpretation
5. [SVD Submap Alignment Covariance](#5-svd-submap-alignment-covariance)
   - 5.1 Alignment Setup
   - 5.2 Overlap Correspondences
   - 5.3 SVD Solve
   - 5.4 Covariance Formula
   - 5.5 Degeneracy Check
   - 5.6 Geometric Interpretation
6. [EKF Integration](#6-ekf-integration)
   - 6.1 Landmark Initialisation
   - 6.2 Observation Update
   - 6.3 Joseph-Form Covariance Update
   - 6.4 SVD Pose Correction Injection
7. [Consistency of the Noise Model](#7-consistency-of-the-noise-model)
8. [Summary Table](#8-summary-table)

---

## 1. Foundations: Why Covariance Matters

In a Kalman-family filter every state estimate is accompanied by a covariance matrix **P**. This matrix encodes two qualitatively different kinds of uncertainty:

- **Self-uncertainty**: how well the filter knows a single state variable (the diagonal entries).
- **Cross-correlation**: how errors in one variable are correlated with errors in another (off-diagonal entries).

Both kinds matter equally. An EKF that carries accurate covariances

1. weights new measurements correctly (small innovation covariance **S** = **HPH**ᵀ + **R** when the filter is confident, large when it is not),
2. rejects outliers reliably via Mahalanobis gating,
3. propagates corrections coherently (a submap SVD correction to the robot pose shifts all correlated landmark estimates in the same direction),
4. avoids the filter inconsistency that arises when **P** is over-optimistic — a well-known failure mode of EKF-SLAM (Julier & Uhlmann, 2001).

The measurement noise matrix **R** for each feature type must therefore be computed carefully. Using a fixed diagonal **R** ignores the geometry of the measurement and is either systematically over-confident or under-confident depending on the wall orientation, number of points, and submap alignment quality. This system computes **R** analytically for all three feature types.

---

## 2. Mathematical Background

### 2.1 The Cramér–Rao Lower Bound

Let **θ** ∈ ℝⁿ be an unknown parameter vector (e.g. wall parameters `(ρ, α)`) observed through a noisy measurement model h(**θ**) + **v**, where **v** ~ N(**0**, **R**_sensor). Any unbiased estimator **θ̂** has covariance bounded below by the inverse of the Fisher Information Matrix:

```
Cov(θ̂) ≥ F(θ)⁻¹,      (CRLB)
```

where

```
F(θ) = E[ (∂ log p(z|θ)/∂θ)(∂ log p(z|θ)/∂θ)ᵀ ]
```

For a Gaussian measurement model with linear-in-θ observations Jᵢ and i.i.d. sensor noise σ², the FIM simplifies to the **Gauss–Newton Hessian**:

```
F(θ) = (1/σ²) Σᵢ Jᵢᵀ Jᵢ  =  (1/σ²) A
```

The CRLB covariance is then:

```
Cov(θ) = σ² A⁻¹
```

This is the formula used for both wall covariance and the submap alignment covariance in this system. It is exact when the measurement model is linear; for nonlinear models it is a first-order approximation.

**Why this is better than a fixed R**: A fixed **R** = σ² **I** assigns the same uncertainty regardless of whether 3 or 300 points were used to fit the feature, and regardless of the geometric arrangement of those points. The CRLB formula naturally gives smaller uncertainty when many points are available (large **A**, small **A**⁻¹), and larger uncertainty when the geometry is degenerate (small eigenvalue in **A**, large eigenvalue in **A**⁻¹).

### 2.1.1 Connecting CRLB, FIM, and Gauss–Newton Hessian

Start from a Gaussian measurement model with residuals **r(θ)** and covariance **R**:

```
p(z | θ) ∝ exp( -1/2 r(θ)ᵀ R⁻¹ r(θ) )
```

The negative log-likelihood is therefore a weighted least-squares cost:

```
L(θ) = 1/2 r(θ)ᵀ R⁻¹ r(θ)
```

The **Fisher Information Matrix** is the expected Hessian of **L(θ)**:

```
I(θ) = E[ ∇² L(θ) ]
```

If the residuals are small at the solution and the noise model is correct, the **Gauss–Newton Hessian** approximates the true Hessian:

```
H_GN = Jᵀ R⁻¹ J
```

Under these conditions, **I(θ) = H_GN**, so the CRLB becomes:

```
Cov(θ̂) ≥ I(θ)⁻¹  ≈  (Jᵀ R⁻¹ J)⁻¹
```

This is the covariance formula used throughout this report. It assumes an unbiased estimator, a correct Gaussian noise model, and a first-order linearisation around the converged solution (residuals near zero). When these assumptions are violated (wrong correspondences, local minima, or unmodeled dynamics), the covariance becomes optimistic. The wall and SVD alignment covariance derivations in Sections 3 and 5 apply this exact chain from likelihood → FIM → Gauss–Newton → CRLB.

### 2.2 First-Order Error Propagation

When a quantity **y** ∈ ℝᵐ is computed as a nonlinear function of uncertain inputs **θ** ∈ ℝⁿ with known covariance Σ_θ, the output covariance is approximated to first order by

```
Cov(y) ≈ J Σ_θ Jᵀ
```

where **J** = ∂**y**/∂**θ** is the Jacobian of the function evaluated at the current estimate. This is the standard linearisation used in the EKF for state propagation and in the corner covariance calculation.

The approximation is valid when the function is approximately linear over the uncertainty region, i.e. when Σ_θ is small relative to the curvature of the function. For corner extraction this requires that the wall uncertainties (ρ, α) are small, which is ensured by the minimum-point and minimum-length gates applied during line extraction.

### 2.3 Landmark Detection Uncertainty

Landmark detection introduces its own uncertainty before any covariance is computed. This uncertainty is not about how precisely a fitted wall or corner is estimated, but whether it is detected at all. It arises from missed detections (true structures discarded), false positives (spurious segments or corners accepted), and sensitivity to the split/merge thresholds used during segmentation.

In this pipeline, residual thresholding, minimum points/length gates, and corner angle thresholds determine which features enter the estimator. These decisions are discrete and do not appear in the CRLB/FIM/Gauss–Newton covariance, which assumes a correct association and models only sensor noise and geometry for a given fitted feature. As a result, the reported covariances should be interpreted as conditional on successful detection.

### 2.4 Uncertainty in Submap Alignment

The uncertainty of the **SVD submap alignment** is captured by its covariance, which describes how the estimated [x, y, θ] correction varies under measurement noise and geometry. High-fidelity approaches such as Monte Carlo simulation can capture more effects but are too expensive for real-time mapping. In this system we use an analytic estimate derived from the SVD singular values and the spatial spread of the matched wall points. This yields a fast, geometry-aware covariance that is consistent with the Cramér–Rao intuition: well-conditioned geometry yields tight uncertainty, while degenerate geometry yields large uncertainty.

The estimate is primarily driven by sensor noise and the configuration of shared-wall correspondences. Errors from wrong associations, local minima, or dynamic objects are not explicitly modeled, so the covariance should be interpreted as a best-case estimate conditioned on the final SVD alignment.

### 2.5 Positive Definiteness and Numerical Conditioning

A covariance matrix must be symmetric positive definite (SPD) by definition. In practice, floating-point arithmetic and near-degenerate geometry can produce matrices that fail this requirement. The consequences are severe: a non-PD covariance matrix leads to a non-PD innovation covariance **S**, which cannot be inverted to compute the Kalman gain, causing the filter to crash or produce NaN states.

Two conditioning steps are applied throughout this system:

**Eigenvalue clamping on A (before inversion)**. Let A = **V** Λ **V**ᵀ (eigen-decomposition). Replace each eigenvalue λᵢ by max(λᵢ, λ_min) before forming A⁻¹. This bounds the maximum output variance (preventing the filter from being infinitely uncertain about a parameter) and ensures A⁻¹ exists even if A is rank-deficient.

**Eigenvalue clamping on the output covariance**. After computing **C** = σ² **A**⁻¹, enforce a minimum eigenvalue σ² × 10⁻⁴. This prevents floating-point underflow from producing a numerically-zero variance on any axis.

Both steps use the eigen-decomposition rather than adding a scalar multiple of the identity. Adding σ² **I** is simpler but destroys the geometric information encoded in the eigenvectors; eigenvalue clamping preserves the directions of principal uncertainty while only regularising the magnitudes.

---

## 3. Wall Landmark Covariance

**Source**: `landmark_features.py:326–364` (`compute_wall_covariance`)

### 3.1 Hessian Normal Form

A wall (planar surface in 2D) is represented in Hessian normal form:

```
ρ = x cos(α) + y sin(α)
```

where:
- **ρ** ≥ 0 is the perpendicular distance from the map origin to the wall,
- **α** ∈ (−π, π] is the angle of the wall's outward normal from the x-axis.

This parameterisation is chosen over endpoint representation (x₁, y₁, x₂, y₂) for two reasons. First, it is minimal (2 parameters instead of 4 for an infinite line). Second, it has bounded derivatives — a small change in sensor orientation does not produce a large change in ρ or α, which would be the case for the slope-intercept form near a vertical wall.

### 3.2 Perpendicular-Distance Measurement Model

Each scan point **p**ᵢ = (pₓ, pᵧ) that belongs to a wall with true parameters (ρ, α) satisfies the perpendicular-distance constraint exactly. With additive i.i.d. range noise vᵢ ~ N(0, σ²), the noisy constraint becomes:

```
hᵢ(ρ, α) = ρ − pₓ cos(α) − pᵧ sin(α) = 0
```

The residual at each point depends on both parameters. The Jacobian of hᵢ with respect to **θ** = (ρ, α) is:

```
∂hᵢ/∂ρ = 1

∂hᵢ/∂α = pₓ sin(α) − pᵧ cos(α)
```

The second partial derivative has a clear geometric meaning: it is the component of the point's position projected onto the tangent direction of the wall (the direction perpendicular to the normal). Points far from the centroid along the wall axis contribute a large ∂h/∂α and therefore provide better angular constraint.

In vector form:

```
Jᵢ = [1,   pₓ sin(α) − pᵧ cos(α)]   ∈ ℝ¹ˣ²
```

### 3.3 Fisher Information Matrix for a Wall

With N scan points all affected by the same sensor noise σ², the FIM is:

```
A = Σᵢ₌₁ᴺ Jᵢᵀ Jᵢ

  = Σᵢ [ 1           pₓᵢ sin(α) − pᵧᵢ cos(α)    ]
        [ pₓᵢ sin(α) − pᵧᵢ cos(α)   (pₓᵢ sin(α) − pᵧᵢ cos(α))² ]
```

Expanding explicitly:

```
A₁₁ = N                           (count of points)
A₁₂ = A₂₁ = Σᵢ (pₓᵢ sin α − pᵧᵢ cos α)
A₂₂ = Σᵢ (pₓᵢ sin α − pᵧᵢ cos α)²
```

Let tᵢ = pₓᵢ sin(α) − pᵧᵢ cos(α) denote the signed distance of point i along the wall tangent (measured from the map origin). Then:

```
A = [ N       Σtᵢ   ]
    [ Σtᵢ    Σtᵢ²  ]
```

This is the **Fisher Information Matrix for a line in Hessian normal form**. It is the same structure as a weighted least-squares normal equation, which confirms that the TLS estimator is efficient (achieves the CRLB) when the noise is isotropic and i.i.d.

**Implementation** (`landmark_features.py:342–346`):
```python
A = np.zeros((2, 2))
for px, py in points:
    dr_d_alpha = px * sin_a - py * cos_a
    J = np.array([[1.0, dr_d_alpha]])
    A += J.T @ J
```

### 3.4 CRLB Covariance

The CRLB covariance is:

```
Cov(ρ, α) = σ² A⁻¹
```

After eigenvalue clamping of **A** to prevent near-singularity:

```
A = V Λ Vᵀ
Λ_clamped = diag(max(λ₁, λ_min), max(λ₂, λ_min))
A⁻¹ = V Λ_clamped⁻¹ Vᵀ
```

where λ_min = σ² × 0.1, which caps the maximum output variance at ~10σ² per parameter.

The final covariance is:

```
C_wall = σ² A⁻¹   ∈ ℝ²ˣ²
```

**Implementation** (`landmark_features.py:348–364`):
```python
min_eig_A = sigma2 * 0.1
eigvals, eigvecs = np.linalg.eigh(A)
eigvals_clamped = np.maximum(eigvals, min_eig_A)
A_inv = eigvecs @ np.diag(1.0 / eigvals_clamped) @ eigvecs.T
cov = sigma2 * A_inv
```

### 3.5 Numerical Regularisation

A second eigenvalue floor is applied to the output covariance itself:

```
λ_min_out = σ² × 10⁻⁴
```

This guards against floating-point cancellation in the matrix product σ² × **V** Λ⁻¹ **V**ᵀ that could produce a slightly negative eigenvalue in the final result.

### 3.6 Geometric Interpretation

The structure of **A** determines how precisely each wall parameter can be estimated:

| Geometry | Effect on A | Effect on Cov(ρ,α) |
|---|---|---|
| Many points, spread along wall | Large A₂₂ = Σtᵢ² | Small Var(α): good angular precision |
| Few points or points bunched at centre | Small A₂₂ | Large Var(α): poor angular precision |
| Short wall segment | Small N and small A₂₂ | Both variances large |
| Long wall far from origin | Large tᵢ values | Very small Var(α) |

This geometry-awareness is the key advantage over a fixed **R**: a wall seen at close range with few points gets a large covariance, while a long wall observed with many well-spread points gets a small covariance, and the EKF automatically down-weights or up-weights the observation accordingly.

---

## 4. Corner Landmark Covariance

**Source**: `landmark_features.py:366–432` (`compute_corner_covariance`)

### 4.1 Corner as the Intersection of Two Walls

A corner is the intersection point of two walls. Given wall A with parameters (ρₐ, αₐ) and wall B with parameters (ρ_b, α_b), the corner position **x** = (xc, yc) satisfies both perpendicular-distance constraints simultaneously:

```
xc cos(αₐ) + yc sin(αₐ) = ρₐ
xc cos(α_b) + yc sin(α_b) = ρ_b
```

In matrix form:

```
A(α) x = b(ρ)

where  A = [ cos αₐ   sin αₐ ]    b = [ ρₐ ]
           [ cos α_b  sin α_b ]        [ ρ_b ]
```

The corner position is therefore:

```
x = A(α)⁻¹ b(ρ)
```

The condition for this system to have a unique solution is det(**A**) ≠ 0:

```
det(A) = cos(αₐ) sin(α_b) − sin(αₐ) cos(α_b) = sin(α_b − αₐ)
```

This equals zero when the two walls are parallel (α_b − αₐ = 0 or π), which is geometrically correct — parallel walls do not intersect at a finite point. The implementation rejects corners when |det(**A**)| < 0.1 (approximately |α_b − αₐ| < 5.7°), providing a numerical safety margin well inside the 50° angle gate applied at the feature extraction stage.

### 4.2 First-Order Jacobian

The corner position **x** depends on the four wall parameters **θ** = (ρₐ, αₐ, ρ_b, α_b). To propagate wall uncertainties to corner position uncertainty, the Jacobian **J** = ∂**x**/∂**θ** ∈ ℝ²ˣ⁴ is needed.

**Sensitivity to ρₐ** (only **b** changes, **A** is fixed):

```
∂x/∂ρₐ = A⁻¹ ∂b/∂ρₐ = A⁻¹ [1, 0]ᵀ
```

**Sensitivity to ρ_b**:

```
∂x/∂ρ_b = A⁻¹ [0, 1]ᵀ
```

**Sensitivity to αₐ** (only **A** changes, **b** is fixed):

Differentiating **A** **x** = **b** with respect to αₐ:

```
(∂A/∂αₐ) x + A (∂x/∂αₐ) = 0

⟹   ∂x/∂αₐ = −A⁻¹ (∂A/∂αₐ) x
```

The derivative of **A** with respect to αₐ is:

```
∂A/∂αₐ = [ −sin αₐ   cos αₐ ]
           [    0         0   ]
```

Therefore:

```
∂x/∂αₐ = −A⁻¹ ( ∂A/∂αₐ · x )
```

**Sensitivity to α_b**:

```
∂A/∂α_b = [    0         0   ]
            [ −sin α_b   cos α_b ]

∂x/∂α_b = −A⁻¹ ( ∂A/∂α_b · x )
```

The full Jacobian is assembled as:

```
J = [ ∂x/∂ρₐ  | ∂x/∂αₐ  | ∂x/∂ρ_b  | ∂x/∂α_b ]   ∈ ℝ²ˣ⁴
```

**Implementation** (`landmark_features.py:398–413`):
```python
dA_dalpha1 = np.array([[-np.sin(alpha_a),  np.cos(alpha_a)],
                        [ 0.0,               0.0           ]])
dA_dalpha2 = np.array([[ 0.0,               0.0           ],
                        [-np.sin(alpha_b),  np.cos(alpha_b)]])

dx_drho1   = A_inv @ np.array([1.0, 0.0])
dx_drho2   = A_inv @ np.array([0.0, 1.0])
dx_dalpha1 = -A_inv @ (dA_dalpha1 @ x)
dx_dalpha2 = -A_inv @ (dA_dalpha2 @ x)

J = np.column_stack([dx_drho1, dx_dalpha1, dx_drho2, dx_dalpha2])
```

### 4.3 Error Propagation

The wall parameters are assumed to be mutually independent (no cross-covariance between wall A and wall B, since they are estimated from different sets of scan points). The joint covariance of the four wall parameters is therefore block-diagonal:

```
Σ_θ = [ C_A    0   ]   ∈ ℝ⁴ˣ⁴
      [   0   C_B  ]
```

where **C_A** = Cov(ρₐ, αₐ) and **C_B** = Cov(ρ_b, α_b) are the 2×2 wall covariances from Section 3.

The corner position covariance follows from first-order propagation:

```
Cov(x_c, y_c) = J Σ_θ Jᵀ   ∈ ℝ²ˣ²
```

**Implementation** (`landmark_features.py:415–423`):
```python
Sigma_theta = np.zeros((4, 4))
Sigma_theta[:2, :2] = cov1    # wall A
Sigma_theta[2:, 2:] = cov2    # wall B

cov_corner = J @ Sigma_theta @ J.T
```

After propagation, symmetry is enforced and negative eigenvalues are clamped:

```python
cov_corner = 0.5 * (cov_corner + cov_corner.T)
eigvals, eigvecs = np.linalg.eigh(cov_corner)
eigvals = np.maximum(eigvals, sigma2 * 1e-4)
```

### 4.4 Geometric Interpretation

The corner covariance captures three coupled geometric effects:

**Wall uncertainty compounds at corners.** A corner position is derived from two walls. Even if each wall is well-estimated individually, the corner inherits uncertainty from both. The corner covariance is always larger than either wall covariance transformed to Cartesian space.

**Sensitivity to intersection angle.** When α_b − αₐ → 0 (walls nearly parallel), det(**A**) → 0 and **A**⁻¹ diverges. The Jacobian terms ∂**x**/∂αₐ and ∂**x**/∂α_b, which include **A**⁻¹, also diverge. This gives a very large corner covariance for shallow intersections — geometrically correct, since a small angular error moves the intersection point far along the walls. The EKF will then down-weight this corner observation.

**Distance from intersection to sensor.** Points observed far from the sensor have larger effective noise in both ρ and α. Through the propagation, distant corners have larger Cartesian covariance — again geometrically correct.

---

## 5. SVD Submap Alignment Covariance

**Source**: `submap_stitcher.py:90–240` (`feature_align_submaps`, `_svd_align`)

### 5.1 Alignment Setup

Submap alignment uses shared **wall landmark IDs** as correspondences. Each shared ID provides a pair of walls:

- **Source**: the current `FeatureMap` wall in the **EKF map frame**.
- **Target**: the corresponding `global_walls` entry in the **global map frame**.

The alignment estimates the rigid transform that maps source → target using a one-shot SVD solve on constructed point pairs.

### 5.2 Overlap Correspondences

Each wall stores scalar extents $(t_{\min}, t_{\max})$ along its own tangent. To build point pairs:

1. Reconstruct the source wall endpoints from $(\rho, \alpha, t_{\min}, t_{\max})$.
2. Project the source extents onto the **target** tangent to compute a 1D overlap interval.
3. Sample $n_{\text{samples}} = 8$ values $t_k$ in the overlap.
4. Generate a point on **each wall’s own geometry**:

```
p_src = ρ_src * n̂_src + t_k * t̂_src
p_tgt = ρ_tgt * n̂_tgt + t_k * t̂_tgt
```

If the overlap length is < 0.3 m, the wall pair is skipped.

### 5.3 SVD Solve

Given matched pairs $\{(\mathbf{p}_i^{\mathrm{src}}, \mathbf{p}_i^{\mathrm{tgt}})\}$, compute centroids and the 2×2 cross-covariance:

```
H = Σ_i (p_i^src − c_src) (p_i^tgt − c_tgt)ᵀ
```

SVD: `H = U Σ Vᵀ`. The rigid transform is:

```
R = V D Uᵀ    (reflection guard)
t = c_tgt − R c_src
```

The condition number is:

```
κ = Σ₀ / (Σ₁ + ε)
```

### 5.4 Covariance Formula

The alignment covariance is derived from the SVD singular values and the spatial spread of source points:

```
σ² = lidar_noise_sigma²
cov_xy = σ² / (Σ₁ + 1e-6)
cov_θ  = σ² / (N * mean_sq_dist + 1e-6)
R_SVD  = diag([cov_xy, cov_xy, cov_θ])
```

where `mean_sq_dist` is the mean squared distance of source points from their centroid, and `N` is the number of point pairs.

### 5.5 Degeneracy Check

If there are fewer than 3 point pairs, or if the condition number exceeds 100, the alignment is rejected and **no pose correction** is injected into the EKF.

### 5.6 Geometric Interpretation

| SVD scenario | Geometry | Implication |
|---|---|---|
| Multiple walls with varied angles | Well-conditioned | Small covariance |
| Parallel walls (corridor) | One singular value small | High κ → reject |
| Few pairs / short overlap | Weak constraints | Large cov_θ |

This geometry-dependence is the key motivation for computing **R_SVD** rather than using a fixed measurement noise for the submap pose correction.

---

## 6. EKF Integration

### 6.1 Landmark Initialisation

When a new landmark is first observed, its initial state and covariance must be set. The observation **z** (in robot frame) is transformed to the map frame using the current robot pose. Since the robot pose is itself uncertain, the landmark initialisation covariance must propagate both the sensor noise and the robot pose uncertainty.

**Wall initialisation** (`ekf_update_feature.py:38–107`):

The wall parameters transform from robot frame (ρ_r, α_r) to map frame (ρ_m, α_m) as:

```
α_m = α_r + θ_r
ρ_m = ρ_r + x_r cos(α_m) + y_r sin(α_m)
```

The Jacobian of (ρ_m, α_m) with respect to the robot pose (x_r, y_r, θ_r):

```
H_r = [ cos α_m    sin α_m    −x_r sin α_m + y_r cos α_m ]
      [    0           0                  1               ]
```

The Jacobian with respect to the observation (ρ_r, α_r):

```
H_z = [ 1    −x_r sin α_m + y_r cos α_m ]
      [ 0                  1              ]
```

The initial landmark covariance combines robot pose uncertainty **P_rr** and measurement noise **R_obs**:

```
P_lm = H_r P_rr H_rᵀ + H_z R_obs H_zᵀ
```

This is the standard formula for the covariance of a transformed uncertain quantity. The cross-covariance between the new landmark and all existing states is:

```
P_rl = P[:, 0:3] H_rᵀ
```

which correctly captures the correlation between the new landmark and the robot pose (and transitively with existing landmarks through the robot pose).

**Corner initialisation**: analogous, using a 2D rotation matrix for the transformation from robot to map frame.

### 6.2 Observation Update

On subsequent observations of an already-initialised landmark, the standard EKF update is applied. The observation model **h** predicts the landmark position in the robot frame given the current state. The innovation is:

```
ν = z − h(state)
```

For walls, angle innovation is wrapped to (−π, π]. The Mahalanobis distance is:

```
D² = νᵀ S⁻¹ ν,    where S = H P Hᵀ + R
```

Observations with D² > 13.8 (χ²(2) at 99.7% confidence) are rejected as outliers. This threshold is higher than the standard 5.991 (95%) to reduce false rejections caused by legitimate large innovations early in the trajectory when the map is still uncertain.

### 6.3 Joseph-Form Covariance Update

The standard EKF covariance update **P** ← (**I** − **KH**)**P** is numerically unstable: it requires the product **KHP** to cancel with **P**, and round-off error causes the result to lose positive definiteness. The Joseph form is unconditionally equivalent but numerically stable:

```
P ← (I − KH) P (I − KH)ᵀ + K R Kᵀ
```

**Implementation** (`ekf_update_feature.py:259–262`):
```python
I_KH = I - K @ H
self.P = I_KH @ self.P @ I_KH.T + K @ measurement_covariance @ K.T
```

The Joseph form guarantees symmetry and positive semi-definiteness regardless of numerical errors in **K** and **H**, which is critical for long-duration EKF-SLAM where thousands of updates accumulate.

### 6.4 SVD Pose Correction Injection

When the submap SVD returns a correction [dx, dy, dθ] and covariance **R_SVD** ∈ ℝ³ˣ³, it is injected into the EKF as a direct observation of the robot pose. The observation model is the identity on the pose sub-state:

```
H_SVD = [I₃  0 ... 0]   ∈ ℝ³ˣⁿ
```

The standard EKF update then propagates the correction to every landmark through the cross-covariance terms **P**_{robot, landmark}. This is the mechanism by which global consistency is maintained: a correction to the robot pose shifts all correlated landmarks coherently. The weight given to the correction is determined by **R_SVD** — degenerate alignments are automatically down-weighted (or rejected entirely).

---

## 7. Consistency of the Noise Model

All three covariance calculations use the same sensor noise standard deviation:

```
σ = 0.01 m   (LDS-01 range noise specification)
```

This ensures internal consistency: the EKF never receives contradictory information about the noise level from different measurement sources. The 0.01 m value is the manufacturer specification for the TurtleBot3 LDS-01 at ranges up to 3.5 m.

There is a subtle point for wall covariance: the perpendicular-distance residual h = ρ − p·n̂ is related to the range noise through the projection of the range vector onto the wall normal. At normal incidence (sensor looking directly at the wall) this projection is exact; at grazing incidence the effective noise in the perpendicular direction is reduced by the cosine of the incidence angle. The implementation uses the full σ = 0.01 m without this cosine correction, which is a conservative (slightly over-stated) uncertainty estimate. This is acceptable because over-stated uncertainty leads to less aggressive updates, which errs on the side of filter stability rather than inconsistency.

---

## 8. Summary Table

| Quantity | Formula | DOF | Source |
|---|---|---|---|
| Wall sensor Jacobian | Jᵢ = [1, pₓ sin α − pᵧ cos α] | 1×2 | `landmark_features.py:345` |
| Wall Fisher matrix | **A** = Σᵢ Jᵢᵀ Jᵢ | 2×2 | `landmark_features.py:346` |
| Wall covariance (CRLB) | **C_w** = σ² **A**⁻¹ | 2×2 | `landmark_features.py:358` |
| Corner Jacobian | **J** = [∂x/∂ρₐ, ∂x/∂αₐ, ∂x/∂ρ_b, ∂x/∂α_b] | 2×4 | `landmark_features.py:413` |
| Corner covariance | **C_c** = **J** Σ_θ **J**ᵀ | 2×2 | `landmark_features.py:423` |
| SVD cross-covariance | **H** = Σ (p_src − c_src)(p_tgt − c_tgt)ᵀ | 2×2 | `submap_stitcher.py:_svd_align` |
| SVD covariance | **R_SVD** = diag(σ²/(Σ₁+ε), σ²/(Σ₁+ε), σ²/(N·d̄²+ε)) | 3×3 | `submap_stitcher.py:feature_align_submaps` |
| EKF update covariance | **P** ← (**I**−**KH**)**P**(**I**−**KH**)ᵀ + **KRK**ᵀ | n×n | `ekf_update_feature.py:262` |
| Sensor noise σ | 0.01 m (LDS-01 spec) | — | all three |

---

## References

1. **Arun, K. S., Huang, T. S., & Blostein, S. D. (1987).** "Least-Squares Fitting of Two 3-D Point Sets." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 9(5), 698–700.

2. **Horn, B. K. P. (1987).** "Closed-Form Solution of Absolute Orientation Using Unit Quaternions." *Journal of the Optical Society of America A*, 4(4), 629–642.

3. **Kay, S. M. (1993).** *Fundamentals of Statistical Signal Processing, Volume I: Estimation Theory*. Prentice Hall.

4. **Julier, S. J., & Uhlmann, J. K. (2001).** "A Counter Example to the Theory of Simultaneous Localisation and Map Building." *Proceedings of IEEE ICRA*, vol. 4, pp. 4238–4243.

5. **Bierman, G. J. (1977).** *Factorization Methods for Discrete Sequential Estimation*. Academic Press. [Source of the Joseph form.]

6. **Dissanayake, M. W. M. G., et al. (2001).** "A Solution to the Simultaneous Localisation and Map Building (SLAM) Problem." *IEEE Transactions on Robotics and Automation*, 17(3), 229–241.

7. **Golub, G. H., & Van Loan, C. F. (2013).** *Matrix Computations* (4th ed.). Johns Hopkins University Press. [TLS via SVD.]

8. **Bar-Shalom, Y., & Fortmann, T. E. (1988).** *Tracking and Data Association*. Academic Press.

9. **Higham, N. J. (2002).** *Accuracy and Stability of Numerical Algorithms* (2nd ed.). SIAM. [Eigenvalue clamping for PD enforcement.]
