# Literature Review: Feature-Based EKF-SLAM for Indoor Mobile Robotics

## Table of Contents
1. [Introduction](#1-introduction)
2. [Foundations of SLAM](#2-foundations-of-slam)
3. [Feature Extraction from 2D LiDAR](#3-feature-extraction-from-2d-lidar)
4. [Feature-Based EKF-SLAM](#4-feature-based-ekf-slam)
5. [Data Association](#5-data-association)
6. [Scan Registration and Global Consistency](#6-scan-registration-and-global-consistency)
7. [Uncertainty Quantification](#7-uncertainty-quantification)
8. [Research Gap and Thesis Contribution](#8-research-gap-and-thesis-contribution)
9. [References](#references)

---

## 1. Introduction

Simultaneous Localisation and Mapping (SLAM) is the problem of building a map of an unknown environment while tracking the robot's position within it. The problem is circular: accurate localisation requires a map, and accurate mapping requires knowing the robot's position (Smith & Cheeseman, 1987). Over four decades of research have produced a mature set of solutions. This review covers the specific literature relevant to the implemented system: geometric feature extraction from 2D LiDAR, EKF-based SLAM with line and corner landmarks, Mahalanobis-distance data association, and ICP-based scan registration with Hessian covariance estimation.

---

## 2. Foundations of SLAM

### 2.1 Probabilistic Formulation

Thrun et al. (2005) provide the standard Bayesian formulation. The SLAM posterior is

$$
p(\mathbf{x}_{0:t},\, \mathbf{m} \mid \mathbf{z}_{1:t},\, \mathbf{u}_{1:t}),
$$

where $\mathbf{x}_{0:t}$ is the robot trajectory, $\mathbf{m}$ is the map, $\mathbf{z}_{1:t}$ are sensor observations, and $\mathbf{u}_{1:t}$ are odometry increments. Different SLAM approaches make different assumptions about this posterior and use different map representations.

### 2.2 Correlation Structure

Smith and Cheeseman (1987) showed that landmarks observed from uncertain robot positions inherit correlated errors. Dissanayake et al. (2001) proved three convergence results for EKF-SLAM:

1. Any submatrix of the map covariance decreases monotonically as observations accumulate.
2. Landmark estimates become fully correlated in the limit.
3. The lower bound on map uncertainty is determined solely by initial vehicle uncertainty.

These results establish that maintaining the full joint covariance — including cross-terms between robot pose and all landmarks — is essential for a consistent filter. Ignoring correlations leads to overconfident estimates and eventual divergence (Durrant-Whyte & Bailey, 2006).

### 2.3 Filter Consistency

Huang et al. (2010) identified a subtle inconsistency in standard EKF-SLAM: the linearised system has two unobservable degrees of freedom (global position and heading), whereas the true nonlinear system has three. This mismatch causes the filter to gain spurious information, producing overconfident pose estimates. The First Estimates Jacobian (FEJ) approach — computing motion and observation Jacobians at the first-ever state estimate rather than the current one — restores correct observability and significantly improves consistency in experiments.

---

## 3. Feature Extraction from 2D LiDAR

### 3.1 Line Extraction

Nguyen et al. (2005) compared line extraction algorithms for 2D laser range data. Split-and-merge (Pavlidis & Horowitz, 1974) provides the best balance of speed, accuracy, and simplicity for structured indoor environments. The algorithm recursively subdivides point sequences based on a perpendicular distance threshold. This system uses an incremental grow variant followed by an adjacent-segment merge step.

The critical choice is which residual to minimise during extraction. An endpoint-to-endpoint residual measures perpendicular distance from each interior point to the line connecting the two segment endpoints. This is dominated by noise at grazing-angle endpoints. Total Least Squares (TLS) minimises perpendicular distances from all points simultaneously and is more robust. The TLS normal and centroid are computed via Singular Value Decomposition (SVD): the normal is the eigenvector corresponding to the smallest eigenvalue of the scatter matrix $\sum_i (\mathbf{p}_i - \bar{\mathbf{p}})(\mathbf{p}_i - \bar{\mathbf{p}})^\top$.

### 3.2 Hessian Normal Form

Siegwart and Nourbakhsh (2011) discuss feature representation choices. A line is represented in Hessian normal form as

$$
x \cos\alpha + y \sin\alpha = \rho,
$$

where $\rho \geq 0$ is the perpendicular distance from the origin to the line and $\alpha \in [-\pi, \pi]$ is the angle of the outward normal. This parameterisation is compact, non-redundant, and avoids the endpoint sensitivity of slope-intercept form. The canonical constraint $\rho \geq 0$ must be enforced explicitly: if an EKF update drives $\rho$ negative, the Hessian normal flips direction and future observations fail the angle-difference check in data association, creating duplicate landmarks.

### 3.3 Corner Detection

Arras et al. (1998) demonstrated that corners — intersections of adjacent wall segments — constrain robot position in two dimensions simultaneously, whereas a single wall constrains only the perpendicular direction. Combining walls and corners improves filter observability. Corners are computed as line-line intersections and stored as Cartesian coordinates $(x, y)$.

---

## 4. Feature-Based EKF-SLAM

### 4.1 State Representation

The EKF state vector contains the robot pose and all landmark parameters jointly:

$$
\mathbf{x} = \bigl[x_r,\, y_r,\, \theta_r,\; \rho_1,\, \alpha_1,\; \ldots,\; \rho_{N_w},\, \alpha_{N_w},\; x_1^c,\, y_1^c,\; \ldots,\; x_{N_c}^c,\, y_{N_c}^c \bigr]^\top,
$$

where $(\rho_i, \alpha_i)$ are wall parameters in Hessian normal form and $(x_j^c, y_j^c)$ are corner positions. The full joint covariance $\mathbf{P}$ is maintained so that a single landmark observation updates both that landmark and every other landmark through the cross-covariance structure.

### 4.2 Prediction

Odometry increments $(\delta_d, \delta_\theta)$ propagate the robot pose through a midpoint-integration motion model (Barfoot, 2017):

$$
\theta_{\mathrm{mid}} = \theta + \tfrac{1}{2}\delta_\theta, \qquad
x' = x + \delta_d \cos\theta_{\mathrm{mid}}, \qquad
y' = y + \delta_d \sin\theta_{\mathrm{mid}}, \qquad
\theta' = \theta + \delta_\theta.
$$

Midpoint integration reduces linearisation error compared to the forward-Euler model. The state Jacobian is

$$
\mathbf{F}_{rr} = \begin{bmatrix}
1 & 0 & -\delta_d \sin\theta_{\mathrm{mid}} \\
0 & 1 &  \delta_d \cos\theta_{\mathrm{mid}} \\
0 & 0 & 1
\end{bmatrix},
$$

and the noise Jacobian (wrt $[\delta_d,\, \delta_\theta]^\top$) is

$$
\mathbf{G}_r = \begin{bmatrix}
\cos\theta_{\mathrm{mid}} & -\tfrac{1}{2}\delta_d \sin\theta_{\mathrm{mid}} \\
\sin\theta_{\mathrm{mid}} &  \tfrac{1}{2}\delta_d \cos\theta_{\mathrm{mid}} \\
0 & 1
\end{bmatrix}.
$$

The $\tfrac{1}{2}$ factor appears in $\mathbf{G}$ (derivative of $x'$ wrt $\delta_\theta$ through $\theta_{\mathrm{mid}}$) but **not** in $\mathbf{F}$ (derivative wrt the state $\theta$, where $\partial\theta_{\mathrm{mid}}/\partial\theta = 1$). The process noise is motion-scaled: $\sigma_d^2 = q_d \delta_d^2 + \epsilon_d$ and $\sigma_\theta^2 = q_\theta \delta_\theta^2 + \epsilon_\theta$, with small minimum variances $\epsilon$ to prevent zero noise when stationary.

### 4.3 Observation Models

**Wall.** The predicted robot-frame observation of map-frame wall $(\rho_m, \alpha_m)$ is

$$
\hat{\rho}_r = \rho_m - (x_r \cos\alpha_m + y_r \sin\alpha_m), \qquad
\hat{\alpha}_r = \alpha_m - \theta_r.
$$

**Corner.** With $\Delta x = x_m - x_r$, $\Delta y = y_m - y_r$, $c = \cos\theta_r$, $s = \sin\theta_r$:

$$
\hat{z}_x = c\,\Delta x + s\,\Delta y, \qquad
\hat{z}_y = -s\,\Delta x + c\,\Delta y.
$$

### 4.4 Covariance Update — Joseph Form

The standard EKF covariance update $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\bar{\mathbf{P}}$ is numerically sensitive: rounding errors in $\mathbf{K}\mathbf{H}$ can make $\mathbf{P}$ non-symmetric or indefinite. The Joseph form (Bierman, 1977) is used instead:

$$
\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\,\bar{\mathbf{P}}\,(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\,\mathbf{R}\,\mathbf{K}^\top.
$$

This is algebraically equivalent but guarantees symmetry and preserves positive semi-definiteness regardless of finite-precision errors in $\mathbf{K}$.

### 4.5 Wall Normalisation

After every EKF update, all wall landmarks are checked for sign drift. If $\rho_i < 0$, the canonical form is restored by:

$$
\rho_i \leftarrow -\rho_i, \qquad
\alpha_i \leftarrow \alpha_i + \pi \pmod{2\pi} \text{ (wrapped to } [-\pi, \pi]).
$$

Both $\rho$ and $\alpha$ are adjusted together. Adjusting only $\rho$ creates an inconsistent pair that would pass the rho-difference check in data association but fail the alpha-difference check, inserting a geometrically incorrect duplicate landmark.

### 4.6 Landmark Management

A new landmark is not added to the state immediately on first observation. It is placed in a provisional buffer and confirmed only after $n_{\min} = 2$ observations. Landmarks not observed for more than $T_{\mathrm{prune}} = 50$ scans are removed by deleting the corresponding rows and columns from $\mathbf{x}$ and $\mathbf{P}$, and decrementing the state indices of all subsequent landmarks.

---

## 5. Data Association

### 5.1 The Correspondence Problem

Data association — matching each observed feature to an existing landmark or declaring it new — is the most failure-prone step in feature-based SLAM. Neira and Tardós (2001) showed that a single incorrect association is sufficient to corrupt the map irrecoverably. False positives (wrong match accepted) are more harmful than false negatives (correct match missed): a false negative produces a duplicate landmark that will be pruned, whereas a false positive introduces a persistent bias.

### 5.2 Mahalanobis Distance

Nearest-neighbour association with Mahalanobis-distance gating is the standard practical approach (Bar-Shalom & Fortmann, 1988). The squared Mahalanobis distance of observation $\mathbf{z}$ against predicted observation $\hat{\mathbf{z}}$ is

$$
D_M^2 = (\mathbf{z} - \hat{\mathbf{z}})^\top \mathbf{S}^{-1} (\mathbf{z} - \hat{\mathbf{z}}),
$$

where $\mathbf{S} = \mathbf{H}\bar{\mathbf{P}}\mathbf{H}^\top + \mathbf{R}$ is the innovation covariance. Unlike Euclidean distance, $D_M^2$ accounts for different variances in each component and for correlations, and is dimensionless.

If the observation genuinely comes from the predicted landmark then $D_M^2 \sim \chi^2(d)$, where $d$ is the observation dimension. For the 2-DOF features used here, a match is accepted if $D_M^2 \leq 5.991$ ($\chi^2_{0.05}(2)$, 95% confidence). Comparing $D_M^2$ to the threshold directly — not to its square — is essential; comparing to $5.991^2 = 35.88$ would make the gate more than six times too loose.

### 5.3 Spatial Pre-Filter

A Euclidean pre-filter reduces the candidate set before the Mahalanobis test. The spatial threshold (5.0 m) must be larger than the LiDAR range (3.5 m). A threshold smaller than the sensor range would reject valid observations before the Mahalanobis test can run.

### 5.4 Joint Compatibility

Neira and Tardós (2001) also introduced Joint Compatibility Branch and Bound (JCBB), which evaluates the joint statistical compatibility of a full set of observation-landmark pairs. JCBB is more robust than nearest-neighbour when landmarks are closely spaced. However, its worst-case complexity is exponential. Nearest-neighbour with the chi-squared gate provides real-time performance at the cost of occasional incorrect associations, which is acceptable for the structured indoor environments targeted here.

---

## 6. Scan Registration and Global Consistency

### 6.1 Iterative Closest Point

Besl and McKay (1992) introduced ICP for aligning point clouds. The algorithm alternates between two steps:
1. **Correspondence**: pair each source point to its nearest target point.
2. **Transform**: compute the optimal rigid transform minimising the sum of squared point-pair distances.

This alternates to a local minimum of

$$
E(\mathbf{R}, \mathbf{t}) = \sum_{i \in \mathcal{C}} \|\mathbf{R}\,\mathbf{p}_i + \mathbf{t} - \mathbf{q}_i\|^2.
$$

ICP is sensitive to initial alignment and can converge to incorrect minima. Providing an initial guess from the EKF pose estimate keeps ICP within the convergence basin.

### 6.2 Submap-Based Stitching

Rather than running ICP scan-by-scan, the system accumulates points into submaps of fixed size ($N = 50$ scans). Each completed submap is registered against the growing global map. This reduces ICP frequency and provides denser point clouds with better overlap for registration. Hess et al. (2016) use a similar submap hierarchy in Google Cartographer for real-time loop closure.

### 6.3 ICP Covariance

Censi (2007) derived a closed-form estimate of ICP covariance from the Gauss–Newton Hessian of the cost function. The information matrix accumulated over all inlier correspondences is

$$
\mathbf{A} = \sum_{i \in \mathcal{C}_{\mathrm{inlier}}} \mathbf{J}_i^\top \mathbf{J}_i,
$$

and the pose covariance is $\mathbf{R}_{\mathrm{ICP}} = \sigma^2 \mathbf{A}^{-1}$, where $\sigma$ is the sensor noise standard deviation. This is the same Cramér–Rao structure used for wall covariance. The result is geometry-aware: alignment in a featureless corridor is poorly constrained along the corridor direction, reflected in a large eigenvalue of $\mathbf{R}_{\mathrm{ICP}}$ in that direction. This automatically downweights degenerate ICP corrections in the EKF update.

---

## 7. Uncertainty Quantification

### 7.1 Cramér–Rao Lower Bound

Kay (1993) establishes that the covariance of any unbiased estimator satisfies $\mathrm{Cov}(\hat{\boldsymbol{\theta}}) \geq \mathcal{I}^{-1}$, where $\mathcal{I}$ is the Fisher information matrix. For a linear Gaussian model $\mathbf{z} = \mathbf{H}\boldsymbol{\theta} + \mathbf{n}$, $\mathbf{n} \sim \mathcal{N}(\mathbf{0}, \sigma^2 \mathbf{I})$, the Fisher information is $\mathcal{I} = \sigma^{-2} \mathbf{H}^\top \mathbf{H}$ and the CRLB is $\sigma^2 (\mathbf{H}^\top \mathbf{H})^{-1}$.

### 7.2 Wall Covariance

The Hessian normal form residual for scan point $(p_x, p_y)$ is

$$
r = p_x \cos\alpha + p_y \sin\alpha - \rho.
$$

The per-point Jacobian is $\mathbf{J}_i = [1,\; p_x \sin\alpha - p_y \cos\alpha]$ (derivatives wrt $\rho$ and $\alpha$ respectively). The Fisher information matrix accumulated over all $N$ supporting points is $\mathbf{A} = \sum_i \mathbf{J}_i^\top \mathbf{J}_i$, and the wall covariance is

$$
\mathrm{Cov}(\rho, \alpha) = \sigma^2 \mathbf{A}^{-1},
$$

with $\sigma = 0.01\;\mathrm{m}$ from the TurtleBot3 LDS-01 specification. Using a fixed sensor noise avoids the optimism of estimating $\sigma$ from residuals, which artificially reduces uncertainty as more points are added.

### 7.3 Corner Covariance

Corner covariance is propagated analytically from the two parent wall covariances. If the corner is the intersection of walls $(\rho_1, \alpha_1)$ and $(\rho_2, \alpha_2)$, first-order error propagation gives

$$
\mathrm{Cov}(x_c, y_c) = \mathbf{J}\,\boldsymbol{\Sigma}_\theta\,\mathbf{J}^\top,
$$

where $\boldsymbol{\Sigma}_\theta = \mathrm{diag}(\mathrm{Cov}(\rho_1, \alpha_1),\, \mathrm{Cov}(\rho_2, \alpha_2))$ is the block-diagonal wall covariance and $\mathbf{J}$ is the $2 \times 4$ Jacobian of the intersection formula wrt $(\rho_1, \alpha_1, \rho_2, \alpha_2)$.

### 7.4 Covariance Conditioning

Near-singular covariance matrices arise when a wall segment contains too few points or the supporting geometry is degenerate. The system uses eigenvalue floor clamping: after computing $\mathbf{A}^{-1}$, eigenvalues of the output covariance are clamped from below at $\sigma^2 \times 10^{-4}$. This is more conservative than pseudoinverse, which can return near-zero covariance for near-singular $\mathbf{A}$.

---

## 8. Research Gap and Thesis Contribution

Most prior EKF-SLAM implementations use point landmarks, which are easier to extract but poorly suited to indoor environments dominated by planar surfaces. Line-based EKF-SLAM has been demonstrated (Arras et al., 1998; Castellanos et al., 1999), but these systems predate the availability of low-cost 360° LiDAR sensors and modern compute platforms.

This thesis implements and evaluates a complete feature-based EKF-SLAM system on the TurtleBot3 Waffle Pi with a 360° LDS-01 LiDAR, targeting practical indoor mapping. The specific contributions are:

1. **TLS-based feature extraction**: Using SVD-based TLS residuals in both the grow and merge phases, rather than endpoint-based residuals, for robustness to grazing-angle noise.

2. **CRLB wall covariance**: Deriving wall covariance from the Fisher information matrix with fixed sensor noise, rather than from fit residuals, to prevent overconfident estimates in under-determined configurations.

3. **Analytical corner covariance**: Propagating corner uncertainty from parent wall covariances through the intersection Jacobian, rather than using ad-hoc heuristics.

4. **Joseph form EKF with wall normalisation**: Using the numerically stable Joseph covariance update and enforcing $\rho \geq 0$ after every state update to prevent sign-drift-induced landmark duplication.

5. **Hessian ICP covariance for EKF feedback**: Computing geometry-aware ICP pose covariance from the Gauss–Newton Hessian and injecting ICP corrections into the EKF as weighted pose observations.

Performance is assessed using absolute trajectory error (ATE), landmark map accuracy, point cloud quality, and filter consistency via the normalised estimation error squared (NEES).

---

## References

Arras, K. O., Tomatis, N., Jensen, B. T., & Siegwart, R. (1998). Multisensor on-the-fly localization using laser and vision. *Robotics and Autonomous Systems*, 34(2–3), 131–143.

Bar-Shalom, Y., & Fortmann, T. E. (1988). *Tracking and Data Association*. Academic Press.

Barfoot, T. D. (2017). *State Estimation for Robotics*. Cambridge University Press.

Besl, P. J., & McKay, N. D. (1992). A method for registration of 3-D shapes. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239–256.

Bierman, G. J. (1977). *Factorization Methods for Discrete Sequential Estimation*. Academic Press.

Castellanos, J. A., Montiel, J. M. M., Neira, J., & Tardós, J. D. (1999). The SPmap: A probabilistic framework for simultaneous localisation and map building. *IEEE Transactions on Robotics and Automation*, 15(5), 948–952.

Censi, A. (2007). An accurate closed-form estimate of ICP's covariance. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 3167–3172.

Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001). A solution to the simultaneous localisation and map building (SLAM) problem. *IEEE Transactions on Robotics and Automation*, 17(3), 229–241.

Durrant-Whyte, H., & Bailey, T. (2006). Simultaneous localisation and mapping: Part I. *IEEE Robotics & Automation Magazine*, 13(2), 99–110.

Hess, W., Kohler, D., Rapp, H., & Andor, D. (2016). Real-time loop closure in 2D LIDAR SLAM. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 1271–1278.

Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010). Observability-based rules for designing consistent EKF SLAM estimators. *The International Journal of Robotics Research*, 29(5), 502–528.

Kay, S. M. (1993). *Fundamentals of Statistical Signal Processing: Estimation Theory*. Prentice Hall.

Neira, J., & Tardós, J. D. (2001). Data association in stochastic mapping using the joint compatibility test. *IEEE Transactions on Robotics and Automation*, 17(6), 890–897.

Nguyen, V., Gächter, S., Martinelli, A., Tomatis, N., & Siegwart, R. (2005). A comparison of line extraction algorithms using 2D range data for indoor mobile robotics. *Autonomous Robots*, 23(2), 97–111.

Pavlidis, T., & Horowitz, S. L. (1974). Segmentation of plane curves. *IEEE Transactions on Computers*, C-23(8), 860–870.

Siegwart, R., & Nourbakhsh, I. R. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

Smith, R. C., & Cheeseman, P. (1987). On the representation and estimation of spatial uncertainty. *The International Journal of Robotics Research*, 5(4), 56–68.

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
