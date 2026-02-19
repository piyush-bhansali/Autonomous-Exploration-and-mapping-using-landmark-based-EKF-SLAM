# Introduction: Literature Review

## Table of Contents
1. [The SLAM Problem](#1-the-slam-problem)
2. [Probabilistic State Estimation](#2-probabilistic-state-estimation)
3. [The Kalman Filter and its Extensions](#3-the-kalman-filter-and-its-extensions)
4. [SLAM Approaches](#4-slam-approaches)
5. [Feature-Based SLAM](#5-feature-based-slam)
6. [Feature Extraction from Range Data](#6-feature-extraction-from-range-data)
7. [Data Association](#7-data-association)
8. [Uncertainty Propagation and Mathematical Foundations](#8-uncertainty-propagation-and-mathematical-foundations)
9. [Filter Consistency and Observability](#9-filter-consistency-and-observability)
10. [Summary](#10-summary)
11. [References](#references)

---

## 1. The SLAM Problem

Mobile robots operating in unknown environments must solve two interdependent problems simultaneously: they must estimate where they are, and they must build a model of the space around them. This is the Simultaneous Localisation and Mapping (SLAM) problem. The challenge is circular — accurate localisation requires a map, and accurate map building requires knowing the robot's position (Smith & Cheeseman, 1987). Early autonomous systems avoided this circularity by assuming a known map or a known starting position. Neither assumption is valid in general deployment.

Smith and Cheeseman (1987) formalised the problem using covariance matrices to represent spatial uncertainty. They observed that landmarks observed from an uncertain robot position inherit correlated errors. Two landmarks observed from the same uncertain position are therefore correlated, even if they are far apart. This correlation structure is not an artefact — it is a fundamental property of the problem. Any SLAM algorithm that discards these correlations produces an inconsistent estimate.

Dissanayake et al. (2001) provided the first convergence proof for SLAM. They showed that the determinant of any submatrix of the joint landmark covariance decreases monotonically as observations accumulate, meaning the map becomes more certain over time. They also proved that landmark estimates become fully correlated in the limit and that the minimum achievable map uncertainty is determined entirely by the initial vehicle uncertainty. These results established that SLAM is solvable in principle. The practical challenge is computational: the full joint covariance over $n$ landmarks requires $O(n^2)$ storage and $O(n^2)$ updates per observation.

Durrant-Whyte and Bailey (2006) and Bailey and Durrant-Whyte (2006) provided comprehensive tutorial treatments of the SLAM problem, unifying the field around a Bayesian filtering formulation and documenting the progress made from 1986 to 2006. By that time SLAM had been demonstrated in outdoor, underwater, and aerial environments. The indoor structured environment — the target of this thesis — remained one of the most tractable settings due to the prevalence of planar surfaces that can be reliably extracted from 2D laser range data.

---

## 2. Probabilistic State Estimation

SLAM is a special case of probabilistic state estimation. The goal is to compute the posterior distribution over the robot state and map given all sensor measurements and control inputs to date. Thrun et al. (2005) provide the definitive treatment in *Probabilistic Robotics*. The SLAM posterior is

$$
p(\mathbf{x}_{0:t},\, \mathbf{m} \mid \mathbf{z}_{1:t},\, \mathbf{u}_{1:t}),
$$

where $\mathbf{x}_{0:t}$ is the robot trajectory, $\mathbf{m}$ is the map, $\mathbf{z}_{1:t}$ are sensor observations, and $\mathbf{u}_{1:t}$ are control inputs. The full trajectory posterior is computationally intractable for most practical systems. Online SLAM solves a marginal of this: the posterior over the current pose and the map only,

$$
p(\mathbf{x}_t,\, \mathbf{m} \mid \mathbf{z}_{1:t},\, \mathbf{u}_{1:t}).
$$

The two dominant families of solutions are Gaussian filters, which approximate the posterior as a multivariate Gaussian over a joint state vector, and particle filters, which represent the posterior as a weighted set of samples. Gaussian filters are exact for linear Gaussian systems and admit principled linearisation for nonlinear systems. Particle filters make fewer distributional assumptions but require careful design to avoid degeneracy as the state dimension grows.

The Bayesian filtering recursion separates into two steps at every time instant. The **prediction** step propagates the prior through the motion model:

$$
p(\mathbf{x}_t \mid \mathbf{z}_{1:t-1}, \mathbf{u}_{1:t}) = \int p(\mathbf{x}_t \mid \mathbf{x}_{t-1}, \mathbf{u}_t)\, p(\mathbf{x}_{t-1} \mid \mathbf{z}_{1:t-1}, \mathbf{u}_{1:t-1})\, d\mathbf{x}_{t-1}.
$$

The **update** step incorporates the new observation:

$$
p(\mathbf{x}_t \mid \mathbf{z}_{1:t}, \mathbf{u}_{1:t}) \propto p(\mathbf{z}_t \mid \mathbf{x}_t)\, p(\mathbf{x}_t \mid \mathbf{z}_{1:t-1}, \mathbf{u}_{1:t}).
$$

For linear Gaussian models this recursion has a closed form: the Kalman filter. For nonlinear models, the Extended Kalman Filter linearises the motion and observation models at the current state estimate.

---

## 3. The Kalman Filter and its Extensions

### 3.1 The Kalman Filter

Kalman (1960) derived the optimal linear estimator for discrete-time linear Gaussian systems. The filter maintains a mean $\hat{\mathbf{x}}$ and covariance $\mathbf{P}$ that are the exact mean and covariance of the posterior. The prediction step is

$$
\hat{\mathbf{x}}^- = \mathbf{F}\,\hat{\mathbf{x}}, \qquad
\mathbf{P}^- = \mathbf{F}\,\mathbf{P}\,\mathbf{F}^\top + \mathbf{Q},
$$

where $\mathbf{F}$ is the state transition matrix and $\mathbf{Q}$ is the process noise covariance. The update step is

$$
\mathbf{K} = \mathbf{P}^- \mathbf{H}^\top (\mathbf{H}\,\mathbf{P}^-\,\mathbf{H}^\top + \mathbf{R})^{-1},
\quad
\hat{\mathbf{x}} = \hat{\mathbf{x}}^- + \mathbf{K}(\mathbf{z} - \mathbf{H}\,\hat{\mathbf{x}}^-),
\quad
\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\,\mathbf{P}^-.
$$

The matrix $\mathbf{K}$ is the Kalman gain. It weights the innovation (the difference between the actual and predicted observation) in proportion to the ratio of prior uncertainty to total innovation uncertainty. A large prior uncertainty relative to measurement noise gives a large gain — the filter trusts the measurement. A small prior uncertainty gives a small gain — the filter trusts its prediction.

The optimality of the Kalman filter is guaranteed only for linear Gaussian models. Jazwinski (1970) provides a rigorous treatment of stochastic filtering theory, establishing the conditions under which the Kalman filter is minimum-variance unbiased.

### 3.2 The Extended Kalman Filter

Robot motion and sensor models are nonlinear. The Extended Kalman Filter (EKF) applies the Kalman filter to nonlinear systems by linearising the motion function $f$ and observation function $h$ at the current state estimate (Gelb, 1974):

$$
\mathbf{F} = \frac{\partial f}{\partial \mathbf{x}}\Bigg|_{\hat{\mathbf{x}}}, \qquad
\mathbf{H} = \frac{\partial h}{\partial \mathbf{x}}\Bigg|_{\hat{\mathbf{x}}}.
$$

The resulting estimate is no longer optimal. Linearisation introduces an approximation error proportional to higher-order terms in the Taylor expansion. For mildly nonlinear systems with small uncertainty, this approximation is adequate. For strongly nonlinear systems or large uncertainties, the EKF can diverge (Julier & Uhlmann, 1997).

### 3.3 Numerical Stability

The standard covariance update $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}^-$ is numerically fragile. In finite-precision arithmetic, rounding errors in $\mathbf{K}\mathbf{H}$ accumulate over many update steps, causing $\mathbf{P}$ to lose symmetry and eventually become indefinite. An indefinite covariance has no physical interpretation and causes the filter to produce nonsensical gains on subsequent steps.

Bierman (1977) developed factored-form filters (UD-decomposition) that propagate $\mathbf{P} = \mathbf{U}\mathbf{D}\mathbf{U}^\top$ directly, avoiding the numerical issues of the standard form. A simpler remedy is the Joseph form of the covariance update (named after B. D. O. Joseph, who proposed it in the early 1960s and documented by Bierman, 1977):

$$
\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\,\mathbf{P}^-\,(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\,\mathbf{R}\,\mathbf{K}^\top.
$$

This is algebraically equivalent to the standard form when $\mathbf{K}$ is the optimal gain, but guarantees that $\mathbf{P}$ remains symmetric and positive semi-definite for any value of $\mathbf{K}$. It is therefore robust to sub-optimal gains and finite-precision errors.

### 3.4 Unscented and Sigma-Point Filters

Julier and Uhlmann (1997) introduced the Unscented Kalman Filter (UKF), which propagates a deterministic set of sigma points through the exact nonlinear functions rather than linearising them. The sigma points are chosen to capture the mean and covariance of the prior exactly. After propagation, the mean and covariance of the posterior are computed from the transformed sigma points. The UKF is accurate to third order in the Taylor expansion for Gaussian distributions, compared to first order for the EKF, without requiring Jacobian computation.

For high-dimensional state spaces — such as joint robot-landmark SLAM states with many landmarks — the UKF is computationally expensive because the number of sigma points scales linearly with state dimension. The EKF remains the dominant choice for large-state SLAM systems.

---

## 4. SLAM Approaches

SLAM algorithms can be divided by the map representation, the estimation method, and the treatment of loop closures.

### 4.1 Dense Scan Matching

Scan matching methods register each new laser scan against a local or global map without extracting discrete features. Lu and Milios (1997) proposed scan-to-scan matching followed by global pose graph optimisation. Each scan pair contributes a relative pose constraint to a graph, and global consistency is obtained by minimising the total constraint error. Gutmann and Konolige (2000) extended this to incremental online mapping.

Grisetti et al. (2007) developed GMapping, which combines scan matching with Rao-Blackwellized particle filters (Murphy, 2000). Each particle maintains an independent map hypothesis. Scan matching sharpens the proposal distribution for particle resampling, dramatically reducing the number of particles needed for accurate inference. GMapping became the standard 2D SLAM baseline for many years due to its public availability and reliable performance.

Hess et al. (2016) introduced Google Cartographer, combining local submap scan matching with a global pose graph for loop closure. Cartographer handles large environments in real time and remains one of the most widely deployed SLAM systems. Kohlbrecher et al. (2011) developed Hector SLAM, which uses scan matching on occupancy grids without odometry, making it robust for platforms on rough terrain.

Dense methods work well in feature-sparse environments such as long corridors and open warehouses. Their main limitations are memory consumption (storing dense point clouds or occupancy grids), sensitivity to initial alignment in ICP, and difficulty associating observations across large revisits without explicit place recognition.

### 4.2 Graph-Based SLAM

Graph SLAM (Thrun & Montemerlo, 2006) separates the front-end (computing relative pose constraints from sensor data) from the back-end (optimising the pose graph). Nodes represent poses; edges represent spatial constraints with associated covariance. Loop closure inserts long-range edges that tighten the graph when the robot revisits a known area.

Kümmerle et al. (2011) developed g2o, a general framework for nonlinear graph optimisation using sparse solvers. Kaess et al. (2012) developed iSAM2, which performs incremental smoothing and maintains a Bayes tree for efficient incremental updates. Graph-based methods have become the dominant back-end for large-scale SLAM because the pose graph is sparse — most poses are connected only to temporally adjacent poses — and sparse linear solvers scale nearly linearly with graph size.

### 4.3 Particle Filter SLAM

Montemerlo et al. (2002) introduced FastSLAM, which exploits a conditional independence property: given the robot trajectory, all landmarks are independent. This allows factoring the joint posterior into a trajectory distribution (represented by particles) and per-landmark posteriors (each a Kalman filter conditioned on one particle's trajectory). FastSLAM scales as $O(M \log N)$, where $M$ is the particle count and $N$ the landmark count, versus $O(N^2)$ for EKF-SLAM.

Particle filters degrade in high-dimensional spaces. As the state dimension grows, the number of particles needed to represent the posterior accurately grows exponentially. FastSLAM mitigates this by conditioning each per-landmark filter on a particle trajectory, keeping each per-landmark problem low-dimensional. However, particle degeneracy after resampling remains a practical concern for large environments with long trajectories.

---

## 5. Feature-Based SLAM

### 5.1 Landmarks and Sparse Maps

Feature-based SLAM represents the environment as a set of discrete landmarks. Each landmark is a geometric primitive — a point, line, or plane — detected reliably and repeatedly from sensor data. The SLAM state is the joint vector of robot pose and all landmark parameters. The full joint covariance is maintained to propagate correlations between the robot and every landmark.

The key advantage of sparse representations over dense ones is efficiency. Storing $n$ landmarks requires $O(n)$ space, compared to $O(A/r^2)$ for an occupancy grid of area $A$ and resolution $r$. For structured indoor environments, a few hundred landmarks suffice to represent a room-scale space. The trade-off is that feature extraction must be reliable: a missed feature creates a temporary localisation gap, and an incorrect feature can corrupt the map permanently.

### 5.2 Point Landmarks

The earliest EKF-SLAM systems used point landmarks (Castellanos et al., 1999; Montemerlo et al., 2002). A point landmark in 2D has state $(x, y)$. The observation model from robot pose $(x_r, y_r, \theta_r)$ is range-and-bearing: $\hat{r} = \sqrt{(x - x_r)^2 + (y - y_r)^2}$, $\hat{\phi} = \arctan2(y - y_r, x - x_r) - \theta_r$. The Jacobian of this model is straightforward to derive.

Point landmarks extracted from 2D LiDAR data are typically corners or isolated objects. In indoor environments most of the sensor return comes from walls, not discrete points. Point landmarks therefore use the sensor data inefficiently and discard the structural regularity of the environment.

### 5.3 Line and Wall Landmarks

Arras et al. (1998) and Castellanos et al. (1999) demonstrated EKF-SLAM with line landmarks extracted from 2D laser range data. Line landmarks describe walls and other planar surfaces compactly. A single line landmark represents hundreds of range returns that would otherwise form hundreds of point landmarks. The compact representation reduces the number of EKF update operations and makes re-observation (loop closure) more reliable.

Siegwart and Nourbakhsh (2011) discuss parameterisation choices for line landmarks. Slope-intercept form $(m, b)$ is singular for vertical lines. A two-point form depends on which points are chosen and is sensitive to endpoint noise. The Hessian normal form $(\rho, \alpha)$ — where $\rho \geq 0$ is the perpendicular distance from the origin to the line and $\alpha \in [-\pi, \pi]$ is the outward normal angle — is non-singular, compact, and uniquely defined. The canonical constraint $\rho \geq 0$ must be enforced; a sign flip in $\rho$ is geometrically equivalent but produces a large innovation in data association, causing the landmark to be re-entered as a duplicate.

### 5.4 Corner Landmarks

Corners provide complementary observability to walls. A wall observation constrains only the component of robot position perpendicular to the wall. A corner observation constrains both position components simultaneously, since it is a point in 2D. Arras et al. (1998) showed that combining wall and corner landmarks improves localisation accuracy compared to walls alone. Corners are detected as intersections of adjacent extracted line segments and stored as Cartesian coordinates $(x, y)$ in the map frame.

---

## 6. Feature Extraction from Range Data

### 6.1 Line Extraction

Nguyen et al. (2005) conducted a systematic comparison of line extraction algorithms for 2D laser range data, evaluating split-and-merge, iterative end-point fit, incremental, RANSAC, and Hough transform approaches. For structured indoor environments, split-and-merge and incremental methods produced the most accurate and consistent results. The key metric is the quality of the Hessian parameters ($\rho$, $\alpha$) and their associated covariance.

The split-and-merge algorithm (Pavlidis & Horowitz, 1974) works in two phases. The split phase recursively divides a point sequence if the maximum residual exceeds a threshold. The merge phase combines adjacent segments whose combined residual also falls below the threshold. Both phases require a residual measure. An endpoint residual — perpendicular distance from each interior point to the line connecting the two segment endpoints — is simple but dominated by endpoint noise, especially at shallow incidence angles. A Total Least Squares (TLS) residual — perpendicular distance from each point to the best-fit line through all points — is more robust because it distributes the influence of all points equally (Golub & Van Loan, 2013).

TLS line fitting minimises the sum of squared perpendicular distances. The solution is the eigenvector of the scatter matrix $\sum_i (\mathbf{p}_i - \bar{\mathbf{p}})(\mathbf{p}_i - \bar{\mathbf{p}})^\top$ corresponding to the smallest eigenvalue, which is the direction orthogonal to the best-fit line. This is equivalent to truncated SVD on the centred point matrix. Golub and Van Loan (2013) provide a full derivation. The TLS fit minimises the algebraically correct residual for Hessian normal form parameters and is therefore preferred over ordinary least squares or endpoint-based approaches.

### 6.2 Line Segment Growing

An alternative to split-and-merge is incremental growing (Arras et al., 1998). Starting from an initial pair of points, the segment is extended one point at a time in the scan order. Each candidate point is added if the updated TLS residual remains below threshold. When a point fails, the segment is finalised and a new segment is started. Growing has $O(n)$ complexity and produces segments in scan order, which simplifies the subsequent merge step. The merge step combines adjacent segments if their combined TLS residual also remains below threshold.

---

## 7. Data Association

### 7.1 The Correspondence Problem

Data association is the mapping from observations to landmarks. It is the most critical and failure-prone step in feature-based SLAM. Neira and Tardós (2001) showed that a single incorrect association is sufficient to permanently corrupt the map: the Kalman update propagates the incorrect constraint through the full joint covariance, shifting all correlated landmark estimates away from their true positions. Recovery without restarting the filter is generally not possible.

The fundamental tension is between false positives (wrong match accepted) and false negatives (correct match missed). A false negative produces a duplicate landmark, which wastes memory and may cause redundant observations in future. A duplicate landmark will typically be pruned once it stops receiving observations. A false positive introduces a systematic bias that compounds over time. Statistical gates should therefore be conservative: it is preferable to accept a false negative than a false positive.

### 7.2 Individual Compatibility and Mahalanobis Gating

Bar-Shalom and Fortmann (1988) established individual compatibility testing as the baseline for data association. An observation $\mathbf{z}$ is individually compatible with predicted observation $\hat{\mathbf{z}}$ if the squared Mahalanobis distance

$$
D_M^2 = (\mathbf{z} - \hat{\mathbf{z}})^\top \mathbf{S}^{-1} (\mathbf{z} - \hat{\mathbf{z}})
$$

falls below a chi-squared threshold. Here $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top + \mathbf{R}$ is the innovation covariance. If the observation genuinely comes from the predicted landmark, $D_M^2 \sim \chi^2(d)$ where $d$ is the observation dimension. Gating at the 95th percentile of $\chi^2(d)$ rejects most spurious associations while accepting 95% of genuine ones.

The Mahalanobis distance is dimensionless and accounts for different scales and correlations in the innovation components. Euclidean distance in observation space is inadequate because it treats all components equally regardless of their uncertainty. A wall $\rho$ innovation of 0.5 m has a very different significance when $\sigma_\rho = 0.01$ m (50 sigma — a clear mismatch) versus $\sigma_\rho = 1.0$ m (0.5 sigma — plausibly the same wall). The Mahalanobis test normalises for this.

### 7.3 Nearest-Neighbour Association

The nearest-neighbour algorithm selects the landmark with the smallest Mahalanobis distance among all candidates passing the gate (Bar-Shalom & Fortmann, 1988). It is computationally efficient and works well when landmarks are well-separated relative to their uncertainty ellipsoids. Its failure mode is in dense, cluttered environments where multiple landmarks lie within each other's gates. In that case, the gate does not uniquely identify the correct landmark.

### 7.4 Joint Compatibility

Neira and Tardós (2001) introduced the Joint Compatibility Branch and Bound (JCBB) algorithm. Rather than testing each observation-landmark pair independently, JCBB tests the joint compatibility of a full assignment of observations to landmarks. A joint assignment passes the gate if the combined innovation — stacking all individual innovations into a single vector — has a Mahalanobis distance below the appropriate chi-squared threshold for the total degrees of freedom. Joint compatibility is more discriminating than individual compatibility because it rejects assignments where individual pairs are each marginally compatible but collectively inconsistent. The cost is exponential worst-case complexity, making it unsuitable for environments with many ambiguous landmarks.

---

## 8. Uncertainty Propagation and Mathematical Foundations

### 8.1 The Cramér–Rao Lower Bound

The Cramér–Rao Lower Bound (CRLB) is the fundamental limit on the precision of an unbiased estimator. Kay (1993) provides a comprehensive treatment. For a parameter vector $\boldsymbol{\theta}$ observed through a noisy model $p(\mathbf{z} ; \boldsymbol{\theta})$, the covariance of any unbiased estimator $\hat{\boldsymbol{\theta}}$ satisfies

$$
\mathrm{Cov}(\hat{\boldsymbol{\theta}}) \geq \mathcal{I}(\boldsymbol{\theta})^{-1},
$$

where $\mathcal{I}(\boldsymbol{\theta})$ is the Fisher Information Matrix (FIM):

$$
\mathcal{I}(\boldsymbol{\theta}) = -\mathbb{E}\left[\frac{\partial^2 \ln p(\mathbf{z};\boldsymbol{\theta})}{\partial \boldsymbol{\theta} \partial \boldsymbol{\theta}^\top}\right].
$$

For the linear model $\mathbf{z} = \mathbf{H}\boldsymbol{\theta} + \mathbf{n}$ with $\mathbf{n} \sim \mathcal{N}(\mathbf{0}, \sigma^2\mathbf{I})$, the FIM is $\mathcal{I} = \sigma^{-2}\mathbf{H}^\top\mathbf{H}$ and the CRLB is $\sigma^2(\mathbf{H}^\top\mathbf{H})^{-1}$. The least-squares estimator achieves this bound — it is efficient (Gauss–Markov theorem).

The CRLB is directly applicable to feature covariance estimation in SLAM. Each range measurement contributes one row to $\mathbf{H}$. The accumulated Fisher information $\mathbf{A} = \mathbf{H}^\top\mathbf{H}$ grows with the number of supporting points, and the covariance $\sigma^2\mathbf{A}^{-1}$ shrinks. Using the fixed sensor noise $\sigma$ from hardware specifications rather than estimating it from residuals is critical: estimating $\sigma$ from residuals gives an empirical quantity that decreases as the number of points grows, artificially reducing the reported uncertainty and making the filter overconfident. A fixed $\sigma$ from the manufacturer datasheet reflects the true measurement noise floor.

### 8.2 First-Order Error Propagation

When a parameter of interest is a nonlinear function of directly measured quantities, its uncertainty is approximated by first-order (linear) error propagation (Taylor, 1997). If $\mathbf{y} = g(\mathbf{x})$ and $\mathbf{x}$ has covariance $\boldsymbol{\Sigma}_x$, then

$$
\boldsymbol{\Sigma}_y \approx \mathbf{J}\,\boldsymbol{\Sigma}_x\,\mathbf{J}^\top,
$$

where $\mathbf{J} = \partial g / \partial \mathbf{x}$ is the Jacobian evaluated at the mean of $\mathbf{x}$. This is a first-order Taylor approximation; it is exact when $g$ is linear and accurate when the uncertainty in $\mathbf{x}$ is small relative to the curvature of $g$.

Error propagation is used throughout the system: corner covariance is propagated from wall covariances through the Jacobian of the line-line intersection formula; submap alignment covariance is derived from the SVD singular values and point spread; landmark initialisation covariance is propagated from observation covariance through the inverse observation model Jacobian.

### 8.3 Total Least Squares and Singular Value Decomposition

Ordinary Least Squares (OLS) minimises the sum of squared residuals in the dependent variable only, assuming the independent variable is error-free. For geometric line fitting, all coordinates are measured and all carry noise. OLS is therefore biased. Total Least Squares (Golub & Van Loan, 2013) treats errors in all variables symmetrically by minimising the sum of squared perpendicular distances from the data points to the fitted line. For the 2D line fitting problem, TLS reduces to a principal component analysis: the line direction is the principal eigenvector of the scatter matrix and the normal is the minor eigenvector.

The connection to Singular Value Decomposition is direct. For the centred data matrix $\tilde{\mathbf{X}} = \mathbf{X} - \bar{\mathbf{x}}\mathbf{1}^\top \in \mathbb{R}^{N \times 2}$, the SVD is $\tilde{\mathbf{X}} = \mathbf{U}\boldsymbol{\Sigma}\mathbf{V}^\top$. The right singular vector corresponding to the smallest singular value gives the TLS normal direction. This is numerically stable and does not require forming the normal equations $\mathbf{X}^\top\mathbf{X}$, which can amplify conditioning problems.

### 8.4 The Gauss–Newton Method

The Gauss–Newton method solves nonlinear least squares problems of the form $\min_{\boldsymbol{\theta}} \sum_i r_i(\boldsymbol{\theta})^2$ by iteratively solving the linearised normal equations (Nocedal & Wright, 2006). At each iteration the Jacobian $\mathbf{J}$ of the residual vector $\mathbf{r}$ is evaluated at the current estimate and the update is

$$
\delta\boldsymbol{\theta} = -(\mathbf{J}^\top\mathbf{J})^{-1}\mathbf{J}^\top\mathbf{r}.
$$

The matrix $\mathbf{H}_{\mathrm{GN}} = \mathbf{J}^\top\mathbf{J}$ approximates the Hessian of the cost function, neglecting second-order terms. At convergence, $\mathbf{H}_{\mathrm{GN}}$ approximates the curvature of the cost around the solution. Censi (2007) recognised that the inverse of this Hessian, scaled by the sensor noise variance, gives a principled covariance estimate for any nonlinear least squares alignment: $\boldsymbol{\Sigma} = \sigma^2 \mathbf{H}_{\mathrm{GN}}^{-1}$. This is precisely the CRLB for the linearised problem. The formula applies to ICP, scan matching, and any other alignment problem cast as nonlinear least squares.

### 8.5 Positive Definite Covariance Matrices

A covariance matrix must be symmetric and positive semi-definite (PSD) by construction. In practice, finite-precision arithmetic can violate both properties. Symmetry is easily restored by replacing $\mathbf{P}$ with $(\mathbf{P} + \mathbf{P}^\top)/2$. Positive definiteness requires that all eigenvalues are strictly positive. Higham (2002) reviews methods for computing the nearest positive definite matrix. A practical approach is eigenvalue clamping: compute the eigendecomposition $\mathbf{P} = \mathbf{V}\boldsymbol{\Lambda}\mathbf{V}^\top$, replace negative or near-zero eigenvalues with a small positive floor $\epsilon$, and reconstruct $\mathbf{P} = \mathbf{V}\boldsymbol{\Lambda}_{\mathrm{clamped}}\mathbf{V}^\top$. The floor $\epsilon$ should reflect the minimum physically meaningful uncertainty — for a system with sensor noise $\sigma$, a floor of $\sigma^2 \times 10^{-4}$ is conservative without being so small as to risk numerical issues.

---

## 9. Filter Consistency and Observability

### 9.1 Consistency

A filter is consistent if the true state lies within the confidence region predicted by the filter covariance with the expected frequency. For a Gaussian filter, consistency is measured by the Normalised Estimation Error Squared (NEES):

$$
\varepsilon_t = (\mathbf{x}_t - \hat{\mathbf{x}}_t)^\top \mathbf{P}_t^{-1} (\mathbf{x}_t - \hat{\mathbf{x}}_t).
$$

If the filter is consistent, $\varepsilon_t \sim \chi^2(n)$ where $n$ is the state dimension. Time-averaged NEES should be approximately $n$. Values significantly above $n$ indicate that the filter is overconfident (covariance is too small relative to the actual error). Values below $n$ indicate underconfidence.

Bar-Shalom et al. (2001) provide statistical tests for consistency based on the NEES distribution. Nüchter et al. (2007) applied NEES evaluation to laser-based SLAM systems, showing that poor data association and incorrect Jacobians both produce inconsistent filters with NEES significantly exceeding the state dimension.

### 9.2 Observability

An estimator can only improve on its prior if the observations contain information about the state. The unobservable subspace of the SLAM system corresponds to the degrees of freedom that no observation can determine: the absolute global position and orientation of the entire map-robot system. Rotating or translating the entire state (robot and all landmarks together) by any fixed amount does not change any observation. The SLAM system therefore has three unobservable degrees of freedom in 2D.

Huang et al. (2010) showed that standard EKF-SLAM, using Jacobians evaluated at the current state estimate, inadvertently renders only two of these three degrees of freedom unobservable. The linearised system gains spurious information about the global orientation. This causes the filter to become overconfident in heading, which then propagates to overconfidence in position and landmark estimates. The First Estimates Jacobian (FEJ) remedy evaluates Jacobians at the first estimate of each state element, ensuring the linearised observability structure matches the true one. Empirical results show FEJ-EKF maintains correct NEES values over long trajectories where standard EKF-SLAM becomes inconsistent.

---

## 10. Summary

The literature establishes several principles that guide the design of the system in this thesis.

**Maintain full correlations.** Dissanayake et al. (2001) proved that the full joint covariance is necessary for a consistent filter. Sparse or independent landmark models discard cross-correlations and produce overconfident estimates over time.

**Use the Joseph form.** The numerical fragility of the standard covariance update is well documented (Bierman, 1977). The Joseph form guarantees positive semi-definiteness at no significant computational cost.

**Use fixed sensor noise in covariance estimation.** Estimating noise from residuals produces optimistic covariances that worsen filter consistency. Hardware-specified noise gives calibrated uncertainty (Kay, 1993).

**Use TLS residuals for line fitting.** OLS and endpoint-based residuals are biased when all coordinates carry measurement noise. TLS via SVD gives the minimum-variance fit and is numerically stable (Golub & Van Loan, 2013).

**Enforce canonical constraints after every update.** The Hessian normal form requires $\rho \geq 0$. EKF updates can violate this. Without enforcement, sign-drifted landmarks fail data association and are re-entered as duplicates.

**Gate conservatively.** A false positive association is more damaging than a false negative (Neira & Tardós, 2001). The Mahalanobis gate at 95% chi-squared confidence is the primary filter for association decisions.

**Use geometry-aware covariance for submap alignment.** The system derives a pose covariance from SVD singular values and the spatial spread of matched wall points. This reflects direction-dependent reliability and provides calibrated weights when alignment corrections are fused into the EKF.

---

## References

Arras, K. O., Tomatis, N., Jensen, B. T., & Siegwart, R. (1998). Multisensor on-the-fly localization using laser and vision. *Robotics and Autonomous Systems*, 34(2–3), 131–143.

Bailey, T., & Durrant-Whyte, H. (2006). Simultaneous localisation and mapping (SLAM): Part II. *IEEE Robotics & Automation Magazine*, 13(3), 108–117.

Bar-Shalom, Y., & Fortmann, T. E. (1988). *Tracking and Data Association*. Academic Press.

Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001). *Estimation with Applications to Tracking and Navigation*. Wiley-Interscience.

Bierman, G. J. (1977). *Factorization Methods for Discrete Sequential Estimation*. Academic Press.

Castellanos, J. A., Montiel, J. M. M., Neira, J., & Tardós, J. D. (1999). The SPmap: A probabilistic framework for simultaneous localisation and map building. *IEEE Transactions on Robotics and Automation*, 15(5), 948–952.

Censi, A. (2007). An accurate closed-form estimate of ICP's covariance. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 3167–3172.

Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001). A solution to the simultaneous localisation and map building (SLAM) problem. *IEEE Transactions on Robotics and Automation*, 17(3), 229–241.

Durrant-Whyte, H., & Bailey, T. (2006). Simultaneous localisation and mapping: Part I. *IEEE Robotics & Automation Magazine*, 13(2), 99–110.

Gelb, A. (Ed.). (1974). *Applied Optimal Estimation*. MIT Press.

Golub, G. H., & Van Loan, C. F. (2013). *Matrix Computations* (4th ed.). Johns Hopkins University Press.

Grisetti, G., Stachniss, C., & Burgard, W. (2007). Improved techniques for grid mapping with Rao-Blackwellized particle filters. *IEEE Transactions on Robotics*, 23(1), 34–46.

Gutmann, J. S., & Konolige, K. (2000). Incremental mapping of large cyclic environments. *Proceedings of IEEE International Symposium on Computational Intelligence in Robotics and Automation (CIRA)*, 318–325.

Hess, W., Kohler, D., Rapp, H., & Andor, D. (2016). Real-time loop closure in 2D LIDAR SLAM. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 1271–1278.

Higham, N. J. (2002). *Accuracy and Stability of Numerical Algorithms* (2nd ed.). SIAM.

Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010). Observability-based rules for designing consistent EKF SLAM estimators. *The International Journal of Robotics Research*, 29(5), 502–528.

Jazwinski, A. H. (1970). *Stochastic Processes and Filtering Theory*. Academic Press.

Julier, S. J., & Uhlmann, J. K. (1997). A new extension of the Kalman filter to nonlinear systems. *Proceedings of SPIE — AeroSense: Signal Processing, Sensor Fusion, and Target Recognition VI*, 3068, 182–193.

Kaess, M., Johannsson, H., Roberts, R., Ila, V., Leonard, J. J., & Dellaert, F. (2012). iSAM2: Incremental smoothing and mapping using the Bayes tree. *The International Journal of Robotics Research*, 31(2), 216–235.

Kalman, R. E. (1960). A new approach to linear filtering and prediction problems. *Journal of Basic Engineering*, 82(1), 35–45.

Kay, S. M. (1993). *Fundamentals of Statistical Signal Processing: Estimation Theory*. Prentice Hall.

Kohlbrecher, S., Meyer, J., von Stryk, O., & Klingauf, U. (2011). A flexible and scalable SLAM system with full 3D motion estimation. *Proceedings of IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)*, 155–160.

Kümmerle, R., Grisetti, G., Strasdat, H., Konolige, K., & Burgard, W. (2011). g2o: A general framework for graph optimization. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 3607–3613.

Lu, F., & Milios, E. (1997). Globally consistent range scan alignment for environment mapping. *Autonomous Robots*, 4(4), 333–349.

Montemerlo, M., Thrun, S., Koller, D., & Wegbreit, B. (2002). FastSLAM: A factored solution to the simultaneous localization and mapping problem. *Proceedings of AAAI National Conference on Artificial Intelligence*, 593–598.

Murphy, K. (2000). Bayesian map learning in dynamic environments. *Advances in Neural Information Processing Systems (NIPS)*, 12.

Neira, J., & Tardós, J. D. (2001). Data association in stochastic mapping using the joint compatibility test. *IEEE Transactions on Robotics and Automation*, 17(6), 890–897.

Nocedal, J., & Wright, S. J. (2006). *Numerical Optimization* (2nd ed.). Springer.

Nguyen, V., Gächter, S., Martinelli, A., Tomatis, N., & Siegwart, R. (2005). A comparison of line extraction algorithms using 2D range data for indoor mobile robotics. *Autonomous Robots*, 23(2), 97–111.

Nüchter, A., Lingemann, K., Hertzberg, J., & Surmann, H. (2007). 6D SLAM — 3D mapping outdoor environments. *Journal of Field Robotics*, 24(8–9), 699–722.

Pavlidis, T., & Horowitz, S. L. (1974). Segmentation of plane curves. *IEEE Transactions on Computers*, C-23(8), 860–870.

Siegwart, R., & Nourbakhsh, I. R. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

Smith, R. C., & Cheeseman, P. (1987). On the representation and estimation of spatial uncertainty. *The International Journal of Robotics Research*, 5(4), 56–68.

Taylor, J. R. (1997). *An Introduction to Error Analysis* (2nd ed.). University Science Books.

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

Thrun, S., & Montemerlo, M. (2006). The GraphSLAM algorithm with applications to large-scale mapping of urban structures. *The International Journal of Robotics Research*, 25(5–6), 403–429.
