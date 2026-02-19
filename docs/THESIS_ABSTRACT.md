# Thesis Abstract

## Feature-Based EKF-SLAM for Autonomous Indoor Mapping on TurtleBot3

Simultaneous localisation and mapping (SLAM) is a core problem in mobile robotics. A robot must build a map of an unknown environment while tracking its own position within that map. This thesis develops and evaluates a feature-based SLAM system for autonomous indoor navigation on the TurtleBot3 Waffle Pi platform.

The system uses a 360° 2D LiDAR as its primary sensor. Raw range scans are processed to extract geometric landmarks — walls and corners — from the structured geometry of indoor spaces. Walls are parameterised in Hessian normal form $(\rho, \alpha)$, where $\rho \geq 0$ is the perpendicular distance from the origin to the wall line and $\alpha \in [-\pi, \pi]$ is the angle of the outward normal. Corners are stored as Cartesian coordinates $(x, y)$. This parameterisation is compact, non-redundant, and free of the endpoint sensitivity that affects other line representations.

Line segments are extracted by incremental growing followed by an adjacent-segment merge step. The residual used in both stages is the maximum perpendicular distance from any point to the total least squares (TLS) best-fit line, computed via singular value decomposition over all candidate points. This is more robust than an endpoint-to-endpoint residual, which can be dominated by noise at grazing-angle scan endpoints. The Hessian parameters and covariances are derived from the same TLS fit. Wall covariance follows from the Cramér–Rao lower bound,

$$\mathrm{Cov}(\rho,\,\alpha) = \sigma^2\,\mathbf{A}^{-1},$$

where $\mathbf{A} = \sum_i \mathbf{J}_i^\top \mathbf{J}_i$ is the Fisher information matrix accumulated from all supporting scan points, $\mathbf{J}_i = [1,\; p_x^i \sin\alpha - p_y^i \cos\alpha]$ is the per-point residual Jacobian, and $\sigma = 0.01\,\mathrm{m}$ is the LiDAR range noise from the TurtleBot3 LDS-01 specification. Corner covariance is propagated analytically from the two parent wall covariances through the Jacobian of the line-intersection formula, using first-order error propagation.

State estimation is performed by an Extended Kalman Filter (EKF). The state vector contains the robot pose $(x, y, \theta)$ and all landmark parameters jointly, so that a single observation updates both the observed landmark and every other landmark through the cross-covariance structure. At each scan, odometry increments propagate the state through a midpoint-integration motion model with motion-scaled process noise. Landmarks are matched to observations using nearest-neighbour association with Mahalanobis-distance gating at the 95% chi-squared confidence level ($\chi^2_{0.05}(2) = 5.99$ for 2-DOF features). After each EKF update, wall parameters are normalised to enforce $\rho \geq 0$, preventing sign drift that would cause a previously observed wall to fail the association gate and be re-entered as a duplicate.

Global consistency is maintained by a submap-stitching layer. Every $N$ scans the FeatureMap point cloud (in the EKF map frame) is registered against the growing global map using **feature-based SVD alignment** of shared wall landmarks. The alignment covariance is derived from the SVD singular values and the spatial spread of the matched points,

$$\mathbf{R}_{\mathrm{SVD}} = \mathrm{diag}\!\left(\frac{\sigma^2}{\Sigma_1},\; \frac{\sigma^2}{\Sigma_1},\; \frac{\sigma^2}{N\bar{d}^2}\right),$$

making it geometry-aware: low uncertainty along well-constrained directions and high uncertainty in degenerate directions such as along a featureless corridor. The SVD pose correction is injected into the EKF as a direct pose observation, propagating the correction to all correlated landmarks.

The system is implemented as a ROS 2 node in Python, tested on TurtleBot3 in structured indoor environments. Performance is assessed using the absolute trajectory error (ATE), landmark map accuracy, point cloud quality, and filter consistency via the normalised estimation error squared (NEES).
