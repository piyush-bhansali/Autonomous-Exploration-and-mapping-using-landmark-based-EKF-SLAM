# Literature Review: Comparative Study of ICP-Based and Feature-Based SLAM with Frontier Exploration

## 1. Introduction

Simultaneous Localization and Mapping (SLAM) has emerged as a fundamental challenge in mobile robotics, enabling autonomous robots to navigate unknown environments without external positioning systems. The problem is characterized by a circular dependency: accurate localization requires a map, while accurate mapping requires knowing the robot's position (Smith & Cheeseman, 1987). Over the past four decades, researchers have developed two predominant approaches to address this challenge: **dense scan matching** using Iterative Closest Point (ICP) algorithms, and **sparse feature-based** methods using Extended Kalman Filters (EKF-SLAM). This literature review examines both approaches, their theoretical foundations, practical implementations, and comparative performance for autonomous exploration tasks.

## 2. Foundations of SLAM

### 2.1 Theoretical Framework

The mathematical foundations of SLAM were established through seminal contributions in the 1980s and 1990s. Smith and Cheeseman (1987) introduced the concept of representing spatial uncertainty using covariance matrices, recognizing that landmarks observed from uncertain vehicle locations inherit correlated errors. This insight about correlation structure became fundamental to all subsequent SLAM research. Smith et al. (1990) expanded this framework to robotic mapping applications, demonstrating that maintaining explicit correlations between robot pose and landmark positions is essential for building consistent maps.

Dissanayake et al. (2001) provided rigorous convergence proofs for SLAM, establishing three fundamental theorems:
1. Any submatrix of the map covariance decreases monotonically as more observations are incorporated
2. Landmark estimates become fully correlated in the limit
3. The lower bound on covariance is determined solely by initial vehicle uncertainty

These theoretical results demonstrated that SLAM is fundamentally solvable under appropriate conditions. Durrant-Whyte and Bailey (2006) presented comprehensive tutorial papers establishing the Bayesian filtering formulation for modern SLAM algorithms, emphasizing that landmark correlations are critical components and that minimizing or ignoring these correlations leads to filter inconsistency.

### 2.2 The SLAM Problem Formulation

The SLAM problem can be formulated probabilistically as computing the posterior distribution over the robot trajectory and map given sensor measurements and control inputs:

$$p(x_{0:t}, m | z_{1:t}, u_{1:t})$$

where $x_{0:t}$ represents the robot trajectory, $m$ is the map, $z_{1:t}$ are sensor observations, and $u_{1:t}$ are control inputs (Thrun et al., 2005). Different SLAM approaches make different assumptions about this posterior distribution and employ different representations for the map $m$.

## 3. ICP-Based SLAM Approaches

### 3.1 Iterative Closest Point Algorithm

The Iterative Closest Point (ICP) algorithm, introduced by Besl and McKay (1992), provides a method for registering point clouds through iterative optimization. The algorithm alternates between two steps:
1. **Correspondence**: Find nearest point pairs between source and target point clouds
2. **Transformation**: Compute the optimal rigid transformation minimizing point-pair distances

This alternating optimization converges to a local minimum of the alignment error. The algorithm has become foundational for scan matching in laser-based SLAM systems.

### 3.2. Evolution of Scan Matching Methods

Lu and Milios (1997) pioneered the use of scan matching for globally consistent SLAM. Their approach constructs a pose graph where nodes represent robot poses and edges represent spatial constraints from scan matching. This formulation enables batch optimization for global map consistency. However, their original approach required storing all scans and performing expensive batch optimization.

Gutmann and Konolige (2000) introduced incremental scan matching for online mapping. Their approach matches each new scan against a locally accumulated map, updating the robot pose incrementally. This enables real-time operation but can accumulate drift over long trajectories. They demonstrated successful mapping of office environments using 2D laser rangefinders.

Censi (2007) made an important theoretical contribution by deriving closed-form ICP covariance estimates. His work established that the Hessian matrix from ICP optimization represents the Fisher Information Matrix. For Gaussian noise, measurement covariance equals sensor noise variance times the inverse Hessian:

$$\Sigma_{ICP} = \sigma^2 H^{-1}$$

This formula separates sensor noise (from hardware specifications) from geometric uncertainty (captured by the Hessian). Censi (2008) extended this work to point-to-line metrics for 2D laser scans, showing improved convergence in structured environments.

### 3.3 Modern ICP Variants and Optimizations

Biber and Straßer (2003) developed the Normal Distributions Transform (NDT) as an alternative to ICP. NDT represents the environment using a grid of cells, each containing a normal distribution fitted to local points. Scan matching optimizes alignment by maximizing the likelihood of source points under the target distribution. This approach provides smooth, differentiable objective functions and often converges faster than ICP.

Segal et al. (2009) introduced Generalized-ICP, which estimates local surface geometry at each point and uses plane-to-plane rather than point-to-point metrics. This yields better convergence properties and higher accuracy, especially for planar environments like building interiors.

Rusinkiewicz and Levoy (2001) systematically analyzed ICP variants, examining different choices for point selection, correspondence matching, weighting, and rejection of outliers. Their experimental comparison demonstrated that combining proper algorithmic choices can improve ICP performance by orders of magnitude compared to naive implementations.

### 3.4 Dense SLAM Approaches

Grisetti et al. (2007) developed GMapping, combining scan matching with particle filters for efficient large-scale SLAM. Each particle maintains a map and trajectory hypothesis. Scan matching provides proposal distributions for particle resampling, significantly reducing the number of particles needed. This approach scales to building-sized environments while maintaining global consistency through loop closure.

Hess et al. (2016) introduced Google Cartographer, which combines scan matching with pose graph optimization for real-time SLAM without loop closure delays. Cartographer uses a multi-resolution approach: local submaps for scan matching and global optimization for consistency. The system can handle large environments (thousands of square meters) in real-time.

Kohlbrecher et al. (2011) developed Hector SLAM specifically for rescue robotics. Their approach uses scan matching on occupancy grid maps without odometry, making it robust for robots operating on irregular terrain or during collisions. The multi-resolution map representation enables efficient matching even with large displacements.

### 3.5 Advantages and Limitations of ICP-Based SLAM

**Advantages:**
- Dense representation captures fine geometric details
- Works in feature-poor environments (e.g., long corridors, warehouses)
- No feature extraction overhead
- Accurate short-term localization through dense point cloud alignment
- Robust to dynamic obstacles through outlier rejection

**Limitations:**
- Computational cost scales with point cloud density
- Memory requirements for storing dense maps
- Accumulates drift without explicit loop closure
- Sensitive to initial alignment (local minima)
- Uncertainty estimation less straightforward than probabilistic filters
- Difficulty in recognizing previously visited areas without additional place recognition

## 4. Feature-Based SLAM Approaches

### 4.1 Extended Kalman Filter SLAM

The Extended Kalman Filter provides a probabilistic framework maintaining a joint Gaussian distribution over robot pose and landmark positions. Thrun et al. (2005) provide comprehensive coverage of EKF-SLAM in "Probabilistic Robotics". The EKF maintains a state vector containing robot pose and landmark parameters:

$$\mathbf{x} = [x_r, y_r, \theta_r, x_{l_1}, y_{l_1}, ..., x_{l_N}, y_{l_N}]^T$$

and a full covariance matrix $\mathbf{P}$ capturing uncertainty and correlations.

The EKF operates through prediction and update cycles:

**Prediction** (using odometry):
$$\mathbf{x}_t^- = f(\mathbf{x}_{t-1}, \mathbf{u}_t)$$
$$\mathbf{P}_t^- = F_t \mathbf{P}_{t-1} F_t^T + G_t Q_t G_t^T$$

**Update** (using landmark observations):
$$K_t = \mathbf{P}_t^- H_t^T (H_t \mathbf{P}_t^- H_t^T + R_t)^{-1}$$
$$\mathbf{x}_t = \mathbf{x}_t^- + K_t (\mathbf{z}_t - h(\mathbf{x}_t^-))$$
$$\mathbf{P}_t = (I - K_t H_t) \mathbf{P}_t^-$$

The key advantage is explicit uncertainty representation with theoretically grounded confidence estimates for every part of the map.

### 4.2 Computational Complexity and Scalability

The computational bottleneck of EKF-SLAM is the update step, which requires $O(n^2)$ operations for $n$ landmarks (Dissanayake et al., 2001). Memory requirements scale as $O(n^2)$ for storing the full covariance matrix. This quadratic scaling limits traditional EKF-SLAM to hundreds of landmarks.

Montemerlo et al. (2002) introduced FastSLAM, which uses Rao-Blackwellized particle filters to achieve $O(M \log N)$ complexity where $M$ is the number of particles and $N$ is the number of landmarks. Each particle maintains an independent estimate of landmark positions conditioned on its trajectory hypothesis. This factorization exploits conditional independence in the SLAM problem.

Eade and Drummond (2006) developed Monocular SLAM using inverse depth parameterization for visual features. Their approach scales to thousands of features in real-time by maintaining only recently observed features and marginalizing out old ones.

### 4.3 Filter Consistency and Observability

A critical discovery came from Huang et al. (2010), who found that standard EKF-SLAM suffers from inconsistency where the filter underestimates true uncertainty. This overconfidence can lead to divergence. The root cause is a mismatch in observability: the true nonlinear SLAM system has three unobservable degrees of freedom (global position and orientation), while the linearized EKF system has only two.

Huang et al. proposed First Estimates Jacobian (FEJ) to restore correct observability. Instead of using current estimates for Jacobian computation, FEJ uses first-ever estimates, preserving the three-dimensional unobservable subspace. Experimental results showed standard EKF violates 95% confidence bounds ~50% of the time, while FEJ-EKF respects confidence bounds ~95% of the time.

### 4.4 Feature Extraction and Representation

Feature extraction from laser range data has been extensively studied. Nguyen et al. (2005) compared line extraction algorithms, finding split-and-merge provides the best balance of speed, accuracy, and simplicity for structured indoor environments. The split-and-merge algorithm (Pavlidis & Horowitz, 1974) recursively subdivides point sequences based on perpendicular distance thresholds, achieving $O(n \log n)$ complexity.

Siegwart and Nourbakhsh (2011) discuss feature representation choices. The Hessian normal form for lines, $\rho = x \cos\alpha + y \sin\alpha$, avoids redundancy issues with other parameterizations. Arras et al. (2001) demonstrated that line-based features achieve 2-3× better localization accuracy than point-based features in structured environments with similar computational cost.

Corner detection provides complementary information, as corners constrain position in both dimensions simultaneously while walls primarily constrain perpendicular direction. Combining walls and corners improves observability (Siegwart & Nourbakhsh, 2011).

### 4.5 Data Association and the Correspondence Problem

Data association—matching observations to existing landmarks—is critical for feature-based SLAM. Incorrect associations corrupt the map irrecoverably (Neira & Tardós, 2001). The standard approach uses gating with Mahalanobis distance:

$$d_M = \sqrt{(\mathbf{z} - \hat{\mathbf{z}})^T S^{-1} (\mathbf{z} - \hat{\mathbf{z}})}$$

where $S$ is the innovation covariance. Bailey et al. (2006) provide comprehensive treatment of data association approaches.

Joint Compatibility Branch and Bound (JCBB) from Neira and Tardós (2001) evaluates joint compatibility of observation-landmark pairs to avoid inconsistent associations. However, JCBB has exponential worst-case complexity. Nearest neighbor with validation gates provides practical real-time performance at the cost of occasional incorrect associations (Bar-Shalom & Li, 1995).

### 4.6 Advantages and Limitations of Feature-Based SLAM

**Advantages:**
- Explicit probabilistic uncertainty representation
- Theoretically grounded confidence estimates
- Efficient representation for structured environments
- Natural loop closure through landmark re-observation
- Bounded memory requirements (scales with number of features, not measurements)
- Well-studied theoretical properties and convergence guarantees

**Limitations:**
- Requires reliable feature extraction
- Struggles in feature-poor environments
- Computational cost scales with number of landmarks
- Linearization errors can cause inconsistency
- Vulnerable to incorrect data association
- Discrete observations create gaps between feature detections

## 5. Comparative Studies of SLAM Approaches

### 5.1 Hybrid and Comparative Approaches

Konolige and Chou (1999) explored combining scan matching with feature-based localization, showing benefits over either method alone. Scan matching provides dense, accurate short-term estimates while feature-based methods provide sparse, consistent long-term estimates.

Hähnel et al. (2003) developed FastSLAM with scan matching for large-scale cyclic environments. The hybrid approach addresses complementary strengths: ICP handles local registration while landmarks enable global consistency through loop closures.

Steux and El Hamzaoui (2010) compared tinySLAM (grid-based scan matching) with CoreSLAM (particle filter with scan matching), finding that algorithm choice significantly impacts performance based on environment characteristics and computational constraints.

### 5.2 Benchmark Datasets and Evaluation Metrics

Kümmerle et al. (2009) introduced evaluation metrics for SLAM systems:
- **Absolute Trajectory Error (ATE)**: Measures global consistency
- **Relative Pose Error (RPE)**: Measures local accuracy
- **Map quality metrics**: Precision, recall for occupancy grids

The Radish dataset repository provides standard benchmarks for comparing SLAM algorithms (Howard & Roy, 2003). The Intel Research Lab and MIT Killian Court datasets are widely used for algorithm comparison.

### 5.3 Performance Comparison Factors

Cadena et al. (2016) provide a comprehensive survey identifying key comparison dimensions:

**Accuracy**: Feature-based methods often achieve better global consistency through explicit loop closure. ICP-based methods provide superior local accuracy through dense matching.

**Computational efficiency**: Scan matching scales with point cloud size. Feature extraction and EKF updates scale with number of features. Optimal choice depends on environment complexity.

**Robustness**: ICP performs better in feature-poor environments. Feature-based methods handle larger loop closures through landmark recognition.

**Memory requirements**: Dense methods require storing point clouds or occupancy grids. Sparse methods store only feature parameters.

**Uncertainty quantification**: EKF-SLAM provides explicit covariance. ICP uncertainty estimation requires additional computation (Censi, 2007).

## 6. Frontier-Based Exploration

### 6.1 Frontier Detection and Selection

Yamauchi (1997) introduced frontier-based exploration as a practical autonomous exploration strategy. Frontiers are boundaries between free space (explored) and unknown space (unexplored). The robot selects frontiers as navigation goals to incrementally expand its map. This greedy approach is computationally efficient and produces reasonable coverage.

Zelinsky et al. (1993) developed distance transforms for efficient frontier identification in occupancy grid maps. The distance transform computes shortest paths to frontiers, enabling cost-based frontier selection.

### 6.2 Information-Theoretic Exploration

Feder et al. (1999) pioneered information-theoretic exploration, where robot motion maximizes information gain about both pose and map. For Gaussian distributions, information gain relates to covariance reduction through mutual information.

Stachniss et al. (2005) extended this to Rao-Blackwellized particle filters, computing information gain for each particle trajectory. The robot selects actions maximizing expected information gain, enabling active exploration that deliberately improves map quality.

Carrillo et al. (2012) compared uncertainty criteria for active SLAM:
- **D-optimality**: Minimizes determinant of covariance (maximizes information)
- **A-optimality**: Minimizes trace of covariance (minimizes average variance)
- **E-optimality**: Minimizes maximum eigenvalue (bounds worst-case uncertainty)

Their experimental comparison revealed trade-offs between criteria for various environments.

### 6.3 Hybrid Frontier-Information Approaches

Combining frontier detection with information-theoretic evaluation selects frontiers maximizing information gain rather than simple proximity. González-Baños and Latombe (2002) developed visibility-based exploration considering sensor range and occlusions.

Sim and Roy (2005) formalized active SLAM as balancing exploration (reducing map uncertainty) and exploitation (using known map information). Optimal strategy depends on mission objectives and time constraints.

### 6.4 Multi-Robot Exploration

Burgard et al. (2000) extended frontier exploration to multi-robot systems, where robots coordinate to avoid redundant coverage while maximizing collective exploration efficiency. Cost-based frontier assignment considers both frontier utility and robot travel cost.

Simmons et al. (2000) developed coordination strategies for heterogeneous robot teams with different sensing capabilities. Their approach allocates frontiers based on robot capabilities and current uncertainty.

## 7. Uncertainty Quantification in SLAM

### 7.1 Theoretical Foundations

The Fisher Information Matrix quantifies measurement information about unknown parameters. The Cramér-Rao bound states that parameter covariance cannot be smaller than the inverse Fisher Information Matrix (Kay, 1993). For Gaussian measurement models:

$$\mathcal{I} = H^T R^{-1} H$$

where $H$ is the measurement Jacobian and $R$ is measurement noise covariance. The parameter covariance bound is $\Sigma \geq \mathcal{I}^{-1}$.

### 7.2 Hessian-Based Covariance Estimation

Censi's (2007) ICP covariance formula provides practical uncertainty quantification for scan matching:

$$\Sigma_{ICP} = \sigma^2 \left(\sum_{i} J_i^T J_i\right)^{-1}$$

where $J_i$ is the Jacobian of the point-to-line distance for the $i$-th correspondence. Using fixed sensor noise $\sigma^2$ from hardware specifications avoids optimism—estimating noise from residuals artificially reduces uncertainty with more measurement points.

The Hessian captures how environment geometry constrains pose estimates:
- Corners provide constraints in all directions → small covariance
- Corridors constrain only perpendicular direction → large covariance in unconstrained directions
- Open spaces provide weak constraints → large covariance

### 7.3 Covariance Propagation and Fusion

Barfoot and Furgale (2014) developed rigorous covariance propagation for SE(3) transformations in Lie groups. Their theoretical framework supports proper uncertainty handling in 3D SLAM.

When fusing ICP and landmark measurements, the EKF update properly combines information sources:

$$\Sigma_{posterior}^{-1} = \Sigma_{prior}^{-1} + H_{ICP}^T R_{ICP}^{-1} H_{ICP} + H_{landmark}^T R_{landmark}^{-1} H_{landmark}$$

This additivity of information matrices enables principled sensor fusion maintaining filter consistency.

## 8. Research Gap and Thesis Contribution

Despite extensive research on both ICP-based and feature-based SLAM, comprehensive comparative studies evaluating both approaches under identical exploration scenarios remain limited. Most existing comparisons focus on localization accuracy or mapping quality in isolation, without considering the coupled effects of exploration strategy and mapping approach.

This thesis addresses this gap by conducting a systematic comparative study of:

1. **ICP-based SLAM** using dense scan matching with Hessian-based uncertainty quantification
2. **Feature-based SLAM** using EKF with wall and corner landmarks in Hessian normal form
3. **Frontier-based autonomous exploration** applied to both approaches under identical conditions

The comparison evaluates:
- Localization accuracy (Absolute Trajectory Error)
- Map quality (precision, completeness)
- Computational efficiency (runtime, memory)
- Exploration efficiency (coverage rate, path length)
- Robustness (success rate in different environments)
- Uncertainty calibration (consistency of confidence estimates)

By implementing both approaches within a unified framework and testing with identical exploration scenarios, this research provides empirical evidence for selecting appropriate SLAM approaches based on application requirements and environmental characteristics.

## 9. Conclusion

The literature review establishes that both ICP-based and feature-based SLAM approaches have solid theoretical foundations and extensive practical validation. ICP-based methods excel in providing dense, accurate local maps and work well in feature-poor environments. Feature-based methods provide efficient sparse representations with explicit uncertainty quantification and natural loop closure capabilities.

Frontier-based exploration offers a practical strategy for autonomous mapping, with well-established algorithms for frontier detection and selection. Recent information-theoretic extensions enable active exploration that explicitly optimizes map quality.

This thesis contributes a systematic comparative evaluation of these approaches under identical exploration conditions, providing empirical guidance for practitioners selecting SLAM strategies for autonomous mobile robotics applications.

## References

Arras, K. O., Castellanos, J. A., Schilt, M., & Siegwart, R. (2001). Feature-based multi-hypothesis localization and tracking using geometric constraints. *Robotics and Autonomous Systems*, 44(1), 41-53.

Bailey, T., & Durrant-Whyte, H. (2006). Simultaneous localization and mapping (SLAM): Part II. *IEEE Robotics & Automation Magazine*, 13(3), 108-117.

Bar-Shalom, Y., & Li, X. R. (1995). *Multitarget-multisensor tracking: Principles and techniques*. YBS Publishing.

Barfoot, T. D., & Furgale, P. T. (2014). Associating uncertainty with three-dimensional poses for use in estimation problems. *IEEE Transactions on Robotics*, 30(3), 679-693.

Besl, P. J., & McKay, N. D. (1992). Method for registration of 3-D shapes. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239-256.

Biber, P., & Straßer, W. (2003). The normal distributions transform: A new approach to laser scan matching. *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2743-2748.

Burgard, W., Moors, M., Stachniss, C., & Schneider, F. E. (2000). Coordinated multi-robot exploration. *IEEE Transactions on Robotics*, 21(3), 376-386.

Cadena, C., Carlone, L., Carrillo, H., Latif, Y., Scaramuzza, D., Neira, J., Reid, I., & Leonard, J. J. (2016). Past, present, and future of simultaneous localization and mapping: Toward the robust-perception age. *IEEE Transactions on Robotics*, 32(6), 1309-1332.

Carrillo, H., Reid, I., & Castellanos, J. A. (2012). On the comparison of uncertainty criteria for active SLAM. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 2080-2087.

Censi, A. (2007). An accurate closed-form estimate of ICP's covariance. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 3167-3172.

Censi, A. (2008). An ICP variant using a point-to-line metric. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 19-25.

Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001). A solution to the simultaneous localization and map building (SLAM) problem. *IEEE Transactions on Robotics and Automation*, 17(3), 229-241.

Durrant-Whyte, H., & Bailey, T. (2006). Simultaneous localization and mapping: Part I. *IEEE Robotics & Automation Magazine*, 13(2), 99-110.

Eade, E., & Drummond, T. (2006). Scalable monocular SLAM. *Proceedings of IEEE Computer Society Conference on Computer Vision and Pattern Recognition (CVPR)*, 469-476.

Feder, H. J. S., Leonard, J. J., & Smith, C. M. (1999). Adaptive mobile robot navigation and mapping. *The International Journal of Robotics Research*, 18(7), 650-668.

González-Baños, H. H., & Latombe, J. C. (2002). Navigation strategies for exploring indoor environments. *The International Journal of Robotics Research*, 21(10-11), 829-848.

Grisetti, G., Stachniss, C., & Burgard, W. (2007). Improved techniques for grid mapping with Rao-Blackwellized particle filters. *IEEE Transactions on Robotics*, 23(1), 34-46.

Gutmann, J. S., & Konolige, K. (2000). Incremental mapping of large cyclic environments. *Proceedings of IEEE International Symposium on Computational Intelligence in Robotics and Automation (CIRA)*, 318-325.

Hähnel, D., Burgard, W., Fox, D., & Thrun, S. (2003). An efficient FastSLAM algorithm for generating maps of large-scale cyclic environments from raw laser range measurements. *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 206-211.

Hess, W., Kohler, D., Rapp, H., & Andor, D. (2016). Real-time loop closure in 2D LIDAR SLAM. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 1271-1278.

Howard, A., & Roy, N. (2003). The robotics data set repository (Radish). http://radish.sourceforge.net/

Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010). Observability-based rules for designing consistent EKF SLAM estimators. *The International Journal of Robotics Research*, 29(5), 502-528.

Kay, S. M. (1993). *Fundamentals of statistical signal processing: Estimation theory*. Prentice Hall.

Kohlbrecher, S., Meyer, J., von Stryk, O., & Klingauf, U. (2011). A flexible and scalable SLAM system with full 3D motion estimation. *Proceedings of IEEE International Symposium on Safety, Security, and Rescue Robotics (SSRR)*, 155-160.

Konolige, K., & Chou, K. (1999). Markov localization using correlation. *Proceedings of International Joint Conference on Artificial Intelligence (IJCAI)*, 1154-1159.

Kümmerle, R., Steder, B., Dornhege, C., Ruhnke, M., Grisetti, G., Stachniss, C., & Kleiner, A. (2009). On measuring the accuracy of SLAM algorithms. *Autonomous Robots*, 27(4), 387-407.

Lu, F., & Milios, E. (1997). Globally consistent range scan alignment for environment mapping. *Autonomous Robots*, 4(4), 333-349.

Montemerlo, M., Thrun, S., Koller, D., & Wegbreit, B. (2002). FastSLAM: A factored solution to the simultaneous localization and mapping problem. *Proceedings of AAAI National Conference on Artificial Intelligence*, 593-598.

Neira, J., & Tardós, J. D. (2001). Data association in stochastic mapping using the joint compatibility test. *IEEE Transactions on Robotics and Automation*, 17(6), 890-897.

Nguyen, V., Gächter, S., Martinelli, A., Tomatis, N., & Siegwart, R. (2005). A comparison of line extraction algorithms using 2D range data for indoor mobile robotics. *Autonomous Robots*, 23(2), 97-111.

Pavlidis, T., & Horowitz, S. L. (1974). Segmentation of plane curves. *IEEE Transactions on Computers*, C-23(8), 860-870.

Rusinkiewicz, S., & Levoy, M. (2001). Efficient variants of the ICP algorithm. *Proceedings of International Conference on 3D Digital Imaging and Modeling (3DIM)*, 145-152.

Segal, A., Haehnel, D., & Thrun, S. (2009). Generalized-ICP. *Proceedings of Robotics: Science and Systems (RSS)*.

Siegwart, R., & Nourbakhsh, I. R. (2011). *Introduction to autonomous mobile robots* (2nd ed.). MIT Press.

Sim, R., & Roy, N. (2005). Global A-optimal robot exploration in SLAM. *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, 661-666.

Simmons, R., Apfelbaum, D., Burgard, W., Fox, D., Moors, M., Thrun, S., & Younes, H. (2000). Coordination for multi-robot exploration and mapping. *Proceedings of AAAI National Conference on Artificial Intelligence*, 852-858.

Smith, R., Self, M., & Cheeseman, P. (1990). Estimating uncertain spatial relationships in robotics. In *Autonomous Robot Vehicles* (pp. 167-193). Springer.

Smith, R. C., & Cheeseman, P. (1987). On the representation and estimation of spatial uncertainty. *The International Journal of Robotics Research*, 5(4), 56-68.

Stachniss, C., Grisetti, G., & Burgard, W. (2005). Information gain-based exploration using Rao-Blackwellized particle filters. *Proceedings of Robotics: Science and Systems (RSS)*, 65-72.

Steux, B., & El Hamzaoui, O. (2010). tinySLAM: A SLAM algorithm in less than 200 lines C-language program. *Proceedings of International Conference on Control, Automation, Robotics and Vision (ICARCV)*, 1975-1979.

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic robotics*. MIT Press.

Yamauchi, B. (1997). A frontier-based approach for autonomous exploration. *Proceedings of IEEE International Symposium on Computational Intelligence in Robotics and Automation (CIRA)*, 146-151.

Zelinsky, A., Jarvis, R. A., Byrne, J. C., & Yuta, S. (1993). Planning paths of complete coverage of an unstructured environment by a mobile robot. *Proceedings of International Conference on Advanced Robotics*, 533-538.
