# Literature Review

## Introduction

Simultaneous Localization and Mapping has been a fundamental problem in mobile robotics for decades. The challenge involves a robot building a map of an unknown environment while simultaneously determining its own location within that map. This creates a circular dependency where the robot needs to know its location to build an accurate map. At the same time, it needs an accurate map to localize itself. Solving this problem is essential for autonomous robots operating in indoor environments without external positioning systems like GPS.

## Foundations of SLAM

The theoretical foundations of SLAM were established through several seminal contributions in the 1980s and 1990s. Smith and Cheeseman introduced the concept of representing spatial uncertainty using covariance matrices in their 1987 paper. They recognized that landmarks observed from uncertain vehicle locations inherit correlated errors through the common vehicle uncertainty. This insight about correlation structure became the cornerstone of all subsequent SLAM research. Smith and colleagues expanded this framework to robotic mapping applications in 1990. They demonstrated that maintaining explicit correlations between the robot pose and landmark positions is essential for building consistent maps.

Dissanayake and colleagues provided the first rigorous convergence proof for SLAM in 2001. They established three fundamental theorems that govern the behavior of SLAM systems. The first theorem states that any submatrix of the map covariance decreases monotonically as more observations are incorporated. The second theorem proves that landmark estimates become fully correlated in the limit. The third theorem shows that the lower bound on covariance is determined solely by initial vehicle uncertainty. These theoretical results demonstrated that SLAM is fundamentally solvable under the right conditions.

Durrant-Whyte and Bailey presented comprehensive tutorial papers in 2006 that established the Bayesian filtering formulation for modern SLAM algorithms. They emphasized that landmark correlations are the critical component of SLAM. Minimizing or ignoring these correlations leads to filter inconsistency. Their work also covered computational complexity issues and data association challenges. Bailey and Durrant-Whyte continued this tutorial series by addressing loop closure detection and various SLAM implementations. These papers remain standard references for researchers entering the field.

## Extended Kalman Filter SLAM

The Extended Kalman Filter provides a probabilistic framework for SLAM that maintains a joint Gaussian distribution over robot pose and all landmark positions. Thrun and colleagues covered the EKF-SLAM algorithm extensively in their 2005 textbook "Probabilistic Robotics". Chapter 10 of their book provides detailed derivations of the prediction and update steps. The EKF maintains a full covariance matrix that captures both uncertainty in individual elements and correlations between the robot and landmarks. This correlation structure enables the filter to update all landmark positions when the robot pose is corrected.

The key advantage of EKF-SLAM is its explicit uncertainty representation. The full covariance matrix provides theoretically grounded confidence estimates for every part of the map. The filter operates incrementally, making it suitable for real-time applications. However, EKF-SLAM has computational limitations. The update step requires matrix operations that scale as O(n³) for n landmarks. The algorithm also relies on linearization of nonlinear motion and observation models. These linearization errors can accumulate over time and lead to inconsistency.

Huang and Dissanayake analyzed convergence and consistency properties of EKF-SLAM in their 2007 paper. They identified conditions under which the filter remains consistent and demonstrated how linearization errors affect performance. Their work provided important insights into the limitations of the standard EKF formulation. This led to subsequent research on improving EKF consistency through better linearization strategies.

## Filter Consistency and Observability

A critical discovery about EKF-SLAM came from Huang and colleagues in 2010. They found that standard EKF-SLAM implementations suffer from inconsistency where the filter underestimates its true uncertainty. This overconfidence can lead to divergence where the filter rejects correct measurements. The root cause is a mismatch in observability properties between the true nonlinear system and the linearized EKF system.

The nonlinear SLAM system has three unobservable degrees of freedom corresponding to global position and orientation. Observations only provide relative measurements. Therefore, a robot can build a consistent map but cannot determine its absolute pose in the world without external reference. The standard EKF uses current state estimates for linearization. This causes the linearized system to have only two unobservable directions instead of three. The missing global orientation component creates spurious information gain that leads to overconfidence.

Huang and colleagues proposed the First Estimates Jacobian solution to restore correct observability. Instead of using current estimates for Jacobian computation, the FEJ approach uses first-ever estimates or prior estimates. This preserves the three-dimensional unobservable subspace and improves consistency. They demonstrated through experiments that standard EKF violates 95% confidence bounds approximately 50% of the time. In contrast, FEJ-EKF respects confidence bounds approximately 95% of the time as expected for a consistent filter. This work fundamentally changed how researchers think about EKF-SLAM implementation.

## Process Noise and Motion Models

Proper modeling of process noise is essential for maintaining filter consistency. Thrun and colleagues discuss odometry error models in Section 5.4 of their book. They explain that odometry errors have physical sources including wheel slip, encoder quantization, ground contact variation, and kinematic model errors. A key observation is that odometry error is proportional to distance traveled. This has been validated experimentally by multiple research groups.

Martinelli and colleagues conducted odometry calibration experiments on real robots in 2007. They found that odometry error follows a model where uncertainty equals a distance-proportional coefficient times distance traveled plus a constant offset. The distance-proportional coefficient typically ranges from 0.01 to 0.05 for wheeled robots. The constant offset represents sensor noise floor. This empirical evidence confirms that uncertainty grows with motion.

The process noise model should therefore combine motion-dependent and minimum floor components. The motion-dependent term represents distance-proportional uncertainty due to cumulative wheel slip and encoder drift. The minimum noise floor prevents singular covariance when the robot is stationary. It also accounts for encoder jitter, IMU bias drift, and numerical precision limits. Bar-Shalom and colleagues discuss process noise modeling extensively in their 2001 textbook on estimation and tracking. They provide guidance on when constant noise is acceptable versus when motion-scaled noise is necessary.

Bailey and colleagues studied the relationship between process noise tuning and filter consistency in 2006. They used Normalized Estimation Error Squared testing to validate filter consistency. Their experimental methodology provides a framework for selecting process noise parameters. Proper tuning ensures that predicted uncertainty accurately reflects true estimation error. This maintains filter consistency over long operation periods.

## Iterative Closest Point and Scan Matching

The Iterative Closest Point algorithm was introduced by Besl and McKay in 1992 for registering three-dimensional shapes. The algorithm iteratively finds correspondences between points in two point clouds. It then computes the optimal rigid transformation that minimizes the sum of squared distances. This alternating optimization converges to a local minimum. ICP has become a foundational algorithm for point cloud registration with numerous variants developed for different applications.

Censi made an important contribution in 2007 by deriving a closed-form estimate of ICP covariance. His work provides the mathematical foundation for Hessian-based uncertainty quantification used in this thesis. The key insight is that the Hessian matrix from the ICP optimization represents the Fisher Information Matrix. For Gaussian noise, the measurement covariance equals the sensor noise variance times the inverse of the Hessian. This formula separates sensor noise from geometric uncertainty. The sensor noise comes from hardware specifications. The geometric uncertainty depends on environment structure through the Hessian.

Censi extended this work in 2008 with an ICP variant using point-to-line metrics for two-dimensional laser scans. This variant shows improved convergence for structured environments. The covariance formulation extends naturally to different ICP variants. Prakhya and colleagues extended Censi's work to three dimensions in 2015. They also developed computational efficiency improvements for real-time applications.

Grisetti and colleagues integrated scan matching with covariance estimation into particle filter SLAM in 2007. Their improved techniques for grid mapping demonstrated the practical value of uncertainty-aware scan matching. The covariance estimates enable proper fusion of scan matching results with other measurements in a Bayesian framework. This maintains filter consistency when combining dense and sparse information sources.

## Hessian-Based Uncertainty Quantification

A unified approach to uncertainty quantification emerges from information theory and estimation theory. The Fisher Information Matrix quantifies how much information measurements provide about unknown parameters. The Cramér-Rao bound states that parameter covariance cannot be smaller than the inverse of the Fisher Information Matrix. For Gaussian measurement models, the information matrix equals the Hessian of the least-squares cost function divided by sensor noise variance.

This theoretical framework applies consistently across different measurement types. For wall landmarks in Hessian form, the information matrix comes from the Jacobian of point-to-line residuals. For corner landmarks in Cartesian coordinates, the information matrix can be computed from point scatter with principal component analysis. For ICP pose measurements, the information matrix accumulates from all point correspondences. In each case, the measurement covariance equals sensor noise variance times the inverse of the information matrix.

Using fixed sensor noise from hardware specifications avoids the optimism problem. If noise is estimated from residuals, more measurement points artificially reduce estimated noise. This makes the filter overconfident. Using fixed sensor noise ensures that uncertainty properly reflects environment geometry rather than point density. The Hessian captures how environment structure constrains pose estimates. Corners provide constraints in all directions, yielding small covariance. Corridors provide constraints in only one direction, yielding large covariance in unconstrained directions.

Barfoot and Furgale developed theoretical foundations for pose uncertainty in Lie groups in 2014. Their work provides rigorous covariance propagation for SE(3) transformations. This extends Hessian-based methods to three-dimensional rotation and translation. Their theoretical framework supports the registration uncertainty methods used in this thesis.

## Landmark Extraction and Feature-Based Methods

Feature extraction from laser range data has been studied extensively for mobile robot navigation. Nguyen and colleagues conducted a comprehensive comparison of line extraction algorithms in 2005. They evaluated split-and-merge, incremental, Hough transform, and RANSAC approaches using two-dimensional laser rangefinders for indoor robotics. Their experimental results showed that split-and-merge provides the best balance of speed, accuracy, and simplicity for structured environments.

The split-and-merge algorithm was originally introduced by Pavlidis and Horowitz in 1974 for image processing. It recursively subdivides point sequences based on perpendicular distance thresholds. The algorithm is fast with O(n log n) complexity and robust to noise. It requires few tuning parameters compared to alternatives. The incremental approach from Duda and Hart in 1973 is faster but more sensitive to point order. The Hough transform from Duda and Hart in 1972 is robust to occlusions but computationally expensive. RANSAC from Fischler and Bolles in 1981 provides excellent outlier rejection but is non-deterministic.

Siegwart and Nourbakhsh discuss feature selection for localization in their 2011 textbook. They explain that line features provide both orientation and distance constraints. The Hessian normal form representation avoids redundancy issues with other line parameterizations. Arras and colleagues demonstrated in 2001 that line-based features achieve two to three times better localization accuracy than point-based features in structured environments. The computational cost is similar between approaches. Therefore, line features are preferred for indoor robot navigation.

Corner detection provides complementary information to wall detection. Corners occur naturally at wall intersections and furniture edges. They provide high information density because they constrain position in both x and y directions simultaneously. Walls constrain position primarily in the direction perpendicular to the wall. Combining walls and corners provides better observability than using either feature type alone.

## Hybrid SLAM Approaches

Combining different SLAM methods can leverage complementary strengths. Konolige and Chou explored markov localization using correlation in 1999. They combined scan matching with feature-based localization for mobile robots. This early hybrid approach showed benefits over using either method alone. Scan matching provides dense, accurate short-term estimates. Feature-based methods provide sparse, consistent long-term estimates with explicit loop closure capability.

Hähnel and colleagues developed an efficient FastSLAM algorithm for large-scale cyclic environments in 2003. Their algorithm combines scan matching with landmark detection within a particle filter framework. This hybrid dense-sparse mapping strategy enables efficient mapping of large environments. The scan matching handles local registration while landmarks enable global consistency through loop closures.

The hybrid approach addresses fundamental trade-offs in SLAM design. Pure ICP-based methods accumulate drift over long distances because they lack explicit landmark tracking. They also require substantial memory for storing dense point clouds. Pure landmark-based methods struggle in feature-poor environments. They suffer from discrete observations that create noisy estimates between detections. Hybrid methods use ICP for dense, accurate short-term corrections. They use landmarks for sparse, consistent long-term structure. The full covariance enables uncertainty-aware decisions about when to trust each information source.

## Information Theory in SLAM

Applying information theory to robotics was pioneered by Feder and colleagues in 1999. They introduced the concept of information-theoretic exploration for autonomous robots. Their key insight was that robot motion should maximize information gain about both robot pose and map. The uncertainty in robot location and map are fundamentally coupled through the covariance matrix. Minimizing map uncertainty requires actively reducing pose uncertainty. Similarly, reducing pose uncertainty improves map quality.

Shannon established the mathematical foundations of information theory in 1948. Cover and Thomas provide comprehensive coverage of information-theoretic methods in their 2006 textbook. Differential entropy quantifies uncertainty for continuous distributions. For Gaussian distributions, entropy depends on the determinant of the covariance matrix. Mutual information quantifies how much observing one variable reduces uncertainty about another variable. These concepts apply naturally to SLAM where observations reduce uncertainty about pose and landmarks.

Stachniss and colleagues extended information-theoretic methods to Rao-Blackwellized particle filters in 2005. They computed information gain for each particle trajectory. The robot selects actions that maximize expected information gain. This enables active exploration where the robot deliberately visits areas that will improve map quality. They demonstrated their approach in large-scale office environments with successful exploration results.

Carrillo and colleagues compared different uncertainty criteria for active SLAM in 2012. They analyzed D-optimality, A-optimality, and E-optimality criteria. D-optimality minimizes the determinant of covariance, which maximizes information. A-optimality minimizes the trace of covariance, which minimizes average variance. E-optimality minimizes the maximum eigenvalue of covariance, which bounds worst-case uncertainty. Their experimental comparison showed trade-offs between different criteria for various environments.

## Active SLAM and Exploration

Sim and Roy formalized active SLAM as an information maximization problem in 2005. The robot selects control inputs that maximize mutual information between the robot path and map. This formulation explicitly balances exploration and exploitation. Exploration reduces map uncertainty by visiting unknown areas. Exploitation uses known map information to navigate efficiently. The optimal strategy depends on mission objectives and time constraints.

Carlone and colleagues used Kullback-Leibler divergence for information gain computation in particle filter-based active SLAM in 2014. The KL divergence measures the information difference between prior and posterior distributions. Their approach extends to multi-robot coordination through information sharing. Multiple robots can coordinate exploration to maximize collective information gain while avoiding redundant coverage.

Valencia and colleagues addressed path planning with uncertainty quantification in 2009. They computed path reliability from pose covariance along planned trajectories. The robot plans paths that maximize the probability of successful navigation. This is critical for safety-critical applications where failures have serious consequences. Their work demonstrated that uncertainty-aware planning significantly improves reliability compared to planning with perfect information assumptions.

Frontier-based exploration was developed as a practical method for autonomous exploration. Frontiers are boundaries between explored and unexplored space. The robot selects frontiers as exploration goals to incrementally expand its map. This greedy approach is computationally efficient and produces reasonable coverage. However, it may not be globally optimal in terms of information gain. Hybrid approaches combine frontier detection with information-theoretic evaluation to select frontiers that maximize information gain.

## Path Planning for Mobile Robots

Path planning algorithms enable robots to navigate from start to goal locations while avoiding obstacles. Sampling-based planners like RRT have become popular for robotics applications. The Rapidly-exploring Random Tree algorithm was introduced for efficiently exploring high-dimensional configuration spaces. RRT builds a tree by randomly sampling configurations and connecting them to the nearest existing node. The algorithm is probabilistically complete, meaning it will find a solution if one exists given sufficient time.

RRT* is an asymptotically optimal variant of RRT introduced by Karaman and Frazzoli in 2011. The algorithm adds rewiring steps that improve solution quality over time. As the number of samples increases, the path cost converges to the optimal path cost. The rewiring radius must be chosen carefully to ensure optimality. The radius typically scales with the logarithm of the number of samples divided by the number of samples. This ensures sufficient connections for optimization while maintaining computational efficiency.

Path tracking controllers enable robots to follow planned paths. The pure pursuit controller is a geometric path tracking algorithm that has been widely used for wheeled robots. The controller computes steering commands to drive the robot toward a lookahead point on the path. The lookahead distance affects tracking performance. Larger lookahead distances produce smoother control but worse tracking accuracy. Smaller lookahead distances improve accuracy but may cause oscillation. Adaptive lookahead strategies adjust the distance based on vehicle speed and path curvature.

Reactive obstacle avoidance complements path planning by handling dynamic obstacles and planning errors. The robot monitors sensor readings for nearby obstacles. If obstacles are detected within a safety threshold, the robot reduces speed or stops to avoid collision. This provides a safety layer that prevents collisions even when the planner produces imperfect paths. The combination of global planning, local tracking, and reactive avoidance creates robust navigation systems.

## Uncertainty Quantification and Map Confidence

Quantifying map confidence enables robots to make informed decisions about navigation and exploration. Bar-Shalom and colleagues provide comprehensive coverage of performance evaluation methods in their 2001 textbook. The Normalized Estimation Error Squared test validates filter consistency. For a consistent filter, the NEES statistic follows a chi-squared distribution. Statistical tests determine whether actual errors are consistent with predicted covariance. Consistent filters satisfy the property that actual error covariance matches estimated covariance.

Bailey and colleagues conducted experimental consistency analysis of EKF-SLAM in 2006. They used NEES testing methodology to identify sources of inconsistency. Their work established standards for consistency testing in SLAM research. Proper validation requires comparing estimates against ground truth over many trials. Statistical tests then determine whether the filter is optimistic, pessimistic, or consistent.

Kümmerle and colleagues developed comprehensive evaluation frameworks for measuring SLAM accuracy in 2009. Their work established metrics for map accuracy, consistency, and robustness. They also introduced benchmark datasets and evaluation protocols. These standards enable fair comparison between different SLAM algorithms. Burgard and colleagues extended this work with graph-based comparison frameworks in 2009. Their relative performance metrics account for the unobservable global reference frame in SLAM.

Information-theoretic confidence metrics provide intuitive measures of map quality. The information matrix is the inverse of covariance. The trace of the information matrix quantifies total information about the state. A normalized confidence score can be computed as one minus the exponential of negative information divided by a scaling parameter. This metric ranges from zero for infinite uncertainty to one for zero uncertainty. The monotonic relationship ensures that more information always increases confidence. These metrics enable per-submap quality assessment and identification of unreliable map regions.

## Multi-Robot SLAM and Covariance Intersection

Multi-robot SLAM introduces challenges in fusing estimates from different robots. Julier and Uhlmann developed covariance intersection for fusing correlated estimates in 2007. When correlations between estimates are unknown, naive fusion produces overconfident results. Covariance intersection provides a conservative fusion rule that maintains consistency even with unknown correlations. The method is critical for multi-robot SLAM where robots may have observed common landmarks without explicitly tracking correlations.

Multi-robot systems enable faster exploration through parallel coverage. However, coordinating exploration requires communication and information sharing. Robots must balance individual exploration with collective benefit. Distributed algorithms enable scalable multi-robot SLAM without centralized coordination. Each robot maintains its own map and occasionally exchanges information with neighbors. The challenge is maintaining global consistency while enabling local autonomy.

## Simulation and Validation

Validating SLAM algorithms requires both simulation and real-world testing. Simulation provides controlled environments with known ground truth. Gazebo has become a standard simulation platform for robotics research. It provides physics simulation, sensor models, and robot models. ROS integration enables running the same software in simulation and on real robots. This facilitates development and testing before deploying to physical hardware.

The TurtleBot platform has become widely used for mobile robotics research. The TurtleBot3 variant features a differential drive base and LDS-01 two-dimensional LiDAR. The LiDAR provides 360-degree scans with one-degree angular resolution. The manufacturer specifies 10-millimeter accuracy at ranges up to 3.5 meters. This accuracy specification provides the sensor noise parameter for Hessian-based covariance estimation. Real-world testing complements simulation by exposing issues like sensor noise, calibration errors, and environmental complexities not captured in simulation.

## Summary

The literature reveals a rich body of work addressing the SLAM problem from multiple perspectives. Theoretical foundations established in the 1980s and 1990s proved that SLAM is fundamentally solvable. EKF-SLAM provides a principled probabilistic framework with explicit uncertainty representation. However, researchers discovered consistency issues arising from observability mismatches. Solutions like First Estimates Jacobian restore correct observability properties. Proper process noise modeling is essential for maintaining consistency over long operation periods.

Scan matching methods like ICP provide dense, accurate pose estimates. Hessian-based covariance estimation enables geometry-aware uncertainty quantification. This separates sensor noise from environmental structure in a theoretically grounded manner. Feature extraction from laser scans enables landmark-based SLAM with efficient computational complexity. Combining ICP and landmarks creates hybrid systems with complementary strengths. Dense matching provides short-term accuracy while landmarks provide long-term consistency.

Information theory provides tools for quantifying map confidence and driving active exploration. Entropy and mutual information quantify uncertainty and information gain. Robots can use these metrics to make informed decisions about navigation and exploration. Sampling-based path planners enable efficient navigation in complex environments. Tracking controllers and reactive avoidance create robust navigation systems.

This thesis builds on these foundations by implementing a complete system that combines EKF-SLAM with ICP-based corrections, frontier-based exploration, and RRT* path planning. The unified Hessian-based uncertainty framework provides consistent uncertainty quantification across all measurement types. The system demonstrates that rigorous uncertainty estimation is achievable in real-time autonomous systems for indoor environments.

## References

### Foundational Papers

1. **Smith, R. C., & Cheeseman, P. (1987).** "On the Representation and Estimation of Spatial Uncertainty." *The International Journal of Robotics Research*, 5(4), 56-68.

2. **Smith, R., Self, M., & Cheeseman, P. (1990).** "Estimating Uncertain Spatial Relationships in Robotics." *Autonomous Robot Vehicles*, pp. 167-193.

3. **Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001).** "A Solution to the Simultaneous Localization and Map Building (SLAM) Problem." *IEEE Transactions on Robotics and Automation*, 17(3), 229-241.

4. **Durrant-Whyte, H., & Bailey, T. (2006).** "Simultaneous Localization and Mapping: Part I." *IEEE Robotics & Automation Magazine*, 13(2), 99-110.

5. **Bailey, T., & Durrant-Whyte, H. (2006).** "Simultaneous Localization and Mapping (SLAM): Part II." *IEEE Robotics & Automation Magazine*, 13(3), 108-117.

### Observability and Consistency

6. **Huang, S., & Dissanayake, G. (2007).** "Convergence and Consistency Analysis for Extended Kalman Filter Based SLAM." *IEEE Transactions on Robotics*, 23(5), 1036-1049.

7. **Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010).** "Observability-based Rules for Designing Consistent EKF SLAM Estimators." *The International Journal of Robotics Research*, 29(5), 502-528.

### Process Noise and Motion Models

8. **Martinelli, A., Tomatis, N., & Siegwart, R. (2007).** "Simultaneous Localization and Odometry Self-Calibration for Mobile Robot." *Autonomous Robots*, 22(1), 75-85.

9. **Borenstein, J., & Feng, L. (1996).** "Measurement and Correction of Systematic Odometry Errors in Mobile Robots." *IEEE Transactions on Robotics and Automation*, 12(6), 869-880.

10. **Bailey, T., Nieto, J., Guivant, J., Stevens, M., & Nebot, E. (2006).** "Consistency of the EKF-SLAM Algorithm." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 3562-3568.

### Textbooks and Tutorials

11. **Thrun, S., Burgard, W., & Fox, D. (2005).** *Probabilistic Robotics*. MIT Press.

12. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation*. Wiley.

13. **Barfoot, T. D. (2017).** *State Estimation for Robotics*. Cambridge University Press.

14. **Siegwart, R., Nourbakhsh, I. R., & Scaramuzza, D. (2011).** *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

### ICP and Hessian-Based Covariance

15. **Besl, P. J., & McKay, N. D. (1992).** "A Method for Registration of 3-D Shapes." *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239-256.

16. **Censi, A. (2007).** "An Accurate Closed-Form Estimate of ICP's Covariance." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 3167-3172.

17. **Censi, A. (2008).** "An ICP Variant Using a Point-to-Line Metric." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 19-25.

18. **Grisetti, G., Stachniss, C., & Burgard, W. (2007).** "Improved Techniques for Grid Mapping with Rao-Blackwellized Particle Filters." *IEEE Transactions on Robotics*, 23(1), 34-46.

19. **Prakhya, S. M., Liu, B., & Lin, W. (2015).** "A Closed-Form Estimate of 3D ICP Covariance." *Proceedings of 14th IAPR International Conference on Machine Vision Applications (MVA)*, pp. 526-529.

### Hybrid SLAM Approaches

20. **Konolige, K., & Chou, K. (1999).** "Markov Localization using Correlation." *Proceedings of International Joint Conference on Artificial Intelligence (IJCAI)*, pp. 1154-1159.

21. **Hähnel, D., Burgard, W., Fox, D., & Thrun, S. (2003).** "An Efficient FastSLAM Algorithm for Generating Maps of Large-Scale Cyclic Environments from Raw Laser Range Measurements." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 206-211.

22. **Barfoot, T. D., & Furgale, P. T. (2014).** "Associating Uncertainty with Three-Dimensional Poses for use in Estimation Problems." *IEEE Transactions on Robotics*, 30(3), 679-693.

### Landmark Extraction

23. **Nguyen, V., Martinelli, A., Tomatis, N., & Siegwart, R. (2005).** "A Comparison of Line Extraction Algorithms using 2D Laser Rangefinder for Indoor Mobile Robotics." *IROS 2005*.

24. **Pavlidis, T., & Horowitz, S. L. (1974).** "Segmentation of Plane Curves." *IEEE Transactions on Computers*, C-23(8), 860-870.

25. **Duda, R. O., & Hart, P. E. (1972).** "Use of the Hough Transformation to Detect Lines and Curves in Pictures." *Communications of the ACM*, 15(1), 11-15.

26. **Duda, R. O., & Hart, P. E. (1973).** *Pattern Classification and Scene Analysis*. Wiley.

27. **Fischler, M. A., & Bolles, R. C. (1981).** "Random Sample Consensus: A Paradigm for Model Fitting with Applications to Image Analysis and Automated Cartography." *Communications of the ACM*, 24(6), 381-395.

28. **Arras, K. O., Castellanos, J. A., Schilt, M., & Siegwart, R. (2001).** "Feature-based Multi-hypothesis Localization and Tracking Using Geometric Constraints." *Robotics and Autonomous Systems*, 44(1), 41-53.

### Information Theory in SLAM

29. **Shannon, C. E. (1948).** "A Mathematical Theory of Communication." *Bell System Technical Journal*, 27(3), 379-423.

30. **Cover, T. M., & Thomas, J. A. (2006).** *Elements of Information Theory* (2nd ed.). Wiley.

31. **Feder, H. J. S., Leonard, J. J., & Smith, C. M. (1999).** "Adaptive Mobile Robot Navigation and Mapping." *The International Journal of Robotics Research*, 18(7), 650-668.

32. **Stachniss, C., Grisetti, G., & Burgard, W. (2005).** "Information Gain-based Exploration Using Rao-Blackwellized Particle Filters." *Proceedings of Robotics: Science and Systems (RSS)*.

33. **Carrillo, H., Reid, I., & Castellanos, J. A. (2012).** "On the Comparison of Uncertainty Criteria for Active SLAM." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 2080-2087.

### Active SLAM and Exploration

34. **Sim, R., & Roy, N. (2005).** "Global A-Optimal Robot Exploration in SLAM." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 661-666.

35. **Carlone, L., Du, J., Ng, M. K., Bona, B., & Indri, M. (2014).** "Active SLAM and Exploration with Particle Filters Using Kullback-Leibler Divergence." *Journal of Intelligent & Robotic Systems*, 75(2), 291-311.

36. **Valencia, R., Morta, M., Andrade-Cetto, J., & Porta, J. M. (2009).** "Planning Reliable Paths with Pose SLAM." *IEEE Transactions on Robotics*, 25(5), 1015-1026.

### Path Planning

37. **Karaman, S., & Frazzoli, E. (2011).** "Sampling-based Algorithms for Optimal Motion Planning." *The International Journal of Robotics Research*, 30(7), 846-894.

### Map Quality and Evaluation

38. **Kümmerle, R., Steder, B., Dornhege, C., Ruhnke, M., Grisetti, G., Stachniss, C., & Kleiner, A. (2009).** "On Measuring the Accuracy of SLAM Algorithms." *Autonomous Robots*, 27(4), 387-407.

39. **Burgard, W., Stachniss, C., Grisetti, G., Steder, B., Kümmerle, R., Dornhege, C., Ruhnke, M., Kleiner, A., & Tardós, J. D. (2009).** "A Comparison of SLAM Algorithms Based on a Graph of Relations." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2089-2095.

### Multi-Robot SLAM

40. **Julier, S. J., & Uhlmann, J. K. (2007).** "Using Covariance Intersection for SLAM." *Robotics and Autonomous Systems*, 55(1), 3-20.

41. **Castellanos, J. A., Neira, J., & Tardós, J. D. (2004).** "Limits to the Consistency of EKF-Based SLAM." *Proceedings of 5th IFAC Symposium on Intelligent Autonomous Vehicles*.

42. **Li, M., & Mourikis, A. I. (2012).** "Improving the Accuracy of EKF-Based Visual-Inertial Odometry." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 828-835.

43. **Jaynes, E. T. (2003).** *Probability Theory: The Logic of Science*. Cambridge University Press.

---

**Document Information:**
- File: `literature_review.md`
- Location: `/home/piyush/thesis_ws/docs/`
- Word Count: ~3,800 words
- Last Updated: 2026-02-09
