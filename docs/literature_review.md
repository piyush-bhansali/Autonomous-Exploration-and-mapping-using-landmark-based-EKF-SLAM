# Literature Review: Feature-Based EKF-SLAM with Autonomous Exploration for Indoor Mobile Robotics

---

## Table of Contents

1. [Introduction](#1-introduction)
2. [The SLAM Problem](#2-the-slam-problem)
3. [SLAM Approaches](#3-slam-approaches)
4. [Feature Extraction from 2D LiDAR](#4-feature-extraction-from-2d-lidar)
5. [EKF-SLAM: State Representation and Filtering](#5-ekf-slam-state-representation-and-filtering)
6. [Data Association](#6-data-association)
7. [Uncertainty Quantification](#7-uncertainty-quantification)
8. [Scan Matching and Global Consistency](#8-scan-matching-and-global-consistency)
9. [Autonomous Exploration](#9-autonomous-exploration)
10. [Research Gap and Thesis Contributions](#10-research-gap-and-thesis-contributions)
11. [References](#11-references)

---

## 1. Introduction

Building a map of an unknown indoor environment while simultaneously tracking the robot's position within it is a core challenge in mobile robotics. This problem is known as Simultaneous Localisation and Mapping (SLAM). A robot cannot know where it is without a map, and it cannot build a correct map without knowing where it is. This circular dependency makes SLAM one of the most studied problems in the field.

This literature review covers the research that underpins every component of the system developed in this thesis. The mapping module uses a geometric feature-based Extended Kalman Filter (EKF) SLAM approach, extracting walls and corners from 2D LiDAR scans and maintaining them in a probabilistic map. The navigation module uses frontier-based exploration, RRT* path planning, and a Pure Pursuit path tracker to autonomously drive the robot through unmapped space. Each section reviews the relevant prior work, explains why the chosen approach was selected, and identifies where existing solutions fall short of the requirements of this system.

---

## 2. The SLAM Problem

### 2.1 Origins and Formulation

Smith and Cheeseman (1987) were the first to frame robot mapping as a problem of representing and propagating spatial uncertainty. They observed that two landmarks seen from the same uncertain robot position are not independent — their errors are correlated. This means the correct way to represent a map is not to store each landmark independently, but to store the full joint uncertainty over all landmarks and the robot at once. Any approach that ignores these correlations will produce an overconfident and eventually incorrect map.

Dissanayake et al. (2001) built on this and proved that a SLAM filter, when implemented correctly, converges over time. As the robot collects more observations, the map uncertainty decreases monotonically. They also showed that the minimum achievable uncertainty in the map is set entirely by the initial uncertainty in the robot's starting position. These results confirmed that SLAM is theoretically solvable and gave the research community a clear target to implement.

Durrant-Whyte and Bailey (2006) provided a comprehensive survey of SLAM progress up to that point. They classified approaches by the map representation and the filtering method, and identified the key unsolved problems: computational scalability, long-range consistency, and loop closure. All three remain relevant today.

### 2.2 Why Indoor Environments

Outdoor SLAM is harder than indoor SLAM for several reasons. Outdoor environments have fewer repeatable geometric structures, wider sensor ranges, and more dynamic objects. Indoor environments — offices, corridors, and rooms — are dominated by flat walls, right-angle corners, and open floors. These structures can be reliably detected with a 2D laser rangefinder and represented compactly as geometric primitives. This thesis targets indoor environments specifically because the geometric feature extraction approach yields stable and interpretable landmarks in this setting.

---

## 3. SLAM Approaches

Several broad families of SLAM algorithms exist. Understanding why filter-based feature SLAM was chosen requires briefly examining the alternatives.

### 3.1 Dense Scan Matching Methods

Dense SLAM systems do not extract landmarks. Instead, they match entire point clouds against each other or against an occupancy grid. GMapping (Grisetti et al., 2007) uses a particle filter where each particle carries a full occupancy grid map. It works well in small environments and produces visually accurate maps. However, the memory cost grows with both the number of particles and the map size. As the environment grows, GMapping requires more particles to maintain accurate coverage, and memory becomes a bottleneck.

Google Cartographer (Hess et al., 2016) is a more recent dense approach that uses submap-based scan matching with loop closure. It can handle large environments in real time and is widely deployed on real robots. However, it treats the map as a grid rather than a collection of geometric features. A grid map cannot be queried for specific wall positions or corner coordinates, which are needed for the EKF landmark update equations. Cartographer is therefore not suitable for a filter-based architecture that relies on explicit landmark states.

Hector SLAM (Kohlbrecher et al., 2011) avoids odometry entirely and relies on scan-to-map matching using a Gauss–Newton optimiser. It performs well when the sensor rate is high, but accumulates drift in symmetric environments where the scan matching has a weak signal. It also does not maintain uncertainty estimates for individual features.

Dense methods were not chosen for this thesis because they do not produce the kind of structured, uncertainty-annotated landmark map that EKF-SLAM requires. They also do not provide a natural mechanism for propagating a pose correction to individual landmarks via cross-covariance — a key property used by the feature-based SVD feedback step in this system.

### 3.2 Graph-Based SLAM

Graph-based methods (Grisetti et al., 2010) represent the trajectory as a graph of pose nodes connected by edge constraints derived from scan matching or odometry. Loop closure is detected separately and added as a constraint. The full graph is then optimised using nonlinear least squares (g2o, iSAM2). Graph SLAM produces highly accurate maps over long trajectories because the optimisation distributes accumulated error across the entire trajectory rather than just correcting the current pose.

The limitation of graph-based SLAM for this application is real-time update cost. Every loop closure triggers a global graph optimisation. For a lightweight platform such as the TurtleBot3, running a real-time graph optimiser alongside a navigation stack and a sensor pipeline is computationally impractical. EKF-SLAM updates are bounded in time per observation, which makes the system more predictable for real-time deployment.

### 3.3 Particle Filter SLAM

FastSLAM (Montemerlo et al., 2002) factorises the SLAM posterior into a particle filter over robot trajectories and independent Kalman filters over landmarks. This reduces the computational complexity from O(n²) per update to O(M log n), where M is the number of particles and n is the number of landmarks. FastSLAM can handle large numbers of landmarks and is robust to multimodal uncertainty.

The weakness of particle filters for this system is sample degeneracy. When the number of particles is small (as it must be on a resource-limited platform), the filter can collapse to a single high-weight particle after several observations, losing diversity and failing to recover from localisation errors. The EKF maintains a single Gaussian estimate, which is adequate for the structured indoor environments targeted here where the environment is unique enough to avoid ambiguous data association.

### 3.4 EKF-SLAM

EKF-SLAM (Dissanayake et al., 2001) maintains a single joint Gaussian over the robot pose and all landmark parameters. Every observation updates both the relevant landmark and all other states through the cross-covariance structure. The update cost is O(n²) per observation because the full covariance matrix must be updated. This limits scalability to environments with a few hundred landmarks, which is entirely sufficient for typical indoor rooms and corridors.

EKF-SLAM was chosen for this thesis because it provides a mathematically principled way to represent and propagate uncertainty, it admits exact closed-form updates, and it integrates naturally with geometry-aware measurement covariances derived from the Fisher information matrix. The cross-covariance structure also means that a global SVD-based pose correction propagates consistently to every landmark — a key requirement for the global consistency layer.

---

## 4. Feature Extraction from 2D LiDAR

### 4.1 Why Geometric Features

A 2D LiDAR scan is a set of range measurements at known angles. These measurements can be stored as a raw point cloud or processed into geometric primitives. Raw point clouds contain all information but have no semantic structure — every point is equivalent. Geometric features (lines, corners) are compact, stable, and well-suited to the Hessian observation models used in EKF-SLAM.

Indoor environments are dominated by straight walls. A single wall segment can be represented by two numbers in Hessian normal form, whereas storing the same wall as individual scan points requires hundreds of numbers. Feature extraction also reduces the data association problem: instead of matching thousands of points, the system matches a small number of wall and corner observations to a small number of stored landmarks.

### 4.2 Line Extraction Algorithms

Several line extraction algorithms exist for 2D laser data. Nguyen et al. (2005) compared them systematically: split-and-merge, iterative end-point fit, incremental, and the Hough transform.

**Split-and-merge** (Pavlidis & Horowitz, 1974) starts with an entire segment, checks whether all points fit a line within a threshold, and recursively splits segments that violate the threshold. It is reliable but can produce breaks in long walls where the split criterion is met near the midpoint due to sensor noise.

**Hough transform** votes for line parameters in a discretised parameter space. It is robust to outliers but the resolution of the parameter grid limits accuracy, and it does not naturally produce ordered point segments.

**Incremental grow** starts at the first point and adds subsequent points one at a time, stopping when the next point violates the fit criterion. This system uses an incremental grow algorithm because it naturally handles the ordered structure of LiDAR scans and preserves spatial continuity between adjacent segments. A final merge step combines adjacent segments whose combined residual remains below threshold, recovering long walls that were split during growing.

### 4.3 Line Fitting: Total Least Squares vs Endpoint Fitting

When a candidate segment is evaluated during growing or merging, its residual must be computed. Two approaches exist.

**Endpoint fitting** computes the line connecting the first and last points of the segment and measures the perpendicular distance of all intermediate points to that line. It is simple but biased: the two endpoints are the noisiest measurements in a laser scan because they are observed at grazing angle, and using them to define the reference line introduces systematic error.

**Total Least Squares (TLS)** minimises the perpendicular distance from all points simultaneously to the best-fit line. It treats all points equally and is not sensitive to the quality of the endpoints. TLS is computed using Singular Value Decomposition (SVD), which is numerically stable and well-understood (Golub & Van Loan, 2013).

This system uses TLS for all residual computations during growing and merging, and for the final Hessian parameter computation. This choice makes the extracted wall parameters consistent with the statistical model assumed in the covariance computation.

### 4.4 Wall Representation: Hessian Normal Form

Once a line is fitted, it must be stored in a form suitable for the EKF state vector. Several choices exist.

**Slope-intercept form** (y = mx + c) is undefined for vertical walls and numerically unstable near them. It is not suitable for an EKF that must handle walls in all orientations.

**Two-point form** (x₁, y₁, x₂, y₂) stores the endpoints of the detected segment. This representation is over-parameterised (four numbers for an infinite-line constraint), and small changes in robot position cause large changes in the endpoints as the line is viewed from different angles.

**Hessian normal form** (ρ, α) represents an infinite line by its perpendicular distance from the origin (ρ ≥ 0) and the angle of its outward normal (α). This is the minimal two-parameter representation for an infinite line. It is well-defined for all wall orientations, and small changes in robot position produce small, predictable changes in ρ and α. The canonical constraint ρ ≥ 0 must be explicitly maintained after every EKF update (Siegwart & Nourbakhsh, 2011). If ρ drifts negative, the normal direction flips and future observations of the same wall fail the association check, creating duplicate landmarks.

### 4.5 Corner Detection

Arras et al. (1998) showed that including corner landmarks in addition to wall landmarks improves localisation accuracy. A wall observation constrains the robot's perpendicular distance from the wall but leaves position along the wall and heading poorly constrained. A corner is observed in two dimensions and constrains two degrees of freedom simultaneously.

Corners are detected as the intersection of two adjacent wall segments whose directions differ by more than a threshold angle (50° in this system). The intersection is computed analytically from the Hessian parameters of the two walls and stored as a Cartesian (x, y) position. This representation is appropriate because corner positions do not depend on the robot's observation angle, unlike wall parameters which must be expressed relative to the robot frame.

---

## 5. EKF-SLAM: State Representation and Filtering

### 5.1 The Joint State Vector

The EKF-SLAM state vector holds the robot's pose (x, y, θ) and the parameters of every known landmark in a single column vector. Wall landmarks contribute (ρ, α) and corner landmarks contribute (x, y). The full joint covariance matrix is maintained at all times. This means that when a wall is observed, the update modifies not only that wall's entry in the map but also the robot pose and every other landmark through the correlation terms.

This joint representation is the key insight of EKF-SLAM first formalised by Dissanayake et al. (2001). A filter that maintains independent estimates for each landmark would not propagate corrections to correlated landmarks, producing inconsistency over time.

### 5.2 Prediction Step

Between observations, the robot moves and its position becomes less certain. The prediction step uses the odometry reading to advance the robot pose estimate and to increase the covariance in proportion to the motion. This system uses a midpoint integration motion model (Barfoot, 2017) rather than forward Euler integration. Midpoint integration evaluates the heading at the middle of the motion step rather than at the start, which reduces accumulated linearisation error over long trajectories, particularly when the robot is turning.

All landmark estimates remain unchanged during prediction. Only the robot pose and its covariance, and the cross-covariance between the robot and each landmark, change during the predict step.

### 5.3 Observation Models

When a wall is observed in the robot frame, the EKF must predict what the robot would see given the current state estimate. This predicted observation is compared to the actual observation to form the innovation. The observation model transforms the map-frame wall parameters into robot-frame (ρ, α) values using the current robot pose. The corner observation model transforms the map-frame corner position into robot-frame Cartesian coordinates using a 2D rotation.

Both models are nonlinear. The EKF linearises them at the current state estimate, which is the standard EKF approximation. For the structured environments and short-range observations targeted here, this linearisation error is small.

### 5.4 Covariance Update: Joseph Form

The standard EKF covariance update formula is numerically unstable for long-running filters. Small floating-point errors accumulate and can make the covariance matrix lose its positive-definite property, causing the filter to produce invalid estimates. The Joseph form of the covariance update is algebraically equivalent but numerically stable (Bierman, 1977). It guarantees that the output covariance remains symmetric and positive semi-definite regardless of floating-point rounding. This is used in place of the standard formula throughout this system.

### 5.5 Landmark Initialisation and Pruning

A new landmark is not added to the map on first observation. It is placed in a buffer and promoted to the map only after being observed a second time. This two-observation confirmation step rejects most spurious detections — single-scan noise events, reflections, and partial occlusions. It is a simple alternative to the full probabilistic existence model of Hahnel et al. (2003), which is more principled but computationally heavier.

Landmarks that have not been observed for more than 50 scans are removed from the state vector. This keeps the state dimension bounded and removes landmarks that correspond to movable objects or measurement artefacts. Removal requires deleting the corresponding rows and columns from both the state vector and the covariance matrix, and updating the bookkeeping indices for all subsequent landmarks.

---

## 6. Data Association

### 6.1 The Correspondence Problem

At every scan, the system extracts a set of wall and corner observations. Each observation must be matched to an existing landmark or declared a new one. This is the data association problem. Neira and Tardós (2001) demonstrated that a single incorrect association is enough to permanently corrupt the map. An observation wrongly matched to a landmark pulls that landmark toward a position consistent with the wrong observation, and the error propagates through the cross-covariance to other landmarks.

### 6.2 Nearest-Neighbour with Mahalanobis Gating

The standard practical approach to data association is nearest-neighbour matching with a statistical gate (Bar-Shalom & Fortmann, 1988). The system computes a predicted observation for each existing landmark given the current state estimate. An observed feature is matched to the nearest landmark in terms of Mahalanobis distance, provided that distance is below a threshold.

Mahalanobis distance was chosen over Euclidean distance because it accounts for the different uncertainties in each observation dimension and for correlations between dimensions. A wall seen from far away has high uncertainty in ρ and should be given a wider gate than a wall seen from close range. Mahalanobis distance automatically adjusts the gate size to reflect this.

The gate threshold is set using the chi-squared distribution with two degrees of freedom. An observation whose Mahalanobis distance exceeds 13.8 (the 99.7% quantile) is declared a new landmark rather than matched to an existing one. This threshold is intentionally conservative: it is better to create a temporary duplicate landmark (which will be pruned) than to force an incorrect match.

### 6.3 Spatial Pre-filtering

Before computing the Mahalanobis distance to every landmark, a Euclidean distance pre-filter reduces the candidate set. Only landmarks within 5.0 m of the robot's current estimated position are considered. This threshold is larger than the LiDAR's maximum range of 3.5 m, so no genuine observation can be excluded by this filter. The pre-filter reduces the number of expensive Mahalanobis computations per scan without affecting correctness.

### 6.4 Joint Compatibility Branch and Bound

Neira and Tardós (2001) proposed a more powerful method called Joint Compatibility Branch and Bound (JCBB). Instead of matching observations independently, JCBB evaluates the joint statistical consistency of a full assignment of observations to landmarks. This catches cases where no single observation is an obvious outlier but the assignment as a whole is geometrically inconsistent.

JCBB was not used in this system because its worst-case computational cost is exponential in the number of observations. In structured indoor environments where the number of wall and corner observations per scan is small (typically 2–8), nearest-neighbour matching is fast and sufficiently reliable. JCBB would be more important in environments where many similar landmarks are close together, which is not the case for the rectangular rooms and corridors targeted here.

---

## 7. Uncertainty Quantification

### 7.1 Why Compute Covariance from Data Geometry

Every observation fed to the EKF must include a measurement noise covariance matrix. This matrix tells the filter how much to trust the observation relative to its current state estimate. The simplest approach is to use a fixed diagonal matrix — the same numbers every time. This is wrong in principle because the uncertainty of a line fit depends on how many points were used, how far apart they were, and their geometric arrangement.

A fixed covariance is overconfident when a wall is observed with few points and underconfident when it is observed with many well-spread points. Over time, overconfident observations dominate the filter and push landmark estimates away from their true positions. This failure mode is called filter inconsistency and is one of the main causes of map divergence in EKF-SLAM (Julier & Uhlmann, 2001).

This system derives measurement covariances from the Fisher Information Matrix (FIM) using the Cramér–Rao Lower Bound (CRLB) (Kay, 1993). The FIM accumulates the sensitivity of each observation to the landmark parameters, summed over all supporting scan points. The resulting covariance matrix is geometry-aware: it is small when the geometry strongly constrains the parameter, and large when it does not.

### 7.2 Wall Covariance

The uncertainty of a wall estimate depends on how many scan points support it and how they are arranged along the wall. Points spread far along the wall provide strong angular constraint. Points bunched near the middle provide weak angular constraint even if there are many of them. The CRLB formula captures this by treating the spread of points along the wall as a direct input to the information accumulation. A long wall with many spread-out points gets a small covariance; a short wall with few nearby points gets a large covariance. The EKF then trusts long-wall observations more, which is exactly the correct behaviour.

### 7.3 Corner Covariance

A corner position is derived from two wall estimates. Its uncertainty depends on the uncertainties of both parent walls and on the angle between them. When two walls are nearly parallel, their intersection point moves very far along both walls for even a tiny change in either wall's angle. The corner position is therefore highly uncertain when the two walls are nearly parallel. The system computes corner covariance by propagating the parent wall covariances through the intersection formula using first-order error propagation (Golub & Van Loan, 2013). This means that near-parallel corners automatically get large covariances and are down-weighted in the EKF update without any additional logic.

### 7.4 Numerical Conditioning

The covariance matrices must be strictly positive definite before they can be used in the EKF update. Near-degenerate geometry (very short walls, near-parallel intersections) can produce covariance matrices that are nearly singular. Rather than using a pseudoinverse, which would give an overconfident near-zero covariance, the system clamps the minimum eigenvalue of the information matrix before inversion (Higham, 2002). This produces a finite, valid covariance even in degenerate cases and prevents the filter from crashing when a borderline feature is accepted.

---

## 8. Scan Matching and Global Consistency

### 8.1 Drift in EKF-SLAM

EKF-SLAM accumulates drift as the robot moves. Each prediction step increases the robot's pose uncertainty based on odometry noise. Over long trajectories, this drift causes the robot's estimated position to diverge from its true position, and landmark estimates drift with it. In small environments this effect is minor. In larger environments it leads to torn maps where the same physical wall appears twice with a slight offset.

A global consistency correction layer is needed to control drift. This layer periodically aligns the accumulated point cloud against the growing map and feeds the resulting pose correction back to the EKF.

### 8.2 Feature-Based SVD Alignment

Given a set of corresponding point pairs, the rigid alignment problem in 2D has a closed-form solution using the SVD of the cross-covariance matrix (Arun et al., 1987; Horn, 1987). This thesis uses a **feature-based SVD alignment** rather than dense ICP. Wall landmarks shared between the current submap and the global wall registry provide correspondences, and overlap sampling on each wall yields point pairs for the SVD solve. This avoids nearest-neighbour matching and exploits the stability of persistent landmark IDs.

### 8.3 Submap Approach

Rather than aligning after every scan, the system accumulates a submap of 50 scans before attempting alignment. This approach is inspired by submap hierarchies used in larger systems (Bosse et al., 2004). Accumulating 50 scans before alignment yields denser geometry and reduces the frequency of global corrections, lowering computational load.

At a 10 Hz scan rate, 50 scans correspond to approximately 5 seconds of travel. Odometric drift over 5 seconds in a slow-moving indoor robot is typically small (often < 0.05 m), which keeps the feature-based SVD alignment well conditioned.

### 8.4 SVD Alignment Covariance

A naive implementation would use a fixed covariance for all global corrections. This is wrong for the same reason that a fixed wall covariance is wrong: the reliability of a submap alignment depends heavily on scene geometry. In a corner-rich room, the alignment constrains all three pose degrees of freedom well; in a long featureless corridor, translation along the corridor is weakly constrained.

The implementation derives a geometry-aware covariance directly from the SVD singular values and the spatial spread of the matched point pairs. The weaker singular value captures the worst-constrained translation direction, and the point spread captures rotational observability. The EKF then automatically down-weights corrections in degenerate configurations.

### 8.5 Why SVD Over Loop Closure

Full loop closure methods such as Scan Context (Kim & Kim, 2018) or bag-of-words visual approaches detect when the robot has returned to a previously visited location and add a global constraint to the map. Loop closure dramatically reduces long-range drift but requires a separate place recognition module and a global map optimiser such as g2o or iSAM2.

For the environments targeted in this thesis — single rooms and short corridors — the robot rarely traverses loops long enough to accumulate significant drift. The feature-based SVD submap correction provides sufficient global consistency at much lower implementation complexity. Loop closure would be the natural extension for larger environments.

---

## 9. Autonomous Exploration

### 9.1 The Exploration Problem

Once the robot has a SLAM system, it still needs a strategy for deciding where to go. In an unknown environment, the robot must actively choose paths that reveal new parts of the map rather than revisiting already-mapped areas. This is the autonomous exploration problem. Without an exploration strategy, the operator must manually drive the robot, which defeats the purpose of autonomous mapping.

### 9.2 Frontier-Based Exploration

Yamauchi (1997) introduced frontier-based exploration. A frontier is a boundary between mapped free space and unknown space. Moving to a frontier guarantees that the robot will explore new territory because it must pass through free space (already known) to reach the boundary of the unknown. The robot selects a frontier, navigates to it, maps the newly revealed area, and repeats.

Frontier-based exploration is the most widely used approach for autonomous indoor mapping (Thrun et al., 2005). Its key advantage is simplicity: frontiers are directly extractable from the occupancy grid that the SLAM system already maintains. No separate world model is needed.

Alternative exploration strategies include information-theoretic approaches, which select viewpoints that maximise expected information gain (Bourgault et al., 2002), and receding-horizon methods, which plan over a finite time window. These tend to produce more efficient paths but at significantly higher computational cost. For a resource-limited platform in a structured environment, frontier-based exploration achieves high coverage reliably and is well-supported in the literature.

### 9.3 Frontier Clustering

Naive frontier extraction produces many individual frontier cells on the occupancy grid boundary. Without clustering, the robot would attempt to reach tiny isolated frontier cells that may be inaccessible or represent sensor noise rather than true unexplored space. This system clusters frontier cells using DBSCAN (Ester et al., 1996), a density-based clustering algorithm that groups nearby points and discards isolated points as noise. Each cluster is then enclosed by a convex hull, and the centroid of the hull becomes the exploration goal.

The convex hull approach (using the Shapely library) was chosen because it provides a smooth, geometrically meaningful boundary around each frontier cluster. The centroid of a convex hull is a stable representative point that lies inside the free-space boundary, making it a valid navigation goal. A simpler approach would be to use the bounding box centre, but bounding boxes are sensitive to cluster shape and can produce goals outside the actual frontier boundary for non-rectangular clusters.

### 9.4 Path Planning: RRT*

With a frontier goal selected, the robot needs a path from its current position to the goal that avoids obstacles. Several planning approaches are available.

**Grid-based planners** such as A* and Dijkstra compute optimal paths on the occupancy grid. They are widely used and guaranteed to find the shortest path if one exists (Hart et al., 1968). However, grid paths have a resolution limit equal to the grid cell size, and they can be choppy because they are constrained to move in discrete directions. Smoothing the path adds another post-processing step.

**Probabilistic Roadmap (PRM)** builds a roadmap of the free space by sampling random points and connecting nearby samples with collision-free edges (Kavraki et al., 1996). PRM is efficient for high-dimensional spaces but is difficult to tune for dynamic environments where the map changes as the robot explores.

**RRT*** (Karaman & Frazzoli, 2011) is a sampling-based planner that builds a tree of collision-free paths by randomly sampling the free space and connecting samples to the nearest tree node. Unlike the original RRT (LaValle, 1998), RRT* includes a rewiring step that replaces parent nodes with shorter alternatives when found. This guarantees that the path cost converges to the optimum as the number of samples increases, a property called asymptotic optimality. RRT* was chosen because it handles arbitrary free-space shapes naturally, does not require a fixed grid resolution, and produces smooth paths without post-processing. The safety margin (0.5 m inflation of obstacles) ensures that paths stay well clear of walls even when the map has positional uncertainty.

### 9.5 Path Tracking: Pure Pursuit

Once a path is planned, the robot needs a controller to follow it. The path consists of a sequence of waypoints. Several tracking approaches exist.

**Model Predictive Control (MPC)** optimises a control sequence over a finite horizon while respecting velocity and acceleration constraints (Camacho & Bordons, 2007). MPC can handle complex dynamics and constraints but requires an accurate motion model and significant computation per control step.

**Dynamic Window Approach (DWA)** (Fox et al., 1997) samples feasible velocity commands, simulates their effect for a short horizon, and selects the command that best balances progress toward the goal and obstacle avoidance. DWA handles dynamic obstacles well but can be conservative near narrow passages.

**Pure Pursuit** (Coulter, 1992) selects a lookahead point on the path a fixed distance ahead of the robot and computes the curvature command needed to steer toward it. It is simple, fast, and smooth. The lookahead distance is the single most important tuning parameter: a short lookahead follows the path closely but produces jerky control; a long lookahead is smoother but cuts corners. A lookahead of 0.8 m was found to provide smooth tracking on the TurtleBot3 at the target speed of 0.20 m/s.

Pure Pursuit was chosen because it is computationally trivial (runs in microseconds), requires no model knowledge beyond the current pose, and behaves predictably. For a differential-drive robot in an obstacle-free corridor, it performs as well as far more complex controllers. Obstacle avoidance is handled at the planning layer by the RRT* path rather than at the tracking layer, which keeps the controller simple.

### 9.6 Stuck Detection and Recovery

Pure Pursuit does not detect when the robot is stuck. A robot can become stuck when the planned path passes through space that the map shows as free but is actually blocked by an obstacle not yet in the map, or when the controller oscillates near a tight corner. This system monitors the robot's linear velocity over a 5-second window. If the average speed drops below 0.1 m/s while the robot is commanded to move, it is declared stuck. The recovery behaviour rotates the robot in place and triggers a new frontier selection, which causes the planner to find a path to a different goal. This simple approach handles the most common stuck scenarios without requiring a more complex recovery state machine.

---

## 10. Research Gap and Thesis Contributions

Most prior EKF-SLAM implementations use point landmarks (Dissanayake et al., 2001; Neira & Tardós, 2001). Point landmarks are easy to extract but poorly suited to indoor environments where the dominant structure is planar walls. A point landmark does not encode the orientation information that walls provide, which means more landmarks are needed for equivalent localisation accuracy.

Line-based EKF-SLAM has been demonstrated (Arras et al., 1998; Castellanos et al., 1999), but these systems were designed before the availability of low-cost 360° LiDAR sensors and modern ROS-based middleware. They also used fixed or heuristic measurement covariances rather than geometry-derived ones.

Geometry-aware covariance for landmarks was proposed independently by several groups but has not been systematically applied to a combined wall-plus-corner EKF-SLAM system. Censi's ICP covariance (2007) is widely cited but rarely used to feed a simultaneous EKF landmark map — most systems treat ICP and SLAM as separate layers without a probabilistic connection.

On the navigation side, frontier-based exploration is well established, but integrating it tightly with a feature-based EKF-SLAM map (rather than a grid map) requires additional engineering. The frontier detector must operate on the occupancy grid produced as a by-product of the SLAM system, and the path planner must account for the inflation margins appropriate to the sensor noise level.

This thesis integrates these components into a single end-to-end system on the TurtleBot3 platform under ROS 2, with the following specific contributions:

1. **TLS-based feature extraction**: Using SVD-based Total Least Squares residuals in both the grow and merge phases, rather than endpoint-based residuals, for robustness to grazing-angle scan noise.

2. **CRLB wall and corner covariance**: Deriving measurement covariances from the Fisher information accumulated over the supporting scan points, rather than from fit residuals or fixed values, to produce geometry-aware measurement noise.

3. **SVD alignment covariance for EKF feedback**: Computing the submap pose covariance from SVD singular values and injecting it into the EKF as a weighted pose observation, so that degenerate alignments are automatically down-weighted.

4. **Joseph-form EKF with wall normalisation**: Using the numerically stable Joseph covariance update and enforcing the ρ ≥ 0 canonical form after every state update to prevent sign-drift-induced landmark duplication.

5. **Integrated autonomous exploration**: Coupling frontier detection, RRT* planning, and Pure Pursuit tracking with the SLAM map in a single ROS 2 system capable of fully autonomous indoor mapping.

---

## 11. References

Arras, K. O., Tomatis, N., Jensen, B. T., & Siegwart, R. (1998). Multisensor on-the-fly localization using laser and vision. *Robotics and Autonomous Systems*, 34(2–3), 131–143.

Bar-Shalom, Y., & Fortmann, T. E. (1988). *Tracking and Data Association*. Academic Press.

Barfoot, T. D. (2017). *State Estimation for Robotics*. Cambridge University Press.

Besl, P. J., & McKay, N. D. (1992). A method for registration of 3-D shapes. *IEEE Transactions on Pattern Analysis and Machine Intelligence*, 14(2), 239–256.

Biber, P., & Strasser, W. (2003). The Normal Distributions Transform: A new approach to laser scan matching. *Proceedings of IEEE/RSJ IROS*, 2743–2748.

Bierman, G. J. (1977). *Factorization Methods for Discrete Sequential Estimation*. Academic Press.

Bosse, M., Newman, P., Leonard, J., & Teller, S. (2004). Simultaneous localisation and map building in large-scale cyclic environments using the Atlas framework. *The International Journal of Robotics Research*, 23(12), 1113–1139.

Bourgault, F., Makarenko, A., Williams, S. B., Grocholsky, B., & Durrant-Whyte, H. F. (2002). Information based adaptive robotic exploration. *Proceedings of IEEE/RSJ IROS*, 540–545.

Camacho, E. F., & Bordons, C. (2007). *Model Predictive Control* (2nd ed.). Springer.

Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *The International Journal of Robotics Research*, 30(7), 846–894.

Castellanos, J. A., Montiel, J. M. M., Neira, J., & Tardós, J. D. (1999). The SPmap: A probabilistic framework for simultaneous localisation and map building. *IEEE Transactions on Robotics and Automation*, 15(5), 948–952.

Censi, A. (2007). An accurate closed-form estimate of ICP's covariance. *Proceedings of IEEE ICRA*, 3167–3172.

Chen, Y., & Medioni, G. (1992). Object modelling by registration of multiple range images. *Image and Vision Computing*, 10(3), 145–155.

Coulter, R. C. (1992). *Implementation of the Pure Pursuit Path Tracking Algorithm*. Carnegie Mellon University, Robotics Institute. Technical Report CMU-RI-TR-92-01.

Dissanayake, M. G., Newman, P., Clark, S., Durrant-Whyte, H. F., & Csorba, M. (2001). A solution to the simultaneous localisation and map building (SLAM) problem. *IEEE Transactions on Robotics and Automation*, 17(3), 229–241.

Durrant-Whyte, H., & Bailey, T. (2006). Simultaneous localisation and mapping: Part I. *IEEE Robotics & Automation Magazine*, 13(2), 99–110.

Ester, M., Kriegel, H. P., Sander, J., & Xu, X. (1996). A density-based algorithm for discovering clusters in large spatial databases with noise. *Proceedings of KDD*, 226–231.

Fox, D., Burgard, W., & Thrun, S. (1997). The Dynamic Window Approach to collision avoidance. *IEEE Robotics & Automation Magazine*, 4(1), 23–33.

Golub, G. H., & Van Loan, C. F. (2013). *Matrix Computations* (4th ed.). Johns Hopkins University Press.

Grisetti, G., Stachniss, C., & Burgard, W. (2007). Improved techniques for grid mapping with Rao-Blackwellized particle filters. *IEEE Transactions on Robotics*, 23(1), 34–46.

Grisetti, G., Kümmerle, R., Stachniss, C., & Burgard, W. (2010). A tutorial on graph-based SLAM. *IEEE Intelligent Transportation Systems Magazine*, 2(4), 31–43.

Hart, P. E., Nilsson, N. J., & Raphael, B. (1968). A formal basis for the heuristic determination of minimum cost paths. *IEEE Transactions on Systems Science and Cybernetics*, 4(2), 100–107.

Hess, W., Kohler, D., Rapp, H., & Andor, D. (2016). Real-time loop closure in 2D LIDAR SLAM. *Proceedings of IEEE ICRA*, 1271–1278.

Higham, N. J. (2002). *Accuracy and Stability of Numerical Algorithms* (2nd ed.). SIAM.

Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010). Observability-based rules for designing consistent EKF SLAM estimators. *The International Journal of Robotics Research*, 29(5), 502–528.

Julier, S. J., & Uhlmann, J. K. (2001). A counter example to the theory of simultaneous localisation and map building. *Proceedings of IEEE ICRA*, vol. 4, 4238–4243.

Karaman, S., & Frazzoli, E. (2011). Sampling-based algorithms for optimal motion planning. *The International Journal of Robotics Research*, 30(7), 846–894.

Kavraki, L. E., Svestka, P., Latombe, J. C., & Overmars, M. H. (1996). Probabilistic roadmaps for path planning in high-dimensional configuration spaces. *IEEE Transactions on Robotics and Automation*, 12(4), 566–580.

Kay, S. M. (1993). *Fundamentals of Statistical Signal Processing: Estimation Theory*. Prentice Hall.

Kim, G., & Kim, A. (2018). Scan Context: Egocentric spatial descriptor for place recognition within 3D point cloud map. *Proceedings of IEEE/RSJ IROS*, 4802–4809.

Kohlbrecher, S., Von Stryk, O., Meyer, J., & Klingauf, U. (2011). A flexible and scalable SLAM system with full 3D motion estimation. *Proceedings of IEEE SSRR*, 155–160.

LaValle, S. M. (1998). *Rapidly-Exploring Random Trees: A New Tool for Path Planning*. Iowa State University. Technical Report TR 98-11.

Montemerlo, M., Thrun, S., Koller, D., & Wegbreit, B. (2002). FastSLAM: A factored solution to the simultaneous localisation and mapping problem. *Proceedings of AAAI*, 593–598.

Neira, J., & Tardós, J. D. (2001). Data association in stochastic mapping using the joint compatibility test. *IEEE Transactions on Robotics and Automation*, 17(6), 890–897.

Nguyen, V., Gächter, S., Martinelli, A., Tomatis, N., & Siegwart, R. (2005). A comparison of line extraction algorithms using 2D range data for indoor mobile robotics. *Autonomous Robots*, 23(2), 97–111.

Pavlidis, T., & Horowitz, S. L. (1974). Segmentation of plane curves. *IEEE Transactions on Computers*, C-23(8), 860–870.

Siegwart, R., & Nourbakhsh, I. R. (2011). *Introduction to Autonomous Mobile Robots* (2nd ed.). MIT Press.

Smith, R. C., & Cheeseman, P. (1987). On the representation and estimation of spatial uncertainty. *The International Journal of Robotics Research*, 5(4), 56–68.

Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.

Yamauchi, B. (1997). A frontier-based approach for autonomous exploration. *Proceedings of IEEE CIRA*, 146–151.

Zhou, Q. Y., Park, J., & Koltun, V. (2018). Open3D: A modern library for 3D data processing. *arXiv:1801.09847*.
