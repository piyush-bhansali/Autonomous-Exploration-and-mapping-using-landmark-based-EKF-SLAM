# System Overview

This thesis presents an autonomous indoor mapping system that integrates real-time feature-based SLAM with feature-based SVD global map stitching. The system uses an Extended Kalman Filter (EKF) as the core state estimator, fusing wheel odometry with geometric landmark observations to maintain a joint estimate of robot pose and map. A separate navigation module provides frontier-based exploration with RRT* path planning and Pure Pursuit path tracking, enabling the robot to explore unknown environments autonomously. The system is implemented in Python using ROS 2 Jazzy as the middleware framework, with the TurtleBot3 Waffle Pi as the robot platform and Gazebo Harmonic for simulation.

---

## System Architecture

The complete system consists of three primary packages: the **Mapping Module** (`map_generation`), the **Navigation Module** (`navigation`), and the **Simulation and Launch infrastructure** (`autonomous_exploration`), all operating within the ROS 2 Jazzy framework.

### High-Level Architecture

The system operates through a hierarchical data flow starting from simulated sensors in Gazebo, processing through the ROS 2 middleware, performing SLAM and navigation computations, and culminating in velocity commands sent back to the robot actuators.

The Gazebo simulation environment provides two sensor streams used by the system:

- **LiDAR**: 360° laser scanner operating at 10 Hz providing 360 range measurements per scan.
- **Odometry**: Wheel encoder-based position estimates at 10 Hz.

These streams are transmitted through the `gz_ros2_bridge` to the ROS 2 ecosystem, where they are consumed by two parallel processing pipelines: the Mapping Module and the Navigation Module.

### Mapping Module Architecture

The Mapping Module implements the complete SLAM pipeline centred around the `local_submap_generator_feature` node. This node performs three operations in sequence for every incoming scan.

**Feature Extraction.** Each LiDAR scan is processed by `LandmarkFeatureExtractor` to extract geometric landmarks. Line segments are grown incrementally using a Total Least Squares (TLS) residual threshold. Segments are merged where geometrically compatible. Each accepted segment is converted to Hessian normal form $(\rho, \alpha)$ with covariance derived from the Fisher information matrix. Corners are detected at adjacent segment pairs and stored as Cartesian coordinates with analytically propagated covariance.

**EKF-SLAM.** The `LandmarkEKFSLAM` class maintains a joint state vector of robot pose and all confirmed landmarks. Each scan produces: an odometry-based prediction step using midpoint integration, a data association step using nearest-neighbour Mahalanobis gating, and an EKF update step for each matched landmark using the Joseph form covariance update. After every update, wall landmarks are normalised to enforce $\rho \geq 0$.

**Submap Generation and Integration.** After every 50 scans, the current **FeatureMap** (walls + corners in the EKF map frame) is converted to a point cloud (5 cm spacing) and stored in a submap-local frame anchored at the submap start pose. The submap is voxel-downsampled and registered against the global map using feature-based SVD alignment: wall landmarks shared between the new submap and the global wall registry are used as correspondences, and a one-shot SVD rigid body solve yields a correction that maps the **current EKF map frame → global map frame**. The SVD covariance ($\mathbf{R}_{\mathrm{SVD}} = \mathrm{diag}(\sigma^2/\Sigma_1, \sigma^2/\Sigma_1, \sigma^2/(N\bar{d}^2))$) is injected into the EKF as a direct pose observation to correct odometric drift.

### Navigation Module Architecture

The Navigation Module implements a finite state machine for autonomous exploration centred around the `SimpleNavigationNode` (`simple_navigation.py`). This node runs a 10 Hz control loop cycling through five states: `WAIT_FOR_MAP`, `DETECT_FRONTIERS`, `PLAN_PATH`, `EXECUTE_PATH`, and `DONE`.

**Frontier Detection.** The `ConvexFrontierDetector` identifies frontiers as candidate positions on the convex hull boundary of the current global point cloud, offset inward by 0.5 m. Raw frontier candidates are clustered using DBSCAN to produce discrete exploration goals. Each frontier is scored based on distance from the robot and angular deviation from current heading.

**Path Planning.** Once a frontier goal is selected, `RRTStar` computes a collision-free path through the point cloud map. The planner uses a KDTree built from the point cloud for obstacle queries, with a safety margin of 0.5 m around obstacles. It runs up to 1500 iterations with step size 0.2 m and goal bias 0.5.

**Path Tracking and Control.** `PurePursuit` tracks the planned path using a lookahead distance of 0.8 m. Maximum linear velocity is 0.20 m/s and maximum angular velocity is 1.0 rad/s. A reactive safety layer monitors the LiDAR scan: if any reading within a 60° forward arc falls below 0.4 m the robot stops and replans.

**Failure Recovery.** The system monitors for path deviation beyond 0.5 m and robot stagnation (less than 0.1 m travel in 5 seconds). On failure, the state machine returns to `DETECT_FRONTIERS` and selects an alternative goal.

### Module Organisation

| Package | Node | Key Files |
|---|---|---|
| `map_generation` | `local_submap_generator_feature` | `local_submap_generator_feature.py`, `feature_slam_manager.py`, `ekf_predict.py`, `ekf_update_feature.py`, `landmark_features.py`, `data_association.py`, `submap_stitcher.py`, `feature_map.py`, `mapping_utils.py`, `transform_utils.py`, `evaluation_utils.py` |
| `navigation` | `simple_navigation` | `simple_navigation.py`, `convex_frontier_detector.py`, `rrt_star.py`, `pure_pursuit_controller.py`, `navigation_utils.py` |
| `autonomous_exploration` | — | `full_system.launch.py`, `qos_profiles.py`, `worlds/maze.sdf`, `models/turtlebot3_waffle_pi/model.sdf` |

---

## Key System Features

### SLAM and Mapping Capabilities

**TLS-Based Feature Extraction.** Line segments are fit using Total Least Squares via SVD. The TLS residual (maximum perpendicular distance from any point to the best-fit line) is used in both the grow and merge phases. This is more robust than endpoint-based residuals, which are dominated by noise at shallow incidence angles.

**CRLB Covariance.** Wall covariance is derived from the Fisher information matrix $\mathbf{A} = \sum_i \mathbf{J}_i^\top \mathbf{J}_i$ with fixed sensor noise $\sigma = 0.01\;\mathrm{m}$ from the LDS-01 specification: $\mathrm{Cov}(\rho, \alpha) = \sigma^2 \mathbf{A}^{-1}$. Corner covariance is propagated analytically through the line-intersection Jacobian.

**Joint EKF State.** The full joint covariance over robot pose and all landmarks is maintained. A single observation updates the observed landmark and all correlated landmarks through the cross-covariance terms. This is the mechanism that makes EKF-SLAM globally consistent.

**Joseph Form Update.** The covariance update uses the Joseph form $\mathbf{P} = (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^\top + \mathbf{K}\mathbf{R}\mathbf{K}^\top$, which guarantees positive semi-definiteness regardless of finite-precision errors in $\mathbf{K}$.

**Wall Normalisation.** After every EKF update all wall landmarks are checked. If $\rho_i < 0$, both $\rho_i$ and $\alpha_i$ are flipped. This prevents sign drift from causing the same physical wall to be re-entered as a duplicate landmark.

**SVD Feature Covariance.** The submap alignment covariance is computed from the SVD singular values of the cross-covariance matrix and the spatial spread of the matched point pairs, giving a geometry-aware $3 \times 3$ covariance in $[x, y, \theta]$ space. In featureless corridors the smaller singular value is near zero and the condition number exceeds the degeneracy threshold, so alignment is rejected rather than producing an unreliable correction.

**GPU Acceleration.** All point cloud operations in `SubmapStitcher` use the Open3D tensor API. CUDA is used where available; the system falls back to CPU automatically.

### Navigation and Exploration Capabilities

**Convex Hull Frontier Detection.** Frontiers are generated as a regularly spaced set of candidate positions on the inward-offset boundary of the convex hull of the current global map. This avoids the need for an occupancy grid and works directly with the native point cloud map representation.

**DBSCAN Clustering.** Raw frontier candidates are grouped using DBSCAN, which determines cluster count automatically and filters noise. Cluster centroids become exploration goals.

**RRT* Path Planning.** The planner uses RRT* rather than basic RRT, incorporating rewiring operations that progressively improve path quality. Initial paths are typically found in well under 1500 iterations, with subsequent iterations refining toward shorter solutions.

**Reactive Safety Layer.** While following a planned path, the node continuously monitors the forward LiDAR arc (±30° from heading). If any reading falls below 0.4 m the robot stops immediately and a replan is triggered.

**Stuck Detection.** If the robot travels less than 0.1 m in any 5-second window during path execution, it is declared stuck. The state machine abandons the current goal and selects a new frontier.

**Thread-Safe Map Access.** The global map is protected by a `threading.Lock`. The navigation and mapping nodes run in separate threads; the lock prevents race conditions between map updates and map reads during frontier detection and path planning.

---

## Technology Stack

| Component | Technology | Version | Purpose |
|---|---|---|---|
| Middleware | ROS 2 | Jazzy | Robot communication framework |
| Simulator | Gazebo | Harmonic | Physics-based simulation |
| Point Cloud | Open3D | 0.18+ | GPU-accelerated voxel operations and tensor point clouds |
| Clustering | scikit-learn | Latest | DBSCAN frontier clustering |
| Geometry | Shapely | Latest | Convex hull frontier boundary computation |
| Numerical | NumPy / SciPy | Latest | Matrix operations, KDTree |

There is no GTSAM dependency. Loop closure and pose graph optimisation are not implemented. Global consistency is maintained solely through the EKF cross-covariance structure and periodic feature-based SVD submap corrections.

### Hardware Requirements

| Component | Specification |
|---|---|
| CPU | Intel Core i7-10750H @ 2.60 GHz (6 cores, 12 threads) |
| RAM | 8 GB |
| GPU | NVIDIA GeForce GTX 1660 Ti (6 GB VRAM, CUDA 12.0) |
| Storage | 1 TB SSD |

---

## ROS 2 Software Architecture

The system runs two primary nodes concurrently. The mapping node processes sensor data and maintains the SLAM state. The navigation node queries the map, plans paths, and sends velocity commands.

### System Nodes

#### Mapping Node: `local_submap_generator_feature`

Implemented in `map_generation/local_submap_generator_feature.py`. Responsible for feature extraction, EKF-SLAM, submap stitching, and publishing the corrected pose and global map.

**Subscribed Topics:**

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/{robot}/scan` | `sensor_msgs/LaserScan` | 10 Hz | 360 range measurements, 1° resolution |
| `/{robot}/odom` | `nav_msgs/Odometry` | 10 Hz | Pose and twist in odometry frame |

**Published Topics:**

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/{robot}/ekf_pose` | `geometry_msgs/PoseStamped` | Per scan | EKF-corrected robot pose |
| `/{robot}/ekf_path` | `nav_msgs/Path` | Per scan | Full trajectory history |
| `/{robot}/scan_features` | `visualization_msgs/MarkerArray` | Per scan | Extracted wall and corner markers |
| `/{robot}/current_submap` | `sensor_msgs/PointCloud2` | Per submap | Current accumulated point cloud |
| `/{robot}/global_map` | `sensor_msgs/PointCloud2` | 1 Hz timer | Stitched global map |

**TF Broadcast:** `map → {robot}/odom` at 10 Hz, computed from the EKF pose minus the latest odometry pose.

#### Navigation Node: `simple_navigation`

Implemented in `navigation/simple_navigation.py`. Runs a 10 Hz control loop that drives the robot through the state machine.

**Subscribed Topics:**

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/{robot}/global_map` | `sensor_msgs/PointCloud2` | 1 Hz | Accumulated map for frontier detection and collision checking |
| `/{robot}/scan` | `sensor_msgs/LaserScan` | 10 Hz | Forward scan for reactive obstacle avoidance |

**Published Topics:**

| Topic | Type | Rate | Content |
|---|---|---|---|
| `/{robot}/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | Linear and angular velocity commands |
| `/{robot}/planned_path` | `nav_msgs/Path` | Per replan | RRT* trajectory waypoints |
| `/{robot}/frontier_markers` | `visualization_msgs/MarkerArray` | Per detection | Frontier cluster visualisation |
| `/{robot}/hull_boundary` | `visualization_msgs/MarkerArray` | Per detection | Convex hull boundary marker |

Robot pose is obtained from the TF tree (`map → {robot}/odom → {robot}/base_footprint`) using `tf2_ros`.

### Quality of Service Profiles

QoS profiles are defined centrally in `autonomous_exploration/qos_profiles.py` and shared by both nodes.

| Topic Class | QoS Policy | Rationale |
|---|---|---|
| `/scan` | RELIABLE | All scans needed for consistent map; dropped scans create coverage gaps |
| `/odom` | RELIABLE | Missing odometry increments cause EKF prediction errors |
| `/cmd_vel` | RELIABLE | Lost velocity commands cause unpredictable motion |
| `/global_map` | RELIABLE | Large messages must arrive complete for planning to function |

### Simulation Module

The simulation infrastructure is managed through `autonomous_exploration/launch/full_system.launch.py`.

**Configurable launch arguments:**

| Argument | Default | Options |
|---|---|---|
| `world` | `maze` | `maze`, `park` |
| `enable_navigation` | `true` | `true`, `false` |
| `use_rviz` | `true` | `true`, `false` |

The number of robots is controlled by `NUM_ROBOTS` (currently 1; supports up to 2). Robot 1 spawns at $(-12, -12)$ m facing east. Robot 2, if enabled, spawns at $(12, 12)$ m facing southwest.

Startup is sequenced with timers: Gazebo at $t = 0$ s, robot spawn at $t = 2$ s, mapping node at $t = 4$ s, RViz at $t = 6$ s, navigation node at $t = 8$ s.

---

## Data Flow

The system processes data through two primary pipelines running in parallel.

### Scan Processing Pipeline (10 Hz)

```
/{robot}/scan  (LaserScan, 10 Hz)
       │
       ▼
LandmarkFeatureExtractor
  ├─ TLS line growing + merge  →  walls (ρ, α) + CRLB covariance
  └─ Corner detection          →  corners (x, y) + Jacobian covariance
       │
       ▼
associate_landmarks()
  ├─ Euclidean pre-filter (6.0 m)
  └─ Mahalanobis gate (D²_M < 5.991)
       ├─── Matched   →  LandmarkEKFSLAM.update_landmark_observation()
       └─── Unmatched →  provisional buffer → add_landmark() after 2 obs
```

### Odometry Prediction Pipeline (10 Hz)

```
/{robot}/odom  (Odometry, 10 Hz)
       │
       ▼
compute_relative_motion_2d()  →  (δ_d, δ_θ)
       │
       ▼
LandmarkEKFSLAM.predict_with_relative_motion()
  ├─ Midpoint integration
  └─ Motion-scaled process noise
```

### Submap Pipeline (every 50 scans)

```
Accumulated scan points  (submap-local frame)
       │
       ▼
SubmapStitcher.integrate_submap_to_global_map()
  ├─ Voxel downsample (0.05 m)
  ├─ Feature SVD align (shared landmark IDs → overlap pairs → SVD solve)
  │    ├─ success: apply correction_4x4 + LandmarkEKFSLAM.update()
  │    └─ fail (degenerate / no shared walls): use EKF pose as-is
  └─ Concatenate + voxel downsample global map
```

### Gazebo–ROS 2 Bridge Configuration

The `config/tb3_bridge.yaml` file maps Gazebo's internal topics to ROS 2. Example entry:

```yaml
- ros_topic_name: "/tb3_1/scan"
  gz_topic_name: "/world/maze_world/model/tb3_1/link/base_scan/sensor/lidar/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
```

Similar mappings exist for `/odom` and `/cmd_vel`.

---

## TurtleBot3 Robot Platform

### Platform Overview

The TurtleBot3 Waffle Pi is a differential-drive mobile robot developed by ROBOTIS. It uses two independently actuated drive wheels and a passive rear caster. Differential drive provides in-place rotation with simple, well-characterised kinematics suited to odometry-based dead reckoning.

### Physical Specifications

| Parameter | Value |
|---|---|
| Length | 281 mm |
| Width | 306 mm |
| Height | 141 mm |
| Mass | 1.8 kg |
| Wheelbase | 287 mm |
| Wheel diameter | 66 mm |
| Ground clearance | 10 mm |

The centre of mass is at $[-0.064, 0, 0.048]$ m relative to the geometric centre — slightly rear-biased due to battery placement.

### Sensor Suite

#### LiDAR Sensor (HLS-LFCD-LDS)

The robot is equipped with a Hitachi-LG LDS-01 triangulation laser distance sensor.

| Parameter | Value |
|---|---|
| Update rate | 10 Hz |
| Angular coverage | 360° |
| Angular resolution | 1.0° (360 samples/scan) |
| Minimum range | 0.12 m |
| Maximum range | 3.5 m (nominal indoor range) |
| Accuracy | ±10 mm (1 σ) |

In Gazebo the LiDAR is modelled with additive Gaussian noise: mean 0, standard deviation 0.01 m — consistent with the manufacturer specification. The sensor is mounted at $[-0.064, 0, 0.121]$ m in the robot base frame. All SLAM covariance calculations use $\sigma = 0.01\;\mathrm{m}$.

The sensor publishes `sensor_msgs/LaserScan` to `/{robot}/scan` at 10 Hz with 360 range values, angular limits $[0, 2\pi]$, and range limits $[0.12, 3.5]$ m.

#### Wheel Odometry

Odometry is generated by the Gazebo differential drive plugin, which integrates simulated wheel velocities to produce pose estimates.

| Parameter | Value |
|---|---|
| Update rate | 10 Hz |
| Position noise (σ) | 0.071 m |
| Orientation noise (σ) | 0.141 rad |

The noise parameters correspond to the `pose_covariance_diagonal` values `[0.005, 0.005, 0.001, 0.001, 0.001, 0.02]` (variances in $[x, y, z, \mathrm{roll}, \mathrm{pitch}, \mathrm{yaw}]$) in the robot SDF model. The 0.141 rad orientation noise reflects the accumulation of differential encoder errors over time.

The odometry frame (`{robot}/odom`) drifts globally. The mapping node's EKF corrects this drift and publishes the `map → {robot}/odom` transform.

**Note:** The system does not use an IMU. The EKF prediction step uses only wheel odometry increments. There is no IMU subscription in `local_submap_generator_feature.py`.

---

## Gazebo Simulation Environment

### Simulation Platform

The system uses Gazebo Harmonic with the Open Dynamics Engine (ODE) physics solver. The simulation runs at an internal timestep of 1 ms. Sensors sample at their configured rates (LiDAR at 10 Hz, odometry at 10 Hz).

### Simulation Plugins

| Plugin | Function |
|---|---|
| Physics | Rigid body dynamics and collision detection (ODE, 1 ms timestep) |
| Scene Broadcaster | Visual state publication for RViz |
| Sensors | LiDAR ray casting with Gaussian noise |
| Differential Drive | Converts `cmd_vel` to wheel torques; publishes odometry |
| GPU LiDAR | GPU-accelerated 360° laser ray casting |

The differential drive plugin implements standard inverse kinematics. Given commanded linear velocity $v$ and angular velocity $\omega$:

$$
v_L = v - \frac{\omega L}{2}, \qquad v_R = v + \frac{\omega L}{2},
$$

where $L = 0.287\;\mathrm{m}$ is the wheelbase. PID controllers apply wheel torques to track the commanded speeds. Forward kinematics integrates actual wheel velocities to produce the odometry estimate.

### Maze World Environment

The primary test environment is defined in `autonomous_exploration/worlds/maze.sdf`.

| Property | Value |
|---|---|
| Wall height | 2.0 m (prevents LiDAR scanning over walls) |
| Wall thickness | 0.2 m |
| Corridor widths | 1.5–3.0 m |
| Environment type | Indoor maze with multiple rooms, corridors, dead ends, and loops |

The loop topology tests whether the EKF-SVD pipeline maintains global consistency after the robot revisits areas.

---

## System Deployment

### Build Process

```bash
cd ~/thesis_ws
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` creates symbolic links to Python files rather than copies. Changes to Python source take effect immediately without rebuilding.

### Launching the System

**Full system (maze world, with navigation and RViz):**
```bash
ros2 launch autonomous_exploration full_system.launch.py
```

**Mapping only (no autonomous navigation):**
```bash
ros2 launch autonomous_exploration full_system.launch.py enable_navigation:=false
```

**Headless (no RViz):**
```bash
ros2 launch autonomous_exploration full_system.launch.py use_rviz:=false
```

### System Monitoring

**List active nodes:**
```bash
ros2 node list
```

**Verify topic publication rates:**
```bash
ros2 topic hz /tb3_1/global_map      # ~0.1 Hz during active exploration
ros2 topic hz /tb3_1/scan            # ~10 Hz
ros2 topic hz /tb3_1/ekf_pose        # ~10 Hz
```

**Inspect current EKF pose:**
```bash
ros2 topic echo /tb3_1/ekf_pose --once
```

**Check node subscriptions and publications:**
```bash
ros2 node info /local_submap_generator_feature
ros2 node info /simple_navigation
```

---

## Performance Characteristics

### Simulation vs. Reality

| Aspect | Simulation | Real World |
|---|---|---|
| LiDAR noise | Parametric Gaussian ($\sigma = 0.01\;\mathrm{m}$) | Non-Gaussian, includes bias drift and temperature effects |
| Wheel slip | Modelled via ODE friction coefficients | Depends on surface, wear, and load distribution |
| Timing | Deterministic 1 ms physics steps | Variable latency, asynchronous sensor delivery |
| Repeatability | Identical results for identical inputs | Non-deterministic due to real-world variability |

Simulation provides the controlled, repeatable environment necessary for systematic evaluation of the SLAM and navigation algorithms. The parametric noise models capture the key characteristics — range uncertainty, odometric drift — that the algorithms must handle. The primary limitation is that correlated noise, sensor bias, and surface-dependent wheel slip are not captured by the Gaussian models used.

### Computational Complexity

| Operation | Complexity | Dominant Cost |
|---|---|---|
| Feature extraction | $O(N)$ per scan | TLS SVD per segment |
| Data association | $O(L)$ per feature | Mahalanobis distance per landmark |
| EKF update | $O(L^2)$ per observation | Full covariance propagation |
| Covariance conditioning | $O(L^3)$ periodic | Eigendecomposition |
| SVD alignment (per submap) | $O(W \cdot n_s)$ | $W$ shared walls, $n_s$ samples per wall; SVD on 2×2 matrix |

where $N$ is the number of scan points, $L$ is the number of landmarks in the state, $W$ is the number of shared wall landmarks, and $n_s = 8$ is the number of overlap sample points per wall. The $O(L^2)$ EKF update is the scalability bottleneck; the system is designed for structured indoor environments where $L$ is bounded to hundreds rather than thousands of landmarks.

---

## Documentation Map

| Document | Content |
|---|---|
| `00_system_overview.md` | This file — full system description, parameters, deployment |
| `01_ekf_slam_theory.md` | EKF-SLAM: state representation, prediction, update, Jacobians |
| `02_landmark_features.md` | Feature extraction: TLS, Hessian form, CRLB covariance, corners |
| `03_data_association.md` | Data association: Mahalanobis gating, chi-squared test, Jacobians |
| `05_submap_management.md` | Submap stitching: feature-based SVD alignment, SVD covariance, EKF feedback |
| `literature_review.md` | Implementation-focused review (algorithms used in this system) |
| `introduction_literature_review.md` | Broad introduction: general SLAM, EKF theory, uncertainty propagation |
