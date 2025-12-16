# Navigation & Autonomous Exploration
## Frontier-Based Exploration with RRT* Path Planning

**Module:** `navigation`
**Main Node:** `simple_navigation`
**Components:** Frontier Detection, RRT* Planner, Pure Pursuit Controller

---

## Overview

The navigation system enables autonomous exploration through a multi-stage pipeline:

1. **Frontier Detection** - Identify unexplored boundaries using DBSCAN clustering
2. **Goal Selection** - Score frontiers based on distance, size, and heading alignment
3. **RRT* Path Planning** - Compute collision-free, asymptotically optimal paths
4. **Pure Pursuit Control** - Execute paths with constant lookahead and adaptive velocity
5. **Reactive Avoidance** - Real-time obstacle response using LiDAR data

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                     Navigation Pipeline                          │
└──────────────────────────────────────────────────────────────────┘
         │
         │  Inputs: Global Map (PointCloud2), Odometry, LaserScan
         │
         ↓
┌─────────────────────────────────────────────────────────────────┐
│  State Machine                                                   │
│  WAIT_FOR_MAP → DETECT_FRONTIERS → PLAN_PATH → EXECUTE_PATH    │
│                      ↑                              │            │
│                      └──────── Replan ──────────────┘            │
└─────────────────────────────────────────────────────────────────┘
         │
         ↓
┌─────────────────────────────────────────────────────────────────┐
│  Frontier Detection (simple_frontiers.py)                       │
│  - Extract free space boundary from map                         │
│  - Sample candidate frontiers (0.8m grid spacing)               │
│  - Filter by clearance (3× robot radius = 0.66m)                │
│  - Check for open unexplored directions                         │
│  - Cluster with DBSCAN (eps=1.0m, min_samples=2)               │
│  - Score: (cluster_size / distance) × heading_factor           │
└─────────────────────────────────────────────────────────────────┘
         │
         │  Best frontier selected
         ↓
┌─────────────────────────────────────────────────────────────────┐
│  RRT* Path Planner (rrt_star.py)                                │
│  - Sample random points (50% goal bias)                         │
│  - Find nearest node using KD-tree                              │
│  - Steer with max step size (0.4m)                              │
│  - Choose best parent within rewire radius                      │
│  - Rewire neighbors for optimality                              │
│  - Post-process: smooth + densify path (0.3m spacing)          │
└─────────────────────────────────────────────────────────────────┘
         │
         │  Path: List of [x, y] waypoints
         ↓
┌─────────────────────────────────────────────────────────────────┐
│  Pure Pursuit Controller (smoothed_pure_pursuit.py)             │
│  - Constant lookahead distance (1.2m)                           │
│  - Adaptive velocity based on curvature                         │
│  - Angular velocity smoothing (EMA: α=0.6)                      │
│  - Linear velocity: 0.08 - 0.18 m/s                             │
│  - Angular velocity: ±0.5 rad/s                                 │
└─────────────────────────────────────────────────────────────────┘
         │
         │  cmd_vel: Twist (linear, angular)
         ↓
┌─────────────────────────────────────────────────────────────────┐
│  Reactive Obstacle Avoidance                                    │
│  - Monitor LiDAR front arc (±60°)                               │
│  - Danger distance: 0.5m → reduce speed                         │
│  - Emergency distance: 0.3m → full stop                         │
│  - Response time: <100ms (10Hz control loop)                    │
└─────────────────────────────────────────────────────────────────┘
         │
         ↓
    Robot Motion
```

---

## Frontier Detection

**File:** `src/navigation/navigation/simple_frontiers.py`
**Class:** `SimpleFrontierDetector`

### Algorithm

The frontier detector uses a **hybrid boundary + gap filtering approach** to identify exploration goals:

#### Step 1: Extract Free Space Boundary
```python
# Use map bounds to define exploration area
x_min, y_min = map_points.min(axis=0)
x_max, y_max = map_points.max(axis=0)
```

#### Step 2: Sample Candidate Frontiers
```python
grid_size = 0.8  # Sample every 0.8m
candidates = sample_boundary(x_min, x_max, y_min, y_max)
```

Sample points 0.3m **inside** the mapped area boundary to ensure reachability.

#### Step 3: Safety Filter
```python
# Check clearance to nearest obstacle
dist, _ = kdtree.query(candidate)

# Require 3× robot radius clearance (0.66m for robot_radius=0.22m)
if dist < robot_radius * 3.0:
    reject_candidate()
```

**Rationale:** Increased from 2× to 3× radius to prevent selecting frontiers too close to walls.

#### Step 4: Gap Filtering (Open Direction Check)
```python
def _has_open_direction(candidate, kdtree, robot_pos):
    """Check if frontier has unexplored space beyond it"""
    num_directions = 8  # Check 8 directions (45° apart)
    check_distance = 1.5m  # Look 1.5m ahead

    for angle in [0°, 45°, 90°, 135°, 180°, 225°, 270°, 315°]:
        check_point = candidate + check_distance * [cos(angle), sin(angle)]
        dist_to_obstacle = kdtree.query(check_point)

        # If >1.0m from any mapped point → unexplored
        if dist_to_obstacle > 1.0m:
            open_count += 1

    return open_count >= 2  # Require at least 2 open directions
```

**Purpose:** Filters out dead-ends and ensures frontiers lead to significant unexplored regions.

#### Step 5: DBSCAN Clustering
```python
from sklearn.cluster import DBSCAN

# Group nearby frontiers into clusters
clustering = DBSCAN(eps=1.0, min_samples=2).fit(candidates)

# Compute cluster centroids
for cluster in clusters:
    centroid = np.mean(cluster_points, axis=0)
```

**Parameters:**
- `eps=1.0m` - Frontiers within 1m are grouped (increased from 0.4m to reduce oscillation)
- `min_samples=2` - Minimum 2 points to form cluster

#### Step 6: Cluster Scoring
```python
def score_frontier(cluster, robot_pos, robot_yaw):
    distance = np.linalg.norm(cluster.centroid - robot_pos)

    # Heading alignment factor
    direction_to_frontier = np.arctan2(dy, dx)
    angle_diff = abs(direction_to_frontier - robot_yaw)
    heading_factor = 1.0 + 0.3 * np.cos(angle_diff)  # Prefer forward

    # Final score (higher is better)
    score = (cluster.size / distance) * heading_factor
    return score
```

**Scoring Components:**
1. **Size** - Larger clusters = more unexplored area
2. **Distance** - Closer frontiers preferred
3. **Heading** - Frontiers aligned with current heading get 30% bonus

### Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `robot_radius` | 0.22m | Safety clearance |
| `grid_size` | 0.8m | Frontier sampling density |
| `clearance_multiplier` | 3.0 | Require 0.66m clearance |
| `check_distance` | 1.5m | Gap filtering lookahead |
| `unexplored_threshold` | 1.0m | Distance to consider unexplored |
| `dbscan_eps` | 1.0m | Clustering radius |
| `min_samples` | 2 | Minimum cluster size |

### Output

**Type:** `List[SimpleFrontier]`

Each frontier contains:
- `position: np.ndarray` - [x, y] centroid
- `score: float` - Composite score (higher = better)
- `size: int` - Number of frontier points in cluster

---

## RRT* Path Planning

**File:** `src/navigation/navigation/rrt_star.py`
**Class:** `RRTStar`

### Algorithm

RRT* (Rapidly-exploring Random Tree Star) is an **asymptotically optimal** sampling-based planner. Unlike RRT, RRT* continuously rewires the tree to improve path quality.

#### Initialization
```python
def __init__(self, map_points, robot_radius=0.22, step_size=0.4,
             goal_bias=0.5, max_iterations=3000, gamma=2.0):
    # Build KD-tree for fast collision checking
    self.kdtree = KDTree(map_points[:, :2])

    # RRT* gamma parameter (controls rewire radius decay)
    self.gamma = gamma
```

#### Main Loop
```python
for iteration in range(max_iterations):
    # 1. Sample random point (50% goal bias)
    if random() < goal_bias:
        sample = goal
    else:
        sample = sample_random_point()

    # 2. Find nearest node in tree (using KD-tree)
    nearest = find_nearest_node(nodes, sample)

    # 3. Steer toward sample (max step = 0.4m)
    new_pos = steer(nearest.position, sample, max_step=0.4)

    # 4. Check collision-free
    if not is_path_collision_free(nearest.position, new_pos):
        continue

    # 5. Compute shrinking rewire radius
    n = len(nodes) + 1
    rewire_radius = min(step_size, gamma * sqrt(log(n) / n))

    # 6. Find near neighbors within rewire radius
    near_nodes = find_near_neighbors(new_pos, rewire_radius)

    # 7. Choose BEST parent (minimum cost)
    best_parent = nearest
    best_cost = nearest.cost + distance(nearest, new_pos)

    for near_node in near_nodes:
        if is_path_collision_free(near_node.position, new_pos):
            cost = near_node.cost + distance(near_node, new_pos)
            if cost < best_cost:
                best_parent = near_node
                best_cost = cost

    # 8. Add node to tree
    new_node = RRTNode(new_pos)
    new_node.parent = best_parent
    new_node.cost = best_cost
    nodes.append(new_node)

    # 9. REWIRE: Check if path through new_node is better for neighbors
    for near_node in near_nodes:
        if is_path_collision_free(new_pos, near_node.position):
            new_cost = new_node.cost + distance(new_node, near_node)
            if new_cost < near_node.cost:
                # Rewire: change parent
                near_node.parent = new_node
                near_node.cost = new_cost

    # 10. Check if goal reached
    if distance(new_pos, goal) < goal_tolerance:
        return extract_path(new_node)
```

#### Post-Processing

After finding a path, apply two optimizations:

**1. Path Smoothing (Shortcut Method)**
```python
def smooth_path(path):
    """Remove unnecessary waypoints using line-of-sight checks"""
    smoothed = [path[0]]
    i = 0

    while i < len(path) - 1:
        # Try to connect to furthest visible point
        for j in range(len(path) - 1, i, -1):
            if is_path_collision_free(path[i], path[j]):
                smoothed.append(path[j])
                i = j
                break

    return smoothed
```

**2. Path Densification**
```python
def densify_path(path, spacing=0.3):
    """Insert waypoints every 0.3m for smooth control"""
    densified = []

    for i in range(len(path) - 1):
        start, end = path[i], path[i+1]
        distance = np.linalg.norm(end - start)
        num_points = int(distance / spacing)

        for j in range(num_points):
            alpha = j / num_points
            densified.append((1 - alpha) * start + alpha * end)

    densified.append(path[-1])
    return densified
```

### Collision Checking

```python
def _is_path_collision_free(self, start, end):
    """Sample along edge and check clearance"""
    distance = np.linalg.norm(end - start)
    num_checks = max(2, int(distance / 0.15))  # Check every 15cm

    for i in range(num_checks + 1):
        alpha = i / num_checks
        check_point = (1 - alpha) * start + alpha * end

        # Query nearest obstacle
        dist_to_obstacle, _ = self.kdtree.query(check_point)

        # Require robot radius clearance
        if dist_to_obstacle < self.robot_radius:
            return False

    return True
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `step_size` | 0.4m | Maximum edge length (η) |
| `goal_bias` | 0.5 | Probability of sampling goal (50%) |
| `max_iterations` | 3000 | Planning timeout |
| `gamma` | 2.0 | RRT* rewiring parameter |
| `goal_tolerance` | 0.3m | Distance to consider goal reached |
| `robot_radius` | 0.22m | Collision clearance |

### Performance

| Metric | Value |
|--------|-------|
| **Planning Time** | 0.5 - 2 seconds (environment dependent) |
| **Path Quality** | Asymptotically optimal (approaches shortest path) |
| **Success Rate** | >95% in maze environments |
| **Rewiring Operations** | ~500-1000 per successful plan |

---

## Pure Pursuit Controller

**File:** `src/navigation/navigation/smoothed_pure_pursuit.py`
**Class:** `SmoothedPurePursuit`

### Overview

Pure Pursuit is a **path-tracking controller** that computes velocity commands to follow a given path. This implementation uses:

1. **Constant lookahead distance** (not adaptive) for consistent behavior
2. **Curvature-based velocity adaptation** for smooth turns
3. **Exponential moving average (EMA)** for angular velocity smoothing

### Constant Lookahead

```python
lookahead_distance = 1.2m  # Fixed value
```

**Why constant?**
- **Consistency:** Behavior doesn't change with speed variations
- **Simplicity:** One less parameter to tune
- **Stability:** Predictable control inputs for EKF sensor fusion

The controller searches forward along the path to find the first waypoint beyond 1.2m from the robot.

### Pure Pursuit Geometry

```
Robot position: (x_r, y_r)
Robot heading: θ_r
Lookahead point: (x_l, y_l)

       (x_l, y_l)  ← Lookahead point
           •
          /|
         / |
        /  | L sin(α)
       /   |
      / α  |
     •─────┘
   (x_r, y_r)

Curvature: κ = 2 sin(α) / L

Angular velocity: ω = v · κ
```

Where:
- `L` = distance to lookahead point (≥ 1.2m)
- `α` = angle between robot heading and lookahead point
- `v` = linear velocity

### Algorithm

```python
def compute_control(self, robot_pos, robot_yaw, path, current_waypoint_index):
    # 1. Find lookahead point
    lookahead_point = find_point_at_distance(path, robot_pos, lookahead=1.2)

    # 2. Compute angle to lookahead
    dx = lookahead_point[0] - robot_pos[0]
    dy = lookahead_point[1] - robot_pos[1]
    target_angle = atan2(dy, dx)

    # Angle error (normalized to [-π, π])
    alpha = normalize_angle(target_angle - robot_yaw)

    # 3. Compute curvature
    distance = sqrt(dx**2 + dy**2)
    if distance < 0.05:  # 5cm safety margin
        return 0.0, 0.0

    curvature = (2.0 * sin(alpha)) / distance

    # 4. Compute desired angular velocity
    omega_desired = v_adaptive * curvature
    omega_desired = clip(omega_desired, -max_w, max_w)

    # 5. Smooth angular velocity (EMA)
    omega_smooth = alpha * omega_desired + (1 - alpha) * omega_smooth_prev

    # 6. Adapt linear velocity based on curvature
    omega_magnitude = abs(omega_smooth)
    velocity_scale = 1.0 - velocity_gain * (omega_magnitude / max_w)
    velocity_scale = clip(velocity_scale, 0.3, 1.0)

    v_adaptive = v_max * velocity_scale
    v_adaptive = clip(v_adaptive, v_min, v_max)

    return v_adaptive, omega_smooth
```

### Angular Velocity Smoothing

Uses **Exponential Moving Average (EMA)** to prevent jerky motion:

```python
omega_smooth[t] = α * omega_desired[t] + (1 - α) * omega_smooth[t-1]
```

Where `α = 0.6` (smoothing factor).

**Effect:**
- `α = 1.0` → No smoothing (instant response)
- `α = 0.0` → Infinite smoothing (no response)
- `α = 0.6` → Balanced smoothing (60% new, 40% previous)

**Benefits:**
1. Reduces control chatter
2. More predictable for EKF (smoother IMU readings)
3. Prevents wheel slip on sharp direction changes

### Velocity Adaptation

Linear velocity reduces on sharp turns:

```python
velocity_scale = 1.0 - velocity_gain * (|ω| / ω_max)

Where:
- velocity_gain = 0.7  (70% reduction at max turn rate)
- ω_max = 0.5 rad/s    (maximum angular velocity)
```

**Example:**
- Straight line: `|ω| = 0` → `scale = 1.0` → `v = 0.18 m/s` (max)
- Medium turn: `|ω| = 0.25` → `scale = 0.65` → `v = 0.12 m/s`
- Sharp turn: `|ω| = 0.5` → `scale = 0.3` → `v = 0.08 m/s` (min)

This ensures the robot slows down on tight corners for stability.

### Goal Handling

```python
dist_to_goal = norm(path[-1] - robot_pos)

if dist_to_goal < goal_tolerance:
    # Stop at goal
    return 0.0, 0.0
```

When within 0.3m of the final waypoint, the controller stops.

### Parameters

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `lookahead_distance` | 1.2m | Constant lookahead for path tracking |
| `max_linear_velocity` | 0.18 m/s | Top speed |
| `min_linear_velocity` | 0.08 m/s | Minimum speed (on sharp turns) |
| `max_angular_velocity` | 0.5 rad/s | Maximum turn rate (~28°/s) |
| `angular_smoothing_factor` | 0.6 | EMA alpha for ω smoothing |
| `goal_tolerance` | 0.3m | Distance to consider goal reached |
| `velocity_gain` | 0.7 | Curvature-based speed reduction |

---

## State Machine

**File:** `src/navigation/navigation/simple_navigation.py`

### States

```python
class State(Enum):
    WAIT_FOR_MAP = 0        # Waiting for first map from SLAM
    DETECT_FRONTIERS = 1    # Finding exploration goals
    PLAN_PATH = 2           # Computing RRT* path to frontier
    EXECUTE_PATH = 3        # Following path with Pure Pursuit
    DONE = 4                # Exploration complete (no frontiers)
```

### State Transitions

```
START
  ↓
WAIT_FOR_MAP ──────────────────────┐
  ↓ (map received)                 │
DETECT_FRONTIERS                   │
  ↓ (frontiers found)              │ (no frontiers)
PLAN_PATH                          │
  ↓ (path computed)                │
EXECUTE_PATH ──────────────────────┤
  ↓ (goal reached)  ↑              │
  └─────────────────┘              │
                                   ↓
                                 DONE
```

### Replanning Triggers

The system can replan while in `EXECUTE_PATH` state if:

#### 1. Stuck Detection
```python
# Check if robot has moved in last 5 seconds
time_since_last_check = current_time - last_stuck_check_time

if time_since_last_check > 5.0:
    distance_moved = norm(current_pos - last_check_pos)

    if distance_moved < 0.1m:
        # Robot stuck! Replan to different frontier
        trigger_replan()
```

#### 2. Path Deviation
```python
# Check every 4 seconds
if time_since_deviation_check > 4.0:
    min_distance_to_path = min([norm(robot_pos - waypoint)
                                 for waypoint in path])

    if min_distance_to_path > 1.5m:
        # Robot deviated from path (e.g., reactive avoidance)
        trigger_replan()
```

#### 3. Better Frontier Found
```python
# Continuously monitor for better goals
new_frontiers = detect_frontiers()
current_frontier_score = current_goal.score

for frontier in new_frontiers:
    score_improvement = frontier.score / current_frontier_score

    if score_improvement > 1.5:  # 50% improvement
        # Found significantly better frontier
        trigger_replan(frontier)
```

#### 4. Timeout
```python
# Maximum time in EXECUTE_PATH state
time_in_execute = current_time - execute_path_start_time

if time_in_execute > 30.0:  # 30 seconds
    # Taking too long, replan
    trigger_replan()
```

### Exploration Termination

Exploration ends when no valid frontiers remain:

```python
frontiers = detect_frontiers()

# Filter by minimum distance
valid_frontiers = [f for f in frontiers
                   if norm(f.position - robot_pos) > min_frontier_distance]

if len(valid_frontiers) == 0:
    state = State.DONE
    publish_zero_velocity()
```

Where `min_frontier_distance = 2.0m` prevents selecting frontiers too close to the robot.

---

## Reactive Obstacle Avoidance

**File:** `src/navigation/navigation/simple_navigation.py`

### LiDAR-Based Safety System

Runs in parallel with path following to prevent collisions:

```python
def scan_callback(self, msg: LaserScan):
    # Extract front arc (±60° = 120° total)
    num_ranges = len(msg.ranges)
    front_indices = get_front_arc_indices(num_ranges, angular_range=60°)

    front_ranges = [msg.ranges[i] for i in front_indices]

    # Find minimum distance in front arc
    min_distance = min(front_ranges)

    # Store for reactive control
    with self.scan_lock:
        self.scan_data = min_distance
```

### Velocity Modulation

```python
def apply_reactive_avoidance(self, v_desired, w_desired):
    with self.scan_lock:
        if self.scan_data is None:
            return v_desired, w_desired

        min_distance = self.scan_data

    # Emergency stop
    if min_distance < scan_emergency_distance:  # 0.3m
        return 0.0, 0.0

    # Slow down
    elif min_distance < scan_danger_distance:  # 0.5m
        # Linear scaling: 0.3m → 0%, 0.5m → 100%
        scale = (min_distance - 0.3) / (0.5 - 0.3)
        v_safe = v_desired * scale
        return v_safe, w_desired

    # Safe distance
    else:
        return v_desired, w_desired
```

### Response Time

| Metric | Value |
|--------|-------|
| **LiDAR Rate** | 10 Hz |
| **Control Loop** | 10 Hz |
| **End-to-End Latency** | <100 ms |

The system can detect and respond to obstacles within 100ms, crucial for safe navigation at 0.18 m/s (moves 18mm in 100ms).

---

## Parameters Summary

### Navigation Node Parameters

Defined in `simple_navigation.py:37-47`:

```python
self.declare_parameter('robot_name', 'tb3_1')
self.declare_parameter('robot_radius', 0.22)
self.declare_parameter('enable_reactive_avoidance', True)
self.declare_parameter('scan_danger_distance', 0.5)
self.declare_parameter('scan_emergency_distance', 0.3)
self.declare_parameter('scan_angular_range', 60.0)
self.declare_parameter('enable_path_deviation_check', True)
self.declare_parameter('path_deviation_threshold', 1.5)
self.declare_parameter('path_deviation_check_interval', 4.0)
```

### Internal Parameters

```python
# Replanning thresholds
replan_score_threshold = 0.50      # 50% score improvement triggers replan
replan_distance_threshold = 4.0    # 4m closer frontier triggers replan
min_frontier_distance = 2.0        # Minimum frontier selection distance

# Stuck detection
stuck_check_window = 5.0           # Check movement every 5s
stuck_distance_threshold = 0.1     # Must move at least 0.1m
stuck_timeout = 30.0               # Max time per path execution
```

### Component Parameters

| Component | Parameter | Value |
|-----------|-----------|-------|
| **Frontier Detector** | `grid_size` | 0.8m |
| | `clearance_multiplier` | 3.0 |
| | `dbscan_eps` | 1.0m |
| **RRT* Planner** | `step_size` | 0.4m |
| | `goal_bias` | 0.5 |
| | `max_iterations` | 3000 |
| | `gamma` | 2.0 |
| **Pure Pursuit** | `lookahead_distance` | 1.2m |
| | `max_linear_velocity` | 0.18 m/s |
| | `min_linear_velocity` | 0.08 m/s |
| | `max_angular_velocity` | 0.5 rad/s |
| | `angular_smoothing_factor` | 0.6 |
| | `velocity_gain` | 0.7 |

---

## Performance Metrics

| Metric | Value | Notes |
|--------|-------|-------|
| **Control Loop Rate** | 10 Hz | Publishes cmd_vel at 10 Hz |
| **Frontier Detection** | 1 Hz | Runs every second in DETECT_FRONTIERS state |
| **RRT* Planning Time** | 0.5 - 2s | Environment-dependent |
| **Replanning Frequency** | Variable | Triggered by conditions, not periodic |
| **Path Tracking Error** | <0.15m | RMS deviation from path |
| **Goal Reach Accuracy** | <0.3m | Defined by goal_tolerance |
| **Exploration Coverage** | 85-90% | Of reachable area in maze |
| **Obstacle Response Time** | <100ms | LiDAR scan to velocity update |

---

## Visualization

The navigation system publishes visualization topics for RViz:

### Published Topics

```python
# Velocity commands
/{robot_name}/cmd_vel (Twist)

# Planned path
/{robot_name}/planned_path (Path)

# Detected frontiers
/{robot_name}/frontier_markers (MarkerArray)
```

### RViz Configuration

Add these displays:
1. **Path** - Shows current RRT* trajectory
2. **MarkerArray** - Shows detected frontier clusters
3. **LaserScan** - Shows LiDAR data for obstacle avoidance

---

## Usage

### Launch with Full System

```bash
ros2 launch multi_robot_mapping full_system.launch.py
```

This starts Gazebo + SLAM + Navigation.

### Launch Navigation Only

```bash
# Assumes SLAM is already running
ros2 run navigation simple_navigation --ros-args \
  -p robot_name:=tb3_1 \
  -p robot_radius:=0.22 \
  -p enable_reactive_avoidance:=true
```

### Monitor Navigation

```bash
# View control commands
ros2 topic echo /tb3_1/cmd_vel

# Check path planning
ros2 topic echo /tb3_1/planned_path --once

# Monitor frontiers
ros2 topic echo /tb3_1/frontier_markers --once

# View logs
ros2 run rqt_console rqt_console
```

---

## Code References

| Component | File Location |
|-----------|---------------|
| **Main Node** | `src/navigation/navigation/simple_navigation.py` |
| **Frontier Detection** | `src/navigation/navigation/simple_frontiers.py` |
| **RRT* Planner** | `src/navigation/navigation/rrt_star.py` |
| **Pure Pursuit** | `src/navigation/navigation/smoothed_pure_pursuit.py` |

---

## Documentation Index

- **[01_SYSTEM_OVERVIEW.md](01_SYSTEM_OVERVIEW.md)** - Architecture, features, quick start
- **[02_SLAM_MAPPING.md](02_SLAM_MAPPING.md)** - Submap mapping, ICP, GPU acceleration
- **[03_EKF_SENSOR_FUSION.md](03_EKF_SENSOR_FUSION.md)** - Multi-sensor localization
- **[04_LOOP_CLOSURE_OPTIMIZATION.md](04_LOOP_CLOSURE_OPTIMIZATION.md)** - GTSAM backend
- **[05_NAVIGATION_MODULE.md](05_NAVIGATION_MODULE.md)** - This document
- **[06_ROS2_INTEGRATION.md](06_ROS2_INTEGRATION.md)** - Nodes, topics, launch files
- **[07_TURTLEBOT3_GAZEBO_PLATFORM.md](07_TURTLEBOT3_GAZEBO_PLATFORM.md)** - Robot platform, simulation
