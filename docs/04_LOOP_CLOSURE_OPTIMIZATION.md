# Loop Closure Detection & Pose Graph Optimization
## GTSAM-Based Backend with Scan Context Features

**Modules:** `feature_extractor`, `loop_closure_detector`, `gtsam_optimizer`
**Code:**
- `src/map_generation/map_generation/feature_extractor.py`
- `src/map_generation/map_generation/loop_closure_detector.py`
- `src/map_generation/map_generation/gtsam_optimizer.py`

---

## Table of Contents

1. [Overview](#overview)
2. [Feature Extraction (Scan Context)](#feature-extraction-scan-context)
3. [Loop Closure Detection](#loop-closure-detection)
4. [Pose Graph Optimization (GTSAM)](#pose-graph-optimization-gtsam)
5. [Integration](#integration)

---

## Overview

Loop closure detection enables the robot to recognize previously visited locations and correct accumulated drift by optimizing the pose graph globally.

### Three-Stage Pipeline

```
┌─────────────────┐
│  New Submap     │
│  (Point Cloud)  │
└────────┬────────┘
         │
         v
┌─────────────────────────────────┐
│  Stage 1: Feature Extraction    │
│  - Scan Context (global)        │
│  - Geometric features (local)   │
└────────┬────────────────────────┘
         │
         v
┌─────────────────────────────────┐
│  Stage 2: Loop Closure Detection│
│  - Scan Context similarity      │
│  - Geometric verification       │
│  - ICP refinement               │
└────────┬────────────────────────┘
         │ (if loop detected)
         v
┌─────────────────────────────────┐
│  Stage 3: Pose Graph Optimization│
│  - Add loop constraint (GTSAM)  │
│  - Optimize all poses           │
│  - Update global map            │
└─────────────────────────────────┘
```

**Key Innovation:** Hybrid approach combines rotation-invariant global descriptor (Scan Context) with local geometric verification for robust place recognition.

---

## Feature Extraction (Scan Context)

**Module:** `feature_extractor.py`
**Purpose:** Convert 3D point cloud to rotation-invariant descriptor for place recognition

### Scan Context Algorithm

**Input:** 3D point cloud (submap)
**Output:** 1D descriptor vector (1200 dimensions)

#### Step 1: Point Cloud Centering

```python
# Center at centroid (not robot pose)
centroid = np.mean(points[:, :2], axis=0)  # [cx, cy]
centered_points = points[:, :2] - centroid
```

**Why centroid?** Makes descriptor represent geometric structure, independent of robot position.

#### Step 2: Polar Coordinate Transformation

```python
r = np.sqrt(centered_points[:, 0]**2 + centered_points[:, 1]**2)
theta = np.arctan2(centered_points[:, 1], centered_points[:, 0])
```

**Polar coordinates:**
- `r`: Radial distance from centroid [0, r_max]
- `theta`: Angular direction [-π, π]

#### Step 3: Binning

```python
# Configuration
num_rings = 20       # Radial bins
num_sectors = 60     # Angular bins
max_range = 10.0     # meters

# Compute bin indices
ring_idx = np.clip((r / max_range * num_rings).astype(int), 0, num_rings-1)
sector_idx = np.clip(((theta + np.pi) / (2*np.pi) * num_sectors).astype(int), 0, num_sectors-1)

# Create 2D grid (binary occupancy)
scan_context = np.zeros((num_rings, num_sectors))
scan_context[ring_idx, sector_idx] = 1.0
```

**Result:** 20×60 grid encoding spatial distribution in polar coordinates

#### Step 4: Flatten

```python
descriptor = scan_context.flatten()  # Shape: (1200,)
```

### Rotation Invariance

**Property:** Rotation in Cartesian space = circular shift in angular dimension

```
Robot rotates by Δθ
  ↓
Scan Context shifts by Δs = ⌊Δθ/(2π) · num_sectors⌋ sectors
```

**Matching strategy:** Compare against all 60 circular shifts → minimum distance = best match

### Implementation

```python
# feature_extractor.py:106-161
def _extract_scan_context(self, pcd: o3d.t.geometry.PointCloud):
    """Extract Scan Context descriptor"""
    # Get points from GPU
    points = pcd.point.positions.cpu().numpy()

    # Center at centroid
    centroid = np.mean(points[:, :2], axis=0)
    centered = points[:, :2] - centroid

    # Polar coordinates
    r = np.linalg.norm(centered, axis=1)
    theta = np.arctan2(centered[:, 1], centered[:, 0])

    # Binning
    ring_idx = np.clip((r / self.max_range * self.num_rings).astype(int),
                       0, self.num_rings - 1)
    sector_idx = np.clip(((theta + np.pi) / (2*np.pi) * self.num_sectors).astype(int),
                         0, self.num_sectors - 1)

    # Create grid
    sc = np.zeros((self.num_rings, self.num_sectors))
    sc[ring_idx, sector_idx] = 1.0

    # Flatten
    descriptor = sc.flatten()

    return descriptor, {'grid': sc, 'centroid': centroid}
```

### Geometric Features

**Purpose:** Local discriminative features for verification

```python
# feature_extractor.py:183-224
def _extract_geometric(self, pcd: o3d.t.geometry.PointCloud, keypoints):
    """Extract eigenvalue-based shape descriptors at keypoints"""
    descriptors = []

    for kp in keypoints:
        # Find neighbors within radius
        neighbors = find_neighbors(kp, radius=0.5)

        # Compute covariance matrix
        cov = np.cov(neighbors.T)

        # Eigenvalue decomposition
        eigenvalues = np.linalg.eigvals(cov)
        eigenvalues = np.sort(eigenvalues)[::-1]  # Descending

        # Shape features
        linearity = (eigenvalues[0] - eigenvalues[1]) / eigenvalues[0]
        planarity = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0]
        sphericity = eigenvalues[2] / eigenvalues[0]

        descriptors.append([linearity, planarity, sphericity])

    return np.array(descriptors)
```

**Features per keypoint:**
- **Linearity:** Edge-like structures (corridors)
- **Planarity:** Wall-like structures
- **Sphericity:** Corner-like structures

---

## Loop Closure Detection

**Module:** `loop_closure_detector.py`
**Purpose:** Identify when robot revisits a previous location

### Two-Stage Verification

```
┌──────────────────────────┐
│ Stage 1: Scan Context    │
│ Coarse Filtering         │
│ - Fast similarity check  │
│ - Rotation-invariant     │
└────────┬─────────────────┘
         │ (Top K candidates)
         v
┌──────────────────────────┐
│ Stage 2: Geometric       │
│ Fine Verification        │
│ - RANSAC matching        │
│ - ICP refinement         │
└────────┬─────────────────┘
         │ (if verified)
         v
    Loop Closure!
```

### Stage 1: Scan Context Similarity

```python
# loop_closure_detector.py:98-143
def detect(self, current_features: Dict, submap_id: int):
    """Detect loop closures"""
    current_sc = current_features['scan_context']

    candidates = []

    # Compare with all previous submaps
    for prev_id, prev_features in self.submap_database.items():
        if submap_id - prev_id < 50:  # Skip recent submaps
            continue

        prev_sc = prev_features['scan_context']

        # Compute cosine similarity with rotation search
        best_similarity = 0.0
        best_shift = 0

        for shift in range(self.num_sectors):  # 60 rotations
            shifted_sc = np.roll(prev_sc, shift)
            similarity = cosine_similarity(current_sc, shifted_sc)

            if similarity > best_similarity:
                best_similarity = similarity
                best_shift = shift

        # Threshold
        if best_similarity > 0.85:  # High similarity required
            candidates.append({
                'id': prev_id,
                'similarity': best_similarity,
                'rotation': best_shift * (2*np.pi / self.num_sectors)
            })

    # Return top 3 candidates
    candidates.sort(key=lambda x: x['similarity'], reverse=True)
    return candidates[:3]
```

**Threshold:** 0.85 cosine similarity (85% match)

### Stage 2: Geometric Verification

```python
# loop_closure_detector.py:240-320
def _verify_geometric(self, candidate):
    """Verify loop closure with geometric matching"""
    current_keypoints = self.current_features['keypoints']
    current_descriptors = self.current_features['geometric']

    prev_keypoints = self.submap_database[candidate['id']]['keypoints']
    prev_descriptors = self.submap_database[candidate['id']]['geometric']

    # 1. Feature matching (nearest neighbor)
    matches = []
    for i, desc1 in enumerate(current_descriptors):
        distances = np.linalg.norm(prev_descriptors - desc1, axis=1)
        j = np.argmin(distances)

        if distances[j] < 0.1:  # Threshold
            matches.append((current_keypoints[i], prev_keypoints[j]))

    if len(matches) < 10:  # Minimum inliers
        return None

    # 2. RANSAC for outlier rejection
    best_inliers = ransac(matches, iterations=100, threshold=0.2)

    if len(best_inliers) < 8:
        return None

    # 3. ICP refinement
    transform = self._refine_with_icp(
        source_pcd=current_submap,
        target_pcd=prev_submap,
        initial_guess=ransac_transform
    )

    # 4. Validate fitness
    if transform['fitness'] > 0.7:  # 70% overlap
        return transform
    else:
        return None
```

### ICP Refinement

```python
# loop_closure_detector.py:365-403
def _refine_with_icp(self, source_pcd, target_pcd, initial_guess):
    """Refine relative pose with ICP"""
    reg_result = o3d.t.pipelines.registration.icp(
        source=source_pcd,
        target=target_pcd,
        max_correspondence_distance=0.5,
        init_source_to_target=initial_guess,
        estimation_method=o3d.t.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.t.pipelines.registration.ICPConvergenceCriteria(max_iteration=100)
    )

    T = reg_result.transformation.cpu().numpy()

    return {
        'transformation': T,
        'fitness': reg_result.fitness,
        'rmse': reg_result.inlier_rmse
    }
```

---

## Pose Graph Optimization (GTSAM)

**Module:** `gtsam_optimizer.py`
**Purpose:** Globally optimize all submap poses given loop closure constraints

### Pose Graph Structure

```
Nodes: Submap poses (x, y, θ)
Edges: Constraints between submaps

Types of Edges:
1. Odometry: Sequential submaps (submap_i → submap_i+1)
2. Loop Closure: Detected revisits (submap_i → submap_j, j << i)
```

### GTSAM Formulation

```python
# gtsam_optimizer.py:35-120
import gtsam

class GTSAMOptimizer:
    def __init__(self):
        self.graph = gtsam.NonlinearFactorGraph()
        self.initial_estimates = gtsam.Values()
        self.current_estimate = gtsam.Values()

        # Noise models
        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.1, 0.1, 0.05]))
        self.loop_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.02]))

    def add_submap(self, submap_id: int, pose: dict):
        """Add new submap node"""
        pose_gtsam = gtsam.Pose2(pose['x'], pose['y'], pose['theta'])

        # Prior on first submap (anchor)
        if submap_id == 0:
            prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.01, 0.01, 0.01]))
            self.graph.add(gtsam.PriorFactorPose2(0, pose_gtsam, prior_noise))

        self.initial_estimates.insert(submap_id, pose_gtsam)

    def add_odometry_constraint(self, from_id: int, to_id: int, relative_pose: dict):
        """Add sequential odometry edge"""
        rel_pose_gtsam = gtsam.Pose2(
            relative_pose['dx'],
            relative_pose['dy'],
            relative_pose['dtheta']
        )

        factor = gtsam.BetweenFactorPose2(
            from_id, to_id,
            rel_pose_gtsam,
            self.odom_noise
        )
        self.graph.add(factor)

    def add_loop_closure(self, from_id: int, to_id: int, relative_pose: dict):
        """Add loop closure edge"""
        rel_pose_gtsam = gtsam.Pose2(
            relative_pose['dx'],
            relative_pose['dy'],
            relative_pose['dtheta']
        )

        factor = gtsam.BetweenFactorPose2(
            from_id, to_id,
            rel_pose_gtsam,
            self.loop_noise  # Higher confidence than odometry
        )
        self.graph.add(factor)

    def optimize(self):
        """Run Levenberg-Marquardt optimization"""
        params = gtsam.LevenbergMarquardtParams()
        params.setVerbosity('ERROR')
        params.setMaxIterations(100)

        optimizer = gtsam.LevenbergMarquardtOptimizer(
            self.graph,
            self.initial_estimates,
            params
        )

        self.current_estimate = optimizer.optimize()

        return self.current_estimate
```

### Optimization Process

```
Initial State: Submaps with odometry-based poses
  ↓
Add Loop Closure: New constraint between distant submaps
  ↓
Optimize: Minimize squared error over all edges
  ↓
Result: Corrected poses that satisfy both odometry and loop constraints
```

**Objective Function:**
```
argmin Σ ||f_i(x)||²_Σi

Where:
- f_i: Constraint i (odometry or loop closure)
- x: All submap poses
- Σi: Covariance (noise model) for constraint i
```

---

## Integration

### Workflow in Submap Stitcher

```python
# submap_stitcher.py:138-245
def process_submap(self, points: np.ndarray, submap_id: int):
    """Process new submap"""
    # 1. Create tensor PointCloud
    pcd = o3d.t.geometry.PointCloud(self.device)
    pcd.point.positions = o3c.Tensor(points, dtype=o3c.float32, device=self.device)

    # 2. Extract features
    features = self.feature_extractor.extract(pcd)

    # 3. Detect loop closures
    loop_candidates = self.loop_detector.detect(features, submap_id)

    # 4. For each candidate, verify and add to pose graph
    for candidate in loop_candidates:
        verified = self.loop_detector.verify(candidate)

        if verified:
            # Extract relative transformation
            T = verified['transformation']
            relative_pose = {
                'dx': T[0, 3],
                'dy': T[1, 3],
                'dtheta': np.arctan2(T[1, 0], T[0, 0])
            }

            # Add loop closure to GTSAM
            self.optimizer.add_loop_closure(
                from_id=submap_id,
                to_id=candidate['id'],
                relative_pose=relative_pose
            )

            # Optimize pose graph
            optimized_poses = self.optimizer.optimize()

            # Update all submap transforms
            self._update_submap_transforms(optimized_poses)

            # Recompute global map
            self._recompute_global_map()

    # 5. Add submap to database
    self.loop_detector.add_to_database(submap_id, features)

    # 6. Merge into global map
    self._merge_submap(pcd, submap_id)
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_submap_separation` | 50 | Min submaps between loop candidates |
| `scan_context_threshold` | 0.85 | Similarity threshold (0-1) |
| `geometric_threshold` | 0.1 | Feature matching distance |
| `min_inliers` | 10 | Minimum RANSAC inliers |
| `icp_fitness_threshold` | 0.7 | ICP overlap requirement |

---

## Performance

| Metric | Value |
|--------|-------|
| **Feature Extraction** | ~50 ms/submap |
| **Loop Detection** | ~100 ms (per submap check) |
| **ICP Verification** | ~200 ms |
| **GTSAM Optimization** | ~500 ms (100 submaps) |
| **Detection Rate** | 85%+ (distinctive locations) |
| **False Positive Rate** | <2% |

---

**Next:** Read [05_NAVIGATION_MODULE.md](05_NAVIGATION_MODULE.md) for frontier-based exploration and path planning.
