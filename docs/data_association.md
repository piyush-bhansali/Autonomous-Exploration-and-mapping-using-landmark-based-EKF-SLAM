# Data Association Algorithm for Feature-Based EKF-SLAM

## Table of Contents
1. [Overview](#overview)
2. [Algorithm Purpose](#algorithm-purpose)
3. [Input and Output](#input-and-output)
4. [Detailed Step-by-Step Process](#detailed-step-by-step-process)
5. [Mathematical Formulation](#mathematical-formulation)
6. [Implementation Details](#implementation-details)
7. [Parameters and Tuning](#parameters-and-tuning)
8. [Edge Cases and Robustness](#edge-cases-and-robustness)
9. [Pseudocode](#pseudocode)
10. [Complexity Analysis](#complexity-analysis)
11. [Integration with EKF-SLAM](#integration-with-ekf-slam)

---

## Overview

Data association is the critical process of determining which observed features from the current sensor scan correspond to which landmarks already stored in the map. This is one of the fundamental challenges in SLAM, as incorrect associations lead to map corruption and localization failures.

The algorithm implements a **greedy nearest-neighbor matching** approach with **Mahalanobis distance gating** and **one-to-one correspondence constraints**. It ensures that each observation matches at most one landmark and vice versa.

**Key Properties:**
- **Statistical Gating:** Uses Mahalanobis distance with chi-square threshold to reject outliers
- **One-to-One Matching:** Each observation matches at most one landmark (prevents ambiguity)
- **Multi-stage Filtering:** Fast geometric tests before expensive covariance computations
- **Type Safety:** Walls only match walls, corners only match corners
- **Spatial Gap Checking:** Prevents matching spatially separated parallel walls

---

## Algorithm Purpose

**Goal:** Given:
- A set of features extracted from the current LiDAR scan (walls and corners)
- The current EKF state containing robot pose and landmark estimates
- The EKF covariance matrix representing uncertainty

**Produce:**
- A list of matched pairs: `(observation_index, landmark_id)`
- A list of unmatched observation indices (new landmarks)
- Optional: Extension information for wall landmark extent updates

**Why This Matters:**
- **Correct matches** → EKF update improves localization and reduces uncertainty
- **Incorrect matches** → Map corruption, divergence, catastrophic failure
- **Unmatched observations** → New landmarks to be initialized

---

## Input and Output

### Inputs

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `observed_features` | `List[Dict]` | Required | Features extracted from current scan. Each dict contains: <br>- `type`: `'wall'` or `'corner'`<br>- For walls: `rho`, `alpha`, `covariance`, `start_point`, `end_point`<br>- For corners: `position`, `covariance` |
| `ekf_slam` | `EKF_SLAM` | Required | EKF-SLAM object containing:<br>- `state`: State vector [x_r, y_r, θ_r, landmark_params...]<br>- `P`: Covariance matrix<br>- `landmarks`: Dict mapping landmark_id → metadata |
| `chi_sq_gate` | `float` | 5.99 | Chi-square threshold for 95% confidence, 2-DOF |
| `max_euclidean_dist` | `float` | 6.0 | Maximum distance (m) for considering a match |
| `wall_angle_tolerance` | `float` | 0.349 | Maximum angular difference (rad ≈ 20°) for wall matching |
| `wall_rho_tolerance` | `float` | 0.5 | Maximum perpendicular distance difference (m) for walls |
| `return_extension_info` | `bool` | False | Whether to return wall endpoint extension data |
| `feature_map` | `FeatureMap` | None | Optional feature map for spatial gap checking |
| `max_gap_ext` | `float` | 0.5 | Maximum spatial gap (m) for wall extent merging |

### Outputs

| Output | Type | Description |
|--------|------|-------------|
| `matched` | `List[Tuple[int, int]]` | List of `(observation_index, landmark_id)` pairs |
| `unmatched` | `List[int]` | List of observation indices with no match (new landmarks) |
| `extension_info` | `Dict[int, Dict]` | Optional: Wall endpoint data for extent updates. Keys are observation indices. |

### Example Return Values

```python
# Scenario: 5 observations, 10 landmarks in map
matched = [(0, 3), (1, 7), (4, 2)]  # obs 0→lm 3, obs 1→lm 7, obs 4→lm 2
unmatched = [2, 3]                   # obs 2 and 3 are new landmarks
extension_info = {
    0: {'new_start': array([1.2, 3.4]), 'new_end': array([2.5, 3.6])},
    1: {'new_start': array([0.8, 1.2]), 'new_end': array([1.1, 4.5])}
}
```

---

## Detailed Step-by-Step Process

### Step 1: Initialization

```python
matched = []          # Stores (obs_idx, landmark_id) pairs
unmatched = []        # Stores unmatched observation indices
extension_info = {}   # Stores wall extent update data
matched_landmarks = set()  # Tracks landmarks already matched (one-to-one constraint)
```

**Key Insight:** Using a `matched_landmarks` set prevents multiple observations from associating with the same landmark, enforcing one-to-one correspondence.

**Why One-to-One?**
- Prevents ambiguous associations
- Each landmark should correspond to one physical feature per scan
- Multiple observations of same feature get matched to the same landmark sequentially across scans

---

### Step 2: Early Exit for Empty Map

```python
if len(ekf_slam.landmarks) == 0:
    # All observations are new landmarks
    return matched, list(range(len(observed_features))), extension_info
```

**When:**
- First iteration of SLAM (no landmarks yet)
- After all landmarks have been pruned due to timeout

**Action:** Mark all observations as unmatched (they will be initialized as new landmarks in the main SLAM loop).

**File Reference:** `data_association.py:28-33`

---

### Step 3: Extract Robot Pose

```python
x_r, y_r, theta_r = ekf_slam.state[0:3]
```

The robot's pose is always stored in the first 3 elements of the state vector:
- **x_r:** Robot x-position in map frame (meters)
- **y_r:** Robot y-position in map frame (meters)
- **theta_r:** Robot heading in map frame (radians, ∈ [-π, π])

**File Reference:** `data_association.py:35`

---

### Step 4: Pre-compute Transformation Matrix

```python
c_r, s_r = np.cos(theta_r), np.sin(theta_r)
R_robot = np.array([[c_r, -s_r],
                    [s_r,  c_r]])
t_robot = np.array([x_r, y_r])
```

**Purpose:** Transform points from robot frame to map frame for spatial gap checking:

```
p_map = R_robot @ p_robot + t_robot
```

This is a standard 2D rotation + translation:

$$
\begin{bmatrix} x_{map} \\ y_{map} \end{bmatrix} =
\begin{bmatrix} \cos\theta_r & -\sin\theta_r \\ \sin\theta_r & \cos\theta_r \end{bmatrix}
\begin{bmatrix} x_{robot} \\ y_{robot} \end{bmatrix} +
\begin{bmatrix} x_r \\ y_r \end{bmatrix}
$$

**File Reference:** `data_association.py:37-40`

---

### Step 5: Iterate Over Each Observed Feature

For each observation, we search for the best matching landmark using a **multi-stage filtering** approach:

```python
for feat_idx, feature in enumerate(observed_features):
    best_landmark_id = None
    best_mahal_sq = float('inf')

    for landmark_id, lm_data in ekf_slam.landmarks.items():
        # Multi-stage filtering (detailed below)
```

**Strategy:** Greedy nearest-neighbor with progressive filtering:
1. Type filtering (O(1))
2. One-to-one constraint check (O(1))
3. Geometric pre-filtering (O(1))
4. Mahalanobis distance computation (O(k³) where k=2)
5. Spatial gap check for walls (O(1))

**File Reference:** `data_association.py:42-162`

---

### Step 5.1: Feature Type Filtering

```python
if feature['type'] != lm_data['feature_type']:
    continue  # Skip: walls can't match corners and vice versa
```

**Rationale:**
- Walls and corners have different observation models
- Incompatible dimensions (walls: 2D [ρ, α], corners: 2D [x, y] but different meaning)
- Prevents nonsensical matches

**File Reference:** `data_association.py:48-49`

---

### Step 5.2: One-to-One Constraint

```python
if landmark_id in matched_landmarks:
    continue  # Skip: this landmark already matched to another observation
```

**Rationale:**
- Each landmark should match at most one observation per scan
- If landmark already matched, try next landmark
- Enforces bijective mapping (one observation ↔ one landmark)

**Example Scenario:**
- Observation 1 matches landmark 5 (Mahalanobis distance = 2.3)
- Observation 2 also compatible with landmark 5 (Mahalanobis distance = 3.1)
- Landmark 5 already in `matched_landmarks`, so observation 2 must find a different landmark

**File Reference:** `data_association.py:51-53`

---

### Step 5.3: Geometric Pre-filtering (Fast Rejection)

Before computing expensive Mahalanobis distances, perform fast geometric tests:

#### For Walls:

```python
# Extract landmark parameters from state vector
lm_rho = ekf_slam.state[idx]      # perpendicular distance
lm_alpha = ekf_slam.state[idx + 1]  # normal angle

# Convert to robot frame
rho_pred = lm_rho - (x_r * np.cos(lm_alpha) + y_r * np.sin(lm_alpha))
alpha_pred = normalize_angle(lm_alpha - theta_r)

# Test 1: Distance check
if abs(rho_pred) > max_euclidean_dist:  # Default: 6.0 m
    continue

# Test 2: Angle check
alpha_diff = abs(normalize_angle(alpha_pred - feature['alpha']))
if alpha_diff > wall_angle_tolerance:  # Default: 20°
    continue

# Test 3: Perpendicular distance check (in map frame)
obs_rho_map, _ = robot_wall_to_map_frame(
    feature['rho'], feature['alpha'], x_r, y_r, theta_r
)
if abs(lm_rho - obs_rho_map) > wall_rho_tolerance:  # Default: 0.5 m
    continue
```

**Test 1 Explanation:** Check if wall is within sensor range
- `rho_pred`: Predicted perpendicular distance from robot to wall
- If > 6m, wall is too far (outside LiDAR range)

**Test 2 Explanation:** Check if walls are parallel (similar orientation)
- Compare predicted angle vs observed angle
- Tolerance: 20° (0.349 rad)
- Rejects walls facing different directions

**Test 3 Explanation:** Check if walls are at similar perpendicular distance
- Both compared in map frame (absolute frame)
- Tolerance: 0.5m
- Prevents matching near wall to far wall on opposite side

**File Reference:** `data_association.py:57-76`

#### For Corners:

```python
lm_x = ekf_slam.state[idx]
lm_y = ekf_slam.state[idx + 1]
dist_to_corner = np.sqrt((lm_x - x_r)**2 + (lm_y - y_r)**2)

if dist_to_corner > max_euclidean_dist:  # Default: 6.0 m
    continue
```

**Test Explanation:** Simple Euclidean distance check
- If corner landmark is > 6m from robot, it can't be observed
- Simpler than walls because corners are point features

**File Reference:** `data_association.py:78-84`

**Performance Impact:** These geometric tests typically reject 80-90% of candidate matches, avoiding expensive matrix operations.

---

### Step 6: Mahalanobis Distance Computation

This is the core statistical test that accounts for uncertainty.

#### Step 6.1: Build Observation Model

```python
H, z_pred = ekf_slam._build_observation(landmark_id)
```

This function constructs:
- **H:** Jacobian of observation function (sparse matrix)
  - Dimensions: (2 × state_dimension)
  - Only non-zero for robot pose and observed landmark
- **z_pred:** Predicted observation based on current state
  - For walls: `[rho_pred, alpha_pred]`
  - For corners: `[x_local, y_local]`

**File Reference:** `data_association.py:89-92`

**Implementation:** See thesis appendix lines 3052-3075 (walls), 3111-3158 (corners)

---

#### Step 6.2: Compute Innovation

```python
if feature['type'] == 'wall':
    z_obs = np.array([feature['rho'], feature['alpha']])
    innovation = z_obs - z_pred
    innovation[1] = normalize_angle(innovation[1])  # Wrap angle to [-π, π]
else:  # corner
    z_obs = feature['position']
    innovation = z_obs - z_pred
```

**Innovation** (also called residual) = measurement error:

$$
\boldsymbol{\nu} = \mathbf{z}_{obs} - \mathbf{z}_{pred}
$$

**Special Handling for Angles:**
- Angles must be wrapped to [-π, π]
- Example: Predicted α = 179°, observed α = -179°
  - Naive difference: -358° (wrong!)
  - Normalized difference: 2° (correct)

**File Reference:** `data_association.py:94-100`

---

#### Step 6.3: Compute Innovation Covariance

```python
S = H @ ekf_slam.P @ H.T + feature['covariance']
```

**Innovation covariance** combines two sources of uncertainty:

1. **State uncertainty:** $\mathbf{H}\mathbf{P}\mathbf{H}^T$
   - Propagates uncertainty from state covariance through observation model
   - Accounts for uncertainty in robot pose and landmark position

2. **Measurement noise:** $\mathbf{R}$ (stored as `feature['covariance']`)
   - Sensor noise (LiDAR noise)
   - Feature extraction error

Full formula:

$$
\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R}
$$

**File Reference:** `data_association.py:102`

---

#### Step 6.4: Compute Mahalanobis Distance

```python
try:
    S_inv = np.linalg.inv(S)
    mahal_dist_sq = innovation.T @ S_inv @ innovation
except np.linalg.LinAlgError:
    continue  # Skip if covariance is singular
```

**Mahalanobis Distance** measures statistical distance accounting for uncertainty:

$$
d_M^2 = \boldsymbol{\nu}^T \mathbf{S}^{-1} \boldsymbol{\nu}
$$

**Geometric Interpretation:**
- Euclidean distance normalized by covariance ellipse
- If innovation lies on 1-sigma ellipse → $d_M = 1$
- If innovation lies on 2-sigma ellipse → $d_M = 2$
- Direction matters: larger distance in low-uncertainty directions

**Why Squared Distance?**
- For Gaussian noise, $d_M^2 \sim \chi^2(n)$ where n = dimension
- Enables statistical hypothesis testing with chi-square distribution

**Singularity Handling:**
- If covariance is ill-conditioned, inversion fails
- This indicates numerical issues (over-confident estimate)
- Skip this landmark and try others

**File Reference:** `data_association.py:104-112`

---

### Step 7: Chi-Square Gating

```python
if mahal_dist_sq < chi_sq_gate and mahal_dist_sq < best_mahal_sq:
    # This is a candidate match
```

**Chi-square gate** (default: 5.99) corresponds to 95% confidence for 2-DOF distribution:

| Confidence Level | 2-DOF Threshold | Interpretation |
|------------------|-----------------|----------------|
| 90% | 4.61 | Accept 90% of true matches |
| 95% | 5.99 | Accept 95% of true matches (default) |
| 98% | 7.38 | Accept 98% of true matches |
| 99% | 9.21 | Accept 99% of true matches |

**Hypothesis Test:**
- **Null hypothesis H₀:** Observation and landmark correspond to the same feature
- **Test statistic:** $d_M^2 \sim \chi^2(2)$ under H₀
- **Decision rule:**
  - If $d_M^2 < 5.99$ → Accept H₀ (match)
  - If $d_M^2 \geq 5.99$ → Reject H₀ (no match)

**Greedy Selection:** Among all candidates passing the gate, select the one with **minimum Mahalanobis distance**.

**File Reference:** `data_association.py:114`

**Visualization:**

```
       d_M²
        │
        │     ╱╲
        │    ╱  ╲
        │   ╱    ╲
        │  ╱      ╲      χ²(2) distribution
        │ ╱        ╲
        │╱__________╲___
        └────────────────
         0    5.99        Accept region: d_M² < 5.99
              ↑
         Chi-square gate
```

---

### Step 8: Spatial Gap Check (Wall-Specific)

For walls, we perform an additional geometric test to prevent matching spatially separated wall segments that happen to be parallel:

```python
if feature_map is not None and feature['type'] == 'wall':
    # Transform observation endpoints to map frame
    obs_s = R_robot @ feature['start_point'] + t_robot
    obs_e = R_robot @ feature['end_point'] + t_robot

    # Get stored landmark's tangent vector
    alpha = wall_data['alpha']
    tangent = np.array([-np.sin(alpha), np.cos(alpha)])

    # Compute extents along tangent
    sto_lo, sto_hi = wall_data['t_min'], wall_data['t_max']  # Stored landmark
    obs_lo = min(np.dot(obs_s, tangent), np.dot(obs_e, tangent))
    obs_hi = max(np.dot(obs_s, tangent), np.dot(obs_e, tangent))

    # Compute gap between segments
    gap = max(0, max(obs_lo - sto_hi, sto_lo - obs_hi))

    if gap > max_gap_ext:  # Default: 0.5 m
        continue  # Spatially separated — reject match
```

**Problem Scenario:** Two parallel walls on opposite sides of a room
- Both have similar Hessian parameters (ρ, α)
- Both pass Mahalanobis gate
- But they are physically distinct walls

**Solution:** Check if the wall segments overlap or are close in **tangential coordinates**

**Tangential Coordinate System:**
- Origin: Wall's closest point to map origin
- Tangent axis: Along the wall direction
- Normal axis: Perpendicular to wall

**Gap Computation:**
```
Stored:     [====sto_lo========sto_hi====]
Observed:                                    [=obs_lo===obs_hi=]
                                            ↑
                                    gap = obs_lo - sto_hi

Stored:                                    [====sto_lo===sto_hi====]
Observed:   [=obs_lo===obs_hi=]
                              ↑
                      gap = sto_lo - obs_hi
```

**Threshold:** Default 0.5m allows small gaps (occlusions, discretization error)

**File Reference:** `data_association.py:116-136`

---

### Step 9: Record Best Match

```python
if best_landmark_id is not None:
    matched.append((feat_idx, best_landmark_id))
    matched_landmarks.add(best_landmark_id)  # Mark as matched

    # Store endpoint extension info for wall extent updates
    if return_extension_info and feature['type'] == 'wall':
        start_map = R_robot @ feature['start_point'] + t_robot
        end_map = R_robot @ feature['end_point'] + t_robot
        extension_info[feat_idx] = {
            'new_start': start_map,
            'new_end': end_map
        }
else:
    unmatched.append(feat_idx)  # No match found — new landmark
```

**Branch 1: Match Found**
- Add `(obs_idx, landmark_id)` to matched list
- Mark landmark as matched (prevent double-matching)
- Store wall endpoint data for extent merging (if requested)

**Branch 2: No Match Found**
- Add observation index to unmatched list
- Main SLAM loop will initialize this as a new landmark

**File Reference:** `data_association.py:141-162`

---

## Mathematical Formulation

### Observation Function for Walls (Hessian Normal Form)

Given:
- Landmark parameters in map frame: $(\rho_m, \alpha_m)$
- Robot pose: $(x_r, y_r, \theta_r)$

**Wall equation in map frame:**

$$
\rho_m = x \cos(\alpha_m) + y \sin(\alpha_m)
$$

**Predicted observation in robot frame:**

$$
\begin{align}
\rho_r &= \rho_m - (x_r \cos\alpha_m + y_r \sin\alpha_m) \\
\alpha_r &= \text{atan2}(\sin(\alpha_m - \theta_r), \cos(\alpha_m - \theta_r))
\end{align}
$$

**Observation vector:**

$$
\mathbf{h}_{wall}(\mathbf{x}_r, \mathbf{m}) =
\begin{bmatrix} \rho_r \\ \alpha_r \end{bmatrix}
$$

**Jacobian for walls:** See thesis appendix lines 3052-3075

---

### Observation Function for Corners (Cartesian)

Given:
- Landmark position in map frame: $(x_m, y_m)$
- Robot pose: $(x_r, y_r, \theta_r)$

**Translation:**

$$
\Delta \mathbf{p} =
\begin{bmatrix} x_m - x_r \\ y_m - y_r \end{bmatrix}
$$

**Rotation into robot frame:**

$$
\mathbf{h}_{corner}(\mathbf{x}_r, \mathbf{m}) =
\begin{bmatrix} x_L \\ y_L \end{bmatrix} =
\begin{bmatrix} \cos\theta_r & \sin\theta_r \\ -\sin\theta_r & \cos\theta_r \end{bmatrix}
\begin{bmatrix} x_m - x_r \\ y_m - y_r \end{bmatrix}
$$

Expanding:

$$
\begin{align}
x_L &= \cos(\theta_r)(x_m - x_r) + \sin(\theta_r)(y_m - y_r) \\
y_L &= -\sin(\theta_r)(x_m - x_r) + \cos(\theta_r)(y_m - y_r)
\end{align}
$$

**Jacobian for corners:** See thesis appendix lines 3111-3158

---

### Mahalanobis Distance

**Definition:**

$$
d_M^2 = (\mathbf{z} - \hat{\mathbf{z}})^T \mathbf{S}^{-1} (\mathbf{z} - \hat{\mathbf{z}})
$$

where:
- $\mathbf{z}$: Actual observation (measured)
- $\hat{\mathbf{z}}$: Predicted observation (from state estimate)
- $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R}$: Innovation covariance

**For 2D features (walls or corners):**

$$
d_M^2 =
\begin{bmatrix} \nu_1 & \nu_2 \end{bmatrix}
\begin{bmatrix} S_{11} & S_{12} \\ S_{21} & S_{22} \end{bmatrix}^{-1}
\begin{bmatrix} \nu_1 \\ \nu_2 \end{bmatrix}
$$

**Relationship to Euclidean Distance:**

If $\mathbf{S} = \sigma^2 \mathbf{I}$ (isotropic, uncorrelated):

$$
d_M^2 = \frac{1}{\sigma^2} \|\mathbf{z} - \hat{\mathbf{z}}\|^2 = \frac{d_{Euclidean}^2}{\sigma^2}
$$

---

### Chi-Square Gating

For 2D features (2-DOF), the gating threshold at 95% confidence:

$$
d_M^2 < \chi^2_{2, 0.05} = 5.99
$$

**Statistical Interpretation:**
- Under null hypothesis (match is correct), $d_M^2 \sim \chi^2(2)$
- 95% of true matches will have $d_M^2 < 5.99$
- 5% false rejection rate
- Controls false positive rate via threshold selection

**Cumulative Distribution Function:**

$$
P(d_M^2 \leq \tau) = \int_0^\tau \frac{x e^{-x/2}}{2} dx
$$

For $\tau = 5.99$: $P(d_M^2 \leq 5.99) = 0.95$

---

## Implementation Details

### File Location
**Primary Implementation:** `src/map_generation/map_generation/data_association.py:11-168`

### Function Signature

```python
def associate_landmarks(
    observed_features: List[Dict],
    ekf_slam,
    chi_sq_gate: float = 5.99,
    max_euclidean_dist: float = 6.0,
    wall_angle_tolerance: float = 0.349066,  # 20 degrees
    wall_rho_tolerance: float = 0.5,
    return_extension_info: bool = False,
    feature_map=None,
    max_gap_ext: float = 0.5
) -> Tuple[List[Tuple[int, int]], List[int], Dict]
```

---

### Key Data Structures

#### Observation Dict (Wall)

```python
{
    'type': 'wall',
    'rho': 1.5,              # perpendicular distance (m)
    'alpha': 0.785,          # normal angle (rad)
    'covariance': np.array([[0.01, 0], [0, 0.01]]),  # 2×2 matrix
    'start_point': np.array([x1, y1]),  # robot frame
    'end_point': np.array([x2, y2])     # robot frame
}
```

#### Observation Dict (Corner)

```python
{
    'type': 'corner',
    'position': np.array([x, y]),  # robot frame (m)
    'covariance': np.array([[0.01, 0], [0, 0.01]])  # 2×2 matrix
}
```

#### Landmark Metadata

```python
{
    'feature_type': 'wall',  # or 'corner'
    'state_index': 5,        # index in state vector
    'last_observed': 42,     # scan number
    'observation_count': 15  # total observations
}
```

#### EKF State Vector Structure

```python
state = [
    x_r,           # Robot x-position (index 0)
    y_r,           # Robot y-position (index 1)
    theta_r,       # Robot heading (index 2)
    # Wall landmarks (Hessian form, 2 params each)
    rho_1,         # Wall 1 distance (index 3)
    alpha_1,       # Wall 1 angle (index 4)
    rho_2,         # Wall 2 distance (index 5)
    alpha_2,       # Wall 2 angle (index 6)
    # Corner landmarks (Cartesian, 2 params each)
    x_c1,          # Corner 1 x-position (index 7)
    y_c1,          # Corner 1 y-position (index 8)
    ...
]
```

---

## Parameters and Tuning

### Parameter Sensitivity Table

| Parameter | Default | Sensitivity | Effect of Increase | Effect of Decrease |
|-----------|---------|-------------|-------------------|-------------------|
| `chi_sq_gate` | 5.99 | **High** | More matches (risk false positives) | Fewer matches (risk missing true matches) |
| `max_euclidean_dist` | 6.0 m | Medium | More candidates considered (slower) | Fewer candidates (faster, but may miss distant landmarks) |
| `wall_angle_tolerance` | 20° | **High** | Accept less parallel walls | Require stricter parallelism |
| `wall_rho_tolerance` | 0.5 m | **High** | Accept walls at different distances | Require walls at same distance |
| `max_gap_ext` | 0.5 m | Medium | Allow larger gaps in wall extents | Require tighter spatial overlap |

---

### Tuning Guidelines

#### Scenario 1: Too Many False Negatives (Observations not matched when they should be)

**Symptoms:**
- Many unmatched observations (check log messages)
- Duplicate landmarks in map (same wall represented multiple times)
- High number of new landmark initializations

**Solution: Loosen constraints**
1. Increase `chi_sq_gate` to 7.38 (98% confidence)
2. Increase `wall_angle_tolerance` to 25° (0.436 rad)
3. Increase `wall_rho_tolerance` to 0.7 m
4. Increase `max_gap_ext` to 0.8 m

---

#### Scenario 2: Too Many False Positives (Wrong matches causing map corruption)

**Symptoms:**
- Map inconsistencies (walls in wrong places)
- Localization divergence (robot "jumps")
- Covariance matrix becoming ill-conditioned

**Solution: Tighten constraints**
1. Decrease `chi_sq_gate` to 4.61 (90% confidence)
2. Decrease `wall_angle_tolerance` to 15° (0.262 rad)
3. Decrease `wall_rho_tolerance` to 0.3 m
4. Decrease `max_gap_ext` to 0.3 m

---

#### Scenario 3: Performance Issues (Slow data association)

**Symptoms:**
- High computation time per scan
- Real-time factor < 1.0

**Solution: Optimize pre-filtering**
1. Decrease `max_euclidean_dist` to 4.0 m (if environment is smaller)
2. Ensure geometric pre-filtering is rejecting most candidates
3. Consider spatial indexing (KD-tree) for landmark lookup (not currently implemented)

---

### Recommended Tuning Process

1. **Start with defaults** and log statistics:
   ```python
   print(f"Matched: {len(matched)}/{len(observed_features)}")
   print(f"Unmatched: {len(unmatched)}/{len(observed_features)}")
   print(f"Match rate: {100*len(matched)/len(observed_features):.1f}%")
   ```

2. **Visualize matches in RViz:**
   - Matched observations should align with landmarks
   - Unmatched observations should be genuinely new features

3. **Validate with ground truth** (if available):
   - Compare matched pairs against known correspondences
   - Compute precision/recall

4. **Iterate parameters** based on observed failure modes

---

## Edge Cases and Robustness

### Case 1: No Landmarks in Map

**Scenario:** First SLAM iteration or all landmarks pruned

**Behavior:** Return all observations as unmatched

**Code Path:**
```python
if len(ekf_slam.landmarks) == 0:
    return matched, list(range(len(observed_features))), extension_info
```

**File Reference:** `data_association.py:28-33`

---

### Case 2: Singular Covariance Matrix

**Scenario:** Landmark covariance becomes ill-conditioned (nearly zero determinant)

**Cause:**
- Over-confident estimate (repeated observations with low noise)
- Numerical issues (eigenvalue collapse)

**Behavior:** Skip this landmark, try others

**Code Path:**
```python
try:
    S_inv = np.linalg.inv(S)
    mahal_dist_sq = innovation.T @ S_inv @ innovation
except np.linalg.LinAlgError:
    logger.debug(f"Singular covariance matrix for {feature['type']} obs {feat_idx}")
    continue
```

**File Reference:** `data_association.py:104-112`

**Prevention:** EKF covariance conditioning (eigenvalue clamping, regularization)

---

### Case 3: Feature Type Mismatch

**Scenario:** Trying to match wall observation to corner landmark (or vice versa)

**Behavior:** Skip immediately (fast rejection)

**Code Path:**
```python
if feature['type'] != lm_data['feature_type']:
    continue
```

**File Reference:** `data_association.py:48-49`

**Why This Matters:** Prevents invalid Jacobian computation (dimensions mismatch)

---

### Case 4: Multiple Observations, One Landmark

**Scenario:** Two wall segments detected as separate observations but belong to same landmark

**Example:**
```
Observed:  [===segment1===]  [===segment2===]
Landmark:  [=============combined=============]
```

**Behavior:**
- First observation matches → Added to `matched`
- Second observation finds same landmark in `matched_landmarks` → Skipped
- Second observation becomes unmatched

**Is This Correct?**
- Yes, for single-scan association
- Wall extent merging happens later (in FeatureMap update)
- Over multiple scans, segments get merged into continuous landmark

**Code Path:**
```python
if landmark_id in matched_landmarks:
    continue  # Already matched to another observation
```

**File Reference:** `data_association.py:52-53`

---

### Case 5: Parallel Walls Across a Corridor

**Scenario:** Two parallel walls on opposite sides of corridor

**Example:**
```
Wall 1: ρ=2.0m, α=0°   (left wall)
Wall 2: ρ=5.0m, α=0°   (right wall, 3m away)
```

**Problem:**
- Both walls have same angle (α=0°)
- Observation of right wall might pass Mahalanobis gate for left wall

**Solution:** Spatial gap check
```python
gap = max(0, max(obs_lo - sto_hi, sto_lo - obs_hi))
if gap > max_gap_ext:
    continue  # Reject match
```

**File Reference:** `data_association.py:116-136`

---

### Case 6: Observation Outside Sensor Range

**Scenario:** Landmark predicted to be in front of robot, but outside LiDAR range

**Example:**
- Landmark at 8m distance
- LiDAR max range: 3.5m

**Behavior:**
- No observation extracted for this landmark
- Landmark not matched (will be pruned after timeout)

**Handled By:** Main SLAM loop landmark pruning (timeout after 25 scans)

---

### Case 7: Occlusion

**Scenario:** Landmark temporarily occluded by dynamic obstacle (person, furniture)

**Example:**
```
Robot → Person → Wall (landmark)
         ↑
      blocks LiDAR
```

**Behavior:**
- No observation of wall in this scan
- Landmark not matched
- Landmark's `last_observed` counter increments
- After 25 scans, landmark pruned

**Is This Correct?**
- Yes, for static maps
- Temporarily occluded landmarks get re-initialized when visible again
- Prevents stale landmarks in map

---

## Pseudocode

```
FUNCTION associate_landmarks(observed_features, ekf_slam, parameters):

    // ============================================================
    // STEP 1: Initialize data structures
    // ============================================================
    matched ← []                    // List of (obs_idx, landmark_id) tuples
    unmatched ← []                  // List of observation indices
    extension_info ← {}             // Dict for wall extent data
    matched_landmarks ← {}          // Set of matched landmark IDs

    // ============================================================
    // STEP 2: Early exit if map is empty
    // ============================================================
    IF ekf_slam.landmarks.size == 0 THEN
        RETURN ([], range(len(observed_features)), {})
    END IF

    // ============================================================
    // STEP 3: Extract robot pose
    // ============================================================
    (x_r, y_r, θ_r) ← ekf_slam.state[0:3]

    // ============================================================
    // STEP 4: Precompute transformation matrix
    // ============================================================
    R_robot ← [[cos(θ_r), -sin(θ_r)],
               [sin(θ_r),  cos(θ_r)]]
    t_robot ← [x_r, y_r]

    // ============================================================
    // STEP 5: For each observation, find best matching landmark
    // ============================================================
    FOR feat_idx, feature IN enumerate(observed_features):
        best_landmark_id ← NULL
        best_mahal_sq ← ∞

        // ========================================================
        // STEP 6: Iterate through all landmarks in map
        // ========================================================
        FOR landmark_id, lm_data IN ekf_slam.landmarks:

            // ====================================================
            // STEP 6.1: Feature type filtering
            // ====================================================
            IF feature.type ≠ lm_data.feature_type THEN
                CONTINUE                // Skip: incompatible types
            END IF

            // ====================================================
            // STEP 6.2: One-to-one constraint check
            // ====================================================
            IF landmark_id IN matched_landmarks THEN
                CONTINUE                // Skip: already matched
            END IF

            // ====================================================
            // STEP 6.3: Geometric pre-filtering
            // ====================================================
            idx ← lm_data.state_index

            IF feature.type == 'wall' THEN
                // Extract landmark parameters
                (ρ_m, α_m) ← ekf_slam.state[idx:idx+2]

                // Convert to robot frame
                ρ_pred ← ρ_m - (x_r·cos(α_m) + y_r·sin(α_m))
                α_pred ← normalize_angle(α_m - θ_r)

                // Fast rejection test 1: Distance
                IF |ρ_pred| > max_euclidean_dist THEN
                    CONTINUE
                END IF

                // Fast rejection test 2: Angle
                α_diff ← |normalize_angle(α_pred - feature.α)|
                IF α_diff > wall_angle_tolerance THEN
                    CONTINUE
                END IF

                // Fast rejection test 3: Perpendicular distance
                ρ_obs_map ← robot_wall_to_map_frame(feature.ρ, feature.α, x_r, y_r, θ_r)
                IF |ρ_m - ρ_obs_map| > wall_rho_tolerance THEN
                    CONTINUE
                END IF

            ELSE IF feature.type == 'corner' THEN
                // Extract landmark position
                (x_m, y_m) ← ekf_slam.state[idx:idx+2]

                // Fast rejection test: Euclidean distance
                dist ← ||[x_m, y_m] - [x_r, y_r]||
                IF dist > max_euclidean_dist THEN
                    CONTINUE
                END IF
            END IF

            // ====================================================
            // STEP 6.4: Build observation model (Jacobian + prediction)
            // ====================================================
            (H, z_pred) ← ekf_slam.build_observation(landmark_id)
            IF H is NULL THEN
                CONTINUE                // Failed to build observation
            END IF

            // ====================================================
            // STEP 6.5: Compute innovation (residual)
            // ====================================================
            IF feature.type == 'wall' THEN
                z_obs ← [feature.ρ, feature.α]
                innovation ← z_obs - z_pred
                innovation[1] ← normalize_angle(innovation[1])  // Wrap angle
            ELSE  // corner
                z_obs ← feature.position
                innovation ← z_obs - z_pred
            END IF

            // ====================================================
            // STEP 6.6: Compute innovation covariance
            // ====================================================
            S ← H · P · H^T + feature.covariance

            // ====================================================
            // STEP 6.7: Compute Mahalanobis distance
            // ====================================================
            TRY:
                S_inv ← inverse(S)
                mahal_sq ← innovation^T · S_inv · innovation
            CATCH LinAlgError:
                CONTINUE                // Singular covariance, skip
            END TRY

            // ====================================================
            // STEP 6.8: Chi-square gating and greedy selection
            // ====================================================
            IF mahal_sq < chi_sq_gate AND mahal_sq < best_mahal_sq THEN

                // ================================================
                // STEP 6.9: Spatial gap check (walls only)
                // ================================================
                IF feature.type == 'wall' AND feature_map is not NULL THEN
                    // Transform endpoints to map frame
                    obs_start_map ← R_robot · feature.start_point + t_robot
                    obs_end_map ← R_robot · feature.end_point + t_robot

                    // Get stored landmark data
                    wall_data ← feature_map.walls[landmark_id]
                    tangent ← [-sin(wall_data.α), cos(wall_data.α)]

                    // Project to tangential coordinates
                    obs_lo ← min(obs_start_map · tangent, obs_end_map · tangent)
                    obs_hi ← max(obs_start_map · tangent, obs_end_map · tangent)

                    // Compute gap
                    gap ← max(0, max(obs_lo - wall_data.t_max,
                                     wall_data.t_min - obs_hi))

                    IF gap > max_gap_ext THEN
                        CONTINUE        // Spatially separated, reject
                    END IF
                END IF

                // Accept as best candidate
                best_mahal_sq ← mahal_sq
                best_landmark_id ← landmark_id
            END IF
        END FOR  // landmarks

        // ========================================================
        // STEP 7: Record best match or mark as unmatched
        // ========================================================
        IF best_landmark_id is not NULL THEN
            // Match found
            matched.append((feat_idx, best_landmark_id))
            matched_landmarks.add(best_landmark_id)

            // Store extension info for wall extent updates
            IF return_extension_info AND feature.type == 'wall' THEN
                start_map ← R_robot · feature.start_point + t_robot
                end_map ← R_robot · feature.end_point + t_robot
                extension_info[feat_idx] ← {
                    'new_start': start_map,
                    'new_end': end_map
                }
            END IF
        ELSE
            // No match found → new landmark
            unmatched.append(feat_idx)
        END IF
    END FOR  // observations

    // ============================================================
    // STEP 8: Return results
    // ============================================================
    RETURN (matched, unmatched, extension_info)
END FUNCTION
```

---

## Complexity Analysis

### Time Complexity

**Overall:** $O(M \times N \times k^3)$

Where:
- $M$ = number of landmarks in map
- $N$ = number of observed features in current scan
- $k$ = dimension of observation (k=2 for walls and corners)

**Breakdown:**

| Operation | Per Match | Total |
|-----------|-----------|-------|
| Feature type check | $O(1)$ | $O(M \times N)$ |
| One-to-one check | $O(1)$ | $O(M \times N)$ |
| Geometric pre-filtering | $O(1)$ | $O(M \times N)$ |
| Build observation (H, z_pred) | $O(k^2)$ | $O(M \times N \times k^2)$ |
| Innovation covariance | $O(k^2 \times n)$ | $O(M \times N \times k^2 \times n)$ |
| Matrix inversion | $O(k^3)$ | $O(M \times N \times k^3)$ |
| Mahalanobis distance | $O(k^2)$ | $O(M \times N \times k^2)$ |
| Spatial gap check | $O(1)$ | $O(M \times N)$ |

**Dominant Term:** Matrix inversion $O(k^3)$ but k=2 is small

**Practical Complexity:** Closer to $O(M \times N)$ because:
- Geometric pre-filtering rejects 80-90% of candidates before expensive operations
- Sparse matrix structure in H reduces covariance computation

---

### Space Complexity

**Overall:** $O(M + N)$

**Breakdown:**
- `matched`: $O(N)$ worst case (all observations matched)
- `unmatched`: $O(N)$ worst case (no observations matched)
- `matched_landmarks`: $O(M)$ worst case (all landmarks matched)
- `extension_info`: $O(N)$ worst case (all walls have extension data)
- Temporary variables: $O(k^2)$ for H, S matrices (k=2 → constant)

---

### Performance Optimization Strategies

1. **Spatial Indexing:** Use KD-tree for landmark lookup (reduce M)
   ```python
   # Instead of iterating all landmarks:
   nearby_landmarks = kdtree.query_radius([x_r, y_r], max_euclidean_dist)
   for landmark_id in nearby_landmarks:
       # Process only nearby landmarks
   ```

2. **Sparse Matrix Operations:** Exploit sparsity in H
   - Only robot pose and observed landmark have non-zero blocks
   - Avoid full matrix multiplication

3. **Early Termination:** If best_mahal_sq very small, stop searching
   ```python
   if best_mahal_sq < 1.0:  # Very confident match
       break  # Don't check remaining landmarks
   ```

4. **Batch Processing:** Vectorize operations across multiple observations
   ```python
   # Compute innovations for all observations at once
   innovations = z_obs_batch - z_pred_batch
   ```

---

## Integration with EKF-SLAM

### Main SLAM Loop

```
1. ODOMETRY CALLBACK:
   - Receive wheel encoder data
   - Store as control input (δ_d, δ_θ)

2. LIDAR CALLBACK:
   - Extract features (walls, corners)
   → observed_features

3. EKF PREDICT:
   - Update state: x̄_t = f(x_{t-1}, u_t)
   - Update covariance: P̄_t = F_t P_{t-1} F_t^T + G_t Q_t G_t^T

4. *** DATA ASSOCIATION *** ← This algorithm
   - Input: observed_features, ekf_slam
   - Output: matched, unmatched

5. EKF UPDATE (for each matched pair):
   FOR (obs_idx, landmark_id) IN matched:
       - Compute innovation: ν = z - h(x̄_t)
       - Compute Kalman gain: K = P̄_t H^T S^{-1}
       - Update state: x_t = x̄_t + K ν
       - Update covariance: P_t = (I - K H) P̄_t (I - K H)^T + K R K^T

6. LANDMARK INITIALIZATION (for each unmatched):
   FOR obs_idx IN unmatched:
       - Initialize new landmark in state vector
       - Initialize covariance with high uncertainty

7. LANDMARK PRUNING:
   FOR landmark_id IN ekf_slam.landmarks:
       IF landmark.last_observed > 25 scans:
           - Remove from state vector
           - Remove from covariance matrix

8. FEATURE MAP UPDATE:
   - Update wall extents using extension_info
   - Merge overlapping segments

9. SUBMAP GENERATION:
   IF scan_count % 50 == 0:
       - Generate point cloud from landmarks
       - Perform ICP alignment with global map
       - Update global map
```

---

### Data Flow Diagram

```
┌─────────────────┐
│  LiDAR Scan     │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Feature Extractor│
└────────┬────────┘
         │ observed_features
         ▼
┌─────────────────────────────────────────┐
│                                         │
│       DATA ASSOCIATION                  │◄─── EKF state (x, P)
│                                         │◄─── Landmark registry
│  • Type filtering                       │
│  • Geometric gating                     │
│  • Mahalanobis distance                 │
│  • Chi-square test                      │
│  • Spatial gap check                    │
│                                         │
└─────────┬───────────────────┬───────────┘
          │                   │
          │ matched           │ unmatched
          ▼                   ▼
┌─────────────────┐   ┌─────────────────┐
│  EKF Update     │   │ Landmark Init   │
│                 │   │                 │
│ • Innovation    │   │ • Add to state  │
│ • Kalman gain   │   │ • Initialize P  │
│ • State update  │   │                 │
│ • Cov update    │   │                 │
└─────────────────┘   └─────────────────┘
```

---

## References

### Code References

- **Primary implementation:** `src/map_generation/map_generation/data_association.py:11-168`
- **Wall observation model:** Thesis appendix lines 3004-3075
- **Corner observation model:** Thesis appendix lines 3076-3158
- **Transform utilities:** `src/map_generation/map_generation/transform_utils.py`

### Theoretical Background

- **Mahalanobis Distance:** Data association lines 102-112
- **Chi-square Gating:** Threshold 5.99 for χ²(2) at 95% confidence
- **EKF Update:** Thesis lines 1476-1574
- **SLAM Problem:** Thesis lines 444-660

### Literature

- **Probabilistic Robotics** (Thrun, Burgard, Fox, 2005)
  - Chapter 10: Data Association
  - Chapter 11: EKF-SLAM

- **SLAM Tutorial** (Durrant-Whyte & Bailey, 2006)
  - Section on nearest-neighbor data association

---

## Glossary

| Term | Definition |
|------|------------|
| **Data Association** | Process of determining correspondence between observations and map landmarks |
| **Innovation** | Difference between observed and predicted measurement (residual) |
| **Mahalanobis Distance** | Statistical distance metric accounting for covariance |
| **Chi-square Gating** | Hypothesis test using χ² distribution to accept/reject matches |
| **One-to-One Matching** | Constraint that each observation matches at most one landmark |
| **Greedy Nearest-Neighbor** | Algorithm that selects closest match for each observation independently |
| **False Positive** | Incorrect match (matching unrelated observation to landmark) |
| **False Negative** | Missed match (failing to match correct observation-landmark pair) |
| **Jacobian (H)** | Matrix of partial derivatives of observation function |
| **Innovation Covariance (S)** | Combined uncertainty from state estimate and measurement noise |
| **Spatial Gap Check** | Geometric test to prevent matching spatially separated parallel walls |

---

**Document Version:** 1.0
**Last Updated:** 2026-02-25
**Author:** Generated from implementation analysis
**File:** `/home/piyush/thesis_ws/docs/data_association.md`
