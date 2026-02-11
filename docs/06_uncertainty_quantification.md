# Uncertainty Quantification and Map Confidence

> **Note:** This document describes uncertainty quantification theory applicable to **both ICP-based and Feature-based mapping modes**. For complete system context, see `methodology_icp_mapping.md` and `methodology_feature_mapping.md`.

## Table of Contents
1. [Introduction](#1-introduction)
2. [Information Theory Foundations](#2-information-theory-foundations)
3. [Confidence Metric Design](#3-confidence-metric-design)
4. [Observability and Information Content](#4-observability-and-information-content)
5. [Thesis Analysis Methods](#5-thesis-analysis-methods)
6. [Implementation](#6-implementation)

---

## 1. Introduction

**Uncertainty quantification** enables **confidence-aware mapping**: the ability to assign reliability scores to different parts of the map. This is critical for:
- **Decision making:** Avoid uncertain regions
- **Active exploration:** Target areas needing more observations
- **System validation:** Quantify mapping quality
- **Multi-robot fusion:** Weight contributions by confidence

### 1.0 Historical Context and Information Theory in SLAM

The application of information theory to SLAM was pioneered by **Feder et al. (1999)**, who introduced the concept of **information-theoretic exploration** for autonomous robots. They established that robot motion should maximize information gain about both the robot's pose and the map.

**Key Insight** (Feder et al., 1999): *"The uncertainty in the robot's location and the map are fundamentally coupled through the covariance matrix. Minimizing map uncertainty requires actively reducing pose uncertainty, and vice versa."*

**Stachniss et al. (2005)** extended this to **Rao-Blackwellized particle filters**:
- Information gain computed for each particle trajectory
- Active exploration by selecting actions maximizing expected information
- Demonstrated in large-scale office environments

**Information-Theoretic Metrics:**

**Entropy** (Shannon, 1948; Cover & Thomas, 2006):

For a continuous Gaussian distribution:

$$
H(\mathbf{x}) = \frac{1}{2} \log((2\pi e)^d |\mathbf{P}|)
$$

**Mutual Information** (Feder et al., 1999):

Between robot pose and landmark:

$$
I(\mathbf{x}_r; \mathbf{m}_i) = H(\mathbf{x}_r) + H(\mathbf{m}_i) - H(\mathbf{x}_r, \mathbf{m}_i)
$$

This quantifies how much observing landmark $i$ reduces uncertainty about the robot pose.

### 1.1 Observability and Filter Consistency

**Huang et al. (2010)** discovered that standard EKF-SLAM suffers from **inconsistency** due to observability properties mismatch (see `01_ekf_slam_theory.md`, Section 9).

**Filter Consistency Definition** (Bar-Shalom et al., 2001):

A filter is **consistent** if:

$$
\mathbb{E}[(\mathbf{x} - \hat{\mathbf{x}})(\mathbf{x} - \hat{\mathbf{x}})^T] = \mathbf{P}
$$

That is, the **actual** error covariance matches the **estimated** covariance.

**Consistency Testing** (Bar-Shalom et al., 2001):

The **Normalized Estimation Error Squared (NEES)**:

$$
\epsilon_k = (\mathbf{x}_k - \hat{\mathbf{x}}_k)^T \mathbf{P}_k^{-1} (\mathbf{x}_k - \hat{\mathbf{x}}_k)
$$

For a consistent filter with $n$-dimensional state:

$$
\epsilon_k \sim \chi^2(n)
$$

**Acceptance Region** (95% confidence):

$$
\chi^2_{0.025}(n) \leq \epsilon_k \leq \chi^2_{0.975}(n)
$$

**Valencia et al. (2009)** used uncertainty quantification for **reliable path planning**:
- Compute path reliability from pose covariance along trajectory
- Plan paths that maximize probability of successful navigation
- Critical for safety-critical applications

### 1.2 Active SLAM and Information-Driven Exploration

**Sim & Roy (2005)** formalized **Active SLAM**:

Maximize mutual information between robot path and map:

$$
\mathbf{u}^* = \arg\max_{\mathbf{u}} I(\mathbf{m}; \mathbf{z}_{1:T} | \mathbf{u}_{1:T})
$$

**Trade-off**: Exploration (reduce map uncertainty) vs. Exploitation (use known map efficiently)

**Carrillo et al. (2012)** developed **D-optimality** criterion for active SLAM:

Minimize determinant of covariance (maximize information):

$$
\mathbf{u}^* = \arg\min_{\mathbf{u}} \log |\mathbf{P}(\mathbf{u})|
$$

This leads to actions that maximize observability of the state.

---

## 2. Information Theory Foundations

### 2.1 Entropy

**Differential entropy** of a Gaussian distribution:

$$
H(\mathbf{x}) = \frac{1}{2} \log((2\pi e)^d |\mathbf{P}|)
$$

Where:
- $d$: Dimension of state
- $|\mathbf{P}|$: Determinant of covariance matrix

**Interpretation:**
- High entropy → high uncertainty
- Low entropy → low uncertainty

### 2.2 Information Matrix

The **Fisher information matrix** is the inverse of covariance:

$$
\mathbf{I} = \mathbf{P}^{-1}
$$

**Properties:**
- $\mathbf{I}$ is positive definite (for non-singular $\mathbf{P}$)
- Large $\mathbf{I}$ → high information → low uncertainty
- Small $\mathbf{I}$ → low information → high uncertainty

**Information Gain:**

When a measurement is incorporated:

$$
\mathbf{I}^+ = \mathbf{I} + \mathbf{H}^T \mathbf{R}^{-1} \mathbf{H}
$$

Where:
- $\mathbf{H}$: Observation Jacobian
- $\mathbf{R}$: Measurement noise covariance

**Interpretation:** Each observation **adds** information.

### 2.3 Mutual Information

The **mutual information** between robot pose and landmark:

$$
I(\mathbf{x}_r; \mathbf{m}_i) = H(\mathbf{x}_r) + H(\mathbf{m}_i) - H(\mathbf{x}_r, \mathbf{m}_i)
$$

Using Gaussian properties:

$$
I(\mathbf{x}_r; \mathbf{m}_i) = \frac{1}{2} \log \frac{|\mathbf{P}_{rr}| |\mathbf{P}_{mm}|}{|\mathbf{P}|}
$$

Where:
- $\mathbf{P}_{rr}$: Robot pose covariance
- $\mathbf{P}_{mm}$: Landmark covariance
- $\mathbf{P}$: Joint covariance (robot + landmark)

**Interpretation:** High mutual information → strong correlation.

---

## 3. Confidence Metric Design

### 3.1 Robot Pose Uncertainty

The robot pose covariance is $\mathbf{P}_{rr} \in \mathbb{R}^{3 \times 3}$:

$$
\mathbf{P}_{rr} = \begin{bmatrix}
\sigma_x^2 & \sigma_{xy} & \sigma_{x\theta} \\
\sigma_{xy} & \sigma_y^2 & \sigma_{y\theta} \\
\sigma_{x\theta} & \sigma_{y\theta} & \sigma_\theta^2
\end{bmatrix}
$$

**Scalar Uncertainty Metrics:**

**1. Trace (Sum of Variances):**

$$
u_{\text{trace}} = \text{tr}(\mathbf{P}_{rr}) = \sigma_x^2 + \sigma_y^2 + \sigma_\theta^2
$$

**2. Determinant (Volume):**

$$
u_{\text{det}} = |\mathbf{P}_{rr}|
$$

**3. Maximum Eigenvalue:**

$$
u_{\text{max}} = \lambda_{\max}(\mathbf{P}_{rr})
$$

**4. Frobenius Norm:**

$$
u_{\text{frob}} = \|\mathbf{P}_{rr}\|_F = \sqrt{\sum_{i,j} P_{ij}^2}
$$

### 3.2 Information-Theoretic Confidence

Our chosen metric uses the **information matrix**:

$$
\mathbf{I}_{rr} = \mathbf{P}_{rr}^{-1}
$$

**Information Content:**

$$
\mathcal{I} = \text{tr}(\mathbf{I}_{rr})
$$

**Normalized Confidence Score:**

$$
\text{confidence} = 1 - \exp\left(-\frac{\mathcal{I}}{\tau}\right)
$$

Where $\tau = 10$ is a scaling parameter.

**Properties:**
- $\mathcal{I} = 0$ (infinite uncertainty) → confidence = 0
- $\mathcal{I} \to \infty$ (zero uncertainty) → confidence → 1
- Monotonic: more information → higher confidence
- Bounded: $\text{confidence} \in [0, 1]$

### 3.3 Submap Confidence

For each submap, we compute:

$$
c_{\text{submap}} = f(\mathbf{P}_{rr}, N_{\text{landmarks}})
$$

Where:
- $\mathbf{P}_{rr}$: Robot pose covariance at submap creation
- $N_{\text{landmarks}}$: Number of landmarks observed in submap

**Extended Formula:**

$$
\text{confidence}_{\text{submap}} = 1 - \exp\left(-\frac{\text{tr}(\mathbf{I}_{rr}) + \alpha N_{\text{landmarks}}}{\tau}\right)
$$

**Rationale:**
- More landmarks → more constraints → higher confidence
- Lower robot uncertainty → higher confidence
- Parameter $\alpha$ weights landmark contribution (e.g., 0.1)

---

## 4. Observability and Information Content

### 4.1 Observability Matrix

The **observability** of a system determines which state variables can be estimated from measurements. For a linear system:

$$
\mathbf{x}_{k+1} = \mathbf{F}_k \mathbf{x}_k + \mathbf{w}_k
$$

$$
\mathbf{z}_k = \mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k
$$

The observability matrix is:

$$
\mathcal{O} = \begin{bmatrix}
\mathbf{H}_0 \\
\mathbf{H}_1 \mathbf{F}_0 \\
\mathbf{H}_2 \mathbf{F}_1 \mathbf{F}_0 \\
\vdots
\end{bmatrix}
$$

The system is **observable** if $\text{rank}(\mathcal{O}) = n$ (full rank).

**Nullspace** (unobservable directions):

$$
\mathcal{N}(\mathcal{O}) = \{\mathbf{v} \mid \mathcal{O} \mathbf{v} = \mathbf{0}\}
$$

### 4.2 SLAM Observability Analysis

**Huang et al. (2010)** showed that SLAM has **3 unobservable directions**:

$$
\dim(\mathcal{N}(\mathcal{O})) = 3
$$

Corresponding to:
1. Global x-position (cannot determine absolute x)
2. Global y-position (cannot determine absolute y)
3. Global orientation (cannot determine absolute heading)

**Physical Interpretation**: We can build a **consistent relative map**, but cannot determine its **absolute pose** in the world without external reference.

### 4.3 Information Matrix and Observability

The **Fisher Information Matrix** quantifies observability:

$$
\mathbf{I} = \sum_{k=0}^T \mathbf{H}_k^T \mathbf{R}_k^{-1} \mathbf{H}_k
$$

**Eigenvalue Spectrum**:

For SLAM with $n$ landmarks (state dimension $3 + 2n$):
- **3 zero eigenvalues** → unobservable directions
- **$(2n)$ positive eigenvalues** → observable (relative landmark positions)

**Condition Number** (Huang et al., 2010):

$$
\kappa(\mathbf{I}) = \frac{\lambda_{\max}(\mathbf{I})}{\lambda_{\min}(\mathbf{I})}
$$

Large $\kappa$ indicates **ill-conditioned** system (numerical instability).

### 4.4 Information Contribution by Observation Type

Different observations contribute different amounts of information:

**Wall Observation** (Hessian form):

Information gain from observing wall $i$:

$$
\Delta \mathbf{I}_{\text{wall}} = \mathbf{H}_{\text{wall}}^T \mathbf{R}_{\text{wall}}^{-1} \mathbf{H}_{\text{wall}}
$$

**Constraints provided**:
- 1 DOF: Perpendicular distance to wall
- 1 DOF: Orientation relative to wall
- **Total**: 2 DOF constraint

**Corner Observation** (Cartesian):

Information gain from observing corner $j$:

$$
\Delta \mathbf{I}_{\text{corner}} = \mathbf{H}_{\text{corner}}^T \mathbf{R}_{\text{corner}}^{-1} \mathbf{H}_{\text{corner}}
$$

**Constraints provided**:
- 2 DOF: x, y position of corner
- **Total**: 2 DOF constraint

**Comparative Analysis**:

| Feature Type | DOF Constrained | Orientation Info | Position Info | Best Use Case |
|--------------|----------------|------------------|---------------|---------------|
| Wall | 2 | Strong | Perpendicular only | Corridors, rooms |
| Corner | 2 | None | Both x, y | Intersections, rooms |
| Two perpendicular walls | 3 | Full | Both directions | Full pose observability |

### 4.5 Degeneracy Detection

**Degenerate Configurations** (Castellanos et al., 2004):

Certain landmark configurations provide **redundant information**, leading to rank-deficient information matrix.

**Examples**:
1. **Collinear landmarks**: All landmarks on a line → cannot observe perpendicular motion
2. **Parallel walls only**: Cannot observe rotation
3. **Single landmark**: Cannot fully observe 3-DOF pose

**Detection**:

Compute smallest eigenvalue of information matrix:

$$
\lambda_{\min}(\mathbf{I}_{rr}) < \epsilon_{\text{threshold}}
$$

If true, configuration is **near-degenerate**.

**Mitigation**:
- Active exploration to observe diverse landmark geometries
- Require minimum 3 non-collinear landmarks for full observability
- Integrate odometry/IMU to constrain unobservable directions

---

## 5. Thesis Analysis Methods

### 5.1 Temporal Analysis

**Confidence Evolution:**

Plot confidence over time:

$$
c(t) = f(\mathbf{P}_{rr}(t))
$$

**Expected Behavior:**
- Initial exploration: confidence increases (gathering information)
- Loop closure: sharp confidence increase (constraints added)
- Steady state: confidence plateaus

**Metrics:**
- Mean confidence: $\bar{c} = \frac{1}{T} \sum_{t=1}^T c(t)$
- Confidence variance: $\sigma_c^2 = \frac{1}{T} \sum_{t=1}^T (c(t) - \bar{c})^2$

### 5.2 Spatial Analysis

**Per-Landmark Confidence:**

For each landmark $i$:

$$
c_i = 1 - \exp\left(-\frac{\text{tr}(\mathbf{I}_{ii})}{\tau}\right)
$$

Where $\mathbf{I}_{ii} = \mathbf{P}_{ii}^{-1}$ is the landmark's information matrix.

**Map Uncertainty Visualization:**

Color-code landmarks by confidence:
- Red: Low confidence ($c < 0.3$)
- Yellow: Medium confidence ($0.3 \leq c < 0.7$)
- Green: High confidence ($c \geq 0.7$)

### 5.3 Correlation Analysis

**Landmarks vs. Confidence:**

Correlation between number of landmarks and submap confidence:

$$
\rho = \text{corr}(N_{\text{landmarks}}, c_{\text{submap}})
$$

**Expected:** Positive correlation (more landmarks → higher confidence).

**Observations vs. Uncertainty:**

For each landmark, plot:
- x-axis: Number of observations
- y-axis: Uncertainty ($\text{tr}(\mathbf{P}_{ii})$)

**Expected:** Negative correlation (more observations → lower uncertainty).

### 5.4 Ground Truth Comparison

Given ground truth pose $\mathbf{x}_{\text{gt}}$ and estimate $\mathbf{x}_{\text{est}}$:

**Position Error:**

$$
e_{\text{pos}} = \sqrt{(x_{\text{est}} - x_{\text{gt}})^2 + (y_{\text{est}} - y_{\text{gt}})^2}
$$

**Orientation Error:**

$$
e_{\theta} = |\text{atan2}(\sin(\theta_{\text{est}} - \theta_{\text{gt}}), \cos(\theta_{\text{est}} - \theta_{\text{gt}}))|
$$

**Consistency Check:**

Is the error within predicted bounds?

$$
e_{\text{pos}} \leq 2\sqrt{\sigma_x^2 + \sigma_y^2} \quad \text{(95\% confidence)}
$$

**Normalized Estimation Error Squared (NEES):**

$$
\text{NEES} = (\mathbf{x}_{\text{est}} - \mathbf{x}_{\text{gt}})^T \mathbf{P}^{-1} (\mathbf{x}_{\text{est}} - \mathbf{x}_{\text{gt}})
$$

If the filter is consistent, $\text{NEES} \sim \chi^2(3)$.

---

## 6. Implementation

### 6.1 Confidence Computation

```python
def compute_ekf_confidence(ekf_slam, ekf_initialized, current_time_ns):
    """
    Compute information-theoretic confidence metric.

    Returns:
        dict: {
            'confidence': float in [0, 1],
            'information': trace of information matrix,
            'robot_uncertainty': trace of pose covariance,
            'num_landmarks': int,
            'timestamp': int
        }
    """
    if not ekf_initialized:
        return {
            'confidence': 0.0,
            'information': 0.0,
            'robot_uncertainty': float('inf'),
            'num_landmarks': 0,
            'timestamp': current_time_ns
        }

    # Robot pose covariance
    P_robot = ekf_slam.P[0:3, 0:3]

    # Uncertainty (trace)
    robot_uncertainty = np.trace(P_robot)

    # Information matrix (inverse)
    try:
        I_robot = np.linalg.inv(P_robot)
        information = np.trace(I_robot)

        # Normalized confidence score
        tau = 10.0
        confidence = 1.0 - np.exp(-information / tau)

    except np.linalg.LinAlgError:
        # Singular covariance
        information = 0.0
        confidence = 0.0

    return {
        'confidence': float(confidence),
        'information': float(information),
        'robot_uncertainty': float(robot_uncertainty),
        'num_landmarks': len(ekf_slam.landmarks),
        'timestamp': current_time_ns
    }
```

### 6.2 CSV Logging

```python
class ConfidenceTracker:
    """
    Log submap confidence metrics for thesis analysis.
    """
    def __init__(self, csv_file_path):
        self.csv_file = open(csv_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'submap_id', 'timestamp', 'confidence', 'information',
            'robot_uncertainty', 'num_landmarks'
        ])

    def log_confidence(self, submap_id, metrics):
        self.csv_writer.writerow([
            submap_id,
            metrics['timestamp'],
            metrics['confidence'],
            metrics['information'],
            metrics['robot_uncertainty'],
            metrics['num_landmarks']
        ])
        self.csv_file.flush()

    def close(self):
        self.csv_file.close()
```

### 6.3 Thesis Plotting

```python
import pandas as pd
import matplotlib.pyplot as plt

# Load confidence data
df = pd.read_csv('submap_confidence.csv')

# Plot confidence over time
plt.figure(figsize=(10, 6))
plt.plot(df['timestamp'] / 1e9, df['confidence'], marker='o')
plt.xlabel('Time (s)')
plt.ylabel('Confidence')
plt.title('Submap Confidence Evolution')
plt.grid()
plt.savefig('confidence_vs_time.png', dpi=300)

# Plot correlation: landmarks vs confidence
plt.figure(figsize=(10, 6))
plt.scatter(df['num_landmarks'], df['confidence'], alpha=0.6)
plt.xlabel('Number of Landmarks')
plt.ylabel('Confidence')
plt.title('Landmarks vs. Confidence Correlation')
plt.grid()

# Compute correlation
correlation = df['num_landmarks'].corr(df['confidence'])
plt.text(0.05, 0.95, f'Correlation: {correlation:.3f}',
         transform=plt.gca().transAxes, fontsize=12,
         verticalalignment='top')
plt.savefig('landmarks_vs_confidence.png', dpi=300)

# Statistics
print(f"Mean confidence: {df['confidence'].mean():.3f}")
print(f"Std confidence: {df['confidence'].std():.3f}")
print(f"Min confidence: {df['confidence'].min():.3f}")
print(f"Max confidence: {df['confidence'].max():.3f}")
```

---

## References

### Information Theory Foundations

1. **Shannon, C. E. (1948).** "A Mathematical Theory of Communication." *Bell System Technical Journal*, 27(3), 379-423.
   - Foundational paper introducing entropy and information theory
   - Established mathematical framework for quantifying information

2. **Cover, T. M., & Thomas, J. A. (2006).** *Elements of Information Theory* (2nd ed.). Wiley.
   - Chapter 8: Differential Entropy for continuous distributions
   - Chapter 9: Gaussian channels and mutual information
   - Standard reference for information-theoretic methods

3. **Jaynes, E. T. (2003).** *Probability Theory: The Logic of Science*. Cambridge University Press.
   - Maximum entropy principle for probabilistic inference
   - Bayesian foundations for uncertainty quantification

### Information Theory in SLAM

4. **Feder, H. J. S., Leonard, J. J., & Smith, C. M. (1999).** "Adaptive Mobile Robot Navigation and Mapping." *The International Journal of Robotics Research*, 18(7), 650-668.
   - Pioneering work on information-theoretic exploration
   - Mutual information between robot path and map
   - Coupled robot-map uncertainty analysis

5. **Stachniss, C., Grisetti, G., & Burgard, W. (2005).** "Information Gain-based Exploration Using Rao-Blackwellized Particle Filters." *Proceedings of Robotics: Science and Systems (RSS)*.
   - Information-driven active exploration
   - Particle filter-based SLAM with information metrics
   - Large-scale experimental validation

6. **Carrillo, H., Reid, I., & Castellanos, J. A. (2012).** "On the Comparison of Uncertainty Criteria for Active SLAM." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 2080-2087.
   - Comparison of D-optimality, A-optimality, and E-optimality criteria
   - Active SLAM formulation and algorithms
   - Experimental comparison of uncertainty metrics

### Filter Consistency and Observability

7. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation*. Wiley.
   - Chapter 5: Information and covariance forms of the Kalman filter
   - Chapter 11: Performance evaluation (NEES, consistency testing)
   - Standard reference for filter validation

8. **Huang, G. P., Mourikis, A. I., & Roumeliotis, S. I. (2010).** "Observability-based Rules for Designing Consistent EKF SLAM Estimators." *The International Journal of Robotics Research*, 29(5), 502-528.
   - Discovered observability-based inconsistency in EKF-SLAM
   - First Estimates Jacobian (FEJ) solution
   - Comprehensive consistency analysis

9. **Castellanos, J. A., Neira, J., & Tardós, J. D. (2004).** "Limits to the Consistency of EKF-Based SLAM." *Proceedings of 5th IFAC Symposium on Intelligent Autonomous Vehicles*.
   - Analysis of EKF-SLAM consistency failures
   - Relationship between linearization errors and inconsistency

### Active SLAM and Exploration

10. **Sim, R., & Roy, N. (2005).** "Global A-Optimal Robot Exploration in SLAM." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 661-666.
    - Formalized active SLAM as information maximization problem
    - A-optimal design for sensor placement
    - Computational approximations for real-time planning

11. **Carlone, L., Du, J., Ng, M. K., Bona, B., & Indri, M. (2014).** "Active SLAM and Exploration with Particle Filters Using Kullback-Leibler Divergence." *Journal of Intelligent & Robotic Systems*, 75(2), 291-311.
    - Kullback-Leibler divergence for information gain computation
    - Particle filter-based active SLAM
    - Multi-robot coordination through information sharing

12. **Valencia, R., Morta, M., Andrade-Cetto, J., & Porta, J. M. (2009).** "Planning Reliable Paths with Pose SLAM." *IEEE Transactions on Robotics*, 25(5), 1015-1026.
    - Path planning with uncertainty quantification
    - Reliability metrics from pose covariance
    - Safety-critical applications

### Consistency Testing and Validation

13. **Bailey, T., Nieto, J., Guivant, J., Stevens, M., & Nebot, E. (2006).** "Consistency of the EKF-SLAM Algorithm." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 3562-3568.
    - Experimental consistency analysis of EKF-SLAM
    - NEES testing methodology
    - Identified sources of inconsistency

14. **Li, M., & Mourikis, A. I. (2012).** "Improving the Accuracy of EKF-Based Visual-Inertial Odometry." *Proceedings of IEEE International Conference on Robotics and Automation (ICRA)*, pp. 828-835.
    - Observability-constrained EKF for consistency
    - FEJ implementation for visual-inertial systems
    - Experimental validation on real datasets

### Map Quality Metrics

15. **Kümmerle, R., Steder, B., Dornhege, C., Ruhnke, M., Grisetti, G., Stachniss, C., & Kleiner, A. (2009).** "On Measuring the Accuracy of SLAM Algorithms." *Autonomous Robots*, 27(4), 387-407.
    - Comprehensive SLAM evaluation framework
    - Metrics for map accuracy, consistency, and robustness
    - Benchmark datasets and evaluation protocols

16. **Burgard, W., Stachniss, C., Grisetti, G., Steder, B., Kümmerle, R., Dornhege, C., Ruhnke, M., Kleiner, A., & Tardós, J. D. (2009).** "A Comparison of SLAM Algorithms Based on a Graph of Relations." *Proceedings of IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, pp. 2089-2095.
    - Relative performance metrics for SLAM systems
    - Graph-based comparison framework
    - Large-scale experimental comparison

### Covariance Intersection and Multi-Robot SLAM

17. **Julier, S. J., & Uhlmann, J. K. (2007).** "Using Covariance Intersection for SLAM." *Robotics and Autonomous Systems*, 55(1), 3-20.
    - Covariance intersection for fusing correlated estimates
    - Critical for multi-robot SLAM with unknown correlations
    - Consistency preservation under fusion

---

**Next:** `07_coordinate_frames.md` — TF management and frame conventions
