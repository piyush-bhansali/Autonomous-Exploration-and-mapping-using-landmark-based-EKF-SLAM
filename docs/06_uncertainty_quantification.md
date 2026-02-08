# Uncertainty Quantification and Map Confidence

## Table of Contents
1. [Introduction](#1-introduction)
2. [Information Theory Foundations](#2-information-theory-foundations)
3. [Confidence Metric Design](#3-confidence-metric-design)
4. [Thesis Analysis Methods](#4-thesis-analysis-methods)
5. [Implementation](#5-implementation)

---

## 1. Introduction

**Uncertainty quantification** enables **confidence-aware mapping**: the ability to assign reliability scores to different parts of the map. This is critical for:
- **Decision making:** Avoid uncertain regions
- **Active exploration:** Target areas needing more observations
- **System validation:** Quantify mapping quality
- **Multi-robot fusion:** Weight contributions by confidence

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

## 4. Thesis Analysis Methods

### 4.1 Temporal Analysis

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

### 4.2 Spatial Analysis

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

### 4.3 Correlation Analysis

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

### 4.4 Ground Truth Comparison

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

## 5. Implementation

### 5.1 Confidence Computation

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

### 5.2 CSV Logging

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

### 5.3 Thesis Plotting

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

1. **Cover, T. M., & Thomas, J. A. (2006).** *Elements of Information Theory*. Wiley. Chapter 8: Differential Entropy.

2. **Stachniss, C., Grisetti, G., & Burgard, W. (2005).** "Information Gain-based Exploration Using Rao-Blackwellized Particle Filters." *Robotics: Science and Systems*.

3. **Valencia, R., Morta, M., Andrade-Cetto, J., & Porta, J. M. (2009).** "Planning Reliable Paths with Pose SLAM." *IEEE Transactions on Robotics*, 25(5), 1015-1026.

4. **Bar-Shalom, Y., Li, X. R., & Kirubarajan, T. (2001).** *Estimation with Applications to Tracking and Navigation*. Wiley. Chapter 5: Information and Covariance Forms.

---

**Next:** `07_coordinate_frames.md` — TF management and frame conventions
