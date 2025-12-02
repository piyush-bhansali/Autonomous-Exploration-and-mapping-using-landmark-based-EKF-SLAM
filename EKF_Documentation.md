# Extended Kalman Filter (EKF) for Robot Localization

## 1. Introduction

This document describes the Extended Kalman Filter (EKF) implementation used for robot pose estimation in the thesis project. The EKF fuses data from multiple sensors (IMU, odometry, and ICP-corrected measurements) to provide accurate and robust robot localization.

**Implementation Location:** `src/map_generation/map_generation/ekf_lib.py`

---

## 2. State Vector and Coordinate System

The EKF estimates a 3-dimensional state vector representing the robot's pose in 2D space:

**State Vector:**
```
x = [x, y, θ]ᵀ
```

Where:
- **x**: Robot position along the x-axis (meters)
- **y**: Robot position along the y-axis (meters)
- **θ**: Robot orientation (heading angle in radians, normalized to [-π, π])

The state is represented in the global odometry frame (`{robot_name}/odom`).

---

## 3. EKF Algorithm Overview

The EKF operates in two main steps that repeat cyclically:

1. **Prediction Step**: Propagate the state estimate forward in time using IMU data
2. **Update Step**: Correct the state estimate using odometry or ICP measurements

### 3.1 Prediction Step (IMU-based)

The prediction step uses the robot's commanded velocity and IMU angular velocity measurements to propagate the state forward.

**Motion Model:**

```
x_{k+1} = x_k + v_x · cos(θ_k) · Δt
y_{k+1} = y_k + v_x · sin(θ_k) · Δt
θ_{k+1} = θ_k + ω · Δt
```

Where:
- **v_x**: Linear velocity in the robot's forward direction (from cmd_vel)
- **ω**: Angular velocity from IMU gyroscope (rad/s)
- **Δt**: Time step between predictions (typically 0.005s at 200 Hz)

**State Jacobian (F):**

The linearized state transition matrix is:

```
F = [1   0   -v_x · sin(θ_k) · Δt]
    [0   1    v_x · cos(θ_k) · Δt]
    [0   0    1                  ]
```

**Covariance Prediction:**

```
P_{k+1|k} = F · P_k · Fᵀ + Q
```

Where:
- **P**: State covariance matrix (3×3)
- **Q**: Process noise covariance matrix

**Process Noise (Q):**

```
Q = [0.0001    0        0     ]
    [0         0.0001   0     ]  (units: m², rad²)
    [0         0        0.0001]
```

This represents:
- Position uncertainty: σ = 0.01 m = 1 cm per 5ms update
- Orientation uncertainty: σ = 0.01 rad ≈ 0.57° per update

The process noise accounts for:
- IMU gyroscope noise (σ = 0.0002 rad/s from Gazebo)
- Velocity model uncertainties
- Unmodeled dynamics

**Implementation:** `ekf_lib.py:62-115` (predict_imu method)

---

### 3.2 Update Step (Measurement Correction)

The update step corrects the predicted state using measurements from odometry or ICP.

**Measurement Model:**

The measurement model is a direct observation of the state:

```
z = H · x + v
```

Where:
- **z = [x_meas, y_meas, θ_meas]ᵀ**: Measurement vector
- **H = I₃**: Measurement matrix (3×3 identity, since we directly observe all state variables)
- **v**: Measurement noise (Gaussian, zero-mean)

**Measurement Noise Covariance (R):**

Two different noise models are used depending on the measurement source:

**Odometry Measurements (R_odom):**
```
R_odom = [0.005    0        0      ]
         [0        0.005    0      ]  (units: m², rad²)
         [0        0        0.02   ]
```
- Position uncertainty: σ = 0.071 m ≈ 7.1 cm
- Orientation uncertainty: σ = 0.141 rad ≈ 8.1°

These values are **matched to the Gazebo differential drive plugin** noise parameters configured in the robot's SDF file (`model.sdf:506-513`), ensuring the EKF's internal model accurately reflects the actual sensor noise characteristics.

**ICP-Corrected Measurements (R_icp):**
```
R_icp = [0.0001    0         0       ]
        [0         0.0001    0       ]  (units: m², rad²)
        [0         0         0.0001  ]
```
- Position uncertainty: σ = 0.01 m = 1 cm
- Orientation uncertainty: σ = 0.01 rad ≈ 0.57°

ICP measurements have significantly lower noise than raw odometry because ICP refines the pose estimate by aligning laser scans with the accumulated map, providing higher accuracy.

**Kalman Gain:**

```
S = H · P_{k|k-1} · Hᵀ + R
K = P_{k|k-1} · Hᵀ · S⁻¹
```

Where:
- **S**: Innovation covariance (3×3)
- **K**: Kalman gain matrix (3×3)

**Innovation (Measurement Residual):**

```
y = z - H · x_{k|k-1}
```

Special handling for angular difference:
```
y[2] = atan2(sin(y[2]), cos(y[2]))  // Normalize to [-π, π]
```

**State Update:**

```
x_{k|k} = x_{k|k-1} + K · y
```

**Covariance Update (Joseph Form):**

```
P_{k|k} = (I - K·H) · P_{k|k-1} · (I - K·H)ᵀ + K · R · Kᵀ
```

This formulation guarantees numerical stability and positive semi-definiteness of the covariance matrix.

**Implementation:** `ekf_lib.py:116-170` (update method)

---

## 4. Sensor Fusion Architecture

The EKF integrates three types of measurements:

### 4.1 IMU (High-Frequency Prediction)

- **Frequency**: ~200 Hz
- **Data**: Angular velocity (ω) from gyroscope
- **Usage**: Prediction step to propagate orientation
- **Callback**: `local_submap_generator.py:224-232`

### 4.2 Odometry (Medium-Frequency Update)

- **Frequency**: Variable (typically 10-50 Hz)
- **Data**: Position (x, y) and orientation (θ) from wheel encoders
- **Usage**: Update step with moderate confidence
- **Callback**: `local_submap_generator.py:246-317`

### 4.3 ICP-Corrected Measurements (Low-Frequency, High-Accuracy Update)

- **Frequency**: Variable (when ICP succeeds)
- **Data**: Refined position and orientation from scan-to-map or submap-to-map ICP
- **Usage**: Update step with high confidence
- **Application**:
  - Scan-to-map ICP: `local_submap_generator.py:337-391`
  - Submap-to-map ICP: `local_submap_generator.py:450-498`

---

## 5. Implementation Details

### 5.1 Initialization

The EKF is initialized with the first odometry measurement:

```python
ekf.initialize(x_odom, y_odom, theta_odom, vx, vy)
```

**Initial Covariance:**
```
P₀ = 0.1 · I₃
```

This represents moderate initial uncertainty (σ = 0.316 m, 0.316 rad).

**Implementation:** `ekf_lib.py:49-60`

---

### 5.2 Angle Normalization

To prevent angle wrapping issues, all angular values are normalized to [-π, π]:

```python
theta = atan2(sin(theta), cos(theta))
```

This is applied to:
- Predicted orientation after motion model
- Innovation (measurement residual)
- Updated state orientation

**Implementation:** Throughout `ekf_lib.py`

---

### 5.3 Covariance Symmetry Enforcement

After each prediction and update step, the covariance matrix is symmetrized to prevent numerical drift:

```python
P = (P + Pᵀ) / 2
```

**Implementation:** `ekf_lib.py:107, 159`

---

### 5.4 Drift Detection

A watchdog monitors consecutive IMU predictions without odometry updates:

- **Threshold**: 100 consecutive predictions (~0.5s at 200 Hz)
- **Action**: Warning issued to indicate potential drift

**Implementation:** `ekf_lib.py:112-114`

---

## 6. Gazebo Sensor Noise Configuration

The EKF noise parameters are calibrated to match the actual sensor noise configured in the Gazebo simulation. This ensures the filter's internal uncertainty model accurately reflects the true sensor characteristics.

### 6.1 IMU Sensor Noise

Configured in `model.sdf:48-96`:

```xml
<angular_velocity>
  <z>
    <noise type="gaussian">
      <mean>0.0</mean>
      <stddev>2e-4</stddev>  <!-- σ = 0.0002 rad/s -->
    </noise>
  </z>
</angular_velocity>
```

The IMU gyroscope noise (σ = 0.0002 rad/s) is implicitly handled through the process noise covariance Q in the prediction step.

### 6.2 LiDAR Sensor Noise

Configured in `model.sdf:158-162`:

```xml
<noise>
  <type>gaussian</type>
  <mean>0.0</mean>
  <stddev>0.01</stddev>  <!-- σ = 1 cm range noise -->
</noise>
```

LiDAR noise affects ICP alignment quality. The ICP measurement noise (R_icp) accounts for both the raw sensor noise and ICP alignment uncertainty.

### 6.3 Odometry Noise (Differential Drive)

Configured in `model.sdf:506-516`:

```xml
<!-- Pose uncertainty (covariance diagonal) -->
<pose_covariance_diagonal>
  0.005 0.005 0.001 0.001 0.001 0.02
</pose_covariance_diagonal>

<!-- Velocity uncertainty -->
<twist_covariance_diagonal>
  0.002 0.002 0.001 0.001 0.001 0.01
</twist_covariance_diagonal>

<!-- Wheel encoder noise -->
<noise>0.015</noise>  <!-- 1.5% encoder error -->
```

**Pose Covariance Breakdown:**
- x, y position: variance = 0.005 m² → σ = 0.071 m ≈ 7.1 cm
- z position: variance = 0.001 m² (not used in 2D navigation)
- roll, pitch: variance = 0.001 rad² (not used in 2D navigation)
- yaw (θ): variance = 0.02 rad² → σ = 0.141 rad ≈ 8.1°

**Velocity Covariance:**
- linear velocities: variance = 0.002 m²/s²
- angular velocity: variance = 0.01 rad²/s²

The EKF's **R_odom** matrix directly mirrors these Gazebo odometry noise parameters.

### 6.4 Noise Parameter Matching

| Parameter | Gazebo Configuration | EKF Matrix | Value |
|-----------|---------------------|------------|-------|
| Odometry x position | pose_covariance[0] | R_odom[0,0] | 0.005 m² |
| Odometry y position | pose_covariance[1] | R_odom[1,1] | 0.005 m² |
| Odometry orientation | pose_covariance[5] | R_odom[2,2] | 0.02 rad² |
| IMU gyroscope | angular_velocity stddev | Q[2,2] (indirect) | 0.0002 rad/s |
| LiDAR range | range noise stddev | R_icp (indirect) | 0.01 m |

This tight coupling between simulation and filter parameters ensures realistic performance evaluation.

---

## 7. Coordinate Frame Transformations

### 7.1 Input Measurements

All input measurements (odometry, ICP corrections) are in the **global odometry frame** (`{robot_name}/odom`).

### 7.2 Robot Frame to Global Frame

The motion model converts robot-frame velocity (v_x) to global frame displacement:

```
Δx_global = v_x · cos(θ) · Δt
Δy_global = v_x · sin(θ) · Δt
```

---

## 7. Uncertainty Quantification

The EKF provides uncertainty estimates through the covariance matrix P.

**Position Uncertainty:**
```
σ_x = √P[0,0]
σ_y = √P[1,1]
σ_pos = √(σ_x² + σ_y²)
```

**Orientation Uncertainty:**
```
σ_θ = √P[2,2]
```

**Convergence Check:**

The EKF is considered converged when:
```
σ_pos < 0.05 m  AND  σ_θ < 0.05 rad
```

**Implementation:** `ekf_lib.py:185-191, 225-229`

---

## 8. Performance Statistics

The EKF tracks performance metrics:

- **imu_prediction_count**: Total IMU-based prediction steps
- **update_count**: Total measurement update steps
- **position_uncertainty**: Current position uncertainty (m)
- **orientation_uncertainty**: Current orientation uncertainty (rad)

**Access:** `ekf.get_statistics()`

**Implementation:** `ekf_lib.py:193-202`

---

## 9. Tuning Parameters

### 9.1 Process Noise (Q)

Controls how much the filter trusts the motion model:
- **Increase Q**: Faster adaptation to changes, but more noise
- **Decrease Q**: Smoother estimates, but slower response

**Current values:** See Section 3.1

### 9.2 Measurement Noise (R)

Controls how much the filter trusts measurements:
- **Increase R**: Rely more on predictions, less on measurements
- **Decrease R**: Rely more on measurements, less on predictions

**Current values:** See Section 3.2

---

## 10. Integration with ICP

### 10.1 Scan-to-Map ICP

**Purpose:** Refine the pose estimate by aligning the current scan with accumulated submap points.

**Process:**
1. Transform scan to world frame using raw odometry
2. Perform ICP alignment with accumulated points
3. Extract pose correction (dx, dy, dθ)
4. Validate correction magnitude (reject if >20cm or >15°)
5. Apply correction to raw odometry and feed to EKF as ICP measurement

**Implementation:** `local_submap_generator.py:337-391`

### 10.2 Submap-to-Map ICP

**Purpose:** Correct accumulated drift when integrating a completed submap into the global map.

**Process:**
1. Align new submap with global map using ICP
2. Extract pose correction
3. Validate correction magnitude (reject if >50cm or >20°)
4. Apply correction to current pose via EKF update

**Implementation:** `local_submap_generator.py:450-498`

---

## 11. Mathematical Summary

**Prediction (IMU):**
```
x̂_{k+1|k} = f(x̂_{k|k}, u_k)
P_{k+1|k} = F_k · P_{k|k} · F_kᵀ + Q
```

**Update (Odometry/ICP):**
```
K_k = P_{k|k-1} · Hᵀ · (H · P_{k|k-1} · Hᵀ + R)⁻¹
x̂_{k|k} = x̂_{k|k-1} + K_k · (z_k - H · x̂_{k|k-1})
P_{k|k} = (I - K_k·H) · P_{k|k-1} · (I - K_k·H)ᵀ + K_k · R · K_kᵀ
```

---

## 12. References

### Code References

- **EKF Library:** `src/map_generation/map_generation/ekf_lib.py`
- **EKF Integration:** `src/map_generation/map_generation/local_submap_generator.py:60-317`
- **Scan-to-Map ICP:** `src/map_generation/map_generation/local_submap_generator.py:337-391`
- **Submap-to-Map ICP:** `src/map_generation/map_generation/local_submap_generator.py:450-498`

### Key Functions

- `EKF.__init__()`: Initialization with noise parameters
- `EKF.initialize()`: Set initial state
- `EKF.predict_imu()`: IMU-based prediction step
- `EKF.update()`: Measurement update step
- `EKF.get_state()`: Retrieve current state estimate
- `EKF.get_covariance()`: Retrieve uncertainty
- `EKF.get_statistics()`: Performance metrics

---

## 13. Conclusion

The implemented EKF provides robust and accurate robot localization by:

1. **High-frequency prediction** using IMU data (200 Hz)
2. **Medium-frequency correction** using odometry (10-50 Hz)
3. **High-accuracy refinement** using ICP-based measurements (variable)

The Joseph form covariance update ensures numerical stability, while angle normalization prevents gimbal lock issues. The dual measurement noise model (odometry vs. ICP) allows the filter to appropriately weight different measurement sources based on their reliability.

This sensor fusion approach effectively mitigates individual sensor limitations:
- IMU drift is corrected by odometry and ICP
- Odometry errors are refined by ICP
- Short-term accuracy from IMU complements long-term stability from ICP

The result is a smooth, accurate, and drift-free pose estimate suitable for high-precision mapping applications.
