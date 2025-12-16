# EKF Sensor Fusion
## Multi-Sensor Localization with Extended Kalman Filter

**Module:** `map_generation.ekf_lib`
**Code:** `src/map_generation/map_generation/ekf_lib.py`

---

## Table of Contents

1. [Overview](#overview)
2. [State Representation](#state-representation)
3. [Prediction Step (IMU)](#prediction-step-imu)
4. [Update Step (Measurements)](#update-step-measurements)
5. [Sensor Fusion Architecture](#sensor-fusion-architecture)
6. [Noise Parameters](#noise-parameters)
7. [Implementation](#implementation)

---

## Overview

The Extended Kalman Filter (EKF) fuses data from three sensors to estimate the robot's 2D pose:

| Sensor | Rate | Data | Usage |
|--------|------|------|-------|
| **IMU** | 200 Hz | Angular velocity (ω) | Prediction step |
| **Odometry** | 10 Hz | Position (x, y, θ) | Update step (moderate confidence) |
| **ICP** | Variable | Corrected pose | Update step (high confidence) |

**Key Advantage:** High-frequency predictions (200 Hz) refined by lower-frequency but accurate measurements.

---

## State Representation

### State Vector

The EKF estimates a 3D state representing the robot's pose in 2D space:

```
x = [x, y, θ]ᵀ
```

Where:
- **x**: Position along x-axis (meters)
- **y**: Position along y-axis (meters)
- **θ**: Heading angle (radians, normalized to [-π, π])

**Frame:** Global odometry frame (`{robot_name}/odom`)

### Covariance Matrix

```
P = [σ²_x    0      0   ]
    [0       σ²_y   0   ]  ∈ ℝ³ˣ³
    [0       0      σ²_θ ]
```

Represents uncertainty in each state dimension.

**Initial Covariance:**
```python
P₀ = 0.1 · I₃  # Moderate initial uncertainty
```

---

## Prediction Step (IMU)

### Motion Model

Using the robot's commanded velocity and IMU angular velocity:

```
x_{k+1} = x_k + v_x · cos(θ_k) · Δt
y_{k+1} = y_k + v_x · sin(θ_k) · Δt
θ_{k+1} = θ_k + ω · Δt
```

**Inputs:**
- `v_x`: Linear velocity (from cmd_vel, stored in EKF)
- `ω`: Angular velocity (from IMU gyroscope)
- `Δt = 0.005s` (200 Hz update rate)

### Jacobian

The linearized state transition matrix:

```
F = [1   0   -v_x · sin(θ_k) · Δt]
    [0   1    v_x · cos(θ_k) · Δt]
    [0   0    1                  ]
```

### Covariance Prediction

```
P_{k+1|k} = F · P_k · Fᵀ + Q
```

**Process Noise (Q):**
```python
Q = np.diag([0.0001, 0.0001, 0.0001])  # Position: ±1cm, Angle: ±0.57°
```

Accounts for:
- IMU gyroscope noise (σ = 0.0002 rad/s from Gazebo)
- Velocity model uncertainties
- Unmodeled dynamics

### Implementation

```python
# ekf_lib.py:62-115
def predict_imu(self, omega: float, dt: float = 0.005):
    """IMU-based prediction step"""
    # Current state
    x, y, theta = self.state[0], self.state[1], self.state[2]

    # Motion model
    x_new = x + self.vx * np.cos(theta) * dt
    y_new = y + self.vx * np.sin(theta) * dt
    theta_new = theta + omega * dt

    # Normalize angle to [-π, π]
    theta_new = np.arctan2(np.sin(theta_new), np.cos(theta_new))

    # Jacobian
    F = np.array([
        [1, 0, -self.vx * np.sin(theta) * dt],
        [0, 1,  self.vx * np.cos(theta) * dt],
        [0, 0,  1]
    ])

    # Covariance prediction
    self.P = F @ self.P @ F.T + self.Q

    # Enforce symmetry
    self.P = (self.P + self.P.T) / 2

    # Update state
    self.state = np.array([x_new, y_new, theta_new])

    self.imu_prediction_count += 1
```

---

## Update Step (Measurements)

### Measurement Model

Direct observation of the state:

```
z = H · x + v
```

Where:
- **z = [x_meas, y_meas, θ_meas]ᵀ**: Measurement vector
- **H = I₃**: Identity matrix (direct state observation)
- **v**: Measurement noise (Gaussian, zero-mean)

### Kalman Gain

```
S = H · P_{k|k-1} · Hᵀ + R
K = P_{k|k-1} · Hᵀ · S⁻¹
```

### Innovation

```
y = z - H · x_{k|k-1}
```

**Special handling for angle:**
```python
y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))  # Normalize to [-π, π]
```

### State Update

```
x_{k|k} = x_{k|k-1} + K · y
```

### Covariance Update (Joseph Form)

```
P_{k|k} = (I - K·H) · P_{k|k-1} · (I - K·H)ᵀ + K · R · Kᵀ
```

**Why Joseph form?** Guarantees numerical stability and positive semi-definiteness.

### Implementation

```python
# ekf_lib.py:116-170
def update(self, measurement: dict, measurement_noise: np.ndarray):
    """Measurement update step"""
    z = np.array([
        measurement['x'],
        measurement['y'],
        measurement['theta']
    ])

    # Measurement matrix (identity)
    H = np.eye(3)

    # Innovation covariance
    S = H @ self.P @ H.T + measurement_noise

    # Kalman gain
    K = self.P @ H.T @ np.linalg.inv(S)

    # Innovation
    y = z - self.state
    y[2] = np.arctan2(np.sin(y[2]), np.cos(y[2]))  # Normalize angle

    # State update
    self.state = self.state + K @ y
    self.state[2] = np.arctan2(np.sin(self.state[2]), np.cos(self.state[2]))

    # Covariance update (Joseph form)
    I_KH = np.eye(3) - K @ H
    self.P = I_KH @ self.P @ I_KH.T + K @ measurement_noise @ K.T

    # Enforce symmetry
    self.P = (self.P + self.P.T) / 2

    self.update_count += 1
```

---

## Sensor Fusion Architecture

### Callback Integration

```
┌─────────────────────────────────────────────────────┐
│         local_submap_generator.py                   │
│                                                      │
│  IMU Callback (200 Hz)                              │
│    ↓                                                 │
│  ekf.predict_imu(omega, dt=0.005)                   │
│    ↓                                                 │
│  [State propagated at 200 Hz]                       │
│                                                      │
│  ─────────────────────────────────────              │
│                                                      │
│  Odom Callback (10 Hz)                              │
│    ↓                                                 │
│  ekf.update(pose, R_odom)                           │
│    ↓                                                 │
│  [Correction with moderate confidence]              │
│                                                      │
│  ─────────────────────────────────────              │
│                                                      │
│  ICP Correction (Variable, when successful)         │
│    ↓                                                 │
│  ekf.update(corrected_pose, R_icp)                  │
│    ↓                                                 │
│  [Correction with high confidence]                  │
└─────────────────────────────────────────────────────┘
```

### Data Flow Timeline

```
t=0.000s: IMU → EKF Predict
t=0.005s: IMU → EKF Predict
t=0.010s: IMU → EKF Predict
...
t=0.100s: IMU → EKF Predict
         Odom → EKF Update (R_odom)
         Scan → ICP → EKF Update (R_icp, if ICP succeeds)
...
```

**Result:** Smooth 200 Hz pose estimate with corrections every 100ms.

---

## Noise Parameters

### Process Noise (Q)

```python
Q = np.diag([0.0001, 0.0001, 0.0001])
```

| Component | Value | Standard Deviation | Notes |
|-----------|-------|-------------------|-------|
| Position (x, y) | 0.0001 m² | σ = 1 cm | Per 5ms update |
| Orientation (θ) | 0.0001 rad² | σ = 0.57° | Accounts for IMU noise |

**Tuning:**
- ↑ Q → Trust predictions less, react faster to measurements
- ↓ Q → Trust predictions more, smoother but slower to adapt

### Measurement Noise - Odometry (R_odom)

```python
R_odom = np.diag([0.005, 0.005, 0.02])
```

| Component | Value | Standard Deviation | Source |
|-----------|-------|-------------------|--------|
| Position (x, y) | 0.005 m² | σ = 7.1 cm | Gazebo wheel encoder noise |
| Orientation (θ) | 0.02 rad² | σ = 8.1° | Gazebo differential drive |

**Matched to Gazebo configuration:**
```xml
<!-- model.sdf:506-513 -->
<pose_covariance_diagonal>
  0.005 0.005 0.001 0.001 0.001 0.02
</pose_covariance_diagonal>
```

### Measurement Noise - ICP (R_icp)

```python
R_icp = np.diag([0.0001, 0.0001, 0.0001])
```

| Component | Value | Standard Deviation | Rationale |
|-----------|-------|-------------------|-----------|
| Position (x, y) | 0.0001 m² | σ = 1 cm | ICP aligns scans accurately |
| Orientation (θ) | 0.0001 rad² | σ = 0.57° | Scan matching precision |

**Why lower noise?** ICP refines pose by aligning geometry → more accurate than raw odometry.

### Noise Parameter Comparison

| Measurement Type | Position σ | Orientation σ | Confidence |
|------------------|-----------|---------------|------------|
| **Odometry** | 7.1 cm | 8.1° | Moderate |
| **ICP** | 1.0 cm | 0.57° | High |
| **Process** | 1.0 cm (per 5ms) | 0.57° | N/A |

---

## Implementation

### Initialization

```python
# ekf_lib.py:49-60
def initialize(self, x: float, y: float, theta: float, vx: float, vy: float):
    """Initialize EKF with first odometry measurement"""
    self.state = np.array([x, y, theta])
    self.vx = vx  # Store linear velocity for predictions
    self.vy = vy
    self.P = 0.1 * np.eye(3)  # Initial uncertainty
    self.ekf_initialized = True
```

**Called from:** `local_submap_generator.py:263-270`

### Angle Normalization

All angles are normalized to [-π, π] to prevent wrapping issues:

```python
theta = np.arctan2(np.sin(theta), np.cos(theta))
```

**Applied to:**
- Predicted state after motion model
- Innovation (measurement residual)
- Updated state after correction

### Drift Detection

```python
# ekf_lib.py:112-114
if self.imu_prediction_count - self.last_update_imu_count > 100:
    # >0.5s without odometry update
    self.get_logger().warn('EKF drift warning: No updates in 0.5s')
```

### Uncertainty Quantification

```python
# ekf_lib.py:185-191
def get_position_uncertainty(self) -> float:
    """Get current position uncertainty (meters)"""
    sigma_x = np.sqrt(self.P[0, 0])
    sigma_y = np.sqrt(self.P[1, 1])
    return np.sqrt(sigma_x**2 + sigma_y**2)

def get_orientation_uncertainty(self) -> float:
    """Get current orientation uncertainty (radians)"""
    return np.sqrt(self.P[2, 2])
```

### Statistics

```python
# ekf_lib.py:193-202
def get_statistics(self) -> dict:
    """Return EKF performance metrics"""
    return {
        'imu_predictions': self.imu_prediction_count,
        'updates': self.update_count,
        'pos_uncertainty': self.get_position_uncertainty(),
        'ori_uncertainty': self.get_orientation_uncertainty()
    }
```

---

## Integration with SLAM

### Usage Pattern

```python
# local_submap_generator.py initialization
ekf = EKF()
ekf_initialized = False

# First odometry message
def odom_callback(msg):
    if not ekf_initialized:
        ekf.initialize(x, y, theta, vx, vy)
        ekf_initialized = True
        return

    # Update with odometry
    ekf.update({'x': x, 'y': y, 'theta': theta}, R_odom)

# IMU stream (200 Hz)
def imu_callback(msg):
    if ekf_initialized:
        ekf.predict_imu(omega=msg.angular_velocity.z, dt=0.005)

# ICP correction (when available)
if icp_success:
    corrected_pose = apply_icp_correction(raw_pose, icp_transform)
    ekf.update(corrected_pose, R_icp)  # High confidence update
```

### State Retrieval

```python
# Get current best estimate
ekf_state = ekf.get_state()  # Returns [x, y, theta]

# Publish TF transform
publish_tf(ekf_state)

# Use for scan transformation
transform_scan_to_world(scan, pose=ekf_state)
```

---

## Tuning Guide

### Problem: Map Drift

**Symptoms:** Map distorts over long runs, walls not straight

**Solutions:**
1. **Reduce process noise:** `Q = diag([0.00005, 0.00005, 0.00005])`
2. **Enable ICP:** `enable_scan_to_map_icp = true`
3. **Check IMU calibration:** Verify gyroscope bias < 0.001 rad/s

### Problem: Jerky Motion

**Symptoms:** Robot pose jumps, not smooth

**Solutions:**
1. **Increase process noise:** `Q = diag([0.0005, 0.0005, 0.0005])`
2. **Increase measurement noise:** `R_odom *= 2`
3. **Verify sensor rates:** IMU at 200 Hz, odom at 10 Hz

### Problem: ICP Corrections Rejected

**Symptoms:** Many "ICP correction too large" warnings

**Solutions:**
1. **Improve initial guess:** Check EKF convergence first
2. **Relax validation:** Increase thresholds in `local_submap_generator.py:370`
3. **More ICP iterations:** `max_iteration = 50` in `mapping_utils.py:46`

---

## Performance Metrics

| Metric | Value | Target |
|--------|-------|--------|
| **Prediction Rate** | 200 Hz | IMU rate |
| **Update Rate** | 10-11 Hz | Odom + ICP |
| **Position Uncertainty** | <5 cm | Converged |
| **Orientation Uncertainty** | <3° | Converged |
| **Convergence Time** | <2 seconds | From initialization |

---

**Next:** Read [04_LOOP_CLOSURE_OPTIMIZATION.md](04_LOOP_CLOSURE_OPTIMIZATION.md) for loop closure detection and GTSAM optimization.
