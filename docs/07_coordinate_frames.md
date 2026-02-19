# Coordinate Frames and Transform Management

> **Note:** This document describes coordinate frame conventions for the **feature-based mapping pipeline** with feature-based SVD submap stitching.

## Table of Contents
1. [Introduction](#1-introduction)
2. [ROS REP-105 Standard](#2-ros-rep-105-standard)
3. [Transform Chain Mathematics](#3-transform-chain-mathematics)
4. [Map Correction Computation](#4-map-correction-computation)
5. [Implementation](#5-implementation)

---

## 1. Introduction

Proper coordinate frame management is critical for SLAM. The system maintains multiple reference frames to separate:
- **Global, drift-free localization** (map frame)
- **Locally-consistent odometry** (odom frame)
- **Sensor measurements** (robot/base frame)

---

## 2. ROS REP-105 Standard

### 2.1 Frame Definitions

**REP-105** (ROS Enhancement Proposal 105) defines standard coordinate frames for mobile robots.

#### **`map` Frame**
- **Origin:** Arbitrary, but fixed for the lifetime of the system
- **Purpose:** Global reference for the environment
- **Properties:**
  - Continuous (no discontinuous jumps)
  - Drift-free (corrected by SLAM)
  - World-fixed

**Use case:** Long-term navigation, global path planning

#### **`odom` Frame**
- **Origin:** Robot's starting position
- **Purpose:** Locally-consistent, continuous odometry
- **Properties:**
  - Continuous (smooth motion)
  - Drift-prone (accumulates error over time)
  - Updated by wheel encoders or visual odometry

**Use case:** Short-term control, velocity estimation

#### **`base_footprint` / `base_link` Frame**
- **Origin:** Robot's center (on ground plane for `base_footprint`)
- **Purpose:** Robot's local reference frame
- **Properties:**
  - Moves with robot
  - Fixed relationship to sensors

**Use case:** Sensor data interpretation, local obstacle avoidance

### 2.2 Transform Hierarchy

```
map
 └─> odom
      └─> base_footprint
           ├─> laser_frame
           ├─> camera_frame
           └─> imu_frame
```

**Key Principle:** The `map → odom` transform is published by the **localization system** (EKF-SLAM in our case).

---

## 3. Transform Chain Mathematics

### 3.1 Homogeneous Transformation Matrices

A 2D rigid transformation is represented as:

$$
\mathbf{T} = \begin{bmatrix}
\cos\theta & -\sin\theta & t_x \\
\sin\theta & \cos\theta & t_y \\
0 & 0 & 1
\end{bmatrix} \in SE(2)
$$

**Apply to point:**

$$
\begin{bmatrix}
x' \\
y' \\
1
\end{bmatrix} = \mathbf{T} \begin{bmatrix}
x \\
y \\
1
\end{bmatrix}
$$

### 3.2 Transform Composition

Given transforms $^A\mathbf{T}_B$ (B expressed in A) and $^B\mathbf{T}_C$ (C expressed in B), the composed transform is:

$$
^A\mathbf{T}_C = \, ^A\mathbf{T}_B \cdot \, ^B\mathbf{T}_C
$$

**Example:**

$$
^{\text{map}}\mathbf{T}_{\text{base}} = \, ^{\text{map}}\mathbf{T}_{\text{odom}} \cdot \, ^{\text{odom}}\mathbf{T}_{\text{base}}
$$

**Order matters:** Matrix multiplication is not commutative.

### 3.3 Inverse Transform

$$
\mathbf{T}^{-1} = \begin{bmatrix}
\mathbf{R}^T & -\mathbf{R}^T \mathbf{t} \\
0 & 1
\end{bmatrix}
$$

**In 2D:**

$$
\mathbf{T}^{-1}(\theta, t_x, t_y) = \mathbf{T}(-\theta, -t_x \cos\theta - t_y \sin\theta, t_x \sin\theta - t_y \cos\theta)
$$

**Simplified for 2D rotation:**

$$
\mathbf{R}^{-1}(\theta) = \mathbf{R}(-\theta) = \mathbf{R}^T(\theta)
$$

---

## 4. Map Correction Computation

### 4.1 Problem Statement

**Given:**
- EKF pose estimate in `map` frame: $^{\text{map}}\mathbf{x}_{\text{base}}^{\text{EKF}} = (x_{\text{ekf}}, y_{\text{ekf}}, \theta_{\text{ekf}})$
- Odometry estimate in `odom` frame: $^{\text{odom}}\mathbf{x}_{\text{base}} = (x_{\text{odom}}, y_{\text{odom}}, \theta_{\text{odom}})$

**Find:**
The `map → odom` transform $^{\text{map}}\mathbf{T}_{\text{odom}}$ such that:

$$
^{\text{map}}\mathbf{T}_{\text{base}}^{\text{EKF}} = \, ^{\text{map}}\mathbf{T}_{\text{odom}} \cdot \, ^{\text{odom}}\mathbf{T}_{\text{base}}
$$

### 4.2 Solution

Rearrange:

$$
^{\text{map}}\mathbf{T}_{\text{odom}} = \, ^{\text{map}}\mathbf{T}_{\text{base}}^{\text{EKF}} \cdot \left( ^{\text{odom}}\mathbf{T}_{\text{base}} \right)^{-1}
$$

**Algorithm:**

**Step 1: Construct EKF transform**

$$
^{\text{map}}\mathbf{T}_{\text{base}}^{\text{EKF}} = \begin{bmatrix}
\cos\theta_{\text{ekf}} & -\sin\theta_{\text{ekf}} & x_{\text{ekf}} \\
\sin\theta_{\text{ekf}} & \cos\theta_{\text{ekf}} & y_{\text{ekf}} \\
0 & 0 & 1
\end{bmatrix}
$$

**Step 2: Invert odometry transform**

$$
\left( ^{\text{odom}}\mathbf{T}_{\text{base}} \right)^{-1} = \begin{bmatrix}
\cos\theta_{\text{odom}} & \sin\theta_{\text{odom}} & -x_{\text{odom}} \cos\theta_{\text{odom}} - y_{\text{odom}} \sin\theta_{\text{odom}} \\
-\sin\theta_{\text{odom}} & \cos\theta_{\text{odom}} & x_{\text{odom}} \sin\theta_{\text{odom}} - y_{\text{odom}} \cos\theta_{\text{odom}} \\
0 & 0 & 1
\end{bmatrix}
$$

**Step 3: Multiply**

$$
^{\text{map}}\mathbf{T}_{\text{odom}} = \, ^{\text{map}}\mathbf{T}_{\text{base}}^{\text{EKF}} \cdot \left( ^{\text{odom}}\mathbf{T}_{\text{base}} \right)^{-1}
$$

**Step 4: Extract correction parameters**

From the resulting matrix:

$$
^{\text{map}}\mathbf{T}_{\text{odom}} = \begin{bmatrix}
\cos\theta_{\text{corr}} & -\sin\theta_{\text{corr}} & x_{\text{corr}} \\
\sin\theta_{\text{corr}} & \cos\theta_{\text{corr}} & y_{\text{corr}} \\
0 & 0 & 1
\end{bmatrix}
$$

### 4.3 Simplified 2D Computation

For computational efficiency in 2D:

**Orientation correction:**

$$
\theta_{\text{corr}} = \theta_{\text{ekf}} - \theta_{\text{odom}}
$$

**Position correction:**

Rotate odometry position by correction angle, then offset:

$$
\begin{bmatrix}
x_{\text{corr}} \\
y_{\text{corr}}
\end{bmatrix} = \begin{bmatrix}
x_{\text{ekf}} \\
y_{\text{ekf}}
\end{bmatrix} - \begin{bmatrix}
\cos\theta_{\text{corr}} & -\sin\theta_{\text{corr}} \\
\sin\theta_{\text{corr}} & \cos\theta_{\text{corr}}
\end{bmatrix} \begin{bmatrix}
x_{\text{odom}} \\
y_{\text{odom}}
\end{bmatrix}
$$

---

## 5. Implementation

### 5.1 Transform Computation

```python
def compute_map_to_odom_transform(ekf_state, odom_pose):
    """
    Compute map->odom transform from EKF and odometry.

    Args:
        ekf_state: (x_ekf, y_ekf, theta_ekf) in map frame
        odom_pose: (x_odom, y_odom, theta_odom) in odom frame

    Returns:
        (x_corr, y_corr, theta_corr): map->odom transform
    """
    x_ekf, y_ekf, theta_ekf = ekf_state
    x_odom, y_odom, theta_odom = odom_pose

    # Orientation correction
    theta_corr = theta_ekf - theta_odom
    theta_corr = np.arctan2(np.sin(theta_corr), np.cos(theta_corr))

    # Position correction
    cos_corr = np.cos(theta_corr)
    sin_corr = np.sin(theta_corr)

    x_corr = x_ekf - (x_odom * cos_corr - y_odom * sin_corr)
    y_corr = y_ekf - (x_odom * sin_corr + y_odom * cos_corr)

    return x_corr, y_corr, theta_corr
```

### 5.2 TF Broadcasting

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def publish_map_to_odom_tf(x_corr, y_corr, theta_corr, robot_name, clock):
    """
    Publish map->odom transform to TF tree.
    """
    t = TransformStamped()

    # Header
    t.header.stamp = clock.now().to_msg()
    t.header.frame_id = 'map'
    t.child_frame_id = f'{robot_name}/odom'

    # Translation
    t.transform.translation.x = x_corr
    t.transform.translation.y = y_corr
    t.transform.translation.z = 0.0

    # Rotation (convert yaw to quaternion)
    qx, qy, qz, qw = yaw_to_quaternion(theta_corr)
    t.transform.rotation.x = qx
    t.transform.rotation.y = qy
    t.transform.rotation.z = qz
    t.transform.rotation.w = qw

    # Broadcast
    tf_broadcaster.sendTransform(t)
```

### 5.3 Quaternion Conversion

```python
def yaw_to_quaternion(yaw):
    """
    Convert yaw angle to quaternion.

    For 2D rotation around z-axis:
    q = [0, 0, sin(yaw/2), cos(yaw/2)]
    """
    qx = 0.0
    qy = 0.0
    qz = np.sin(yaw / 2.0)
    qw = np.cos(yaw / 2.0)

    return qx, qy, qz, qw

def quaternion_to_yaw(qx, qy, qz, qw):
    """
    Extract yaw angle from quaternion.

    For 2D: yaw = atan2(2(qw*qz + qx*qy), 1 - 2(qy² + qz²))
    """
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return yaw
```

### 5.4 Update Frequency

The `map → odom` transform should be published at **high frequency** (10+ Hz) to:
- Provide smooth visualization in RViz
- Enable other nodes to use corrected pose
- Prevent TF tree timeouts

**Implementation:**
```python
# In LocalSubmapGenerator.__init__()
self.create_timer(0.1, self._publish_tf_callback)  # 10 Hz

def _publish_tf_callback(self):
    """Publish map->odom TF at fixed rate."""
    if self.current_pose is not None and self.latest_odom_pose is not None:
        self._publish_map_to_odom_tf()
```

---

## 6. Common Issues and Debugging

### 6.1 TF Tree Validation

```bash
# View full TF tree
ros2 run tf2_tools view_frames

# Check specific transform
ros2 run tf2_ros tf2_echo map tb3_1/base_footprint
```

### 6.2 Frame Convention Errors

**Symptom:** Robot appears to move in wrong direction or teleport.

**Cause:** Incorrect transform inversion or composition order.

**Fix:** Verify:
1. Frame labels match intended parent-child relationship
2. Inverse computed correctly
3. Composition order: parent × child

### 6.3 Timestamp Issues

**Symptom:** TF extrapolation errors, jumpy visualization.

**Cause:** Stale timestamps or inconsistent clocks.

**Fix:**
- Always use current time: `self.get_clock().now().to_msg()`
- Publish TF at regular intervals
- Check system time synchronization

---

## 7. Coordinate Frame Best Practices

### 7.1 Right-Hand Rule

Follow the right-hand coordinate system:
- **x-axis:** Forward
- **y-axis:** Left
- **z-axis:** Up
- **Rotation:** Counter-clockwise is positive

### 7.2 Units

**Standard Units:**
- Position: meters (m)
- Orientation: radians (rad)
- Time: seconds (s)

### 7.3 Frame Naming

**Convention:**
- Namespace frames with robot name: `tb3_1/odom`, `tb3_2/odom`
- Global frames without namespace: `map`, `earth`
- Descriptive suffixes: `_link`, `_frame`, `_footprint`

---

## References

1. **ROS REP-105:** "Coordinate Frames for Mobile Platforms." https://www.ros.org/reps/rep-0105.html

2. **ROS REP-103:** "Standard Units of Measure and Coordinate Conventions." https://www.ros.org/reps/rep-0103.html

3. **Foote, T. (2013).** "tf: The Transform Library." *IEEE International Conference on Technologies for Practical Robot Applications (TePRA)*.

4. **Corke, P. (2017).** *Robotics, Vision and Control*. Springer. Chapter 2: Representing Position and Orientation.

5. **Murray, R. M., Li, Z., & Sastry, S. S. (1994).** *A Mathematical Introduction to Robotic Manipulation*. CRC Press. Chapter 2: Rigid Body Motion.

---

## Summary

Current core documentation files:

1. **`00_system_overview.md`** — System architecture and design rationale
2. **`01_ekf_slam_theory.md`** — EKF-SLAM mathematical derivation
3. **`02_landmark_features.md`** — Feature extraction and Hessian normal form
4. **`03_data_association.md`** — Mahalanobis distance and statistical gating
5. **`05_submap_management.md`** — Submap creation and global map stitching
6. **`06_uncertainty_quantification.md`** — Information theory and confidence metrics
7. **`07_coordinate_frames.md`** — TF management and frame conventions

**Primary Documentation:**
- `methodology_feature_mapping.md` — Complete feature-based SLAM methodology
- `literature_review.md` — Academic background for comparative study
- `00_system_overview.md` — System architecture overview

These technical documents provide supplementary mathematical and theoretical foundations for feature-based SLAM with SVD-based submap stitching.
