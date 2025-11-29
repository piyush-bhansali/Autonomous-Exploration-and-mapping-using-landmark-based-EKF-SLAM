# How Odometry Works - Complete Explanation

**Date**: 2025-11-27
**Question**: "What about odometer, how was that working?"
**Answer**: Odometry comes from the DiffDrive plugin, not a sensor!

---

## Key Difference: Odometry is NOT a Sensor

### Sensors vs Actuator Plugins

| Type | Purpose | Example | Publishes |
|------|---------|---------|-----------|
| **Sensor** | Measures environment | IMU, Lidar, Camera | Sensor data |
| **Actuator Plugin** | Controls robot + computes motion | DiffDrive | Commands + Odometry |

**Odometry is a byproduct of the actuator plugin!**

---

## How DiffDrive Works

### The DiffDrive Plugin Does 3 Things:

```
┌────────────────────────────────────────────────────────┐
│         gz-sim-diff-drive-system Plugin                │
│                                                         │
│  1. LISTENS to cmd_vel                                 │
│     /model/tb3_1/cmd_vel                               │
│     ↓                                                   │
│  2. CONTROLS wheels (applies torques)                  │
│     • Reads: linear.x, angular.z                       │
│     • Computes: left_wheel_vel, right_wheel_vel        │
│     • Applies: torques to wheel joints                 │
│     ↓                                                   │
│  3. COMPUTES odometry (from wheel positions)           │
│     • Reads: wheel joint positions (encoders)          │
│     • Calculates: robot pose (x, y, θ)                 │
│     • Calculates: robot velocity (vx, vy, ω)           │
│     • Publishes: /model/tb3_1/odometry @ 50 Hz         │
│                                                         │
└────────────────────────────────────────────────────────┘
```

### Configuration in Your Robot

**File**: `src/multi_robot_mapping/models/turtlebot3_waffle_pi/model.sdf`

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">

  <!-- 1. WHICH WHEELS TO CONTROL -->
  <left_joint>wheel_left_joint</left_joint>
  <right_joint>wheel_right_joint</right_joint>

  <!-- 2. ROBOT DIMENSIONS (for kinematics) -->
  <wheel_separation>0.287</wheel_separation>  <!-- Distance between wheels -->
  <wheel_radius>0.033</wheel_radius>          <!-- Wheel radius -->

  <!-- 3. VELOCITY LIMITS -->
  <max_linear_velocity>0.26</max_linear_velocity>
  <max_angular_velocity>1.82</max_angular_velocity>

  <!-- 4. ODOMETRY CONFIGURATION -->
  <frame_id>odom</frame_id>                   <!-- Parent frame -->
  <child_frame_id>base_footprint</child_frame_id>  <!-- Robot frame -->
  <odom_publisher_frequency>50</odom_publisher_frequency>  <!-- 50 Hz -->

  <!-- 5. NOISE MODEL (simulates encoder errors) -->
  <noise>0.015</noise>                        <!-- 1.5% noise on wheel encoders -->

  <pose_covariance_diagonal>
    0.005 0.005 0.001 0.001 0.001 0.02
  </pose_covariance_diagonal>

  <twist_covariance_diagonal>
    0.002 0.002 0.001 0.001 0.001 0.01
  </twist_covariance_diagonal>

</plugin>
```

---

## Odometry Calculation (How it Computes Position)

### Step-by-Step Process

**At 50 Hz** (every 20ms), the DiffDrive plugin:

#### 1. Read Wheel Encoder Positions
```
left_wheel_angle = joint_position(wheel_left_joint)    # radians
right_wheel_angle = joint_position(wheel_right_joint)  # radians
```

#### 2. Calculate Wheel Distances Traveled
```
left_distance = left_wheel_angle * wheel_radius
right_distance = right_wheel_angle * wheel_radius
```

#### 3. Calculate Robot Motion (Differential Drive Kinematics)
```
# Linear distance (average of both wheels)
linear_distance = (left_distance + right_distance) / 2

# Angular distance (difference between wheels)
angular_distance = (right_distance - left_distance) / wheel_separation

# Update robot pose (dead reckoning)
x += linear_distance * cos(theta)
y += linear_distance * sin(theta)
theta += angular_distance
```

#### 4. Calculate Velocities (derivative)
```
dt = time_since_last_update  # 0.02 seconds (50 Hz)

vx = (x - x_prev) / dt
vy = (y - y_prev) / dt
omega = (theta - theta_prev) / dt
```

#### 5. Add Noise (simulates real wheel encoders)
```
noise_factor = gaussian_random(mean=0, stddev=0.015)
vx *= (1 + noise_factor)
omega *= (1 + noise_factor)
# (simplified - actual noise is more complex)
```

#### 6. Publish Odometry Message
```
Publishes to: /model/tb3_1/odometry
Rate: 50 Hz
Content:
  - pose: (x, y, z, roll, pitch, yaw)
  - twist: (vx, vy, vz, roll_rate, pitch_rate, yaw_rate)
  - covariances: uncertainty estimates
```

---

## Why Odometry Worked (But IMU Didn't)

### Odometry Path

```
┌─────────────────────────────────────────────────────────────┐
│                  GAZEBO SIMULATION                          │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │         DiffDrive Plugin                           │    │
│  │  (gz-sim-diff-drive-system)                        │    │
│  │                                                     │    │
│  │  ✅ Already has plugin tag in SDF                  │    │
│  │  ✅ Reads wheel joint positions from physics       │    │
│  │  ✅ Computes odometry (dead reckoning)             │    │
│  │  ✅ Publishes at 50 Hz                             │    │
│  └──────────────┬─────────────────────────────────────┘    │
│                 │                                           │
│                 ↓                                           │
│   Gazebo Topic: /model/tb3_1/odometry                      │
│                 ↓                                           │
└─────────────────┼───────────────────────────────────────────┘
                  │
      ┌───────────▼──────────────┐
      │  ROS-Gazebo Bridge       │
      │  (parameter_bridge)      │
      └───────────┬──────────────┘
                  │
      ┌───────────▼──────────────┐
      │  ROS Topic               │
      │  /tb3_1/odom             │
      │  @ 50 Hz ✅              │
      └──────────────────────────┘
```

### IMU Path (BEFORE Fix)

```
┌─────────────────────────────────────────────────────────────┐
│                  GAZEBO SIMULATION                          │
│                                                              │
│  ┌────────────────────────────────────────────────────┐    │
│  │         IMU Sensor                                 │    │
│  │  (sensor type="imu")                               │    │
│  │                                                     │    │
│  │  ❌ NO PLUGIN TAG IN SDF!                          │    │
│  │  ❌ Nobody reads physics state                     │    │
│  │  ❌ Nobody computes IMU data                       │    │
│  │  ❌ Nothing published                              │    │
│  └─────────────────────────────────────────────────────┘    │
│                                                              │
│   Gazebo Topic: /world/maze_world/model/tb3_1/imu          │
│                 ↓ (EMPTY - no data!)                        │
└─────────────────┼───────────────────────────────────────────┘
                  │
      ┌───────────▼──────────────┐
      │  ROS-Gazebo Bridge       │
      │  (creates topic but      │
      │   no messages to forward)│
      └───────────┬──────────────┘
                  │
      ┌───────────▼──────────────┐
      │  ROS Topic               │
      │  /tb3_1/imu              │
      │  @ 0 Hz ❌ (exists but   │
      │          no data!)       │
      └──────────────────────────┘
```

---

## Key Differences

| Aspect | Odometry (DiffDrive) | IMU (Before Fix) | IMU (After Fix) |
|--------|---------------------|------------------|-----------------|
| **Type** | Actuator Plugin | Sensor | Sensor |
| **Plugin?** | ✅ Yes (DiffDrive) | ❌ No (missing!) | ✅ Yes (IMU plugin) |
| **Data Source** | Wheel joint positions | Physics (velocity, accel) | Physics (velocity, accel) |
| **Computation** | In plugin code | ❌ Nobody doing it! | ✅ Plugin does it |
| **Publishing** | Plugin publishes | ❌ Nothing to publish | ✅ Plugin publishes |
| **Rate** | 50 Hz | 0 Hz | 100 Hz |
| **Working?** | ✅ Always worked | ❌ Never worked | ✅ Now works |

---

## What Each Plugin Does

### 1. DiffDrive Plugin (`gz-sim-diff-drive-system`)

**Purpose**: Control differential drive robot

**Responsibilities**:
- ✅ Subscribe to cmd_vel
- ✅ Apply forces to wheel joints
- ✅ **Compute odometry from wheel encoders**
- ✅ Publish odometry messages
- ✅ Publish TF (if enabled)

**Why odometry works**: Built into the motor controller plugin!

---

### 2. IMU Plugin (`gz-sim-imu-system`) - NOW ADDED

**Purpose**: Simulate IMU sensor

**Responsibilities**:
- ✅ Read robot velocity from physics engine
- ✅ Compute linear acceleration (derivative of velocity)
- ✅ Read angular velocity from physics engine
- ✅ Apply gravity vector (9.81 m/s² downward)
- ✅ Add noise (from sensor config)
- ✅ Publish IMU messages

**Why IMU didn't work before**: This plugin was missing!

---

### 3. Joint State Publisher Plugin

**Purpose**: Publish wheel positions

```xml
<plugin filename="gz-sim-joint-state-publisher-system"
        name="gz::sim::systems::JointStatePublisher">
  <joint_name>wheel_left_joint</joint_name>
  <joint_name>wheel_right_joint</joint_name>
</plugin>
```

**Publishes**: `/tb3_1/joint_states` (wheel angles, velocities)

---

## Real World Analogy

Think of a real robot:

### Odometry = Motor Controller
```
Motor Controller (one device does both):
├── Reads: Wheel encoder ticks
├── Controls: Motor voltages
└── Computes & Publishes: Odometry (x, y, θ, vx, ω)
```
In Gazebo: **DiffDrive plugin = Motor controller**

### IMU = Separate Sensor
```
IMU Chip (separate device):
├── Accelerometer (measures acceleration)
├── Gyroscope (measures rotation rate)
└── Publishes: IMU data (ax, ay, az, ωx, ωy, ωz)
```
In Gazebo: **IMU sensor + IMU plugin = IMU chip**

---

## Why This Matters for EKF

### Data Flow to EKF

```
┌─────────────────────────────────────────────────┐
│             LOCAL SUBMAP GENERATOR              │
│                (EKF Node)                        │
│                                                  │
│  ┌──────────────────────────────────────────┐  │
│  │  odom_callback()                         │  │
│  │  Topic: /tb3_1/odom @ 50 Hz ✅           │  │
│  │  • Position: (x, y, θ)                   │  │
│  │  • Velocity: (vx, vy, ω) ← LAGGED!       │  │
│  │  • Used for: EKF corrections            │  │
│  └──────────────────────────────────────────┘  │
│                                                  │
│  ┌──────────────────────────────────────────┐  │
│  │  imu_callback()                          │  │
│  │  Topic: /tb3_1/imu @ 100 Hz ✅ (NOW!)    │  │
│  │  • Angular velocity: ω                   │  │
│  │  • Used for: EKF predictions @ 100 Hz    │  │
│  └──────────────────────────────────────────┘  │
│                                                  │
│  ┌──────────────────────────────────────────┐  │
│  │  cmd_vel_callback()                      │  │
│  │  Topic: /tb3_1/cmd_vel ✅                │  │
│  │  • Commanded velocity: vx                │  │
│  │  • Used for: EKF predictions (no lag!)   │  │
│  └──────────────────────────────────────────┘  │
│                                                  │
│  ┌──────────────────────────────────────────┐  │
│  │  EKF FILTER                              │  │
│  │  • Predicts @ 100 Hz (using cmd_vel+IMU) │  │
│  │  • Updates @ 50 Hz (using odometry)      │  │
│  │  • Result: Smooth 100 Hz position        │  │
│  └──────────────────────────────────────────┘  │
└─────────────────────────────────────────────────┘
```

### Before IMU Fix

```
Odometry: 50 Hz ✅  ─┐
                    ├─→ EKF updates @ 50 Hz only
cmd_vel: ~10 Hz ✅  ─┤   (no predictions between!)
                    │
IMU: 0 Hz ❌ ────────┘   Result: Laggy, inaccurate
```

### After IMU Fix

```
Odometry: 50 Hz ✅  ─┐
                    ├─→ EKF predicts @ 100 Hz
cmd_vel: ~10 Hz ✅  ─┤      updates @ 50 Hz
                    │
IMU: 100 Hz ✅ ──────┘   Result: Smooth, accurate!
```

---

## Summary

### Question: "How was odometry working?"

**Answer**:

**Odometry comes from the DiffDrive plugin, not a sensor!**

1. **DiffDrive plugin** (always had plugin ✅):
   - Controls wheel motors
   - Reads wheel encoder positions
   - **Computes odometry** (dead reckoning)
   - Publishes `/model/tb3_1/odometry` → bridge → `/tb3_1/odom`

2. **IMU sensor** (had no plugin ❌):
   - Sensor defined but no plugin to compute data
   - Nothing published
   - **Fixed by adding `gz-sim-imu-system` plugin**

3. **Key difference**:
   - Odometry = byproduct of actuator (motor controller)
   - IMU = dedicated sensor (needs own plugin)

### Why You Didn't Notice Before

- Odometry worked ✅ → Robot moved, map built
- IMU silent ❌ → But you weren't using predictions yet
- Only when trying EKF predictions did you discover IMU missing

---

**Status**: ✅ Now you understand the complete picture!
**Next**: Restart system and verify IMU publishes alongside odometry
