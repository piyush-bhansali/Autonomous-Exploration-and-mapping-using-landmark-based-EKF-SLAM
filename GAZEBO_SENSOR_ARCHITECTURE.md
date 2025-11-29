# Gazebo Sensor Architecture - Complete Explanation

**Date**: 2025-11-27
**Topic**: How sensors work in Gazebo and what was missing

---

## Your Robot's Sensors

### Sensors in TurtleBot3 Model

```
turtlebot3_waffle_pi/
├── IMU (tb3_imu)                    ← FIXED: Added plugin
├── Lidar (hls_lfcd_lds)             ← Works: Uses global sensor system
├── Camera (camera)                  ← Works: Uses global sensor system
└── Actuators:
    ├── DiffDrive (wheels)           ← Has plugin
    └── JointStatePublisher          ← Has plugin
```

---

## How Gazebo Sensors Work

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────┐
│                    GAZEBO SIMULATION                         │
│  ┌─────────────────────────────────────────────────────────┐│
│  │         Physics Engine (dart/bullet)                    ││
│  │  • Robot poses, velocities, accelerations               ││
│  │  • Collisions, forces, torques                          ││
│  └───────────────┬─────────────────────────────────────────┘│
│                  │                                           │
│  ┌───────────────▼────────┐      ┌───────────────────────┐ │
│  │  Sensor Systems        │      │  Rendering Engine     │ │
│  │  (per-sensor plugins)  │      │  (global for all      │ │
│  │                        │      │   visual sensors)     │ │
│  │  • IMU ←─────┐         │      │                       │ │
│  │  • Contact   │ NEED    │      │  • Lidar   ┐          │ │
│  │  • ForceTorque PLUGIN  │      │  • Camera  │ AUTO-    │ │
│  │              │         │      │  • Depth   │ MATIC   │ │
│  │              ↓         │      │            ↓          │ │
│  │  Compute sensor data   │      │  Raycast/render data │ │
│  └────────────┬───────────┘      └────────┬──────────────┘ │
│               │                           │                 │
│               └───────────┬───────────────┘                 │
│                           ↓                                 │
│               ┌─────────────────────────┐                   │
│               │  Gazebo Internal Topics │                   │
│               │  /world/.../model/.../  │                   │
│               │    • /imu               │                   │
│               │    • /scan              │                   │
│               │    • /camera/image      │                   │
│               └───────────┬─────────────┘                   │
└───────────────────────────┼─────────────────────────────────┘
                            │
                ┌───────────▼──────────────┐
                │  ROS-Gazebo Bridge       │
                │  (parameter_bridge)      │
                └───────────┬──────────────┘
                            │
                ┌───────────▼──────────────┐
                │     ROS 2 Topics         │
                │     /tb3_1/imu           │
                │     /tb3_1/scan          │
                │     /tb3_1/camera/image  │
                └──────────────────────────┘
```

---

## Sensor Type Comparison

### Type 1: Rendering Sensors (Automatic)

**Lidar Example**:
```xml
<sensor name="hls_lfcd_lds" type="gpu_lidar">
  <always_on>true</always_on>
  <update_rate>10</update_rate>
  <!-- ❌ NO PLUGIN NEEDED! -->
  <ray>
    <scan>
      <horizontal>
        <samples>360</samples>
        <min_angle>0.0</min_angle>
        <max_angle>6.28</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.12</min>
      <max>5.0</max>
    </range>
  </ray>
</sensor>
```

**How it works**:
1. Gazebo's **global sensor system** (`gz-sim-sensors-system`) automatically:
   - Detects all lidar/camera sensors
   - Performs raycasting for lidar
   - Renders images for cameras
2. Publishes to Gazebo topic: `/world/maze_world/model/tb3_1/link/base_scan/sensor/hls_lfcd_lds/scan`
3. Bridge forwards to ROS: `/tb3_1/scan`

**Status**: ✅ **Working** - No plugin needed!

---

### Type 2: Physics Sensors (Need Plugin)

**IMU - BEFORE Fix**:
```xml
<sensor name="tb3_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <!-- ❌ NO PLUGIN = NO DATA! -->
  <imu>
    <angular_velocity>
      <z>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>2e-4</stddev>
        </noise>
      </z>
    </angular_velocity>
    <linear_acceleration>
      <!-- ... -->
    </linear_acceleration>
  </imu>
  <!-- ❌ MISSING PLUGIN! -->
</sensor>
```

**What happened**:
1. ❌ Sensor defined but **no plugin** to compute IMU data
2. ❌ Gazebo **does not** automatically handle IMU (it's not a rendering sensor)
3. ❌ No data published to `/world/maze_world/model/tb3_1/imu`
4. ❌ Bridge created ROS topic `/tb3_1/imu` but **no data flowing through**
5. ❌ EKF IMU callback **never triggered** (no messages to receive!)

---

**IMU - AFTER Fix**:
```xml
<sensor name="tb3_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <imu>
    <angular_velocity>
      <!-- ... same config ... -->
    </angular_velocity>
    <linear_acceleration>
      <!-- ... same config ... -->
    </linear_acceleration>
  </imu>
  <!-- ✅ PLUGIN ADDED! -->
  <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
  </plugin>
</sensor>
```

**How it works now**:
1. ✅ Plugin (`gz-sim-imu-system`) reads robot state from physics engine:
   - Linear velocity → compute linear acceleration (derivative)
   - Angular velocity → read directly from physics
   - Gravity vector → apply to accelerometer
2. ✅ Adds Gaussian noise (from `<noise>` config)
3. ✅ Publishes to Gazebo topic: `/world/maze_world/model/tb3_1/imu`
4. ✅ Bridge forwards to ROS: `/tb3_1/imu` at **100 Hz**
5. ✅ EKF IMU callback triggered → predictions work!

**Status**: ✅ **Fixed** - Plugin added!

---

## Why Other Sensors Worked

### Lidar (`gpu_lidar`)
- **Type**: Rendering sensor
- **Handler**: Global `gz-sim-sensors-system` (loaded automatically)
- **Plugin needed?**: ❌ No
- **Working?**: ✅ Yes (you see laser scans in RViz)

### Camera (`camera`)
- **Type**: Rendering sensor
- **Handler**: Global `gz-sim-sensors-system` (loaded automatically)
- **Plugin needed?**: ❌ No
- **Working?**: ✅ Yes (if you subscribed to it)

### DiffDrive (wheel controller)
- **Type**: Actuator plugin
- **Plugin**: `gz-sim-diff-drive-system`
- **Purpose**:
  - Subscribes to `/tb3_1/cmd_vel`
  - Applies forces to wheel joints
  - Publishes odometry to `/model/tb3_1/odometry`
- **Status**: ✅ Working (robot moves when you send cmd_vel)

### JointStatePublisher
- **Type**: Publisher plugin
- **Plugin**: `gz-sim-joint-state-publisher-system`
- **Purpose**: Publishes joint angles/velocities
- **Status**: ✅ Working (you see joint states)

---

## The Fix

### What I Changed

**File**: `src/multi_robot_mapping/models/turtlebot3_waffle_pi/model.sdf`

**Lines 94-95**: Added IMU plugin
```xml
<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu">
</plugin>
```

**That's it!** Just 2 lines.

---

## Why This Was Missing

Possible reasons:

1. **Different Gazebo version**:
   - Older Gazebo Classic (not Gazebo Sim/Ignition) had different plugin system
   - Some plugins were loaded automatically
   - Migration to Gazebo Sim requires explicit plugins

2. **Copy-paste from example**:
   - Example model might have been for Gazebo Classic
   - Or IMU was disabled/not used in example

3. **Testing without IMU**:
   - If you only tested with odometry (no EKF), IMU wasn't needed
   - Bug only appeared when trying to use IMU for predictions

4. **Assumed automatic loading**:
   - Easy to assume IMU works like lidar (automatic)
   - But IMU is different sensor type (physics vs rendering)

---

## Verification Checklist

After restarting the system, verify:

### 1. Gazebo Topic
```bash
gz topic -e -t /world/maze_world/model/tb3_1/imu
```
Should see IMU messages streaming at 100 Hz:
```
header {
  stamp { sec: 123 nsec: 456789000 }
}
angular_velocity { x: 0.0 y: 0.0 z: 0.031 }
linear_acceleration { x: 0.05 y: 0.01 z: 9.81 }
```

### 2. ROS Topic
```bash
ros2 topic hz /tb3_1/imu
```
Should see:
```
average rate: 100.000
  min: 0.010s max: 0.010s std dev: 0.00000s window: 100
```

### 3. ROS Topic Content
```bash
ros2 topic echo /tb3_1/imu --once
```
Should see IMU message with:
- `angular_velocity.z` ≠ 0 when rotating
- `linear_acceleration.z` ≈ 9.81 (gravity)

### 4. EKF Console
Should see:
```
[INFO] [...]: [IMU] Logging predictions to ./imu_logs/imu_predictions_tb3_1.csv
[INFO] [...]: [IMU] #200: ω=0.0000 rad/s | This step: Δ=(1.000mm, ...) | ...
```

### 5. Log File
```bash
ls -lh ./imu_logs/imu_predictions_tb3_1.csv
head -20 ./imu_logs/imu_predictions_tb3_1.csv
```

### 6. EKF Predictions
Console should show:
```
[1. ODOM] Raw: (2.0000, 0.0000, 0.00°)
[2. UPDATE] Predicted: (1.9980, 0.0000, 0.00°) | Diff from odom: 2.0mm ✅
```
**Not** 2000mm like before!

---

## Summary

| Sensor | Type | Plugin Needed? | Status Before | Status After |
|--------|------|----------------|---------------|--------------|
| **IMU** | Physics | ✅ Yes | ❌ Missing → No data | ✅ Added → 100 Hz |
| **Lidar** | Rendering | ❌ No (auto) | ✅ Working | ✅ Working |
| **Camera** | Rendering | ❌ No (auto) | ✅ Working | ✅ Working |
| **DiffDrive** | Actuator | ✅ Yes | ✅ Has plugin | ✅ Working |
| **Joints** | Publisher | ✅ Yes | ✅ Has plugin | ✅ Working |

**The problem**: IMU is a **physics sensor**, not a rendering sensor. It needs an individual plugin (`gz-sim-imu-system`) to compute and publish data.

**The fix**: Added the missing plugin (2 lines).

**The impact**:
- Before: 0 Hz IMU → No predictions → 2000mm error
- After: 100 Hz IMU → 100 Hz predictions → < 10mm error (expected)

---

**Status**: ✅ Fixed - Ready to test!
**Next**: Restart system and verify IMU publishes at 100 Hz
