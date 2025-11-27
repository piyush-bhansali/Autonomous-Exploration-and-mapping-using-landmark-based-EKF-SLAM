# Debugging Enhancements for Odometry Scaling Error

**Date**: 2025-11-26
**Purpose**: Comprehensive debugging to identify the source of ~3.7x odometry scaling error

---

## Problem Statement

After all previous fixes (QoS matching, frame namespacing, topic routing), the odometry scaling error persists:

- **Gazebo Reality**: Robot moves correctly in simulation (user confirmed)
- **Odometry Reports**: ~5.0m traveled
- **EKF Estimates**: ~1.3m traveled
- **Error Ratio**: ~3.7x scaling factor

**Critical Insight**: The error is NOT in Gazebo physics, but in how odometry data is processed/converted in ROS.

---

## Debugging Code Added

### 1. Wheel Joint Velocity Monitoring

**File**: `src/map_generation/map_generation/local_submap_generator.py`
**Lines**: 100-106, 258-301

**What it does**:
- Subscribes to `/tb3_1/joint_states` topic
- Monitors `wheel_left_joint` and `wheel_right_joint` velocities (rad/s)
- Calculates expected linear and angular velocities using differential drive kinematics
- Logs every 2 seconds

**Expected Output**:
```
[WHEEL VEL] left=2.345 rad/s, right=2.567 rad/s
[WHEEL CALC] Expected: v=0.161 m/s, ω=0.025 rad/s (1.4°/s)
```

**Purpose**:
- Verify wheel speeds are correct from Gazebo
- Compare calculated velocities against odometry message velocities
- Identify if error originates in DiffDrive plugin or bridge conversion

**Formula Used**:
```
Wheel radius (r) = 0.033m
Wheel separation (L) = 0.287m

Linear velocity:  v = r × (ωL + ωR) / 2
Angular velocity: ω = r × (ωR - ωL) / L
```

---

### 2. Enhanced Odometry Message Logging

**File**: `src/map_generation/map_generation/local_submap_generator.py`
**Lines**: 303-378

**What it does**:
- Tracks first odometry pose (baseline for total distance)
- Tracks last odometry pose and timestamp (for delta calculations)
- Calculates total distance traveled from start
- Calculates instantaneous distance change between messages
- Calculates average speed from deltas
- Logs comprehensive odometry data every 2 seconds

**Expected Output**:
```
[ODOM RAW] frame_id=tb3_1/odom, child_frame_id=tb3_1/base_footprint
[ODOM RAW] pos=(4.997, 0.869), θ=-0.81°, vel=(0.200, 0.000) m/s, ω=0.000 rad/s
[ODOM RAW] Total dist from start: 4.997m, Count: 250
[ODOM RAW] Delta: 0.0040m in 0.020s, avg speed: 0.200 m/s
```

**Purpose**:
- Verify frame_ids are correctly namespaced (`tb3_1/odom`, not `odom`)
- Monitor raw position values being fed into EKF
- Track odometry message frequency and timestamping
- Calculate if position changes match velocity × time
- Identify if odometry is incrementing correctly or if there are jumps

---

### 3. Existing Debug Comparison (Already Present)

**File**: `src/map_generation/map_generation/local_submap_generator.py`
**Lines**: 165-184 (approximately)

**What it does**:
- Compares odometry position vs EKF position every 0.5 seconds
- Calculates linear and angular error

**Expected Output**:
```
[EKF DEBUG] Odom: (4.997, 0.869, -0.81°) | EKF: (1.334, 0.313, 8.23°) | Error: 3.705m, 54.75°
```

---

## Debug Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│ Gazebo DiffDrive Plugin                                     │
│ - Reads wheel joint velocities from physics engine          │
│ - Integrates velocities to calculate pose                   │
│ - Publishes /model/tb3_1/odometry (gz.msgs.Odometry)        │
│ - Publishes /model/tb3_1/joint_states (gz.msgs.JointState)  │
└─────────────────────────────────────────────────────────────┘
                    ↓                           ↓
┌──────────────────────────────────┐  ┌─────────────────────┐
│ ros_gz_bridge                    │  │ ros_gz_bridge       │
│ - Converts gz.msgs → ROS msgs    │  │ - Converts          │
│ → /tb3_1/odom (Odometry)         │  │ → /tb3_1/joint_states│
└──────────────────────────────────┘  └─────────────────────┘
                    ↓                           ↓
┌──────────────────────────────────┐  ┌─────────────────────┐
│ [NEW] odom_callback()            │  │ [NEW]               │
│ - Logs frame_ids                 │  │ joint_states_callback│
│ - Logs raw positions             │  │ - Logs wheel speeds │
│ - Tracks total distance          │  │ - Calculates v, ω   │
│ - Calculates deltas              │  │ - Compares to odom  │
└──────────────────────────────────┘  └─────────────────────┘
                    ↓
┌──────────────────────────────────┐
│ EKF (Extended Kalman Filter)     │
│ - Fuses odometry + IMU + ICP     │
│ - Publishes corrected pose       │
└──────────────────────────────────┘
                    ↓
┌──────────────────────────────────┐
│ [EXISTING] _debug_statistics     │
│ - Compares Odom vs EKF           │
│ - Shows 3.7x error               │
└──────────────────────────────────┘
```

---

## What to Look For in Output

### Scenario 1: Wheel velocities are wrong
**Symptom**: `[WHEEL CALC]` shows velocities that don't match command or Gazebo visual

**Likely Cause**: Gazebo DiffDrive plugin configuration error (wheel radius/separation)

**Action**: Check model.sdf wheel parameters

---

### Scenario 2: Wheel velocities correct, but odometry position grows too fast
**Symptom**:
- `[WHEEL CALC] Expected: v=0.200 m/s`
- `[ODOM RAW] vel=(0.200) m/s` ← matches
- `[ODOM RAW] Delta: 0.040m in 0.020s` ← should be 0.004m (0.2 × 0.02)

**Likely Cause**: DiffDrive integration error (timestep issue, integration bug)

**Action**: Check Gazebo Harmonic version, report bug to gz-sim

---

### Scenario 3: Odometry velocity in message doesn't match wheel calculation
**Symptom**:
- `[WHEEL CALC] Expected: v=0.200 m/s`
- `[ODOM RAW] vel=(0.740) m/s` ← doesn't match!

**Likely Cause**: DiffDrive plugin using wrong wheel parameters for velocity calculation

**Action**: Verify wheel_radius and wheel_separation in SDF match physical model

---

### Scenario 4: Frame ID mismatch (should be fixed already)
**Symptom**: `[ODOM RAW] frame_id=odom` (missing namespace)

**Likely Cause**: Launch file substitution didn't apply

**Action**: Verify `/tmp/tb3_1_spawned.sdf` has namespaced frame_ids

---

### Scenario 5: Position jumps instead of smooth integration
**Symptom**:
- `[ODOM RAW] Delta: 0.0040m in 0.020s, avg speed: 0.200 m/s` (correct)
- Next log: `Delta: 1.2000m in 0.020s, avg speed: 60.0 m/s` (jump!)

**Likely Cause**: Coordinate frame transformation error, TF tree corruption

**Action**: Monitor `/tf` topic for anomalies

---

## Diagnostic Commands to Run Alongside

### 1. Monitor Gazebo's Raw Odometry Output
```bash
gz topic -e -t /model/tb3_1/odometry -n 10
```
**Purpose**: See what Gazebo is actually publishing (raw values before bridge)

---

### 2. Monitor ROS Odometry
```bash
ros2 topic echo /tb3_1/odom --once
```
**Purpose**: Verify bridge conversion is correct

---

### 3. Monitor Joint States
```bash
ros2 topic echo /tb3_1/joint_states --once
```
**Purpose**: See raw wheel velocities

---

### 4. Compare Timestamps
```bash
ros2 topic echo /tb3_1/odom --field header.stamp
```
**Purpose**: Verify timestamp progression is smooth (20ms intervals at 50Hz)

---

### 5. Monitor TF Tree
```bash
ros2 run tf2_tools view_frames
```
**Purpose**: Generate PDF showing TF tree structure, verify no duplicate frames

---

## Next Steps

1. **Rebuild the workspace**:
   ```bash
   cd ~/thesis_ws
   colcon build --packages-select map_generation
   source install/setup.bash
   ```

2. **Launch the system**:
   ```bash
   ros2 launch multi_robot_mapping full_system.launch.py
   ```

3. **Wait for robot to move** (~10 seconds for navigation to start)

4. **Observe debug output** in the terminal:
   - Look for `[WHEEL VEL]` messages
   - Look for `[WHEEL CALC]` messages
   - Look for `[ODOM RAW]` messages
   - Look for `[EKF DEBUG]` messages

5. **Compare values**:
   - Do wheel velocities → calculated velocities match odometry velocities?
   - Does odometry delta distance match velocity × time?
   - Does total odometry distance grow linearly or in jumps?

6. **Run Gazebo comparison**:
   ```bash
   # In a new terminal
   gz topic -e -t /model/tb3_1/odometry
   ```
   Compare Gazebo's reported position against ROS odometry position

---

## Expected Result

This comprehensive debugging should reveal **exactly where the 3.7x scaling factor is introduced**:

- **If in Gazebo**: Wheel velocities won't match expected from joint_states
- **If in DiffDrive integration**: Delta distance won't match velocity × time
- **If in Bridge**: Gazebo topic values won't match ROS topic values
- **If in EKF**: Odometry will be correct but EKF will diverge (we already see this is NOT the case)

---

## Files Modified

1. **`src/map_generation/map_generation/local_submap_generator.py`**:
   - Added `JointState` import (line 6)
   - Added `joint_states` subscription (lines 100-106)
   - Added `joint_states_callback()` method (lines 258-301)
   - Enhanced `odom_callback()` with comprehensive tracking (lines 303-378)

---

**Status**: Ready for testing
**Next**: Rebuild, launch, and analyze output
