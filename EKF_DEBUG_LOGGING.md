# EKF Pipeline Debug Logging

**Date**: 2025-11-27
**Purpose**: Clean, informative debug logging showing the complete EKF sensor fusion pipeline
**Status**: ✅ Implemented (Updated with cmd_vel velocity source)

---

## New Debug Output Format

The debug logging now shows the complete sensor fusion pipeline in numbered steps:

### Example Output Sequence

```
[EKF INIT] Initialized at odom pose: (0.000, 0.000, 0.00°)

[1. ODOM] Raw: (0.2000, 0.0100, 1.50°)
[2. UPDATE] EKF after odom update: (0.2000, 0.0100, 1.50°) | Diff: 0.0mm | σ=1.00mm

[3. PREDICT] After 200 IMU predictions: Δ=(0.15mm, 0.08mm, 0.02°) | ω_avg=0.5°/s

[4. ICP] Correction: (1.5mm, 0.8mm, 0.10°) | Odom: (3.1234, 0.5678) → Corrected: (3.1249, 0.5686) | EKF after: (3.1249, 0.5686)
```

---

## Debug Message Breakdown

### [EKF INIT] - EKF Initialization
**When**: First odometry message received
**Frequency**: Once per session
**Shows**: Initial pose from odometry

```
[EKF INIT] Initialized at odom pose: (0.000, 0.000, 0.00°)
```

**What it means**:
- EKF state initialized to first odometry reading
- All subsequent updates are relative to this starting point

---

### [1. ODOM] - Raw Odometry Measurement
**When**: Every ~1 second (every 50th odometry message at 50 Hz)
**Location**: `odom_callback()` in `local_submap_generator.py`
**Shows**: Raw odometry pose BEFORE any EKF filtering

```
[1. ODOM] Raw: (3.1234, 0.5678, 18.50°)
```

**Fields**:
- `(x, y, θ)`: Position in meters and orientation in degrees
- This is the **measurement** being fed to EKF

**What it means**:
- Direct reading from wheel encoders via Gazebo DiffDrive plugin
- In simulation: highly accurate (~1mm precision)
- In real robot: subject to wheel slip, calibration errors

---

### [2. UPDATE] - EKF After Odometry Update
**When**: Every ~1 second (same timing as [1. ODOM])
**Location**: `odom_callback()` after `ekf.update()`
**Shows**: EKF state AFTER fusing odometry measurement

```
[2. UPDATE] EKF after odom update: (3.1234, 0.5678, 18.50°) | Diff: 0.5mm | σ=1.00mm
```

**Fields**:
- `(x, y, θ)`: EKF-filtered position
- `Diff`: Distance between EKF and raw odometry (should be tiny!)
- `σ`: Position uncertainty (standard deviation in mm)

**What it means**:
- EKF has incorporated the odometry measurement
- **If Diff is large** (>1cm): R_odom too high, EKF not trusting odometry
- **If σ grows unbounded**: Q_imu too high, filter diverging
- **Expected**: Diff < 2mm, σ < 2mm (with current parameters)

---

### [3. PREDICT] - IMU Prediction Step
**When**: Every ~1 second (after 200 IMU messages at 200 Hz)
**Location**: `imu_callback()` after 200 calls to `ekf.predict_imu()`
**Shows**: Cumulative change from 200 IMU-based predictions

```
[3. PREDICT] After 200 IMU predictions: Δ=(0.15mm, 0.08mm, 0.02°) | ω_avg=0.5°/s
```

**Fields**:
- `Δ=(dx, dy, dθ)`: How much EKF state changed over 200 predictions (~1 second)
- `ω_avg`: Average angular velocity during this period

**What it means**:
- Between odometry updates (20ms gaps), IMU maintains the estimate
- IMU predictions use velocity from **cmd_vel** (commanded velocity) + angular rate from gyroscope
- **Small Δ when stationary** (<0.5mm): Good, filter is stable
- **Large Δ when moving** (~40mm): Expected, robot moving at 0.2 m/s × 0.2s = 40mm per 200 IMU updates
- **Growing Δ over time**: Q_imu might be too high

**IMPORTANT - Velocity Source**:
- EKF now uses **cmd_vel** (commanded velocity) for predictions, NOT odometry velocity
- Reason: Odometry velocity is calculated from position derivatives (vx = Δpos / Δt)
- This creates lag during acceleration - velocity reported AFTER robot has moved
- cmd_vel represents INTENDED motion with no lag
- Odometry position/orientation still used for corrections

---

### [4. ICP] - ICP Correction Applied
**When**: When laser scan processed and ICP finds correction
**Frequency**: Variable (10-30 Hz depending on scan rate)
**Location**: `scan_callback()` after `scan_to_map_icp()`
**Shows**: ICP correction magnitude and effect on EKF

```
[4. ICP] Correction: (1.5mm, 0.8mm, 0.10°) | Odom: (3.1234, 0.5678) → Corrected: (3.1249, 0.5686) | EKF after: (3.1249, 0.5686)
```

**Fields**:
- `Correction`: ICP-measured error in odometry (dx, dy, dθ)
- `Odom`: Raw odometry position
- `Corrected`: Odometry + ICP correction (fed to EKF as measurement)
- `EKF after`: EKF state after incorporating ICP correction

**What it means**:
- ICP aligns current scan to accumulated map points
- Finds small drift in odometry-based scan placement
- Correction applied to odometry, then fed to EKF
- **Tiny corrections** (~1-5mm): Odometry is accurate, ICP just fine-tuning
- **Large corrections** (>5cm): Possible odometry drift or poor scan alignment
- **No ICP messages**: Either ICP disabled or insufficient accumulated scans

---

## Data Flow Visualization

```
Time=0.00s: [EKF INIT] Initialized at (0.000, 0.000)

Time=1.00s:
    [1. ODOM] Raw odometry: (0.200, 0.010, 1.5°)
    [2. UPDATE] EKF: (0.200, 0.010, 1.5°) | Diff: 0.0mm ✅

Time=2.00s:
    [3. PREDICT] IMU predicted: Δ=(0.10mm, 0.05mm, 0.01°)
    [1. ODOM] Raw odometry: (0.400, 0.020, 3.0°)
    [2. UPDATE] EKF: (0.400, 0.020, 3.0°) | Diff: 0.1mm ✅

Time=2.50s:
    [4. ICP] Correction: (1.2mm, 0.5mm, 0.05°)
         Odom (0.450, 0.025) → Corrected (0.4512, 0.0255)
         EKF after: (0.4512, 0.0255) ✅

Time=3.00s:
    [3. PREDICT] IMU predicted: Δ=(0.15mm, 0.08mm, 0.02°)
    [1. ODOM] Raw odometry: (0.600, 0.030, 4.5°)
    [2. UPDATE] EKF: (0.600, 0.030, 4.5°) | Diff: 0.2mm ✅
```

---

## Interpreting the Logs

### Healthy System Indicators

✅ **Perfect tracking**:
```
[1. ODOM] Raw: (8.000, 0.100, 45.0°)
[2. UPDATE] EKF after odom update: (8.000, 0.100, 45.0°) | Diff: 0.5mm | σ=1.2mm
[4. ICP] Correction: (2.0mm, 1.0mm, 0.15°)
```
- EKF matches odometry within 1mm
- ICP corrections tiny (1-3mm)
- Uncertainty bounded (<2mm)

---

### Problem: EKF Not Tracking Odometry

❌ **Symptoms**:
```
[1. ODOM] Raw: (8.000, 0.100, 45.0°)
[2. UPDATE] EKF after odom update: (3.200, 0.050, 12.0°) | Diff: 4800mm | σ=1.0mm
```

**Diagnosis**: R_odom too high → Kalman gain too low → EKF ignoring measurements

**Fix**: Reduce R_odom in `ekf_lib.py`:
```python
self.R_odom = np.diag([
    0.000001,  # Reduce from 0.0001 → 0.000001
    0.000001,
    0.00001
])
```

---

### Problem: EKF Uncertainty Growing

❌ **Symptoms**:
```
[2. UPDATE] ... | Diff: 0.5mm | σ=5.0mm
(1 second later)
[2. UPDATE] ... | Diff: 0.8mm | σ=15.0mm
(1 second later)
[2. UPDATE] ... | Diff: 1.2mm | σ=45.0mm
```

**Diagnosis**: Q_imu too high → Process noise accumulating → Covariance exploding

**Fix**: Reduce Q_imu in `ekf_lib.py`:
```python
self.Q_imu = np.diag([
    0.00001,  # Reduce from 0.0001 → 0.00001
    0.00001,
    0.0001
])
```

---

### Problem: Large ICP Corrections

❌ **Symptoms**:
```
[4. ICP] Correction: (150.0mm, 80.0mm, 5.00°)
```

**Possible causes**:
1. **Odometry drift** (real robot): Wheel slip, poor calibration
2. **EKF not tracking**: Check [2. UPDATE] Diff - if large, fix R_odom
3. **Poor scan matching**: Feature-poor environment, moving objects

**Fix**:
- If odometry is drifting (real robot): Increase R_odom to allow EKF to trust ICP more
- If EKF not tracking: Reduce R_odom as shown above
- If scan matching failing: Tune ICP parameters in `mapping_utils.py`

---

## Removed Debug Messages

The following verbose debug messages were removed for cleaner output:

### Removed: [ODOM RAW]
**Old**:
```
[ODOM RAW] frame_id=tb3_1/odom, child_frame_id=tb3_1/base_footprint
[ODOM RAW] pos=(3.123, 0.567), θ=18.50°, vel=(0.200, 0.000) m/s, ω=0.000 rad/s
[ODOM RAW] Total dist from start: 3.123m, Count: 156
[ODOM RAW] Delta: 0.0040m in 0.020s, avg speed: 0.200 m/s
```

**New**: Replaced with concise `[1. ODOM]` format (see above)

---

### Removed: [WHEEL VEL] and [WHEEL CALC]
**Old**:
```
[WHEEL VEL] left=6.061 rad/s, right=6.061 rad/s
[WHEEL CALC] Expected: v=0.200 m/s, ω=0.000 rad/s (0.0°/s)
```

**Reason**: Odometry already incorporates wheel velocities. Redundant for normal operation.

---

### Removed: [IMU DEBUG]
**Old**:
```
[IMU DEBUG] ω=0.031 rad/s (1.8°/s)
```

**New**: Replaced with `[3. PREDICT]` showing cumulative effect of 200 predictions

---

### Removed: [EKF STATUS]
**Old**:
```
[EKF STATUS] Pose: (3.123, 0.567, 18.50°) | Uncertainty: σ_pos=1.2mm, σ_θ=0.30°
```

**New**: Incorporated into `[2. UPDATE]` format

---

## Files Modified

1. **`src/map_generation/map_generation/local_submap_generator.py`**:
   - Lines 397-425: Updated odometry callback logging
   - Lines 242-274: Updated IMU callback logging
   - Lines 543-549: Updated ICP correction logging
   - Lines 276-279: Removed joint_states verbose logging
   - Line 368: Removed verbose ODOM RAW logging

**Total**: ~150 lines modified/removed for cleaner output

---

## Testing the New Debug Output

```bash
# Rebuild
source ~/thesis_ws/install/setup.bash

# Launch system
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# Run straight line test
source ~/thesis_ws/install/setup.bash
ros2 run navigation test_straight_line --ros-args -p robot_name:=tb3_1 -p distance:=8.0 -p velocity:=0.2
```

**Expected clean output**:
```
[EKF INIT] Initialized at odom pose: (0.000, 0.000, 0.00°)

[1. ODOM] Raw: (0.2000, 0.0100, 1.50°)
[2. UPDATE] EKF after odom update: (0.2000, 0.0100, 1.50°) | Diff: 0.0mm | σ=1.00mm

[3. PREDICT] After 200 IMU predictions: Δ=(0.40mm, 0.20mm, 0.00°) | ω_avg=0.0°/s

... (robot moves) ...

[1. ODOM] Raw: (8.0000, 0.1000, 0.50°)
[2. UPDATE] EKF after odom update: (8.0000, 0.1000, 0.50°) | Diff: 0.2mm | σ=1.05mm
[4. ICP] Correction: (1.5mm, 0.8mm, 0.10°) | Odom: (8.000, 0.100) → Corrected: (8.0015, 0.1008) | EKF after: (8.0015, 0.1008)
```

---

## Quick Reference Card

| Message | Shows | Normal Value | Problem Indicator |
|---------|-------|--------------|-------------------|
| `[1. ODOM]` | Raw odometry | Position in meters | N/A (ground truth) |
| `[2. UPDATE] Diff` | EKF vs odom error | < 2mm | > 10mm → Fix R_odom |
| `[2. UPDATE] σ` | Position uncertainty | < 2mm | Growing → Fix Q_imu |
| `[3. PREDICT] Δ` | IMU prediction drift | < 0.5mm when stopped | Growing → Fix Q_imu |
| `[4. ICP]` | Scan alignment correction | 1-5mm | > 50mm → Check odom/ICP |

---

## CMD_VEL Velocity Implementation (2025-11-27)

### Problem Identified
EKF predictions were 13x too slow despite correct velocity readings from odometry:
- Robot traveled 0.176m in 1 second
- EKF predicted only 0.013m movement
- Odometry showed vx=0.200 m/s (correct)
- But predictions didn't match expected motion

### Root Cause
Odometry velocity is calculated from position derivatives:
```
vx_odom = Δposition / Δt
```

This creates **velocity lag**:
- During acceleration, velocity is reported AFTER robot has moved
- Odometry says "vx=0.2 m/s" but that's the PAST velocity, not current
- EKF uses this lagged velocity for future predictions → wrong predictions

### Solution: Use cmd_vel for Predictions
**Implementation**: Subscribe to `/tb3_1/cmd_vel` topic

**Data flow**:
```
Navigation → cmd_vel (intent) → EKF predictions
Gazebo → odometry (measurement) → EKF corrections
IMU → angular velocity → EKF predictions
```

**Key changes**:
1. Added cmd_vel subscription in `local_submap_generator.py:112-118`
2. Created `cmd_vel_callback()` that updates `ekf.vx` and `ekf.vy`
3. Removed velocity updates from odometry callback
4. Odometry now only used for position/orientation corrections

### Code Implementation

**File**: `src/map_generation/map_generation/local_submap_generator.py`

```python
# Line 8: Added Twist import
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist

# Lines 112-118: Subscribe to cmd_vel
self.cmd_vel_sub = self.create_subscription(
    Twist,
    f'/{self.robot_name}/cmd_vel',
    self.cmd_vel_callback,
    10
)

# Lines 250-269: Cmd_vel callback
def cmd_vel_callback(self, msg):
    """
    Update EKF velocity from commanded velocity.

    We use COMMANDED velocity (cmd_vel) for EKF predictions, not odometry velocity.
    Reason: Odometry velocity is calculated from position derivatives, which:
    - Lags behind actual motion (calculated AFTER robot moves)
    - Has quantization noise
    - Doesn't reflect intended motion during acceleration

    Cmd_vel represents the INTENDED motion, which is better for predictions.
    Odometry position is still used for corrections via EKF updates.
    """
    if not self.ekf_initialized:
        return

    # Update EKF velocity from commanded velocity
    self.ekf.vx = msg.linear.x
    self.ekf.vy = msg.linear.y

# Lines 341-343: Removed velocity update from odometry
# NOTE: We NO LONGER update velocity from odometry!
# Velocity now comes from cmd_vel callback (commanded velocity)
# Odometry is only used for position/orientation corrections

# Lines 352, 373: Modified EKF updates
self.ekf.update(x_odom, y_odom, theta_odom, vx_odom=None)
```

### Expected Results

**Before (using odometry velocity)**:
```
[1. ODOM] Raw: (0.1765, 0.0100, 1.50°) | vx=0.200 m/s
[2. UPDATE] Predicted: (0.0130, 0.0010, 1.50°, ekf.vx=0.200) | Updated: (0.1765, 0.0100, 1.50°) | Diff: 163.5mm
```
❌ Prediction wrong despite correct velocity

**After (using cmd_vel velocity)**:
```
[1. ODOM] Raw: (0.1765, 0.0100, 1.50°) | vx=0.200 m/s
[2. UPDATE] Predicted: (0.1750, 0.0095, 1.50°, ekf.vx=0.200) | Updated: (0.1765, 0.0100, 1.50°) | Diff: 1.5mm
```
✅ Prediction matches odometry within 2mm

### Testing

**Test script**: `/tmp/test_cmd_vel_ekf.py`
- Monitors cmd_vel vs odometry velocity
- Verifies commanded velocity matches measured motion
- Helps diagnose any remaining prediction issues

**Test command**:
```bash
# Terminal 1: Launch system
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# Terminal 2: Run straight line test
ros2 run navigation test_straight_line --ros-args -p robot_name:=tb3_1 -p distance:=8.0 -p velocity:=0.2

# Terminal 3: Monitor cmd_vel (optional)
python3 /tmp/test_cmd_vel_ekf.py
```

---

**Status**: ✅ Implemented, ready for testing
**Next**: Run test and verify EKF predictions now match odometry
