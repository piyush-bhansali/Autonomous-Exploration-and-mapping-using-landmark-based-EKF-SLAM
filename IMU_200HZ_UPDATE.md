# IMU Update Rate - Corrected to 200 Hz

**Date**: 2025-11-27
**Issue**: IMU was configured for 100 Hz, but EKF designed for 200 Hz
**Fix**: Changed `update_rate` from 100 to 200 Hz

---

## Good Catch!

You correctly noticed that the EKF code was designed for **200 Hz** IMU predictions:

### EKF Code Evidence

**File**: `ekf_lib.py`

```python
# Line 45: Comment says 200 Hz
# Applied at ~200 Hz, so per-update noise must be very small

# Line 97-99: Default dt for 200 Hz
dt = 0.005  # Default to 200 Hz (0.005s = 5ms = 200 Hz)

# Line 141: Drift warning threshold
if self.consecutive_predictions_without_update > 100:  # ~0.5s at 200Hz
```

### Local Submap Generator Evidence

**File**: `local_submap_generator.py`

```python
# Line 248: IMU callback logs every 200 messages
if self.imu_callback_count % 200 == 0:
    # This would be ~2 seconds at 100Hz, but ~1 second at 200Hz
```

---

## The Fix

### Before (Wrong)
```xml
<sensor name="tb3_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>  <!-- ❌ Too slow! -->
  ...
</sensor>
```

### After (Correct)
```xml
<sensor name="tb3_imu" type="imu">
  <always_on>true</always_on>
  <update_rate>200</update_rate>  <!-- ✅ Matches EKF design! -->
  ...
</sensor>
```

**File**: `src/multi_robot_mapping/models/turtlebot3_waffle_pi/model.sdf:50`

---

## Why 200 Hz Matters

### IMU Prediction Frequency

**Purpose**: High-frequency predictions between odometry updates

```
Timeline (50ms):

Odometry @ 50 Hz (every 20ms):
t=0ms    t=20ms   t=40ms   t=60ms
  |        |        |        |
  O--------O--------O--------O   Odometry updates

IMU @ 200 Hz (every 5ms):
t=0  5  10 15 20 25 30 35 40 45 50 55 60
 |   |   |   |   |   |   |   |   |   |   |   |
 I   I   I   I   I   I   I   I   I   I   I   I   IMU predictions

EKF Output @ 200 Hz:
 Smooth, high-frequency pose estimates!
```

### Benefits of 200 Hz vs 100 Hz

| Frequency | Period | Predictions Between Odom | Smoothness |
|-----------|--------|---------------------------|------------|
| **50 Hz** (odom only) | 20ms | 0 | ❌ Choppy |
| **100 Hz** | 10ms | 1 | ⚠️ Better |
| **200 Hz** | 5ms | 3 | ✅ Smooth |

**200 Hz gives 3 predictions between each odometry update!**

### Impact on TF Publishing

With 200 Hz IMU:
- Robot pose updated every 5ms
- TF tree updated at 200 Hz
- Visualization in RViz: smooth motion
- Scan alignment: better timing

With 100 Hz IMU:
- Robot pose updated every 10ms
- Only 1 prediction between odom updates
- Less smooth, more interpolation error

---

## Process Noise Implications

### Q_imu Values Were Tuned for 200 Hz

**File**: `ekf_lib.py:54-58`

```python
self.Q_imu = np.diag([
    0.0001,   # x: σ = 0.316mm per 5ms step (200 Hz)
    0.0001,   # y: σ = 0.316mm per 5ms step
    0.001     # theta: σ = 0.0316 rad per 5ms step
])
```

**At 200 Hz** (5ms per step):
- Per-step noise: 0.316mm
- Over 1 second (200 steps): √(200 × 0.0001) = 0.141m ≈ 14cm uncertainty growth
- **Reasonable** - allows healthy Kalman gain

**At 100 Hz** (10ms per step) with same Q:
- Per-step noise: 0.316mm (same variance)
- But larger dt means MORE position change per step!
- Over 1 second (100 steps): √(100 × 0.0001) = 0.10m = 10cm
- Process noise too small relative to actual motion
- P would still collapse!

**Solution**: Run at 200 Hz as designed, or retune Q_imu for 100 Hz

We chose: **Run at 200 Hz** (matches original design)

---

## Expected Test Results

### After Restart with 200 Hz IMU

```bash
$ ros2 topic hz /tb3_1/imu
average rate: 200.000
  min: 0.005s max: 0.005s std dev: 0.00000s window: 200
```

### Console Output

```
[IMU] #200: ω=0.0000 rad/s |
      This step: Δ=(1.000mm, 0.000mm, 0.000°) |
      Last 200 steps: Δ=(200.0mm, 0.0mm) |
      ekf.vx=0.200 m/s
```

**Note**: "Last 200 steps" now = **1 second** (not 2 seconds)

### CSV Analysis

**File**: `./imu_logs/imu_predictions_tb3_1.csv`

Expected dt between rows:
```
timestamp,callback_count,omega_z,vx,...
10.000,1000,0.0,0.2,...
10.005,1001,0.0,0.2,...  ← 5ms later (200 Hz)
10.010,1002,0.0,0.2,...  ← 5ms later
10.015,1003,0.0,0.2,...  ← 5ms later
```

Should see **0.005s = 5ms** intervals (200 Hz)

---

## Summary

### Changes Made

1. ✅ **Added IMU plugin** (`gz-sim-imu-system`)
2. ✅ **Increased update rate** from 100 Hz → 200 Hz

### Why 200 Hz?

- EKF code designed for 200 Hz (dt = 0.005s)
- Process noise Q_imu tuned for 200 Hz
- Provides 3 predictions between odometry updates
- Matches documentation and comments

### Files Modified

1. **`model.sdf:50`**: `<update_rate>200</update_rate>`
2. **`model.sdf:94-95`**: Added `<plugin>` tag

**Total**: 3 lines (1 changed, 2 added)

---

## Testing

After restart, verify:

```bash
# Check rate
ros2 topic hz /tb3_1/imu
# Should show: average rate: 200.000

# Check dt in CSV
head -20 ./imu_logs/imu_predictions_tb3_1.csv
# Timestamps should differ by 0.005s (5ms)

# Check console logs
# [IMU] messages should appear every ~1 second (200 callbacks at 200Hz)
```

---

**Status**: ✅ Corrected to 200 Hz
**Next**: Test and verify IMU publishes at 200 Hz
