# IMU Prediction Logging and Analysis

**Date**: 2025-11-27
**Purpose**: Log every IMU prediction (200 Hz) to diagnose EKF prediction issues
**Status**: ✅ Implemented

---

## Overview

The EKF prediction step runs at **200 Hz** (every IMU callback). To diagnose why predictions might be incorrect, we now log **every single prediction** to a CSV file.

### What Gets Logged

Each row in the CSV contains:
- **timestamp**: Simulation time (seconds)
- **callback_count**: IMU callback number (1, 2, 3, ...)
- **omega_z**: Angular velocity from IMU (rad/s)
- **vx, vy**: Commanded velocity in body frame (m/s) - from cmd_vel
- **x_before, y_before, theta_before**: EKF state BEFORE prediction
- **x_after, y_after, theta_after**: EKF state AFTER prediction
- **dx, dy, dtheta**: Change in state from this single prediction step

---

## Usage

### 1. Run the System

```bash
# Terminal 1: Launch system
cd ~/thesis_ws
source install/setup.bash
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# Terminal 2: Send velocity commands (or let navigation run)
ros2 run navigation test_straight_line --ros-args \
  -p robot_name:=tb3_1 \
  -p distance:=8.0 \
  -p velocity:=0.2
```

**Log file location**: `./imu_logs/imu_predictions_tb3_1.csv`

The log file is created when the first IMU callback runs after EKF initialization.

### 2. Analyze the Log

After stopping the system (Ctrl+C), run the analysis script:

```bash
# Install dependencies if needed
pip install pandas matplotlib numpy

# Run analysis (from thesis_ws directory)
python3 /tmp/analyze_imu_predictions.py ./imu_logs/imu_predictions_tb3_1.csv
```

**Output**:
- Console summary with statistics and issue detection
- Plot window showing 6 graphs
- PNG file: `./imu_logs/imu_predictions_tb3_1_analysis.png`
- Summary CSV (1Hz samples): `./imu_logs/imu_predictions_tb3_1_summary.csv`

---

## What to Look For

### ✅ Healthy Predictions

**Console output should show**:
```
📊 Dataset Info:
   Total predictions: 20000
   Duration: 100.00 seconds
   Average rate: 200.0 Hz  ← Should be ~200 Hz

📈 Velocity Statistics:
   Commanded vx: min=0.000, max=0.200, mean=0.150 m/s  ← Robot moving

📏 Position Change Statistics (per prediction step ~5ms):
   dx: min=0.000mm, max=1.000mm, mean=0.750mm  ← Positive dx when moving forward
   dy: min=-0.100mm, max=0.100mm, mean=0.000mm  ← Small, near zero

🔍 Issue Detection:
   Non-zero dx predictions: 15000/20000 (75.0%)  ← Should be >50% when moving
   Non-zero vx: 15000/20000 (75.0%)  ← Should match movement time
   Mean dt: 5.00ms (expected: ~5ms for 200Hz)  ← Should be ~5ms
   Expected dx per step (0.2 m/s): 1.000mm
   Actual mean dx (when vx>0.1): 0.950mm  ← Should be close to expected
```

**Plots should show**:
- Trajectory: Smooth path in X-Y plane
- Velocity: Jumps from 0 to 0.2 m/s when robot starts
- Position: Smooth ramp up (not flat line!)
- dx per step: Jumps to ~1mm when moving (not staying at 0!)

---

### ❌ Problem 1: Predictions Not Happening

**Symptoms**:
```
🔍 Issue Detection:
   Non-zero dx predictions: 10/20000 (0.1%)  ← ❌ Too few!
   ⚠️  WARNING: Less than 10% of predictions moved the robot!
```

**Possible causes**:
1. **vx = 0**: Robot stationary entire time
2. **dt = 0**: Time not advancing (simulation paused?)
3. **Sign error**: Positive vx creating negative dx
4. **State not updating**: Calculation correct but not applied

**Check**:
- Look at `vx` column - is it non-zero when robot should be moving?
- Look at `dx` column - is it always zero even when vx > 0?
- Check dt calculation in `ekf_lib.py:85-103`

---

### ❌ Problem 2: CMD_VEL Not Updating Velocity

**Symptoms**:
```
📈 Velocity Statistics:
   Commanded vx: min=0.000, max=0.000, mean=0.000 m/s  ← ❌ Always zero!

🔍 Issue Detection:
   Non-zero vx: 0/20000 (0.0%)
   ⚠️  WARNING: Robot moved but vx was zero most of the time!
      Possible cause: cmd_vel not being received or not updating ekf.vx
```

**Possible causes**:
1. **cmd_vel not published**: Navigation not running
2. **cmd_vel callback not updating ekf.vx**: Check `cmd_vel_callback()` at line 250
3. **cmd_vel callback gated**: Was returning early before EKF initialized (we fixed this!)

**Check console logs**:
```bash
# Should see these messages:
[CMD_VEL] #10: vx=0.200 m/s, vy=0.000 m/s, ω=0.000 rad/s | EKF: vx 0.000 → 0.200 m/s
```

If you don't see `[CMD_VEL]` messages, cmd_vel is not being published or subscribed.

---

### ❌ Problem 3: Predictions 10x Too Small

**Symptoms**:
```
📏 Position Change Statistics (per prediction step ~5ms):
   dx: min=0.000mm, max=0.100mm, mean=0.075mm  ← ❌ Should be ~1mm!

🔍 Issue Detection:
   Expected dx per step (0.2 m/s): 1.000mm
   Actual mean dx (when vx>0.1): 0.100mm  ← ❌ 10x too small!
   ❌ ERROR: Predictions are 10x too small!
```

**Possible causes**:
1. **dt wrong**: If dt = 0.0005 instead of 0.005, predictions 10x too small
2. **vx wrong units**: If vx in mm/s instead of m/s
3. **Covariance collapse**: K → 0, updates don't apply (already fixed with increased Q_imu)
4. **Sign error**: Negative velocity canceling positive position

**Check**:
- Mean dt in analysis output (should be ~5ms)
- `[PREDICT DEBUG]` messages showing dt value
- Compare vx in CSV to cmd_vel commands (should match)

---

### ❌ Problem 4: Wrong Reference Frame

**Symptoms**:
```
Plot shows trajectory spiraling or moving in wrong direction relative to heading
```

**Current implementation** (lines 107-108 in `ekf_lib.py`):
```python
dx = self.vx * np.cos(theta) * dt  # Transform body→world
dy = self.vx * np.sin(theta) * dt
```

This assumes:
- `self.vx` is in **body frame** (forward velocity)
- `theta` is robot heading in world frame
- Transformation: body frame velocity → world frame displacement

**If trajectory is wrong**:
- Check if cmd_vel.linear.x is in body frame (it should be!)
- Check if theta is in radians (it should be!)
- Check sign convention: +theta = counter-clockwise?

---

## Manual CSV Analysis

If you want to manually check the CSV:

```bash
# View first 20 rows
head -20 ./imu_logs/imu_predictions_tb3_1.csv

# Count non-zero dx values
awk -F',' 'NR>1 && $12 != 0 {count++} END {print count, "non-zero dx predictions"}' ./imu_logs/imu_predictions_tb3_1.csv

# Show vx values when robot is moving
awk -F',' 'NR>1 && $4 > 0.01 {print "vx:", $4, "dx:", $12}' ./imu_logs/imu_predictions_tb3_1.csv | head -20
```

---

## Expected Data Example

**When robot moves forward at 0.2 m/s**:

| timestamp | callback_count | omega_z | vx | x_before | x_after | dx |
|-----------|----------------|---------|----|-----------|---------|----|
| 10.000 | 1000 | 0.000 | 0.200 | 0.5000 | 0.5010 | 0.0010 |
| 10.005 | 1001 | 0.000 | 0.200 | 0.5010 | 0.5020 | 0.0010 |
| 10.010 | 1002 | 0.000 | 0.200 | 0.5020 | 0.5030 | 0.0010 |

**Analysis**:
- dt = 0.005s (5ms between predictions)
- vx = 0.200 m/s (constant velocity)
- dx = vx × dt = 0.200 × 0.005 = 0.001m = 1mm ✅
- Position increases by 1mm each step ✅

---

## Performance Notes

**File size**: At 200 Hz, expect:
- ~200 rows/second
- ~20KB/second
- ~1.2 MB/minute
- ~12 MB for 10-minute run

**Flushing**: File is flushed every 100 callbacks (0.5 seconds) to ensure data isn't lost if system crashes.

**Shutdown**: File is properly closed when node shuts down via `shutdown()` method.

---

## Files Modified

1. **`local_submap_generator.py:225-285`**: Added IMU prediction logging
   - Opens CSV file on first IMU callback after initialization
   - Logs every prediction (200 Hz)
   - Flushes every 100 callbacks

2. **`local_submap_generator.py:678-689`**: Added file close in shutdown
   - Ensures file is properly closed on Ctrl+C

3. **Created `/tmp/analyze_imu_predictions.py`**: Analysis script
   - Reads CSV
   - Calculates statistics
   - Detects common issues
   - Generates plots

---

## Quick Diagnosis Workflow

1. **Run system for ~10-30 seconds** to collect data
2. **Stop system** (Ctrl+C)
3. **Run analysis script**:
   ```bash
   python3 /tmp/analyze_imu_predictions.py
   ```
4. **Check console output** for warnings:
   - Non-zero dx < 10%? → Predictions not happening
   - Non-zero vx = 0%? → cmd_vel not updating
   - Mean dt ≠ 5ms? → Timing issue
   - Actual dx ≪ expected? → Scale/unit error
5. **Check plots**:
   - Position flat line? → Not moving
   - Velocity always zero? → cmd_vel issue
   - dx always zero? → Prediction not applying
6. **Check CSV manually** if needed

---

**Status**: ✅ Ready to use
**Next**: Run test and analyze IMU predictions to find root cause
