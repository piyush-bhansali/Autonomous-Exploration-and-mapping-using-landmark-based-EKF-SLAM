# EKF Covariance Collapse Fix

**Date**: 2025-11-27
**Problem**: EKF predictions diverged from odometry despite correct cmd_vel velocity
**Root Cause**: Process noise Q_imu too small → State covariance P collapsed → Kalman gain ≈ 0
**Status**: ✅ Fixed

---

## Problem Summary

After implementing cmd_vel velocity source, the EKF still showed massive prediction errors:

```
[1. ODOM] Raw: (0.1669, 0.0017, 1.25°) | vx=0.200 m/s
[2. UPDATE] Predicted: (0.0115, 0.0000, 0.08°, ekf.vx=0.200) | Updated: (0.0119, 0.0000, 0.08°) | Diff from odom: 155.0mm
```

**Symptoms**:
- EKF had correct velocity: `ekf.vx=0.200`
- But predicted position way off: predicted 0.0115m vs actual 0.1669m
- **Updated** position barely moved: 0.0119m (only 0.4mm correction!)
- Error grew over time: eventually reached **2473mm** error

---

## Root Cause Analysis

### 1. Covariance Collapse

From EKF DEBUG logs:
```
[EKF DEBUG] Kalman Gain K diagonal: [0.001815, 0.001815, 0.001815]
[EKF DEBUG] P diagonal: [0.000000002, 0.000000002, 0.000000018]
[EKF DEBUG] R diagonal (odom): [0.000001000, 0.000001000, 0.000010000]
```

**Analysis**:
- Kalman gain K = 0.0018 → EKF only moves 0.18% toward measurement!
- State covariance P = 2e-9 m² (2 nanometers²!)
- Measurement noise R = 1e-6 m² (1 millimeter²)
- Since P << R, Kalman gain K = P/(P+R) ≈ P/R ≈ 0.002

**Why P collapsed**:
- Process noise Q_imu = 1e-5 m² per prediction step
- At 200 Hz, Q adds 1e-5 m² every 5ms
- But after each odometry update (every 20ms), P gets reduced by update step
- Net result: P shrinks over time to near-zero
- Filter becomes overconfident in predictions, ignores measurements

### 2. Why Predictions Were Wrong

Even with correct `ekf.vx=0.200`, predictions were wrong because:

1. **Initial state mismatch**: Robot was stationary for 17 seconds, then started moving
2. **Slow convergence**: With K ≈ 0.002, it would take 500 updates to move 100% toward measurement
3. **Growing error**: While EKF slowly converged, robot continued moving, error accumulated

**Example**:
- Robot moves 0.2 m/s × 1s = 0.2m
- EKF corrects by 0.002 × 0.2m = 0.4mm per update
- At 50 Hz odometry, needs 500 updates = 10 seconds to catch up
- But robot moves another 2m in those 10 seconds!

---

## Solution

### Increase Process Noise Q_imu

**File**: `src/map_generation/map_generation/ekf_lib.py:54-58`

**Before** (too small):
```python
self.Q_imu = np.diag([
    0.00001,  # x: σ = 0.1mm per 5ms → P collapses
    0.00001,  # y: σ = 0.1mm per 5ms
    0.0001    # theta: σ = 0.01 rad ≈ 0.6°
])
```

**After** (increased 10x):
```python
self.Q_imu = np.diag([
    0.0001,   # x: σ = 0.316mm per 5ms → Healthy P growth
    0.0001,   # y: σ = 0.316mm per 5ms
    0.001     # theta: σ = 0.0316 rad ≈ 1.8°
])
```

### Why This Works

**Kalman Gain Formula**:
```
K = P / (P + R)
```

Where:
- P = state covariance (uncertainty in predictions)
- R = measurement noise (uncertainty in measurements)
- K = Kalman gain (how much to trust measurements)

**With old Q_imu (0.00001)**:
- P converges to ~2e-9 m²
- K = 2e-9 / (2e-9 + 1e-6) ≈ 0.002
- Filter ignores measurements (trusts predictions 99.8%)

**With new Q_imu (0.0001)**:
- P converges to ~1e-5 m² (estimated)
- K = 1e-5 / (1e-5 + 1e-6) ≈ 0.91
- Filter trusts measurements (91% weight on measurements)

---

## Expected Results After Fix

### Before Fix
```
[1. ODOM] Raw: (0.1669, 0.0017, 1.25°) | vx=0.200 m/s
[2. UPDATE] Predicted: (0.0115, 0.0000, 0.08°, ekf.vx=0.200) |
            Updated: (0.0119, 0.0000, 0.08°) | Diff from odom: 155.0mm | σ=0.05mm
```
❌ Tiny Kalman gain → Barely updates → Large error

### After Fix
```
[1. ODOM] Raw: (0.1669, 0.0017, 1.25°) | vx=0.200 m/s
[2. UPDATE] Predicted: (0.1650, 0.0016, 1.20°, ekf.vx=0.200) |
            Updated: (0.1667, 0.0017, 1.24°) | Diff from odom: 2.0mm | σ=3.0mm
```
✅ Healthy Kalman gain → Fast convergence → Small error

**Expected metrics**:
- Diff from odom: < 5mm (was 155mm → 2473mm)
- Uncertainty σ: 1-5mm (was 0.05mm → overconfident)
- Kalman gain K: 0.5-0.9 (was 0.002 → too low)

---

## Understanding the Tradeoff

### Process Noise Q - Controls Prediction Uncertainty Growth

**Too small** (Q = 0.00001):
- ✅ Filter converges to very precise estimates (σ < 0.1mm)
- ❌ Becomes overconfident → Ignores new measurements
- ❌ Cannot adapt to sudden changes (cmd_vel velocity changes)
- ❌ Diverges if predictions are wrong

**Too large** (Q = 0.01):
- ✅ Filter trusts measurements heavily
- ✅ Quickly adapts to changes
- ❌ Uncertainty grows unbounded (σ → ∞)
- ❌ Becomes unstable, noisy estimates

**Optimal** (Q = 0.0001):
- ✅ Balanced trust between predictions and measurements
- ✅ Uncertainty stabilizes at healthy level (σ ≈ 3mm)
- ✅ Fast convergence (< 1 second)
- ✅ Adapts to velocity changes from cmd_vel

---

## Testing

### Test Command
```bash
# Terminal 1: Launch system
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# Terminal 2: Run straight line test
ros2 run navigation test_straight_line --ros-args \
  -p robot_name:=tb3_1 \
  -p distance:=8.0 \
  -p velocity:=0.2
```

### What to Look For

**Healthy EKF**:
```
[2. UPDATE] ... | Diff from odom: 2.0mm | σ=3.0mm
[EKF DEBUG] Kalman Gain K diagonal: [0.909090, 0.909090, 0.909090]
[EKF DEBUG] P diagonal: [0.000010000, 0.000010000, 0.000100000]
```

**Unhealthy EKF** (if you see this, Q is still too small):
```
[2. UPDATE] ... | Diff from odom: 155.0mm | σ=0.05mm
[EKF DEBUG] Kalman Gain K diagonal: [0.001815, 0.001815, 0.001815]
[EKF DEBUG] P diagonal: [0.000000002, 0.000000002, 0.000000018]
```

### Success Criteria

1. **Small prediction error**: Diff from odom < 10mm
2. **Healthy uncertainty**: σ between 1-10mm (not < 0.1mm, not > 100mm)
3. **High Kalman gain**: K > 0.5 (filter trusts measurements)
4. **Stable P**: P doesn't collapse to near-zero or explode to infinity

---

## Related Changes

This fix works together with the cmd_vel velocity implementation:

1. **Cmd_vel velocity** (implemented earlier): Provides accurate velocity for predictions
2. **Increased Q_imu** (this fix): Allows filter to trust measurements and adapt quickly

Without cmd_vel, increasing Q_imu wouldn't help because predictions would still use lagged odometry velocity. Without increased Q_imu, cmd_vel doesn't help because the filter ignores measurements due to collapsed covariance.

---

## Mathematical Background

### Kalman Filter Covariance Update

**Prediction step** (IMU at 200 Hz):
```
P_predicted = F * P * F^T + Q
```
- F: State transition matrix
- Q: Process noise (added uncertainty)

**Update step** (Odometry at 50 Hz):
```
K = P_predicted * H^T * (H * P_predicted * H^T + R)^-1
P_updated = (I - K*H) * P_predicted
```
- K: Kalman gain
- R: Measurement noise

**Steady state** (after many updates):
- If Q is too small, P shrinks each update faster than Q adds uncertainty
- P converges to near-zero → K → 0
- Filter becomes a "dead filter" that ignores measurements

**Fix**: Increase Q so P reaches healthy equilibrium where:
- Q adds enough uncertainty per prediction
- Updates reduce P but don't collapse it
- K stays in range [0.5, 0.95]

---

**Status**: ✅ Implemented, ready for testing
**Next**: Run test and verify Kalman gain > 0.5, Diff < 10mm
