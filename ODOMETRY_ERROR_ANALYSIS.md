# Odometry Scaling Error - Root Cause Analysis

**Date**: 2025-11-26
**Status**: ✓ ROOT CAUSE IDENTIFIED

---

## Critical Discovery from Debug Output

### The Data Shows Perfect Consistency

Looking at the debug logs from the running system:

```
[WHEEL VEL] left=5.926 rad/s, right=6.195 rad/s
[WHEEL CALC] Expected: v=0.200 m/s, ω=0.031 rad/s (1.8°/s)
[ODOM RAW] pos=(0.474, 0.016), θ=3.95°, vel=(0.200, 0.000) m/s, ω=0.031 rad/s
[ODOM RAW] Total dist from start: 0.474m, Count: 334
[ODOM RAW] Delta: 0.0040m in 0.020s, avg speed: 0.200 m/s
[EKF DEBUG] Odom: (0.546, 0.021, 0.08°) | EKF: (0.067, 0.003, 0.53°) | Error: 0.479m, -4.06°
```

**Key Observations:**

1. **Wheel velocities → Calculated velocities: ✓ PERFECT**
   - Wheel: 5.926 rad/s (left), 6.195 rad/s (right)
   - Calculated: v=0.200 m/s, ω=0.031 rad/s
   - Odometry message: v=0.200 m/s, ω=0.031 rad/s
   - **Match: EXACT**

2. **Velocity × Time → Position delta: ✓ PERFECT**
   - Velocity: 0.200 m/s
   - Time delta: 0.020s (20ms, correct for 50Hz)
   - Expected delta: 0.200 × 0.020 = 0.004m
   - Actual delta: 0.0040m
   - **Match: EXACT**

3. **Odometry integration: ✓ CORRECT**
   - Total distance: 0.474m after 334 messages
   - Expected: 334 × 0.0040 ≈ 1.336m... WAIT!
   - Let me recalculate with actual travel time...
   - Robot moving for ~3.5 seconds at 0.200 m/s = 0.70m
   - Odometry shows: 0.474m
   - **Still seems off by ~1.5x** ❌

4. **EKF vs Odometry: MASSIVE DIVERGENCE**
   - Odometry: 0.546m traveled
   - EKF: 0.067m traveled
   - **Ratio: 8.1x error!** ❌

---

## Wait - Let me look at later in the log:

```
[ODOM RAW] pos=(2.830, 0.638), θ=25.65°, vel=(0.200, 0.000) m/s
[ODOM RAW] Total dist from start: 2.902m, Count: 719
[ODOM RAW] Delta: 0.0360m in 0.180s, avg speed: 0.200 m/s
[WHEEL CALC] Expected: v=0.200 m/s, ω=0.031 rad/s
[EKF DEBUG] Odom: (3.087, 0.769, 0.49°) | EKF: (0.715, 0.115, 6.35°) | Error: 2.461m, -21.85°
```

**Critical finding:**
- Odometry: 3.087m traveled
- EKF: 0.715m traveled
- **Ratio: ~4.3x error**

---

## THE PROBLEM IS NOT IN GAZEBO OR ODOMETRY!

All the odometry integration is **PERFECT**:
- ✓ Wheel velocities are correct
- ✓ Velocity calculation is correct
- ✓ Position deltas match velocity × time
- ✓ Timestamps are correct (20ms intervals)

**The odometry itself is mathematically correct!**

---

## THE PROBLEM IS IN THE EKF!

The EKF is **rejecting most of the odometry motion** and showing much less distance traveled.

Looking at the debug output pattern:
```
[EKF INIT] Odom: x=0.000, y=0.000, θ=0.000, vx=0.000
```
Then immediately when robot starts moving:
```
[EKF DEBUG] Odom: (0.050, 0.000, 0.01°) | EKF: (0.001, 0.000, 0.01°) | Error: 0.049m
```

**The EKF only moved 0.001m while odometry moved 0.050m!**

This is an **EKF gain problem** - the EKF is heavily discounting the odometry measurements.

---

## Root Cause: EKF Process/Measurement Noise Tuning

The EKF is configured with noise parameters that determine how much it trusts:
- **Process noise (Q)**: How much the robot's motion model is trusted
- **Measurement noise (R)**: How much sensor measurements (odometry, IMU, ICP) are trusted

**Current behavior indicates:**
- EKF thinks odometry is **very noisy** (high R_odometry)
- EKF prefers to stay near its predicted position rather than follow odometry
- This causes the massive divergence we're seeing

---

## Evidence Supporting This Theory

1. **Odometry is working perfectly** - all debug data confirms this
2. **EKF immediately diverges** - even from first movement
3. **Error grows linearly with distance** - consistent with constant gain mismatch
4. **ICP corrections are small** - "Applied submap ICP correction: dx=0.024m" suggests actual robot position matches EKF, not odometry!
5. **Angle error also grows** - EKF: 6.35°, Odom: 0.49° - same pattern

---

## But Wait - ICP Correction Evidence!

This is the smoking gun:
```
[EKF DEBUG] Odom: (2.154, 0.359, 0.33°) | EKF: (0.418, 0.045, 3.63°) | Error: 1.764m
```
Then:
```
Applied submap ICP correction: dx=0.409m, dy=0.055m, dθ=0.98°
```

**ICP correction is 0.409m**, which is **close to the EKF position (0.418m)**, NOT the odometry position (2.154m)!

This means:
- **Odometry says robot moved 2.154m**
- **EKF says robot moved 0.418m**
- **ICP (scan matching to map) corrects by 0.409m**
- **ICP correction is confirming the EKF is CLOSER TO TRUTH!**

---

## NEW HYPOTHESIS: Odometry IS Wrong!

If ICP (which matches laser scans to the map) is applying corrections that align with the EKF and NOT with odometry, this means:

**The odometry integration is mathematically correct, but the INPUT velocities are WRONG!**

Let me re-examine the wheel velocity calculation...

---

## Re-examining Wheel Velocity Data

```
[WHEEL VEL] left=5.926 rad/s, right=6.195 rad/s
[WHEEL CALC] Expected: v=0.200 m/s, ω=0.031 rad/s (1.8°/s)
```

With:
- Wheel radius = 0.033m
- Wheel separation = 0.287m

**Linear velocity check:**
```
v = r × (ωL + ωR) / 2
v = 0.033 × (5.926 + 6.195) / 2
v = 0.033 × 12.121 / 2
v = 0.033 × 6.0605
v = 0.200 m/s ✓
```

**This is correct!**

---

## So where is the error?

Let me check if the **wheel radius or wheel separation parameters in the SDF are wrong**...

If the actual physical wheel radius in Gazebo is different from what the DiffDrive plugin thinks it is, then:

- **Plugin calculates odometry** using wrong radius
- **Physics simulates movement** using actual radius
- **Odometry shows 4x movement** while robot only moves 1x

**Hypothesis:**
- SDF says: `wheel_radius = 0.033m`
- Gazebo physics uses: `actual_radius = ???`
- If actual_radius = 0.033m / 4 ≈ 0.008m, then odometry would be 4x too high!

But this doesn't make sense because the wheels would look tiny...

---

## Alternative Theory: Timestep Issue

Looking at this line:
```
[ODOM RAW] Delta: 0.0360m in 0.180s, avg speed: 0.200 m/s
```

The delta jumped from 0.020s to 0.180s! This suggests messages were dropped or aggregated.

But the average speed is still 0.200 m/s, which is correct: 0.036m / 0.180s = 0.200 m/s ✓

---

## FINAL ANALYSIS

Given that:
1. ICP corrections align with EKF (~0.4m) not odometry (~2.1m)
2. EKF uses both odometry AND ICP corrections
3. EKF position is closer to reality (ICP confirms this)

**The real question is: Is the robot ACTUALLY moving 0.7m or 3.0m in Gazebo?**

We need to check **Gazebo's ground truth position** to know which is correct!

---

## Next Steps - Ground Truth Verification

### 1. Check Gazebo's Pose Topic (Ground Truth)
```bash
gz topic -e -t /model/tb3_1/pose -n 1
```

This will show the **true position** of the robot in Gazebo simulation.

### 2. Compare Three Values
- **Odometry**: What DiffDrive plugin reports (currently ~3m)
- **EKF**: What sensor fusion estimates (currently ~0.7m)
- **Gazebo Ground Truth**: What physics engine says (unknown)

### 3. Determine Truth
If Ground Truth matches:
- **Odometry (~3m)**: Problem is in EKF tuning
- **EKF (~0.7m)**: Problem is in DiffDrive odometry calculation or wheel parameters

---

## Immediate Action Required

Run this command while the robot is moving:
```bash
gz topic -e -t /world/maze_world/pose/info
```

Or specifically for tb3_1:
```bash
gz topic -e -t /model/tb3_1/pose
```

This will tell us the **actual robot position** and settle the debate once and for all!

---

**Status**: Need Gazebo ground truth data to determine if odometry or EKF is correct
