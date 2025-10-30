# Quick Start Guide

## When Does Exploration Stop?

The robot stops exploring when **any** of these happen:

1. **⏱️ Time limit reached**: Default = **60 minutes** (3600 seconds)
2. **✓ No more frontiers**: All areas explored
3. **✓ All frontiers visited**: Already been to all detected frontiers

## Test in 30 Seconds

```bash
# Build and launch
cd /home/piyush/thesis_ws
source install/setup.bash
ros2 launch multi_robot_mapping full_system.launch.py
```

**Wait 15 seconds**, then check:
- ✓ Gazebo running with robot
- ✓ RViz showing map
- ✓ Robot moving autonomously

## How to Change Exploration Time

Edit `/home/piyush/thesis_ws/src/multi_robot_mapping/launch/full_system.launch.py`

**Line 216**: Change `'max_exploration_time': 3600.0`

```python
'max_exploration_time': 300.0    # 5 minutes (quick test)
'max_exploration_time': 900.0    # 15 minutes
'max_exploration_time': 1800.0   # 30 minutes
'max_exploration_time': 3600.0   # 1 hour (default)
```

## Monitor Progress

```bash
# Check exploration status
ros2 topic echo /tb3_1/exploration_status

# Watch robot moving
ros2 topic hz /tb3_1/cmd_vel

# See detected frontiers
ros2 topic echo /tb3_1/frontiers

# Check map updates
ros2 topic hz /tb3_1/global_map
```

## Test Without Navigation (Manual Control)

```bash
# Launch mapping only
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# Control manually (new terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tb3_1/cmd_vel
```

## Check Results

```bash
# View generated submaps
ls -lh submaps/tb3_1/

# Count submaps created
ls submaps/tb3_1/*.pcd | wc -l
```

## Stop Everything

```bash
# Press Ctrl+C in launch terminal
# Or kill all:
killall gz
killall rviz2
```

## Common States

Monitor `/tb3_1/exploration_status`:
- `IDLE` - Starting up
- `DETECTING_FRONTIERS` - Looking for unexplored areas
- `PLANNING_PATH` - Computing RRT* path
- `EXECUTING_PATH` - Following path to frontier
- `EXPLORATION_COMPLETE` - **Done! All explored**

## Quick Verification

```bash
# Are all nodes running?
ros2 node list
# Should see:
# - /tb3_1_submap_generator
# - /tb3_1_navigation (if enabled)

# Is robot getting data?
ros2 topic hz /tb3_1/scan         # ~30 Hz
ros2 topic hz /tb3_1/odom         # ~30 Hz
ros2 topic hz /tb3_1/global_map   # ~0.5 Hz
```

## Expected Timeline

- **0-4s**: Gazebo starts
- **4-7s**: Robot spawns
- **7-15s**: Mapping begins, initial map builds
- **15s+**: Navigation starts, robot explores
- **5-60 min**: Exploration continues
- **End**: Robot stops, exploration complete

## For More Details

See: `/home/piyush/thesis_ws/TESTING_GUIDE.md`
