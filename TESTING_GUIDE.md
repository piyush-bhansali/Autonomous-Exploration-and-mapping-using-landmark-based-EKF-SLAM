# Testing Guide - Multi-Robot SLAM & Navigation System

## Exploration Termination Conditions

The exploration stops when **any** of these conditions are met:

### 1. **Time Limit Reached** (Default: 1 hour)
- **Parameter**: `max_exploration_time: 3600.0` (seconds)
- The robot explores for **60 minutes** maximum
- You can change this in the launch file

### 2. **No More Frontiers Detected**
- When all frontiers have been explored
- When no unexplored regions remain
- This means **exploration is complete** ✓

### 3. **All Frontiers Already Visited**
- Frontiers within 1m of previously explored locations are filtered out
- When all detected frontiers are "already explored"

## How to Test the System

### **Test 1: Quick Smoke Test (2-3 minutes)**

Verify the system starts correctly:

```bash
# Terminal 1: Launch system
cd /home/piyush/thesis_ws
source install/setup.bash
ros2 launch multi_robot_mapping full_system.launch.py

# Wait ~15 seconds for all components to start
```

**What to check**:
1. ✓ Gazebo opens with maze world and robot
2. ✓ RViz opens showing robot and environment
3. ✓ Robot starts moving (after ~15 seconds)
4. ✓ Point clouds appear in RViz

**Terminal 2: Monitor topics**
```bash
source install/setup.bash

# Check mapping is working
ros2 topic hz /tb3_1/global_map
# Should show ~0.5 Hz (updates every 2 seconds)

# Check navigation is publishing commands
ros2 topic hz /tb3_1/cmd_vel
# Should show ~10 Hz when robot is moving

# Check exploration status
ros2 topic echo /tb3_1/exploration_status --once
```

**Expected output**: `data: 'DETECTING_FRONTIERS'` or `'PLANNING_PATH'` or `'EXECUTING_PATH'`

---

### **Test 2: Short Exploration Run (5 minutes)**

Test with reduced exploration time:

```bash
# Edit the launch file to reduce time
# Change line 216 in full_system.launch.py:
'max_exploration_time': 300.0  # 5 minutes instead of 3600

# Then launch:
ros2 launch multi_robot_mapping full_system.launch.py
```

**What to observe**:
1. Robot moves to detected frontiers (orange spheres in RViz)
2. Green path lines show planned routes
3. Map grows as robot explores
4. After 5 minutes, robot stops (exploration complete)

---

### **Test 3: Manual Control + Mapping Only**

Test mapping without navigation:

```bash
# Launch without navigation
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# In another terminal, control robot manually
sudo apt install ros-jazzy-teleop-twist-keyboard  # If not installed
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/tb3_1/cmd_vel
```

**Controls**:
- `i` = forward
- `,` = backward
- `j` = turn left
- `l` = turn right
- `k` = stop
- `q/z` = increase/decrease speed

**What to observe**:
- Map builds in RViz as you drive
- Current submap (green) and global map (white) update
- Check saved submaps: `ls -lh /home/piyush/thesis_ws/submaps/tb3_1/`

---

### **Test 4: Monitor Exploration Statistics**

Track exploration progress in real-time:

```bash
# Terminal 1: Launch system
ros2 launch multi_robot_mapping full_system.launch.py

# Terminal 2: Monitor node logs
ros2 node info /tb3_1_navigation

# Terminal 3: Watch frontiers
ros2 topic echo /tb3_1/frontiers

# Terminal 4: Watch planned path
ros2 topic echo /tb3_1/planned_path
```

---

### **Test 5: Verify Map Quality**

After exploration, check the generated map:

```bash
# List generated submaps
ls -lh /home/piyush/thesis_ws/submaps/tb3_1/

# Count submaps
ls /home/piyush/thesis_ws/submaps/tb3_1/*.pcd | wc -l

# View a submap
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('/home/piyush/thesis_ws/submaps/tb3_1/submap_0000.pcd')
print(f'Points: {len(pcd.points)}')
o3d.visualization.draw_geometries([pcd])
"
```

---

## Testing Checklist

### Startup Phase (0-15 seconds)
- [ ] Gazebo launches with maze world
- [ ] Robot appears in Gazebo at position (-12, -12)
- [ ] RViz opens with configured view
- [ ] Clock bridge is running: `ros2 topic hz /clock`
- [ ] Laser scan publishing: `ros2 topic hz /tb3_1/scan` (~30 Hz)
- [ ] Odometry publishing: `ros2 topic hz /tb3_1/odom` (~30 Hz)

### Mapping Phase (7+ seconds)
- [ ] Submap generator node running: `ros2 node list | grep submap`
- [ ] Current submap visible in RViz (green points)
- [ ] Global map growing: `ros2 topic hz /tb3_1/global_map`
- [ ] Submaps saving to disk: `watch -n 1 "ls submaps/tb3_1/ | wc -l"`

### Navigation Phase (15+ seconds)
- [ ] Navigation node running: `ros2 node list | grep navigation`
- [ ] Frontiers detected: `ros2 topic echo /tb3_1/frontiers --once`
- [ ] Path planned: `ros2 topic echo /tb3_1/planned_path --once`
- [ ] Robot moving: `ros2 topic echo /tb3_1/cmd_vel`
- [ ] Velocity commands non-zero when executing path

### Exploration Completion
- [ ] Status changes to `EXPLORATION_COMPLETE`
- [ ] Robot stops moving (cmd_vel = 0)
- [ ] Final map saved
- [ ] All frontiers explored

---

## RViz Visualization Setup

In RViz, add these displays:

1. **RobotModel** - Shows TurtleBot3
   - Topic: `/robot_description`

2. **LaserScan** - Raw sensor data
   - Topic: `/tb3_1/scan`
   - Color: Red

3. **PointCloud2: Current Submap** - Active submap being built
   - Topic: `/tb3_1/current_submap`
   - Color: Green
   - Size: 0.02

4. **PointCloud2: Global Map** - Complete stitched map
   - Topic: `/tb3_1/global_map`
   - Color: White
   - Size: 0.01

5. **MarkerArray: Frontiers** - Detected exploration targets
   - Topic: `/tb3_1/frontiers`

6. **MarkerArray: Path** - Planned RRT* path
   - Topic: `/tb3_1/planned_path`

7. **Set Fixed Frame**: `map`

---

## Common Issues and Solutions

### Issue: Robot doesn't move
**Check**:
```bash
ros2 topic echo /tb3_1/exploration_status
```
**Solution**:
- If stuck in `DETECTING_FRONTIERS`: Map may be too sparse. Drive manually first to build initial map.
- If `EXPLORATION_COMPLETE` too early: Reduce `min_frontier_size` in frontier_detector.py

### Issue: No frontiers detected
**Reason**: Map is too uniform or too sparse
**Solution**:
- Drive robot manually for 30 seconds first
- Or use a world with more features (maze has more obstacles than park)

### Issue: Planning always fails
**Check**:
```bash
ros2 topic echo /tb3_1/global_map --once
```
**Solution**:
- Increase `max_iterations` in RRT* planner (currently 3000)
- Reduce frontier search radius

### Issue: System uses too much CPU/Memory
**Solution**:
- Reduce `scans_per_submap` from 250 to 150
- Increase `voxel_size` from 0.05 to 0.1
- Disable loop closure (already disabled by default)

---

## Performance Benchmarks

**Expected performance** (WSL2 + NVIDIA GPU):

| Metric | Value |
|--------|-------|
| Mapping frequency | 0.5 Hz (every 2s) |
| Scan processing | 30-50 Hz |
| ICP registration | 0.1-0.3s per submap |
| RRT* planning | 1-3s per path |
| Frontier detection | 0.2-0.5s |
| Control loop | 10 Hz |

---

## Adjusting Exploration Duration

**Quick test (5 minutes)**:
```python
# In full_system.launch.py, line 216:
'max_exploration_time': 300.0  # 5 minutes
```

**Medium test (15 minutes)**:
```python
'max_exploration_time': 900.0  # 15 minutes
```

**Full exploration (1 hour - default)**:
```python
'max_exploration_time': 3600.0  # 1 hour
```

**Unlimited (until all frontiers explored)**:
```python
'max_exploration_time': 86400.0  # 24 hours (effectively unlimited)
```

---

## Saving Results

### Submaps
```bash
# Submaps saved automatically to:
/home/piyush/thesis_ws/submaps/tb3_1/
```

### Trajectory
```bash
# Robot trajectory saved in:
/home/piyush/thesis_ws/test_results/tb3_1/
```

### Screenshots
```bash
# Take RViz screenshots: File → Export Image
```

---

## Quick Test Command Summary

```bash
# 1. Full autonomous (1 hour)
ros2 launch multi_robot_mapping full_system.launch.py

# 2. Quick test (5 min) - Edit launch file first
ros2 launch multi_robot_mapping full_system.launch.py

# 3. Mapping only (manual control)
ros2 launch multi_robot_mapping full_system.launch.py enable_navigation:=false

# 4. No visualization (faster)
ros2 launch multi_robot_mapping full_system.launch.py use_rviz:=false

# 5. Different world
ros2 launch multi_robot_mapping full_system.launch.py world:=park
```

---

## Expected Results

After successful exploration:
- ✓ 10-30 submaps generated (depends on duration)
- ✓ Complete point cloud map of environment
- ✓ Robot covered majority of accessible space
- ✓ Frontiers exhausted or time limit reached
- ✓ Clean termination (no errors)

---

## Troubleshooting Commands

```bash
# List all ROS2 nodes
ros2 node list

# Check node health
ros2 node info /tb3_1_submap_generator
ros2 node info /tb3_1_navigation

# List all topics
ros2 topic list

# Monitor CPU usage
htop

# Check Gazebo is running
ps aux | grep gz

# Kill everything
killall gz
killall rviz2
```

---

## Next Steps After Testing

1. **Analyze map quality**: View saved submaps
2. **Tune parameters**: Adjust for better coverage/speed
3. **Export data**: Save maps, trajectories for analysis
4. **Scale up**: Test with longer durations or multiple robots

---

## Questions?

Check system logs:
```bash
ros2 run rqt_console rqt_console
```

Or check this README:
```bash
cat /home/piyush/thesis_ws/README.md
```
