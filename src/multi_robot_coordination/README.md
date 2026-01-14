# Multi-Robot Coordination

Decentralized multi-robot coordination system for collaborative SLAM.

## Overview

This package enables multiple robots to:
- Detect each other via ROS2 discovery + proximity
- Align their coordinate frames using ICP on LiDAR scans
- Maintain a distributed pose graph of inter-robot transformations
- Share minimal data (transforms, positions, environment bounds only)
- Collaborate without centralized coordination

## Architecture

### Key Components

1. **DistributedFrameManager** (`frame_manager.py`)
   - Maintains pose graph of robot-to-robot transformations
   - Computes transitive transforms using BFS graph traversal
   - Handles reference frame updates
   - Validates loop closure consistency

2. **InterRobotICP** (`inter_robot_icp.py`)
   - Aligns robots using current LiDAR scans
   - Multi-metric verification:
     - ICP fitness score
     - RANSAC inlier ratio
     - Translation/rotation sanity checks
     - Scan overlap percentage
   - GPU-accelerated using Open3D tensors

3. **CommunicationManager** (`communication_manager.py`)
   - ROS2 pub/sub for robot info broadcasting
   - Service interface for alignment requests
   - Manages communication with multiple robots

4. **RobotCoordinator** (`robot_coordinator.py`)
   - Main coordination node
   - State machine: SOLO → DETECTING → ALIGNING → ALIGNED → COLLABORATIVE
   - Proximity detection and alignment triggering
   - Frame update broadcasting to mapping module

### State Machine

```
SOLO: No other robots detected, mapping alone
  ↓ (robot detected on network)
DETECTING: Other robot(s) visible, checking proximity
  ↓ (distance < threshold)
ALIGNING: Performing ICP scan-to-scan alignment
  ↓ (alignment successful)
ALIGNED: Successfully aligned to shared reference frame
  ↓ (multiple robots communicating)
COLLABORATIVE: Active multi-robot coordination
```

## Messages and Services

### Messages

**RobotInfo.msg**
```
string robot_id
float64 x, y, theta
float64[4] quaternion
string reference_frame_id
uint32 num_submaps
builtin_interfaces/Time timestamp
```

**EnvironmentBounds.msg**
```
string robot_id
float64 x_min, x_max, y_min, y_max
builtin_interfaces/Time timestamp
```

**FrameUpdate.msg**
```
string from_frame, to_frame
float64[16] transform_matrix
builtin_interfaces/Time timestamp
```

### Services

**RequestAlignment.srv**

Request:
```
string requesting_robot_id
float64[] current_scan_ranges
float64 scan_angle_min, scan_angle_max, scan_angle_increment
float64 range_min, range_max
float64[3] position
```

Response:
```
bool success
float64[16] transform_matrix
float64 fitness, inlier_ratio, overlap
string failure_reason
```

## Parameters

**robot_coordinator node:**
- `robot_name` (string, default: 'tb3_1'): Robot identifier
- `proximity_threshold` (float, default: 3.0): Distance threshold for alignment (meters)
- `alignment_timeout` (float, default: 10.0): Service call timeout (seconds)
- `info_broadcast_rate` (float, default: 1.0): Robot info broadcast frequency (Hz)
- `bounds_broadcast_rate` (float, default: 0.1): Environment bounds broadcast frequency (Hz)

## Usage

### Build

```bash
cd ~/thesis_ws
colcon build --packages-select multi_robot_coordination
source install/setup.bash
```

### Run Coordinator Node

```bash
ros2 run multi_robot_coordination robot_coordinator --ros-args -p robot_name:=tb3_1
```

### Launch with Multi-Robot System

(To be integrated with `full_system.launch.py` later)

```python
# Add to launch file for each robot:
coordinator_node = Node(
    package='multi_robot_coordination',
    executable='robot_coordinator',
    name=f'{robot_name}_coordinator',
    parameters=[{
        'use_sim_time': True,
        'robot_name': robot_name,
        'proximity_threshold': 3.0
    }]
)
```

## Design Decisions

### Distributed Pose Graph
- **Why**: Handles arbitrary meeting orders, no single point of failure
- **How**: Each robot maintains local pose graph, uses BFS to compute transitive transforms

### ROS2 Discovery + Proximity
- **Why**: Efficient, leverages ROS2's built-in discovery mechanism
- **How**: Detect when other robot nodes appear, verify distance before alignment

### Adaptive ICP Verification
- **Why**: ICP can fail or give false positives, need robust verification
- **How**: Multiple metrics (fitness, RANSAC, overlap) all must pass threshold

### Minimal Data Sharing
- **Why**: Avoid exponential growth in network traffic and map size
- **How**: Share only transforms (4×4 matrix), positions (3 floats), bounds (4 floats)

## Testing Strategy

### Test 1: Two Robots, Head-On Meeting
- Spawn tb3_1 at (-5, 0), tb3_2 at (5, 0)
- Let them explore toward each other
- Verify alignment when distance < 3m
- Check transform accuracy

### Test 2: Three Robots, Sequential Meetings
- tb3_1 explores alone (becomes reference)
- tb3_2 meets tb3_1 → aligns to tb3_1's frame
- tb3_3 meets tb3_2 → aligns transitively to tb3_1's frame
- Verify all robots in same coordinate system

### Test 3: Loop Closure
- tb3_1 ↔ tb3_2 align
- tb3_2 ↔ tb3_3 align
- tb3_3 ↔ tb3_1 meet (loop)
- Verify loop consistency check detects/corrects drift

## Integration with Existing System

### Mapping Module Integration

The coordinator publishes `FrameUpdate` messages when the reference frame changes. The mapping module should:

1. Subscribe to `/{robot_name}/frame_update`
2. Apply transformation to all submaps
3. Rebuild global map with new poses

**Example integration** (to be added to `local_submap_generator.py`):

```python
def handle_frame_update(self, msg):
    """Apply frame transformation from coordinator."""
    T_new = np.array(msg.transform_matrix).reshape(4, 4)
    self.stitcher.apply_frame_transform(T_new)
    self.current_pose = transform_pose(self.current_pose, T_new)
```

### Navigation Module Integration

The navigation module can subscribe to `/multi_robot/robot_info` to:
- Avoid other robots during path planning
- Use merged environment bounds for frontier detection
- Coordinate exploration to avoid overlap

## Future Enhancements

1. **Inter-Robot Loop Closure**: Share scan context descriptors for loop detection
2. **Distributed Pose Graph Optimization**: Periodic GTSAM optimization across robots
3. **Bandwidth Optimization**: Compress transforms, adaptive broadcast rates
4. **Failure Recovery**: Handle robot disconnections, reference frame fallback
5. **Map Merging**: Optional full map exchange for high-accuracy applications

## Dependencies

- ROS2 Jazzy
- Python 3.10+
- NumPy
- SciPy
- Open3D (with CUDA support)
- sensor_msgs, nav_msgs, geometry_msgs

## License

MIT

## Author

Piyush - Thesis Project (Multi-Robot SLAM)
