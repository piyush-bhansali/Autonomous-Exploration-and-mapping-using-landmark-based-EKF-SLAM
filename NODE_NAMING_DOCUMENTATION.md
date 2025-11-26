# Node Naming Documentation
## Multi-Robot SLAM System - Complete Node Naming Reference

**Generated**: 2025-11-26
**Purpose**: Document where and how all ROS node names are defined in the system

---

## Overview

ROS node names are defined in **two places**:
1. **Python source code** - Inside the node class `__init__()` method (base name)
2. **Launch file** - When creating the `Node()` object (can override with `name` parameter)

The **final node name** visible in `ros2 node list` is determined by the launch file's `name` parameter, which typically adds the robot namespace prefix.

---

## Node Naming Pattern

For multi-robot support, we follow this naming convention:
```
{robot_name}_{functional_name}
```

**Examples:**
- `tb3_1_submap_generator` (robot: tb3_1, function: submap_generator)
- `tb3_1_navigation` (robot: tb3_1, function: navigation)
- `tb3_2_submap_generator` (robot: tb3_2, function: submap_generator)

---

## 1. Mapping Node (EKF + SLAM)

### Python Source Code
**File**: `src/map_generation/map_generation/local_submap_generator.py`

**Line 35**: Node class initialization
```python
class LocalSubmapGenerator(Node):

    def __init__(self):
        super().__init__('local_submap_generator')  # ← Base node name
```

**Base Name**: `local_submap_generator`

---

### Launch File Configuration
**File**: `src/multi_robot_mapping/launch/full_system.launch.py`

**Lines 232-247**: Node instantiation
```python
submap_generator = Node(
    package='map_generation',
    executable='local_submap_generator',
    name=f'{robot_name}_submap_generator',  # ← Final node name
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'robot_name': robot_name,
        'scans_per_submap': 50,
        'save_directory': './submaps',
        'voxel_size': 0.08,
        'feature_method': 'hybrid',
        'enable_loop_closure': True,
        'enable_scan_to_map_icp': True
    }]
)
```

**Final Name**: `{robot_name}_submap_generator` → **`tb3_1_submap_generator`** (for robot tb3_1)

**How to Change**:
- **Base name**: Modify line 35 in `local_submap_generator.py`
- **Prefix**: Modify the `name` parameter in launch file (line 235)
- **Note**: The `name` parameter in the launch file **overrides** the base name from the Python code

---

## 2. Navigation Node (Frontier Exploration)

### Python Source Code
**File**: `src/navigation/navigation/simple_navigation.py`

**Line ~40**: Node class initialization (exact line may vary)
```python
class SimpleNavigation(Node):

    def __init__(self):
        super().__init__('simple_navigation')  # ← Base node name
```

**Base Name**: `simple_navigation`

---

### Launch File Configuration
**File**: `src/multi_robot_mapping/launch/full_system.launch.py`

**Lines 255-266**: Node instantiation
```python
navigation_node = Node(
    package='navigation',
    executable='simple_navigation',
    name=f'{robot_name}_navigation',  # ← Final node name
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'robot_name': robot_name,
        'robot_radius': 0.22
    }],
    condition=launch.conditions.IfCondition(enable_navigation)
)
```

**Final Name**: `{robot_name}_navigation` → **`tb3_1_navigation`** (for robot tb3_1)

**How to Change**:
- **Base name**: Modify `super().__init__()` in `simple_navigation.py`
- **Prefix**: Modify the `name` parameter in launch file (line 258)

---

## 3. Bridge Node (Gazebo ↔ ROS)

### Launch File Configuration
**File**: `src/multi_robot_mapping/launch/full_system.launch.py`

**Lines 193-202**: Node instantiation
```python
robot_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name=f'{robot_name}_bridge',  # ← Node name
    output='screen',
    parameters=[{'config_file': temp_config}]
)
```

**Final Name**: `{robot_name}_bridge` → **`tb3_1_bridge`**

**Note**: This uses the ROS package `ros_gz_bridge`, so there's no Python source code in our project. The node name is entirely controlled by the launch file.

---

## 4. Robot State Publisher

### Launch File Configuration
**File**: `src/multi_robot_mapping/launch/full_system.launch.py`

**Lines 210-220**: Node instantiation
```python
robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name=f'{robot_name}_state_publisher',  # ← Node name
    output='screen',
    parameters=[{
        'robot_description': urdf_content,
        'frame_prefix': ''
    }],
    remappings=[('/joint_states', f'/{robot_name}/joint_states')]
)
```

**Final Name**: `{robot_name}_state_publisher` → **`tb3_1_state_publisher`**

**Note**: This is a standard ROS package, no custom Python code.

---

## 5. RViz Visualization

### Launch File Configuration
**File**: `src/multi_robot_mapping/launch/full_system.launch.py`

**Lines 275-279**: Node instantiation
```python
rviz_node_tb3_1 = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz2_tb3_1',     # ← Node name
    namespace='tb3_1',       # ← Namespace
    # ...
)
```

**Final Name**: `/tb3_1/rviz2_tb3_1` (namespaced)

**Note**: RViz is a standard ROS package. The node uses both `name` and `namespace`.

---

## 6. Clock Bridge

### Launch File Configuration
**File**: `src/multi_robot_mapping/launch/full_system.launch.py`

**Lines ~127-133**: Node instantiation
```python
clock_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='clock_bridge',  # ← Node name
    output='screen',
    arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    parameters=[{'use_sim_time': True}]
)
```

**Final Name**: `clock_bridge`

**Note**: This is a system-level bridge (not per-robot), so no robot prefix.

---

## 7. Complete Node List (for tb3_1)

When running the full system with robot `tb3_1`, these nodes are created:

| Node Name | Package | Purpose | Defined In |
|-----------|---------|---------|------------|
| `clock_bridge` | ros_gz_bridge | Sync Gazebo/ROS time | Launch file line ~127 |
| `tb3_1_bridge` | ros_gz_bridge | Bridge sensor/cmd data | Launch file line 193 |
| `tb3_1_state_publisher` | robot_state_publisher | Publish robot TF tree | Launch file line 210 |
| `tb3_1_submap_generator` | map_generation | EKF + Mapping | Launch file line 232 + Python |
| `tb3_1_navigation` | navigation | Frontier exploration | Launch file line 255 + Python |
| `/tb3_1/rviz2_tb3_1` | rviz2 | Visualization | Launch file line 275 |

---

## 8. How Node Names Are Resolved

### Step-by-Step Resolution

1. **Python Code Defines Base Name**
   ```python
   super().__init__('local_submap_generator')
   ```
   → Creates node with name: `local_submap_generator`

2. **Launch File Can Override**
   ```python
   Node(
       package='map_generation',
       executable='local_submap_generator',
       name=f'{robot_name}_submap_generator'  # Overrides base name
   )
   ```
   → Final name becomes: `tb3_1_submap_generator`

3. **Namespace Can Be Added**
   ```python
   Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2_tb3_1',
       namespace='tb3_1'  # Adds namespace prefix
   )
   ```
   → Final name becomes: `/tb3_1/rviz2_tb3_1`

---

## 9. Changing Node Names

### To Change a Custom Node Name (map_generation, navigation)

**Option 1: Change in Python (affects all instances)**
```python
# In local_submap_generator.py
super().__init__('my_new_name')  # Changes base name
```

**Option 2: Change in Launch File (per-robot customization)**
```python
# In full_system.launch.py
submap_generator = Node(
    name=f'{robot_name}_my_custom_name'  # Overrides Python base name
)
```

**Recommendation**: Use **Option 2** for multi-robot systems to maintain the `{robot_name}_` prefix pattern.

---

### To Change a System Node Name (bridge, state_publisher)

**Only option: Modify launch file**
```python
robot_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    name='my_custom_bridge_name'  # Only way to set name
)
```

---

## 10. Verification Commands

### List All Active Nodes
```bash
ros2 node list
```

**Example Output:**
```
/clock_bridge
/tb3_1/rviz2_tb3_1
/tb3_1/transform_listener_impl_5d69d5e5ebd0
/tb3_1_bridge
/tb3_1_navigation
/tb3_1_state_publisher
/tb3_1_submap_generator
```

---

### Get Detailed Info About a Node
```bash
ros2 node info /tb3_1_submap_generator
```

**Shows:**
- Publishers
- Subscribers
- Services
- Actions

---

### Find Launch File for a Running Node
```bash
ros2 node info /tb3_1_submap_generator | head -1
```

**Then search launch files:**
```bash
grep -r "tb3_1_submap_generator" src/multi_robot_mapping/launch/
```

---

## 11. Multi-Robot Scaling

### Current System (1 Robot: tb3_1)
- `tb3_1_submap_generator`
- `tb3_1_navigation`
- `tb3_1_bridge`

### With 2 Robots (tb3_1 + tb3_2)
- `tb3_1_submap_generator` ← Robot 1
- `tb3_1_navigation` ← Robot 1
- `tb3_1_bridge` ← Robot 1
- `tb3_2_submap_generator` ← Robot 2
- `tb3_2_navigation` ← Robot 2
- `tb3_2_bridge` ← Robot 2

**Naming ensures no conflicts!**

---

## 12. File Locations Summary

### Python Node Definitions (Base Names)
```
src/map_generation/map_generation/local_submap_generator.py:35
src/navigation/navigation/simple_navigation.py:~40
src/map_generation/map_generation/pose_comparison_test.py:36
src/navigation/navigation/test_straight_line.py:~35
```

### Launch File Node Instantiation (Final Names)
```
src/multi_robot_mapping/launch/full_system.launch.py:
  - Line 127: clock_bridge
  - Line 193: tb3_1_bridge
  - Line 210: tb3_1_state_publisher
  - Line 232: tb3_1_submap_generator
  - Line 255: tb3_1_navigation
  - Line 275: rviz2_tb3_1
```

---

## 13. Quick Reference

| Want to Change | Edit This File | Line | What to Modify |
|----------------|----------------|------|----------------|
| Mapping node base name | `local_submap_generator.py` | 35 | `super().__init__('...')` |
| Navigation node base name | `simple_navigation.py` | ~40 | `super().__init__('...')` |
| Mapping node final name | `full_system.launch.py` | 235 | `name=f'{robot_name}_...'` |
| Navigation node final name | `full_system.launch.py` | 258 | `name=f'{robot_name}_...'` |
| Bridge node name | `full_system.launch.py` | 193 | `name=f'{robot_name}_...'` |
| RViz node name | `full_system.launch.py` | 277 | `name='...'` |

---

**End of Documentation**
