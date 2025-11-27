# Odometry Data Flow - Complete Path
## From Gazebo DiffDrive Plugin to ROS EKF

**Generated**: 2025-11-26
**Purpose**: Understand where and how odometry is calculated and transmitted

---

## Overview

Odometry data flows through **4 major components**:

```
┌─────────────────────────────────────────────────────────────────────┐
│ 1. GAZEBO PHYSICS ENGINE                                            │
│    - Simulates wheel rotation based on cmd_vel                      │
│    - Updates joint positions/velocities                             │
└─────────────────┬───────────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│ 2. DIFFDRIVE PLUGIN (Gazebo C++ code)                               │
│    - Reads wheel joint velocities from physics                      │
│    - Calculates odometry using differential drive kinematics        │
│    - Publishes to Gazebo topic: /model/tb3_1/odometry               │
└─────────────────┬───────────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│ 3. ROS_GZ_BRIDGE                                                     │
│    - Subscribes to Gazebo topic: /model/tb3_1/odometry              │
│    - Converts gz.msgs.Odometry → nav_msgs/msg/Odometry              │
│    - Publishes to ROS topic: /tb3_1/odom                            │
└─────────────────┬───────────────────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│ 4. YOUR ROS NODES                                                    │
│    - tb3_1_submap_generator: EKF subscribes to /tb3_1/odom          │
│    - tb3_1_navigation: Uses odometry for localization               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## 1. Gazebo Physics Engine

### Location
**Binary**: `/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-physics-system.so`

**Source Code** (if you want to inspect):
```bash
# Gazebo source (not in your workspace)
https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/physics/Physics.cc
```

### What It Does
1. **Receives velocity commands** from DiffDrive plugin
2. **Simulates wheel physics**:
   - Applies torque to wheel joints
   - Calculates angular velocity: `ω = v / r`
   - Updates joint positions based on timestep
3. **Handles collisions** and friction
4. **Updates robot pose** in the world

### Key Physics Parameters
```xml
<!-- From maze.sdf world file -->
<physics name="1ms" type="ignored">
  <max_step_size>0.001</max_step_size>  <!-- 1ms timestep -->
  <real_time_factor>1</real_time_factor> <!-- Run at real-time speed -->
</physics>
```

**This means:**
- Physics updates **1000 times per second** (1ms steps)
- Simulation tries to match **real-time** (1x speed)

---

## 2. DiffDrive Plugin - WHERE ODOMETRY IS CALCULATED

### Location
**Binary**: `/opt/ros/jazzy/opt/gz_sim_vendor/lib/gz-sim-8/plugins/libgz-sim-diff-drive-system.so`

**Source Code**:
```bash
https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/diff_drive/DiffDrive.cc
```

### Configuration (Your SDF File)
**File**: `src/multi_robot_mapping/models/turtlebot3_waffle_pi/model.sdf:480-518`

```xml
<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">

  <!-- WHEEL JOINTS - Read from physics engine -->
  <left_joint>wheel_left_joint</left_joint>
  <right_joint>wheel_right_joint</right_joint>

  <!-- KINEMATICS - Used for odometry calculation -->
  <wheel_separation>0.287</wheel_separation>  <!-- Distance between wheels (meters) -->
  <wheel_radius>0.033</wheel_radius>          <!-- Wheel radius (meters) -->

  <!-- ODOMETRY OUTPUT -->
  <frame_id>odom</frame_id>                   <!-- Parent frame in odom message -->
  <child_frame_id>base_footprint</child_frame_id>
  <odom_publisher_frequency>50</odom_publisher_frequency>  <!-- 50 Hz = every 20ms -->

  <!-- NOISE MODEL -->
  <noise>0.015</noise>  <!-- Wheel encoder noise (1.5% error) -->
</plugin>
```

---

### How DiffDrive Calculates Odometry

**Every physics update (1ms), the plugin:**

#### Step 1: Read Wheel Joint Velocities
```cpp
// Simplified from DiffDrive.cc
double leftWheelVelocity = leftJoint->GetVelocity(0);   // rad/s
double rightWheelVelocity = rightJoint->GetVelocity(0); // rad/s
```

#### Step 2: Apply Encoder Noise
```cpp
// Add Gaussian noise to simulate encoder errors
leftWheelVelocity += noise * gaussianRandom();
rightWheelVelocity += noise * gaussianRandom();
```

#### Step 3: Calculate Linear and Angular Velocities
```cpp
// Differential drive kinematics
double wheelRadius = 0.033;        // meters
double wheelSeparation = 0.287;    // meters

// Linear velocity of each wheel
double vLeft = leftWheelVelocity * wheelRadius;
double vRight = rightWheelVelocity * wheelRadius;

// Robot's linear velocity (forward)
double linearVelocity = (vLeft + vRight) / 2.0;

// Robot's angular velocity (turning)
double angularVelocity = (vRight - vLeft) / wheelSeparation;
```

**Example Calculation:**
```
Given:
  leftWheelVelocity = 6.06 rad/s
  rightWheelVelocity = 6.06 rad/s
  wheelRadius = 0.033 m
  wheelSeparation = 0.287 m

Calculate:
  vLeft = 6.06 * 0.033 = 0.200 m/s
  vRight = 6.06 * 0.033 = 0.200 m/s

  linearVelocity = (0.200 + 0.200) / 2 = 0.200 m/s  ✓
  angularVelocity = (0.200 - 0.200) / 0.287 = 0 rad/s  (going straight)
```

#### Step 4: Integrate to Get Position
```cpp
// Time since last update
double dt = currentTime - lastUpdateTime;  // typically 0.02s (50 Hz)

// Update orientation
yaw += angularVelocity * dt;

// Update position
x += linearVelocity * cos(yaw) * dt;
y += linearVelocity * sin(yaw) * dt;
```

**Example Integration (at 50 Hz):**
```
Given:
  linearVelocity = 0.200 m/s
  dt = 0.02 s (50 Hz)
  yaw = 0 rad (facing forward)

Calculate:
  Δx = 0.200 * cos(0) * 0.02 = 0.004 m
  Δy = 0.200 * sin(0) * 0.02 = 0.000 m

  After 50 updates (1 second):
    x += 0.004 * 50 = 0.200 m  ✓ (correct!)
```

#### Step 5: Publish Odometry Message
```cpp
// Create Gazebo odometry message
gz::msgs::Odometry odomMsg;
odomMsg.header().set_frame_id("odom");
odomMsg.set_child_frame_id("base_footprint");

// Set position
odomMsg.mutable_pose()->mutable_position()->set_x(x);
odomMsg.mutable_pose()->mutable_position()->set_y(y);
odomMsg.mutable_pose()->mutable_position()->set_z(0);

// Set orientation (convert yaw to quaternion)
odomMsg.mutable_pose()->mutable_orientation()->set_z(sin(yaw/2));
odomMsg.mutable_pose()->mutable_orientation()->set_w(cos(yaw/2));

// Set velocities
odomMsg.mutable_twist()->mutable_linear()->set_x(linearVelocity);
odomMsg.mutable_twist()->mutable_angular()->set_z(angularVelocity);

// Publish to Gazebo topic
publisher.Publish(odomMsg);
```

**Gazebo Topic**: `/model/tb3_1/odometry`

---

## 3. ROS-Gazebo Bridge

### Location
**Package**: `ros_gz_bridge`
**Binary**: `/opt/ros/jazzy/lib/ros_gz_bridge/parameter_bridge`

### Configuration
**File**: `src/multi_robot_mapping/config/tb3_bridge.yaml:7-11`

```yaml
- ros_topic_name: "/tb3_1/odom"
  gz_topic_name: "/model/tb3_1/odometry"
  ros_type_name: "nav_msgs/msg/Odometry"
  gz_type_name: "gz.msgs.Odometry"
  direction: GZ_TO_ROS
```

### What It Does

#### Input (Gazebo Message)
```protobuf
// gz.msgs.Odometry
message Odometry {
  Header header = 1;
  Pose pose = 2;
  Twist twist = 3;
}
```

#### Output (ROS Message)
```cpp
// nav_msgs/msg/Odometry
std_msgs/Header header
  string frame_id = "odom"
  builtin_interfaces/Time stamp
string child_frame_id = "base_footprint"
geometry_msgs/PoseWithCovariance pose
  Pose pose
    Point position (x, y, z)
    Quaternion orientation
  float64[36] covariance
geometry_msgs/TwistWithCovariance twist
  Twist twist
    Vector3 linear (vx, vy, vz)
    Vector3 angular (ωx, ωy, ωz)
  float64[36] covariance
```

**Conversion Process:**
```cpp
// Simplified bridge logic
void ConvertMessage(gz::msgs::Odometry& gzMsg, nav_msgs::msg::Odometry& rosMsg) {
  // Copy position
  rosMsg.pose.pose.position.x = gzMsg.pose().position().x();
  rosMsg.pose.pose.position.y = gzMsg.pose().position().y();
  rosMsg.pose.pose.position.z = gzMsg.pose().position().z();

  // Copy orientation
  rosMsg.pose.pose.orientation.x = gzMsg.pose().orientation().x();
  rosMsg.pose.pose.orientation.y = gzMsg.pose().orientation().y();
  rosMsg.pose.pose.orientation.z = gzMsg.pose().orientation().z();
  rosMsg.pose.pose.orientation.w = gzMsg.pose().orientation().w();

  // Copy velocities
  rosMsg.twist.twist.linear.x = gzMsg.twist().linear().x();
  rosMsg.twist.twist.angular.z = gzMsg.twist().angular().z();

  // Copy frame IDs
  rosMsg.header.frame_id = gzMsg.header().data(0).value()[0];
  rosMsg.child_frame_id = gzMsg.child_frame_id();
}
```

**ROS Topic**: `/tb3_1/odom`

---

## 4. Your ROS Nodes (Subscribers)

### EKF Node
**File**: `src/map_generation/map_generation/local_submap_generator.py:91-97`

```python
# Subscribe to odometry
self.odom_sub = self.create_subscription(
    Odometry,
    f'/{self.robot_name}/odom',  # /tb3_1/odom
    self.odom_callback,
    odom_qos  # RELIABLE QoS
)
```

**Callback** (line 229-260):
```python
def odom_callback(self, msg):
    # Extract position from odometry message
    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y

    # Extract orientation (convert quaternion to yaw)
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    theta_odom = quaternion_to_yaw(qx, qy, qz, qw)

    # Extract velocities
    vx = msg.twist.twist.linear.x
    vy = msg.twist.twist.linear.y
    omega = msg.twist.twist.angular.z

    # Initialize or update EKF
    if not self.ekf_initialized:
        self.ekf.initialize(x_odom, y_odom, theta_odom, vx, 0.0)
        self.ekf_initialized = True
    else:
        self.ekf.update(x_odom, y_odom, theta_odom, vx)
```

---

## Complete Data Flow Example

### Starting Condition
- Robot at origin: `(x=0, y=0, yaw=0)`
- Command: `linear.x = 0.2 m/s, angular.z = 0 rad/s`

### T = 0.000s - Command Received
```
Navigation Node publishes:
  Topic: /tb3_1/cmd_vel
  Message: Twist(linear.x=0.2, angular.z=0)
```

### T = 0.001s - Bridge Forwards Command
```
Bridge converts and publishes:
  Topic: /model/tb3_1/cmd_vel (Gazebo)
  Message: gz.msgs.Twist(linear.x=0.2, angular.z=0)
```

### T = 0.002s - DiffDrive Receives Command
```
DiffDrive plugin calculates required wheel velocities:
  targetLinear = 0.2 m/s
  targetAngular = 0 rad/s

  # Inverse kinematics
  leftWheelVel = (0.2 - 0*0.287/2) / 0.033 = 6.06 rad/s
  rightWheelVel = (0.2 + 0*0.287/2) / 0.033 = 6.06 rad/s
```

### T = 0.003s - Physics Applies Torque
```
Physics engine applies torque to wheel joints:
  leftJoint.SetVelocity(6.06 rad/s)
  rightJoint.SetVelocity(6.06 rad/s)
```

### T = 0.020s - First Odometry Update (50 Hz)
```
DiffDrive reads actual wheel velocities:
  leftVel = 6.06 rad/s (from physics)
  rightVel = 6.06 rad/s

DiffDrive calculates odometry:
  linearVel = (6.06*0.033 + 6.06*0.033) / 2 = 0.200 m/s ✓
  angularVel = (6.06*0.033 - 6.06*0.033) / 0.287 = 0 rad/s ✓

  dt = 0.02s
  yaw = 0 + 0*0.02 = 0 rad
  x = 0 + 0.200*cos(0)*0.02 = 0.004 m
  y = 0 + 0.200*sin(0)*0.02 = 0.000 m

DiffDrive publishes to Gazebo:
  Topic: /model/tb3_1/odometry
  Position: (0.004, 0.000, 0)
  Velocity: (0.200, 0.000, 0)
```

### T = 0.021s - Bridge Converts
```
Bridge receives Gazebo message and converts:
  Input: gz.msgs.Odometry
  Output: nav_msgs/msg/Odometry

Bridge publishes to ROS:
  Topic: /tb3_1/odom
  Position: (0.004, 0.000, 0)
  Velocity: (0.200, 0.000, 0)
```

### T = 0.022s - EKF Receives
```
EKF node callback executes:
  x_odom = 0.004 m
  y_odom = 0.000 m
  theta_odom = 0 rad
  vx = 0.200 m/s

EKF updates state and publishes TF:
  Topic: /tf
  Transform: tb3_1/odom → tb3_1/base_footprint
  Position: (0.004, 0.000, 0)
```

### T = 1.000s - After 1 Second
```
50 odometry updates later:
  Expected position: x = 0.200 m (0.004 * 50)

Gazebo reports: (0.200, 0.000, 0)  ✓ CORRECT
```

---

## Where The Odometry Error Occurs

Based on your diagnostics showing **~7x error**, the problem is likely in **one of these places**:

### Hypothesis 1: Physics Simulation Issue
**Location**: Gazebo Physics Engine
**Symptom**: Wheels rotate slower than commanded
**Check**:
```bash
ros2 topic echo /tb3_1/joint_states
# Compare reported wheel velocities to expected 6.06 rad/s
```

### Hypothesis 2: DiffDrive Calculation Error ⚠️ MOST LIKELY
**Location**: DiffDrive Plugin Odometry Integration
**Symptom**: Odometry accumulates error over time
**Evidence**: Your logs show odometry = 4.66m, actual = 0.7m

**Possible causes:**
- Integration timestep mismatch (`dt` calculation)
- Double-counting wheel rotations
- Incorrect `wheel_radius` or `wheel_separation` parameters

### Hypothesis 3: Bridge Conversion Error
**Location**: ros_gz_bridge
**Symptom**: Message values corrupted during conversion
**Check**:
```bash
# Compare Gazebo vs ROS odometry
gz topic -e -t /model/tb3_1/odometry &
ros2 topic echo /tb3_1/odom
```

---

## Debugging Commands

### 1. Check Physics Wheel Velocities
```bash
# See actual wheel joint states
ros2 topic echo /tb3_1/joint_states

# Expected output (at 0.2 m/s):
# velocity: [6.06, 6.06]  # left, right wheels in rad/s
```

### 2. Compare Gazebo vs ROS Odometry
```bash
# Terminal 1: Gazebo odometry
gz topic -e -t /model/tb3_1/odometry | grep "position"

# Terminal 2: ROS odometry
ros2 topic echo /tb3_1/odom | grep "position" -A 3
```

### 3. Monitor Odometry Frequency
```bash
# Should be 50 Hz
ros2 topic hz /tb3_1/odom
```

### 4. Check DiffDrive Parameters
```bash
# View spawned robot SDF
cat /tmp/tb3_1_spawned.sdf | grep -A 10 "wheel_radius"
```

---

## Key Parameters Affecting Odometry

| Parameter | Value | Location | Impact |
|-----------|-------|----------|--------|
| `wheel_radius` | 0.033 m | model.sdf:488 | Linear velocity = ω × r |
| `wheel_separation` | 0.287 m | model.sdf:487 | Angular velocity = Δv / L |
| `odom_publisher_frequency` | 50 Hz | model.sdf:502 | Update rate (dt = 0.02s) |
| `max_step_size` | 0.001s | maze.sdf | Physics timestep |
| `noise` | 0.015 | model.sdf:514 | Encoder error (1.5%) |

---

## Source Code References

If you want to inspect the actual DiffDrive implementation:

```bash
# Clone Gazebo source
git clone https://github.com/gazebosim/gz-sim.git -b gz-sim8
cd gz-sim

# DiffDrive plugin code
# File: src/systems/diff_drive/DiffDrive.cc
# Lines ~400-600: Odometry calculation
# Lines ~700-800: Odometry publishing
```

**Key functions:**
- `UpdateVelocity()`: Reads wheel velocities
- `UpdateOdometry()`: Calculates position/orientation
- `PublishOdometry()`: Publishes to Gazebo topic

---

**End of Document**
