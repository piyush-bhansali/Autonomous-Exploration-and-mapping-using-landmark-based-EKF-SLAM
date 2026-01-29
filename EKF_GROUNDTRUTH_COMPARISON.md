# EKF vs Ground Truth Comparison Guide

This system compares the EKF localization estimates with Gazebo ground truth to evaluate localization accuracy.

## Setup Complete

The following components have been added:

### 1. **Bridge Configuration** (`src/multi_robot_mapping/config/tb3_bridge.yaml:50-54`)
- Added ground truth pose topic bridge from Gazebo to ROS2
- Topic: `/{robot_name}/ground_truth_pose`

### 2. **Data Logger** (`src/map_generation/map_generation/local_submap_generator.py`)
- Subscribes to ground truth pose from Gazebo
- Logs EKF and ground truth data to CSV file
- Automatically calculates position and orientation errors
- Data saved to: `./submaps/{robot_name}/ekf_vs_groundtruth.csv`

### 3. **MATLAB Plotting Script** (`plot_ekf_vs_groundtruth.m`)
- Comprehensive visualization of EKF vs ground truth
- Error statistics (RMSE, mean, max, std)
- Multiple plot types for detailed analysis

## Usage

### Step 1: Rebuild the Package
```bash
cd ~/thesis_ws
colcon build --packages-select multi_robot_mapping map_generation
source install/setup.bash
```

### Step 2: Run the Simulation
```bash
ros2 launch multi_robot_mapping full_system.launch.py
```

The system will automatically:
- Subscribe to Gazebo ground truth pose
- Log EKF and ground truth data to CSV
- Calculate errors in real-time

### Step 3: Run Your Robot
Let the robot explore for a while to collect trajectory data.

### Step 4: Stop and Analyze
Press `Ctrl+C` to stop the simulation. The CSV file will be saved automatically.

### Step 5: Plot in MATLAB
```matlab
cd ~/thesis_ws
plot_ekf_vs_groundtruth
```

## CSV Data Format

The CSV file contains the following columns:
- `timestamp`: Time in seconds
- `ekf_x`, `ekf_y`, `ekf_theta`: EKF estimated pose
- `gt_x`, `gt_y`, `gt_theta`: Ground truth pose from Gazebo
- `pos_error`: Euclidean position error (meters)
- `orient_error`: Orientation error (radians)

## MATLAB Output

The script generates 3 figures:

### Figure 1: Overview
- 2D trajectory comparison
- Position error over time
- Orientation error over time
- X/Y components comparison

### Figure 2: Detailed Error Analysis
- X, Y, Theta individual errors
- Error distributions (histograms)
- Position vs orientation error scatter plot

### Figure 3: Trajectory Details (if enough data)
- Zoomed views of start, middle, and end of trajectory

### Console Statistics
The script prints:
- Position RMSE, mean, max, std
- Orientation RMSE, mean, max, std
- Number of data points and duration

## File Locations

- CSV data: `./submaps/tb3_1/ekf_vs_groundtruth.csv`
- MATLAB script: `./plot_ekf_vs_groundtruth.m`
- Bridge config: `./src/multi_robot_mapping/config/tb3_bridge.yaml`
- Data logger: `./src/map_generation/map_generation/local_submap_generator.py`

## Notes

- The CSV file is created when the node starts and closed on shutdown
- Data is flushed immediately after each write (no buffering)
- Ground truth comes directly from Gazebo's physics engine (perfect accuracy)
- EKF uses sensor fusion (odometry + IMU + ICP corrections)

## Interpreting Results

**Good EKF performance:**
- Position RMSE < 0.05 m (5 cm)
- Orientation RMSE < 0.05 rad (~3 degrees)
- Errors stay bounded over time (no drift)

**Signs of issues:**
- Increasing error over time = drift
- Large spikes = missed ICP corrections or bad odometry
- Systematic bias = sensor calibration issues
