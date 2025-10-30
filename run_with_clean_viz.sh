#!/bin/bash
# Run mapping system with clean global map visualization
# This shows ONLY the stitched global map in real-time, no scans or submaps

# Get pattern argument (default: long_corridor)
PATTERN="${1:-long_corridor}"

echo "========================================"
echo "STARTING MAPPING WITH CLEAN VISUALIZATION"
echo "========================================"
echo ""
echo "Pattern: $PATTERN"
echo ""
echo "This will launch:"
echo "  1. Gazebo + Robot + Mapping (no default RViz)"
echo "  2. Clean RViz (global map only)"
echo ""
echo "Press Ctrl+C to stop"
echo ""

cd ~/thesis_ws
source install/setup.bash

# Launch main system in background
ros2 launch multi_robot_mapping test_mapping.launch.py use_rviz:=false pattern:=$PATTERN &
MAIN_PID=$!

# Wait for system to initialize
echo "Waiting for system to initialize..."
sleep 8

# Launch clean visualization
echo ""
echo "Launching clean global map visualization..."
ros2 launch multi_robot_mapping visualize_global_map.launch.py

# Cleanup on exit
kill $MAIN_PID 2>/dev/null
