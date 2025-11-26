#!/bin/bash
# Verification script for TF tree after fixing Gazebo TF conflict
# Run this after launching your system to verify the fix worked

echo "================================================"
echo "TF Tree Verification Script"
echo "================================================"
echo ""
echo "Make sure your system is running first!"
echo "  ros2 launch multi_robot_mapping full_system.launch.py"
echo ""
echo "Press Enter to continue..."
read

echo ""
echo "1. Checking if local_submap_generator node is running..."
if ros2 node list | grep -q "tb3_1_submap_generator"; then
    echo "   ✅ tb3_1_submap_generator is running"
else
    echo "   ❌ ERROR: tb3_1_submap_generator not found!"
    echo "   Make sure the system is fully launched (wait ~10 seconds after launch)"
fi

echo ""
echo "2. Checking TF publishers..."
echo "   Publishers on /tf topic:"
ros2 topic info /tf --verbose | grep "Publisher count:" -A 10
echo ""

echo ""
echo "3. Testing TF transform: odom -> tb3_1/base_footprint"
timeout 2 ros2 run tf2_ros tf2_echo odom tb3_1/base_footprint 2>&1 | head -15
if [ $? -eq 0 ]; then
    echo "   ✅ Transform available"
else
    echo "   ❌ Transform not available - check if EKF node is publishing"
fi

echo ""
echo "4. Testing TF transform: odom -> tb3_1/base_scan (for navigation)"
timeout 2 ros2 run tf2_ros tf2_echo odom tb3_1/base_scan 2>&1 | head -15
if [ $? -eq 0 ]; then
    echo "   ✅ Transform available"
else
    echo "   ❌ Transform not available - check TF chain"
fi

echo ""
echo "5. Checking for orphaned 'base_footprint' frame (should NOT exist)..."
timeout 2 ros2 run tf2_ros tf2_echo odom base_footprint 2>&1 | head -5
if echo "$?" | grep -q "124"; then
    echo "   ✅ No orphaned base_footprint frame (good!)"
else
    echo "   ⚠️  base_footprint frame exists - TF bridge might still be active"
fi

echo ""
echo "6. Generating complete TF tree visualization..."
echo "   Running: ros2 run tf2_tools view_frames"
echo "   This will create frames.pdf in current directory..."
cd /tmp
timeout 10 ros2 run tf2_tools view_frames 2>&1

if [ -f "/tmp/frames.pdf" ]; then
    echo "   ✅ frames.pdf created"
    echo ""
    echo "   Opening frames.pdf..."
    if command -v evince &> /dev/null; then
        evince /tmp/frames.pdf &
    elif command -v xdg-open &> /dev/null; then
        xdg-open /tmp/frames.pdf &
    else
        echo "   View the file manually: /tmp/frames.pdf"
    fi
    echo ""
    echo "   WHAT TO LOOK FOR IN frames.pdf:"
    echo "   ✅ Single tree: odom -> tb3_1/base_footprint -> tb3_1/base_link -> ..."
    echo "   ✅ Authority for odom->tb3_1/base_footprint: local_submap_generator"
    echo "   ✅ Authority for robot links: robot_state_publisher"
    echo "   ❌ NO orphaned 'base_footprint' (without tb3_1/ prefix)"
    echo "   ❌ NO multiple authorities for same transform"
else
    echo "   ❌ Failed to create frames.pdf"
fi

echo ""
echo "7. Checking topics..."
echo "   Odometry topic:"
ros2 topic info /tb3_1/odom | head -5

echo ""
echo "   Current submap topic:"
ros2 topic info /tb3_1/current_submap | head -5

echo ""
echo "   Global map topic:"
ros2 topic info /tb3_1/global_map | head -5

echo ""
echo "================================================"
echo "Verification Complete!"
echo "================================================"
echo ""
echo "EXPECTED RESULTS:"
echo "  ✅ odom -> tb3_1/base_footprint transform exists"
echo "  ✅ odom -> tb3_1/base_scan transform exists"
echo "  ✅ NO orphaned base_footprint frame"
echo "  ✅ Single authority per transform"
echo "  ✅ Clean TF tree in frames.pdf"
echo ""
echo "If all checks pass, your multi-robot namespacing is working correctly!"
echo ""
