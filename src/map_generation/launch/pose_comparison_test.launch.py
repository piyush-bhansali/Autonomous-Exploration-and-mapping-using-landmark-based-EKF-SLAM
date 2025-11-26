#!/usr/bin/env python3
"""
Launch file for pose comparison test.

This test node subscribes to:
- Ground truth from Gazebo (/tf)
- Raw odometry (/tb3_1/odom)
- EKF estimate (/tb3_1/ekf_pose)

And prints detailed comparison every second.

Usage:
    ros2 launch map_generation pose_comparison_test.launch.py

    # Or specify robot name:
    ros2 launch map_generation pose_comparison_test.launch.py robot_name:=tb3_2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for pose comparison test"""

    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='tb3_1',
        description='Robot name (e.g., tb3_1, tb3_2)'
    )

    robot_name = LaunchConfiguration('robot_name')

    # Pose comparison test node
    test_node = Node(
        package='map_generation',
        executable='pose_comparison_test',
        name='pose_comparison_test',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        test_node,
    ])
