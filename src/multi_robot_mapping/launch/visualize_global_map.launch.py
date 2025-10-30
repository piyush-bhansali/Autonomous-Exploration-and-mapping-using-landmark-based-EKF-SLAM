#!/usr/bin/env python3
"""
Global Map Visualization Launch File

Launches ONLY RViz2 with global map visualization - no scans or submaps.
Use this to cleanly visualize how the stitched global map is evolving.

Usage:
    # Launch the main system in one terminal:
    ros2 launch multi_robot_mapping full_system.launch.py use_rviz:=false

    # Launch this visualization in another terminal:
    ros2 launch multi_robot_mapping visualize_global_map.launch.py

    # Or for test mapping:
    ros2 launch multi_robot_mapping test_mapping.launch.py use_rviz:=false pattern:=long_corridor
    ros2 launch multi_robot_mapping visualize_global_map.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for global map visualization only"""

    # Package directory
    pkg_multi_robot = get_package_share_directory('multi_robot_mapping')

    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='tb3_1',
        description='Robot namespace for topics'
    )

    # RViz configuration file
    rviz_config_file = os.path.join(
        pkg_multi_robot,
        'rviz',
        'global_map_only.rviz'
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_global_map',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        rviz_node,
    ])
