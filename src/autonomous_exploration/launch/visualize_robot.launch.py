#!/usr/bin/env python3
"""
Launch RViz for a specific robot

Usage:
    ros2 launch autonomous_exploration visualize_robot.launch.py robot_name:=tb3_1
    ros2 launch autonomous_exploration visualize_robot.launch.py robot_name:=tb3_2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for per-robot RViz visualization"""

    pkg_multi_robot = get_package_share_directory('autonomous_exploration')

    # Launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='tb3_1',
        description='Robot name (tb3_1, tb3_2, etc.)'
    )

    robot_name = LaunchConfiguration('robot_name')

    # RViz config path (per-robot)
    rviz_config = PathJoinSubstitution([
        pkg_multi_robot,
        'rviz',
        [robot_name, '_visualization.rviz']
    ])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name=['rviz2_', robot_name],
        namespace=robot_name,
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_name_arg,
        rviz_node
    ])
