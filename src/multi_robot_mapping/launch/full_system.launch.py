#!/usr/bin/env python3
"""
Complete Multi-Robot SLAM and Navigation System

Launches:
1. Gazebo simulation with world
2. TurtleBot3 robots
3. Local submap generation (mapping)
4. Frontier exploration (navigation)
5. RViz visualization

Usage:
    ros2 launch multi_robot_mapping full_system.launch.py
    ros2 launch multi_robot_mapping full_system.launch.py num_robots:=2 world:=maze
"""

from launch import LaunchDescription
import launch.conditions
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate complete system launch description"""

    # Package directories
    pkg_multi_robot = get_package_share_directory('multi_robot_mapping')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze',
        description='World file (maze or park)'
    )

    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Number of robots to spawn'
    )

    enable_navigation_arg = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Enable autonomous exploration'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    # Configuration
    world_name = LaunchConfiguration('world')
    num_robots = LaunchConfiguration('num_robots')
    enable_navigation = LaunchConfiguration('enable_navigation')
    use_rviz = LaunchConfiguration('use_rviz')

    # File paths
    world_file = PathJoinSubstitution([
        pkg_multi_robot,
        'worlds',
        [world_name, '.sdf']
    ])

    robot_sdf = os.path.join(pkg_multi_robot, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    urdf_file = os.path.join(pkg_multi_robot, 'urdf', 'turtlebot3_waffle_pi.urdf')
    bridge_common_yaml = os.path.join(pkg_multi_robot, 'config', 'tb3_bridge_common.yaml')
    bridge_robot_yaml = os.path.join(pkg_multi_robot, 'config', 'tb3_bridge.yaml')
    rviz_config = os.path.join(pkg_multi_robot, 'rviz', 'mapping_visualization.rviz')

    # World name mapping for Gazebo (default to maze_world)
    world_name_map = {
        'maze': 'maze_world',
        'park': 'park_world'
    }

    # Read robot SDF
    with open(robot_sdf, 'r') as f:
        robot_sdf_content = f.read()

    # Fix mesh paths
    mesh_path = os.path.join(pkg_multi_robot, 'meshes')
    robot_sdf_content = robot_sdf_content.replace(
        'package://multi_robot_mapping/meshes',
        f'file://{mesh_path}'
    )

    # ========================================================================
    # 1. GAZEBO SIMULATION
    # ========================================================================
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r -v 4'],
            'on_exit_shutdown': 'true'
        }.items()
    )

    # ========================================================================
    # 2. CLOCK BRIDGE
    # ========================================================================
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{'config_file': bridge_common_yaml}]
    )

    clock_bridge_delayed = TimerAction(period=1.0, actions=[clock_bridge])

    # ========================================================================
    # 3. ROBOT SPAWN AND CONTROL (tb3_1)
    # ========================================================================
    robot_name = 'tb3_1'

    # Get Gazebo world name (from world parameter, default to maze)
    # This needs to be resolved at runtime, so we'll use a default
    # In production, extract from world parameter properly
    gazebo_world_name = 'maze_world'  # Will be dynamically set based on world arg

    # Spawn robot at corner position
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{robot_name}',
        output='screen',
        arguments=[
            '-world', gazebo_world_name,
            '-name', robot_name,
            '-string', robot_sdf_content,
            '-x', '-12.0',
            '-y', '-12.0',
            '-z', '0.01',  # Minimal height to avoid ground penetration
            '-Y', '0.0'
        ]
    )

    # Robot bridge (ROS-Gazebo communication)
    with open(bridge_robot_yaml, 'r') as f:
        template_content = f.read()

    config_content = template_content.replace('{robot_name}', robot_name)
    config_content = config_content.replace('{world_name}', gazebo_world_name)

    temp_config = f'/tmp/{robot_name}_bridge.yaml'
    with open(temp_config, 'w') as f:
        f.write(config_content)

    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_bridge',
        output='screen',
        parameters=[{'config_file': temp_config}]
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{robot_name}_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file, 'r').read(),
            'frame_prefix': f'{robot_name}/'
        }],
        remappings=[('/joint_states', f'/{robot_name}/joint_states')]
    )

    robot_spawn_delayed = TimerAction(
        period=4.0,
        actions=[spawn_robot, robot_bridge, robot_state_publisher]
    )

    # ========================================================================
    # 4. MAPPING: LOCAL SUBMAP GENERATOR
    # ========================================================================
    submap_generator = Node(
        package='map_generation',
        executable='local_submap_generator',
        name=f'{robot_name}_submap_generator',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'scans_per_submap': 50,  # 50 scans @ 8.5Hz = ~6s = ~1.2m at 0.2 m/s (OPTIMIZED for real-time!)
            'min_distance_between_submaps': 1.0,  # 1.0m for faster updates
            'save_directory': './submaps',
            'voxel_size': 0.08,  # 8cm voxel - coarser but faster processing
            'feature_method': 'hybrid',
            'enable_loop_closure': True,  # ENABLED for better mapping
            'enable_scan_to_map_icp': True  # Enable real-time scan matching
        }]
    )

    submap_generator_delayed = TimerAction(period=7.0, actions=[submap_generator])

    # ========================================================================
    # 5. NAVIGATION: FRONTIER EXPLORATION
    # ========================================================================
    navigation_node = Node(
        package='navigation',
        executable='simple_navigation',
        name=f'{robot_name}_navigation',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'robot_radius': 0.22
        }],
        condition=launch.conditions.IfCondition(enable_navigation)
    )

    navigation_delayed = TimerAction(period=15.0, actions=[navigation_node])

    # ========================================================================
    # 6. RVIZ VISUALIZATION
    # ========================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=launch.conditions.IfCondition(use_rviz)
    )

    rviz_delayed = TimerAction(period=8.0, actions=[rviz_node])

    # ========================================================================
    # LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Arguments
        world_arg,
        num_robots_arg,
        enable_navigation_arg,
        use_rviz_arg,

        # System startup
        LogInfo(msg='========================================'),
        LogInfo(msg='MULTI-ROBOT SLAM & NAVIGATION SYSTEM'),
        LogInfo(msg='========================================'),
        LogInfo(msg=['World: ', world_name]),
        LogInfo(msg=['Robots: ', num_robots]),
        LogInfo(msg='========================================'),

        # Components (timed sequence)
        gazebo_server,
        clock_bridge_delayed,
        robot_spawn_delayed,
        submap_generator_delayed,
        rviz_delayed,
        navigation_delayed,

        LogInfo(msg='✓ System launching...'),
    ])
