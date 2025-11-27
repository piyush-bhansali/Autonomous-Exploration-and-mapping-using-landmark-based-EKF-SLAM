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
    LogInfo,
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
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
    # Per-robot RViz configs (supports multi-robot independent visualization)
    rviz_config_tb3_1 = os.path.join(pkg_multi_robot, 'rviz', 'tb3_1_visualization.rviz')
    rviz_config_tb3_2 = os.path.join(pkg_multi_robot, 'rviz', 'tb3_2_visualization.rviz')

    # World name mapping for Gazebo (default to maze_world)
    world_name_map = {
        'maze': 'maze_world',
        'park': 'park_world'
    }

    # Read robot SDF (will be substituted per-robot below)
    mesh_path = os.path.join(pkg_multi_robot, 'meshes')

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

    # Map world name to Gazebo world name
    # For Python string operations, we use the map. For launch substitutions, we build dynamically.
    world_name_map = {
        'maze': 'maze_world',
        'park': 'park_world'
    }
    # Default to maze_world for Python operations (bridge config)
    gazebo_world_name_str = 'maze_world'
    # For spawn command, use dynamic substitution
    gazebo_world_name_dynamic = [world_name, '_world']

    # Prepare robot SDF with substitutions
    with open(robot_sdf, 'r') as f:
        robot_sdf_content = f.read()

    # Substitute mesh paths and robot name
    robot_sdf_content = robot_sdf_content.replace('package://multi_robot_mapping/meshes', f'file://{mesh_path}')
    robot_sdf_content = robot_sdf_content.replace('{robot_name}', robot_name)

    # Replace gz_frame_id for sensors to add namespace (for proper TF tree)
    robot_sdf_content = robot_sdf_content.replace(
        '<gz_frame_id>base_scan</gz_frame_id>',
        f'<gz_frame_id>{robot_name}/base_scan</gz_frame_id>'
    )

    # Fix DiffDrive plugin frame_id to include robot namespace
    # This ensures odometry message uses namespaced frame
    robot_sdf_content = robot_sdf_content.replace(
        '<frame_id>odom</frame_id>',
        f'<frame_id>{robot_name}/odom</frame_id>'
    )

    # Fix DiffDrive plugin child_frame_id to include robot namespace
    # This prevents odometry from accumulating in wrong coordinate frame
    robot_sdf_content = robot_sdf_content.replace(
        '<child_frame_id>base_footprint</child_frame_id>',
        f'<child_frame_id>{robot_name}/base_footprint</child_frame_id>'
    )

    # Debug: Save modified SDF for inspection
    debug_sdf_path = f'/tmp/{robot_name}_spawned.sdf'
    with open(debug_sdf_path, 'w') as f:
        f.write(robot_sdf_content)

    # Spawn robot at corner position
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{robot_name}',
        output='screen',
        arguments=[
            '-world', gazebo_world_name_dynamic,
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
    config_content = config_content.replace('{world_name}', gazebo_world_name_str)

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
    # Read URDF and substitute ${namespace} with robot name
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    urdf_content = urdf_content.replace('${namespace}', f'{robot_name}/')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{robot_name}_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'frame_prefix': ''  # Empty since namespace already in URDF
        }],
        remappings=[('/joint_states', f'/{robot_name}/joint_states')]
    )

    # Spawn robot FIRST - EKF needs sensor data to initialize!
    # Robot must be spawned and publishing sensor data before EKF can start
    robot_spawn_delayed = TimerAction(
        period=2.0,  # Spawn robot early so sensor data is available for EKF
        actions=[spawn_robot, robot_state_publisher, robot_bridge]
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
            'use_sim_time': True,  # CRITICAL: Use Gazebo simulation time for TF timestamps
            'robot_name': robot_name,
            'scans_per_submap': 50,  # 50 scans @ 8.5Hz = ~6s = ~1.2m at 0.2 m/s (OPTIMIZED for real-time!)
            'save_directory': './submaps',
            'voxel_size': 0.08,  # 8cm voxel - coarser but faster processing
            'feature_method': 'hybrid',
            'enable_loop_closure': True,  # ENABLED for better mapping
            'enable_scan_to_map_icp': True  # Enable real-time scan matching
        }]
    )

    # Start EKF AFTER robot has spawned and is publishing sensor data
    submap_generator_delayed = TimerAction(period=4.0, actions=[submap_generator])

    # ========================================================================
    # 5. NAVIGATION: FRONTIER EXPLORATION
    # ========================================================================
    navigation_node = Node(
        package='navigation',
        executable='simple_navigation',
        name=f'{robot_name}_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,  # Use Gazebo simulation time
            'robot_name': robot_name,
            'robot_radius': 0.22
        }],
        condition=launch.conditions.IfCondition(enable_navigation)
    )

    # Start navigation AFTER EKF has initialized and published initial map
    navigation_delayed = TimerAction(period=8.0, actions=[navigation_node])

    # ========================================================================
    # 6. RVIZ VISUALIZATION (Per-Robot)
    # ========================================================================
    # Launch RViz for tb3_1 with its own map frame
    rviz_node_tb3_1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_tb3_1',
        namespace='tb3_1',
        arguments=['-d', rviz_config_tb3_1],
        output='screen',
        parameters=[{'use_sim_time': True}],  # Use Gazebo simulation time
        condition=launch.conditions.IfCondition(use_rviz)
    )

    # RVIZ starts after both robot and EKF are running
    # Starts at t=6s to ensure TF tree is fully established:
    #   t=2s: Robot spawns, robot_state_publisher starts
    #   t=4s: EKF starts, begins publishing odom->base_footprint TF
    #   t=6s: RViz starts with full TF tree available
    rviz_delayed = TimerAction(period=6.0, actions=[rviz_node_tb3_1])

    # NOTE: For tb3_2, launch separately with:
    # ros2 run rviz2 rviz2 -d /path/to/tb3_2_visualization.rviz

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
        # t=0s:   Gazebo simulation starts
        # t=1s:   Clock bridge starts
        # t=2s:   Robot spawns (sensor data now available)
        # t=4s:   EKF/mapping starts (can now consume sensor data)
        # t=6s:   RViz starts (TF tree fully established)
        # t=8s:   Navigation starts (map available)
        gazebo_server,
        clock_bridge_delayed,          # t=1s
        robot_spawn_delayed,           # t=2s - Robot FIRST
        submap_generator_delayed,      # t=4s - EKF after robot
        rviz_delayed,                  # t=6s
        navigation_delayed,            # t=8s

        LogInfo(msg='✓ System launching...'),
    ])
