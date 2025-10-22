#!/usr/bin/env python3
"""
Complete test launch file for mapping system evaluation.

This launch file starts:
1. Gazebo simulation with maze world
2. Single robot (tb3_1)
3. Local submap generator with visualization
4. Robot controller for automated test patterns

Usage:
    ros2 launch multi_robot_mapping test_mapping.launch.py pattern:=square
    ros2 launch multi_robot_mapping test_mapping.launch.py pattern:=long_corridor
    ros2 launch multi_robot_mapping test_mapping.launch.py pattern:=figure_eight
    ros2 launch multi_robot_mapping test_mapping.launch.py pattern:=spiral
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessStart, OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for mapping test"""

    # Package directories
    pkg_multi_robot = get_package_share_directory('multi_robot_mapping')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    pattern_arg = DeclareLaunchArgument(
        'pattern',
        default_value='square',
        description='Movement pattern: square, long_corridor, figure_eight, spiral'
    )

    use_ekf_arg = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Use EKF for pose estimation'
    )

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='false',
        description='Enable Open3D visualization (may fail on Wayland)'
    )

    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Use GPU acceleration'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze',
        description='World file to load (maze or park)'
    )

    # Robot configuration
    robot_name = 'tb3_1'
    world_name = LaunchConfiguration('world')
    pattern = LaunchConfiguration('pattern')

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

    # Read and prepare robot SDF
    with open(robot_sdf, 'r') as f:
        robot_sdf_content = f.read()

    # Replace package:// URIs with file:// URIs for Gazebo
    mesh_path = os.path.join(pkg_multi_robot, 'meshes')
    robot_sdf_content = robot_sdf_content.replace(
        'package://multi_robot_mapping/meshes',
        f'file://{mesh_path}'
    )

    # ============================================================================
    # 1. GAZEBO SIMULATION
    # ============================================================================

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                pkg_ros_gz_sim,
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': [world_file, ' -r']  # -r: run on start (WITH GUI)
        }.items()
    )

    # ============================================================================
    # 2. CLOCK BRIDGE (after 1 second)
    # ============================================================================

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'config_file': bridge_common_yaml}]
    )

    clock_bridge_delayed = TimerAction(
        period=1.0,
        actions=[clock_bridge]
    )

    # ============================================================================
    # 3. SPAWN ROBOT (after 4 seconds)
    # ============================================================================

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{robot_name}',
        output='screen',
        arguments=[
            '-world', 'maze_world',
            '-name', robot_name,
            '-string', robot_sdf_content,
            '-x', '-23.0',
            '-y', '-23.0',
            '-z', '0.02',
            '-Y', '0.0'
        ]
    )

    # Robot-specific bridge - create temp config with placeholders replaced
    with open(bridge_robot_yaml, 'r') as f:
        template_content = f.read()

    # Replace placeholders with actual robot name and world name
    config_content = template_content.replace('{robot_name}', robot_name)
    config_content = config_content.replace('{world_name}', 'maze')

    # Write to temporary config file
    temp_config = f'/tmp/{robot_name}_test_bridge.yaml'
    with open(temp_config, 'w') as f:
        f.write(config_content)

    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{robot_name}_bridge',
        output='screen',
        parameters=[{
            'config_file': temp_config
        }]
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
        remappings=[
            ('/joint_states', f'/{robot_name}/joint_states'),
        ]
    )

    robot_spawn_delayed = TimerAction(
        period=4.0,
        actions=[spawn_robot, robot_bridge, robot_state_publisher]
    )

    # ============================================================================
    # 4. LOCAL SUBMAP GENERATOR (after 7 seconds)
    # ============================================================================

    submap_generator = Node(
        package='map_generation',
        executable='local_submap_generator',
        name=f'{robot_name}_submap_generator',
        output='screen',
        parameters=[{
            'robot_name': robot_name,
            'use_ekf': LaunchConfiguration('use_ekf'),
            'distance_threshold': 2.0,
            'angle_threshold': 0.5,
            'min_scans_per_submap': 50,
            'save_directory': './test_results/submaps',
            'voxel_size': 0.05,
            'visualize_open3d': LaunchConfiguration('visualize'),
            'use_gpu': LaunchConfiguration('use_gpu'),
            'gpu_device_id': 0
        }]
    )

    submap_generator_delayed = TimerAction(
        period=7.0,
        actions=[submap_generator]
    )

    # ============================================================================
    # 5. TEST ROBOT CONTROLLER (after 10 seconds)
    # ============================================================================

    # Wait for everything to start, then begin test
    test_controller = ExecuteProcess(
        cmd=[
            'ros2', 'run',
            'map_generation',
            'test_robot_controller',
            robot_name,
            pattern
        ],
        output='screen',
        shell=False
    )

    test_controller_delayed = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg='🚀 Starting automated test pattern...'),
            test_controller
        ]
    )

    # ============================================================================
    # LAUNCH DESCRIPTION
    # ============================================================================

    return LaunchDescription([
        # Arguments
        pattern_arg,
        use_ekf_arg,
        visualize_arg,
        use_gpu_arg,
        world_arg,

        # Nodes (with timing)
        gazebo_server,
        clock_bridge_delayed,
        robot_spawn_delayed,
        submap_generator_delayed,
        test_controller_delayed,

        # Info
        LogInfo(msg='========================================='),
        LogInfo(msg='MAPPING TEST LAUNCH'),
        LogInfo(msg='========================================='),
        LogInfo(msg=['Pattern: ', pattern]),
        LogInfo(msg=['World: ', world_name]),
        LogInfo(msg='========================================='),
    ])
