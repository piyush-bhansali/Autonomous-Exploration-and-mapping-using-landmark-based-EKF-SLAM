#!/usr/bin/env python3

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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    def reject_unknown_launch_arguments(context):
        allowed_args = {'world', 'enable_navigation', 'use_rviz'}
        unknown_args = sorted(
            key for key in context.launch_configurations.keys()
            if key not in allowed_args
        )

        if unknown_args:
            unknown_str = ', '.join(unknown_args)
            raise RuntimeError(
                f"Unsupported launch argument(s): {unknown_str}. "
                "This launch file supports only: world, enable_navigation, use_rviz."
            )
        return []

    # ========================================================================
    # CONFIGURATION
    # ========================================================================
    ROBOT_NAME = 'tb3_1'
    SPAWN_X = -12.0
    SPAWN_Y = -12.0
    SPAWN_Z = 0.01
    SPAWN_YAW = 0.0

    # Timer delays (in seconds)
    SPAWN_DELAY = 2.0
    MAPPING_DELAY = 4.0
    RVIZ_DELAY = 6.0
    NAVIGATION_DELAY = 8.0

    # Package directories
    pkg_exploration = get_package_share_directory('autonomous_exploration')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='maze',
        description='World file (maze or park)'
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
    enable_navigation = LaunchConfiguration('enable_navigation')
    use_rviz = LaunchConfiguration('use_rviz')

    # File paths
    world_file = PathJoinSubstitution([
        pkg_exploration,
        'worlds',
        [world_name, '.sdf']
    ])

    robot_sdf = os.path.join(pkg_exploration, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    urdf_file = os.path.join(pkg_exploration, 'urdf', 'turtlebot3_waffle_pi.urdf')
    bridge_common_yaml = os.path.join(pkg_exploration, 'config', 'tb3_bridge_common.yaml')
    bridge_robot_yaml = os.path.join(pkg_exploration, 'config', 'tb3_bridge.yaml')
    mesh_path = os.path.join(pkg_exploration, 'meshes')
    rviz_config = os.path.join(pkg_exploration, 'rviz', f'{ROBOT_NAME}_visualization.rviz')

    # World name configuration for Gazebo
    gazebo_world_name_str = 'maze_world'  # Default to maze_world for bridge config
    gazebo_world_name_dynamic = [world_name, '_world']  # For spawn command substitution

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
            'gz_args': [world_file, ' -r -v 2'],  # v2=WARN, v3=INFO, v4=DEBUG
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
    # 3. ROBOT COMPONENTS
    # ========================================================================

    # Prepare robot SDF with substitutions
    with open(robot_sdf, 'r') as f:
        robot_sdf_content = f.read()

    # Substitute mesh paths and robot name
    robot_sdf_content = robot_sdf_content.replace('package://autonomous_exploration/meshes', f'file://{mesh_path}')
    robot_sdf_content = robot_sdf_content.replace('{robot_name}', ROBOT_NAME)

    # Replace gz_frame_id for sensors to add namespace (for proper TF tree)
    robot_sdf_content = robot_sdf_content.replace(
        '<gz_frame_id>base_scan</gz_frame_id>',
        f'<gz_frame_id>{ROBOT_NAME}/base_scan</gz_frame_id>'
    )

    # Fix DiffDrive plugin frame_id to include robot namespace
    robot_sdf_content = robot_sdf_content.replace(
        '<frame_id>odom</frame_id>',
        f'<frame_id>{ROBOT_NAME}/odom</frame_id>'
    )

    # Fix DiffDrive plugin child_frame_id to include robot namespace
    robot_sdf_content = robot_sdf_content.replace(
        '<child_frame_id>base_footprint</child_frame_id>',
        f'<child_frame_id>{ROBOT_NAME}/base_footprint</child_frame_id>'
    )

    # Fix DiffDrive plugin tf_topic to use proper namespaced topic for bridge
    robot_sdf_content = robot_sdf_content.replace(
        '<tf_topic>/model/{robot_name}/tf</tf_topic>',
        f'<tf_topic>/model/{ROBOT_NAME}/tf</tf_topic>'
    )

    # Spawn robot node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name=f'spawn_{ROBOT_NAME}',
        output='screen',
        arguments=[
            '-world', gazebo_world_name_dynamic,
            '-name', ROBOT_NAME,
            '-string', robot_sdf_content,
            '-x', str(SPAWN_X),
            '-y', str(SPAWN_Y),
            '-z', str(SPAWN_Z),
            '-Y', str(SPAWN_YAW)
        ]
    )

    # Robot bridge configuration
    with open(bridge_robot_yaml, 'r') as f:
        template_content = f.read()

    config_content = template_content.replace('{robot_name}', ROBOT_NAME)
    config_content = config_content.replace('{world_name}', gazebo_world_name_str)

    temp_config = f'/tmp/{ROBOT_NAME}_bridge.yaml'
    with open(temp_config, 'w') as f:
        f.write(config_content)

    robot_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{ROBOT_NAME}_bridge',
        output='screen',
        parameters=[{'config_file': temp_config}]
    )

    # Robot state publisher
    with open(urdf_file, 'r') as f:
        urdf_content = f.read()
    urdf_content = urdf_content.replace('${namespace}', f'{ROBOT_NAME}/')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name=f'{ROBOT_NAME}_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': urdf_content,
            'frame_prefix': ''  # Empty since namespace already in URDF
        }],
        remappings=[('/joint_states', f'/{ROBOT_NAME}/joint_states')]
    )

    robot_spawn_delayed = TimerAction(
        period=SPAWN_DELAY,
        actions=[spawn_robot, robot_state_publisher, robot_bridge]
    )

    # Submap generator (Feature SLAM)
    submap_generator_feature = Node(
        package='map_generation',
        executable='local_submap_generator_feature',
        name=f'{ROBOT_NAME}_submap_generator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_name': ROBOT_NAME,
            'save_directory': './submaps'
        }]
    )

    submap_generator_delayed = TimerAction(
        period=MAPPING_DELAY,
        actions=[submap_generator_feature]
    )

    # Navigation node
    navigation_node = Node(
        package='navigation',
        executable='simple_navigation',
        name=f'{ROBOT_NAME}_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_name': ROBOT_NAME,
            'enable_reactive_avoidance': True,
            'scan_emergency_distance': 0.5,
            'scan_angular_range': 60.0
        }],
        condition=launch.conditions.IfCondition(enable_navigation)
    )

    navigation_delayed = TimerAction(period=NAVIGATION_DELAY, actions=[navigation_node])

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name=f'rviz2_{ROBOT_NAME}',
        namespace=ROBOT_NAME,
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    rviz_delayed = TimerAction(period=RVIZ_DELAY, actions=[rviz_node])

    # ========================================================================
    # 4. LAUNCH DESCRIPTION
    # ========================================================================

    return LaunchDescription([
        # Arguments
        world_arg,
        enable_navigation_arg,
        use_rviz_arg,
        OpaqueFunction(function=reject_unknown_launch_arguments),

        # System startup
        LogInfo(msg='========================================'),
        LogInfo(msg='FEATURE-BASED SLAM & NAVIGATION SYSTEM'),
        LogInfo(msg='========================================'),
        LogInfo(msg=['World: ', world_name]),
        LogInfo(msg=f'Robot: {ROBOT_NAME}'),
        LogInfo(msg='Mapping Mode: Feature SLAM with SVD-based submap stitching'),
        LogInfo(msg='========================================'),

        # Core system
        gazebo_server,
        clock_bridge_delayed,          # t=1s

        # Robot components
        robot_spawn_delayed,           # t=2s
        submap_generator_delayed,      # t=4s
        navigation_delayed,            # t=8s
        rviz_delayed,                  # t=6s

        LogInfo(msg='✓ System launching...')
    ])
