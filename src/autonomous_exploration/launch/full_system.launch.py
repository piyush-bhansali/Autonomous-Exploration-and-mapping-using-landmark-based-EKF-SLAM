#!/usr/bin/env python3

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
    # ========================================================================
    # CONFIGURATION
    # ========================================================================
    NUM_ROBOTS = 1  # Change this to spawn more robots (1, 2, etc.)

    # Initial spawn positions for each robot: (x, y, z, yaw)
    ROBOT_SPAWN_POSES = {
        1: {'x': -12.0, 'y': -12.0, 'z': 0.01, 'yaw': 0.0},      # Southwest corner
        2: {'x': 12.0, 'y': 12.0, 'z': 0.01, 'yaw': 3.14159},    # Northeast corner, facing SW
    }

    # Timer delay configuration (in seconds)
    BASE_SPAWN_DELAY = 2.0        # First robot spawns at t=2s
    BASE_MAPPING_DELAY = 4.0      # First robot mapping starts at t=4s
    BASE_RVIZ_DELAY = 6.0         # First robot rviz starts at t=6s
    BASE_NAVIGATION_DELAY = 8.0   # First robot navigation starts at t=8s
    ROBOT_DELAY_OFFSET = 1.0      # Each additional robot adds 1s to all timers

    # Package directories
    pkg_multi_robot = get_package_share_directory('autonomous_exploration')
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
        pkg_multi_robot,
        'worlds',
        [world_name, '.sdf']
    ])

    robot_sdf = os.path.join(pkg_multi_robot, 'models', 'turtlebot3_waffle_pi', 'model.sdf')
    urdf_file = os.path.join(pkg_multi_robot, 'urdf', 'turtlebot3_waffle_pi.urdf')
    bridge_common_yaml = os.path.join(pkg_multi_robot, 'config', 'tb3_bridge_common.yaml')
    bridge_robot_yaml = os.path.join(pkg_multi_robot, 'config', 'tb3_bridge.yaml')
    mesh_path = os.path.join(pkg_multi_robot, 'meshes')

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
    # 3. HELPER FUNCTION: CREATE ROBOT COMPONENTS
    # ========================================================================
    def create_robot_components(robot_id, spawn_pose):
       
        robot_name = f'tb3_{robot_id}'

        # Calculate delays for this robot (stagger based on robot_id)
        delay_offset = (robot_id - 1) * ROBOT_DELAY_OFFSET
        spawn_delay = BASE_SPAWN_DELAY + delay_offset
        mapping_delay = BASE_MAPPING_DELAY + delay_offset
        rviz_delay = BASE_RVIZ_DELAY + delay_offset
        nav_delay = BASE_NAVIGATION_DELAY + delay_offset

        # RViz config file path
        rviz_config = os.path.join(pkg_multi_robot, 'rviz', f'{robot_name}_visualization.rviz')

        # Prepare robot SDF with substitutions
        with open(robot_sdf, 'r') as f:
            robot_sdf_content = f.read()

        # Substitute mesh paths and robot name
        robot_sdf_content = robot_sdf_content.replace('package://autonomous_exploration/meshes', f'file://{mesh_path}')
        robot_sdf_content = robot_sdf_content.replace('{robot_name}', robot_name)

        # Replace gz_frame_id for sensors to add namespace (for proper TF tree)
        robot_sdf_content = robot_sdf_content.replace(
            '<gz_frame_id>base_scan</gz_frame_id>',
            f'<gz_frame_id>{robot_name}/base_scan</gz_frame_id>'
        )

        # Fix DiffDrive plugin frame_id to include robot namespace
        robot_sdf_content = robot_sdf_content.replace(
            '<frame_id>odom</frame_id>',
            f'<frame_id>{robot_name}/odom</frame_id>'
        )

        # Fix DiffDrive plugin child_frame_id to include robot namespace
        robot_sdf_content = robot_sdf_content.replace(
            '<child_frame_id>base_footprint</child_frame_id>',
            f'<child_frame_id>{robot_name}/base_footprint</child_frame_id>'
        )

        # Spawn robot node
        spawn_robot = Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{robot_name}',
            output='screen',
            arguments=[
                '-world', gazebo_world_name_dynamic,
                '-name', robot_name,
                '-string', robot_sdf_content,
                '-x', str(spawn_pose['x']),
                '-y', str(spawn_pose['y']),
                '-z', str(spawn_pose['z']),
                '-Y', str(spawn_pose['yaw'])
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

        # Wrap spawn components in TimerAction
        robot_spawn_delayed = TimerAction(
            period=spawn_delay,
            actions=[spawn_robot, robot_state_publisher, robot_bridge]
        )

        # Mapping: Local submap generator
        submap_generator = Node(
            package='map_generation',
            executable='local_submap_generator',
            name=f'{robot_name}_submap_generator',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name,
                'save_directory': './submaps',
                'enable_loop_closure': True
            }]
        )

        submap_generator_delayed = TimerAction(period=mapping_delay, actions=[submap_generator])

        # Navigation: Frontier exploration
        navigation_node = Node(
            package='navigation',
            executable='simple_navigation',
            name=f'{robot_name}_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'robot_name': robot_name,
                'enable_reactive_avoidance': True,
                'scan_emergency_distance': 0.5,
                'scan_angular_range': 60.0
            }],
            condition=launch.conditions.IfCondition(enable_navigation)
        )

        navigation_delayed = TimerAction(period=nav_delay, actions=[navigation_node])

        # RViz visualization
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name=f'rviz2_{robot_name}',
            namespace=robot_name,
            arguments=['-d', rviz_config],
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=launch.conditions.IfCondition(use_rviz)
        )

        rviz_delayed = TimerAction(period=rviz_delay, actions=[rviz_node])

        # Return all delayed actions for this robot
        return [robot_spawn_delayed, submap_generator_delayed, navigation_delayed, rviz_delayed]

    # ========================================================================
    # 4. LAUNCH DESCRIPTION
    # ========================================================================

    # Build the launch description
    launch_entities = [
        # Arguments
        world_arg,
        enable_navigation_arg,
        use_rviz_arg,

        # System startup
        LogInfo(msg='========================================'),
        LogInfo(msg='MULTI-ROBOT SLAM & NAVIGATION SYSTEM'),
        LogInfo(msg='========================================'),
        LogInfo(msg=['World: ', world_name]),
        LogInfo(msg=f'Robots: {NUM_ROBOTS}'),
        LogInfo(msg='========================================'),

        # Core system
        gazebo_server,
        clock_bridge_delayed,          # t=1s
    ]

    # Create and add components for each robot using loop
    for robot_id in range(1, NUM_ROBOTS + 1):
        if robot_id not in ROBOT_SPAWN_POSES:
            raise ValueError(f"No spawn pose defined for robot {robot_id}. Please add it to ROBOT_SPAWN_POSES.")

        robot_components = create_robot_components(
            robot_id=robot_id,
            spawn_pose=ROBOT_SPAWN_POSES[robot_id]
        )
        launch_entities.extend(robot_components)

    launch_entities.append(LogInfo(msg='✓ Multi-robot system launching...'))

    return LaunchDescription(launch_entities)
