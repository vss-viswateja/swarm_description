"""
Combined Launch File: Jackal Control + Nav2

This launch file merges jackal_control.launch.py and jackal_nav2.launch.py:
1. Gazebo simulation with Jackal robot
2. Robot state publisher with TF frame_prefix
3. ros2_control controllers (joint_state_broadcaster, diff_drive_controller)
4. Robot localization (EKF)
5. Nav2 navigation stack
6. Static TF publisher for map->odom

Timing sequence:
- 0s:   robot_state_publisher
- 3s:   Spawn robot in Gazebo
- 13s:  joint_state_broadcaster controller
- 19s:  diff_drive_controller
- 5s:   Static TF (map->odom)
- 0s:   Nav2 navigation stack

Usage:
    # Default launch
    ros2 launch swarm_description jackal_bringup.launch.py
    
    # Custom namespace
    ros2 launch swarm_description jackal_bringup.launch.py robot_namespace:=robot1
    
    # Custom map and world
    ros2 launch swarm_description jackal_bringup.launch.py \
        world_file:=/path/to/world.sdf map_path:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import (
    DeclareLaunchArgument, 
    SetEnvironmentVariable, 
    IncludeLaunchDescription, 
    RegisterEventHandler, 
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for Jackal + Nav2 simulation."""
    
    # ======================= Package Paths =======================
    package_name = 'swarm_description'
    pkg_path = get_package_share_directory(package_name)
    world_pkg_path = get_package_share_directory('swarm_bringup')
    
    xacro_path = os.path.join(pkg_path, 'urdf', 'jackal.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'jackal_nav2.rviz')
    controller_param = os.path.join(pkg_path, 'config', 'jackal_control.yaml')
    localization_param = os.path.join(pkg_path, 'config', 'jackal_ekf.yaml')
    
    # Check if required files exist
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Xacro file not found: {xacro_path}")
    
    # ======================= Launch Arguments =======================
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world_file')
    robot_namespace = LaunchConfiguration('robot_namespace')
    map_path = LaunchConfiguration('map_path')
    nav2_params_path = LaunchConfiguration('nav2_params_path')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Path to RViz configuration file'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(world_pkg_path, 'worlds', 'test_world_v2.sdf'),
        description='Path to the world file to load'
    )
    
    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='jackal',
        description='Namespace for the robot (e.g., "robot1", "robot2"). Leave empty for single robot.'
    )
    
    declare_map_path_cmd = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(world_pkg_path, 'maps', 'construction_site_v1.yaml'),
        description='Path to the map of the environment'
    )
    
    declare_nav2_params_path_cmd = DeclareLaunchArgument(
        'nav2_params_path',
        default_value=os.path.join(pkg_path, 'config', 'jackal_nav2_params.yaml'),
        description='Path to the Nav2 parameters file'
    )

    # ======================= Environment Variables =======================
    robot_desc_pkg_prefix = get_package_prefix('swarm_description')
    resource_path = os.path.join(robot_desc_pkg_prefix, 'share') + ':' + '/home/viswa/Desktop/Gazebo_models'
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    # ======================= Robot Description =======================
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' robot_namespace:=', robot_namespace]), 
        value_type=str
    )
    
    frame_prefix = PythonExpression(["'", robot_namespace, "/' if '", robot_namespace, "' else ''"])

    # ======================= Robot State Publisher =======================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': frame_prefix
        }],
    )

    # ======================= Spawn Robot in Gazebo =======================
    spawn_jackal = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_namespace,
        arguments=[
            '-name', robot_namespace,
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '-3.0',
            '-z', '0.22',
        ],
        output='screen',
    )
    
    # Use event handler to spawn robot after robot_state_publisher starts
    spawn_robot_after_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[spawn_jackal]
                )
            ]
        )
    )

    # ======================= Controllers (Delayed) =======================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=['joint_state_broadcaster'],
    )
    
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=['diff_drive_controller'],
    )
    
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=13.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_diff_drive_controller_spawner = TimerAction(
        period=19.0,
        actions=[diff_drive_controller_spawner]
    )

    # ======================= Robot Localization (EKF) =======================
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=robot_namespace,
        name='ekf_node',
        output='screen',
        parameters=[localization_param, {'use_sim_time': use_sim_time}]
    )

    # ======================= Nav2 Navigation Stack =======================
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_description'), 'launch', 'bringup.launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
            'params_file': nav2_params_path,
            'use_namespace': 'true',
            'namespace': robot_namespace,
        }.items(),
    )

    # ======================= Static TF Publisher (map->odom) =======================
    odom_frame = [robot_namespace, '/odom']
    
    static_tf_publisher_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['--x', '0', '--y', '0', '--z', '0', 
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'map', '--child-frame-id', odom_frame],
        output='screen'
    )

    delayed_map_tf = TimerAction(
        period=5.0,
        actions=[static_tf_publisher_map]
    )

    # ======================= RViz2 (Optional) =======================
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    # ======================= Gazebo-ROS Bridge =======================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'jackal_bridge.yaml')
        }]
    )

    # ======================= Launch Description =======================
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time_cmd,
        declare_rviz_config_cmd,
        declare_world_file_cmd,
        declare_robot_namespace_cmd,
        declare_map_path_cmd,
        declare_nav2_params_path_cmd,
        
        # Environment
        ign_resource_path,
        gz_resource_path,
        
        # Phase 1: Robot setup
        robot_state_publisher_node,
        spawn_robot_after_rsp,
        
        # Phase 2: Controllers
        delayed_joint_state_broadcaster_spawner,
        delayed_diff_drive_controller_spawner,
        
        # Phase 3: Localization
        robot_localization_node,
        
        # Phase 4: Navigation
        navigation,
        delayed_map_tf,
        
        # Uncomment below as needed:
        # rviz2_node,
        # bridge,
    ])
