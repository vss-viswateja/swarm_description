"""
Combined Launch File: Mobile Manipulator Control + Nav2 + MoveIt2

This launch file includes three sub-launch files:
1. mobile_manipulator.launch.py: Gazebo simulation with Mobile Manipulator (Jackal + UR5),
   robot state publisher, ros2_control controllers, and robot localization (EKF)
2. mobman_nav2.launch.py: Nav2 navigation stack with static TF publisher
3. mobman_moveit.launch.py: MoveIt2 motion planning for UR5 arm

Usage:
    # Default launch (all components)
    ros2 launch swarm_description mobman_bringup.launch.py
    
    # Custom namespace
    ros2 launch swarm_description mobman_bringup.launch.py robot_namespace:=robot1
    
    # Custom map and world
    ros2 launch swarm_description mobman_bringup.launch.py \
        world_file:=/path/to/world.sdf map_path:=/path/to/map.yaml
    
    # Disable specific components
    ros2 launch swarm_description mobman_bringup.launch.py \
        launch_nav2:=false launch_moveit:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """Generate launch description for Mobile Manipulator + Nav2 + MoveIt2 simulation."""
    
    # ======================= Package Paths =======================
    package_name = 'swarm_description'
    pkg_path = get_package_share_directory(package_name)
    world_pkg_path = get_package_share_directory('swarm_bringup')
    moveit_pkg_path = get_package_share_directory('mobman_moveit_config')
    
    rviz_config_path = os.path.join(pkg_path, 'config', 'mobman.rviz')
    
    # ======================= Launch Arguments =======================
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world_file')
    robot_namespace = LaunchConfiguration('robot_namespace')
    map_path = LaunchConfiguration('map_path')
    nav2_params_path = LaunchConfiguration('nav2_params_path')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_moveit = LaunchConfiguration('launch_moveit')
    use_rviz = LaunchConfiguration('use_rviz')
    
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
        default_value='mobman',
        description='Namespace for the robot (e.g., "robot1", "robot2"). Leave empty for single robot.'
    )
    
    declare_map_path_cmd = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(world_pkg_path, 'maps', 'construction_site_v1.yaml'),
        description='Path to the map of the environment'
    )
    
    declare_nav2_params_path_cmd = DeclareLaunchArgument(
        'nav2_params_path',
        default_value=os.path.join(pkg_path, 'config', 'mobman_nav2_params.yaml'),
        description='Path to the Nav2 parameters file'
    )
    
    declare_launch_nav2_cmd = DeclareLaunchArgument(
        'launch_nav2',
        default_value='true',
        description='Whether to launch Nav2 navigation stack'
    )
    
    declare_launch_moveit_cmd = DeclareLaunchArgument(
        'launch_moveit',
        default_value='true',
        description='Whether to launch MoveIt2 motion planning'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Whether to launch RViz with MoveIt plugin'
    )

    # ======================= Include: mobile_manipulator.launch.py =======================
    # This includes: Gazebo, robot_state_publisher, spawn robot, controllers, EKF, bridge
    mobile_manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'mobile_manipulator.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz_config': rviz_config,
            'world_file': world_file,
            'robot_namespace': robot_namespace,
        }.items(),
    )

    # ======================= Include: mobman_nav2.launch.py =======================
    # This includes: Nav2 navigation stack, static TF publisher (map->odom)
    mobman_nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'mobman_nav2.launch.py')
        ),
        launch_arguments={
            'map_path': map_path,
            'nav2_params_path': nav2_params_path,
            'robot_namespace': robot_namespace,
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    # ======================= Include: mobman_moveit.launch.py (Delayed) =======================
    # This includes: MoveIt2 move_group node for arm motion planning
    # Delayed to ensure controllers are fully loaded before MoveIt starts
    mobman_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg_path, 'launch', 'mobman_moveit.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
        }.items(),
        condition=IfCondition(launch_moveit),
    )
    
    # Delay MoveIt launch to ensure controllers are ready (after arm_controller at 12s)
    delayed_moveit_launch = TimerAction(
        period=15.0,
        actions=[mobman_moveit_launch]
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
        declare_launch_nav2_cmd,
        declare_launch_moveit_cmd,
        declare_use_rviz_cmd,
        
        # Include sub-launch files
        mobile_manipulator_launch,      # Phase 1: Gazebo + Robot + Controllers + Localization
        mobman_nav2_launch,             # Phase 2: Nav2 navigation stack
        delayed_moveit_launch,          # Phase 3: MoveIt2 (delayed 15s for controllers)
    ])
