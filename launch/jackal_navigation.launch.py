import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Complete launch file for Jackal robot with Nav2 navigation.
    
    This launch file starts everything needed for autonomous navigation:
    1. Gazebo simulation with the world
    2. Jackal robot spawn and control
    3. Nav2 navigation stack
    4. RViz2 for visualization and waypoint setting
    
    Usage:
    ros2 launch swarm_description jackal_navigation.launch.py
    
    Then in RViz:
    - Use "2D Pose Estimate" to set initial pose (if using AMCL with a map)
    - Use "Nav2 Goal" to send navigation goals
    """
    
    package_name = 'swarm_description'
    bringup_pkg = 'swarm_bringup'
    
    # Get package directories
    pkg_path = get_package_share_directory(package_name)
    bringup_path = get_package_share_directory(bringup_pkg)
    
    # Launch configuration variables
    world_file = LaunchConfiguration('world_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    
    # Declare launch arguments
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(bringup_path, 'worlds', 'empty_world.sdf'),
        description='Path to the world file to load'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value='',
        description='Full path to map yaml file. Leave empty to use SLAM or navigation without a map.'
    )
    
    # Launch Gazebo with the world (from bringup_ign but without robots)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path, 'launch', 'bringup_ign.launch.py')
        ),
        launch_arguments={
            'world_file': world_file,
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Launch Jackal robot with control
    jackal_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'jackal_control.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world_file': world_file,
        }.items()
    )
    
    # Delay jackal spawn to ensure Gazebo is ready
    delayed_jackal_launch = TimerAction(
        period=5.0,
        actions=[jackal_launch]
    )
    
    # Launch Nav2 navigation stack (using the simple version that works)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_path, 'launch', 'jackal_nav2_simple.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Delay Nav2 to ensure robot is spawned and controllers are loaded
    delayed_nav2_launch = TimerAction(
        period=15.0,
        actions=[nav2_launch]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(declare_world_file_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    
    # Add launch actions in order
    ld.add_action(gazebo_launch)
    ld.add_action(delayed_jackal_launch)
    ld.add_action(delayed_nav2_launch)
    
    return ld
