import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    package_name = 'swarm_bringup'
    package_path = get_package_share_directory(package_name)
    param_package = get_package_share_directory('swarm_description')
    
    # Set launch configuration variables
    map_path = LaunchConfiguration('map_path')
    nav2_params_path = LaunchConfiguration('nav2_params_path')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Declare the launch variables with default values
    declare_map_path_cmd = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(package_path, 'maps', 'construction_site_v1.yaml'),
        description='Path to the map of the environment'
    )

    declare_nav2_param_path_cmd = DeclareLaunchArgument(
        'nav2_params_path',
        default_value=os.path.join(param_package, 'config', 'nav2_params.yaml'),
        description='Describing the navigation parameters'
    )
    
    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='jackal',
        description='Namespace for the robot (used for TF frame prefix)'
    )
 
    # Launch the navigation file
    # NOTE: For multi-robot, create separate nav2_params_<robot_name>.yaml files
    # with appropriate frame prefixes, and pass the correct file via nav2_params_path
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')]),
        launch_arguments={
            'use_sim_time': 'true',
            'map': map_path,
            'params_file': nav2_params_path,
            'use_namespace': 'true',
            'namespace': robot_namespace,
        }.items(),
    )
    
    # Use proper substitution for dynamic frame name
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
    

    return LaunchDescription([
        declare_map_path_cmd,
        declare_nav2_param_path_cmd,
        declare_robot_namespace_cmd,
        navigation,
        delayed_map_tf,        
    ])

