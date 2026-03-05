import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():

    package_name = 'swarm_bringup'
    package_path = get_package_share_directory(package_name)
    param_package = get_package_share_directory('swarm_description')
    
    # Set launch configuration variables
    map_path = LaunchConfiguration('map_path')
    nav2_params_path = LaunchConfiguration('nav2_params_path')
    robot_namespace = LaunchConfiguration('robot_namespace')
    pose_x = LaunchConfiguration('pose_x')
    pose_y = LaunchConfiguration('pose_y')
    pose_z = LaunchConfiguration('pose_z')

    # Build the default params filename dynamically: <namespace>_nav2_params.yaml
    default_params_file = PathJoinSubstitution([
        param_package, 'config',
        [robot_namespace, '_nav2_params.yaml']
    ])

    # Declare the launch variables with default values
    declare_map_path_cmd = DeclareLaunchArgument(
        'map_path',
        default_value=os.path.join(package_path, 'maps', 'construction_site_v2.yaml'),
        description='Path to the map of the environment'
    )

    declare_nav2_param_path_cmd = DeclareLaunchArgument(
        'nav2_params_path',
        default_value=default_params_file,
        description='Describing the navigation parameters'
    )
    
    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='mobman',
        description='Namespace for the robot (used for TF frame prefix)'
    )

    declare_pose_x_cmd = DeclareLaunchArgument(
        'pose_x',
        default_value='1.0',
        description='Initial x position of the robot'
    )

    declare_pose_y_cmd = DeclareLaunchArgument(
        'pose_y',
        default_value='-3.0',
        description='Initial y position of the robot'
    )

    declare_pose_z_cmd = DeclareLaunchArgument(
        'pose_z',
        default_value='0.22',
        description='Initial z position of the robot'
    )
 
    # Launch the navigation file
    # NOTE: For multi-robot, create separate nav2_params_<robot_name>.yaml files
    # with appropriate frame prefixes, and pass the correct file via nav2_params_path
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
    
    # Use proper substitution for dynamic frame name
    odom_frame = [robot_namespace, '/odom']

    static_tf_publisher_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        namespace=robot_namespace,
        arguments=['--x', pose_x, '--y', pose_y, '--z', pose_z, 
                   '--roll', '0', '--pitch', '0', '--yaw', '0',
                   '--frame-id', 'gz_world', '--child-frame-id', odom_frame],
        remappings=[('/tf_static', '/tf_static')],
        output='screen'
    )

    delayed_map_tf = TimerAction(
        period=5.0,
        actions=[static_tf_publisher_map]
    )
    

    return LaunchDescription([
        declare_map_path_cmd,
        declare_pose_x_cmd,
        declare_pose_y_cmd,
        declare_pose_z_cmd, 
        declare_nav2_param_path_cmd,
        declare_robot_namespace_cmd,
        navigation,
        delayed_map_tf,        
    ])

