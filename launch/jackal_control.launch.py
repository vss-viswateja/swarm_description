import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction callback that resolves LaunchConfiguration values
    and creates properly namespaced nodes for each robot.
    """
    package_name = 'swarm_description'
    
    # Get package directories
    pkg_path = get_package_share_directory(package_name)
    world_pkg_path = get_package_share_directory('swarm_bringup')
    xacro_path = os.path.join(pkg_path, 'urdf', 'jackal.urdf.xacro')
    controller_param = os.path.join(pkg_path, 'config', 'jackal_control.yaml')  
    localization_param = os.path.join(pkg_path, 'config', 'jackal_ekf.yaml') 
    
    # Resolve LaunchConfiguration values to actual strings
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    robot_namespace = LaunchConfiguration('robot_namespace').perform(context)
    pose_x = LaunchConfiguration('pose_x').perform(context)
    pose_y = LaunchConfiguration('pose_y').perform(context)
    pose_z = LaunchConfiguration('pose_z').perform(context)
    
    # Convert use_sim_time string to boolean for parameters
    use_sim_time_bool = use_sim_time.lower() == 'true'
    
    # Build frame names with resolved namespace
    odom_frame = f'{robot_namespace}/odom'
    base_link_frame = f'{robot_namespace}/base_link'
    world_frame = f'{robot_namespace}/odom'
    map_frame = f'{robot_namespace}/map'
    frame_prefix = f'{robot_namespace}/'
    
    # Process xacro with resolved namespace
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' robot_namespace:=', robot_namespace]), 
        value_type=str
    )

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

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time_bool,
            'frame_prefix': frame_prefix
        }],
    )

    spawn_jackal = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_namespace,
        arguments=[
            '-name', robot_namespace,
            '-topic', 'robot_description',
            '-x', pose_x,
            '-y', pose_y,
            '-z', pose_z,
        ],
        output='screen',
    )

    # Controller spawners
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

    # Robot localization node with resolved frame names
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        namespace=robot_namespace,
        name='ekf_node',
        output='screen',
        parameters=[localization_param, 
                    {
                        'use_sim_time': use_sim_time_bool,
                        'odom_frame': odom_frame,
                        'base_link_frame': base_link_frame,
                        'world_frame': world_frame,
                        'map_frame': map_frame,
                    }]
    )
    
    # ============ EVENT HANDLERS ============
    
    # Event handler to spawn robot after robot_state_publisher starts
    spawn_robot_after_rsp = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[spawn_jackal]
                )
            ]
        )
    )
    
    # Event-based controller spawning - wait for robot to spawn in Gazebo
    spawn_controllers_after_robot = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_jackal,
            on_exit=[
                TimerAction(
                    period=5.0,
                    actions=[joint_state_broadcaster_spawner, diff_drive_controller_spawner]
                )
            ]
        )
    )
    
    # Event-based localization - wait for diff_drive_controller to be ready
    # Using TimerAction to ensure controllers are up before starting EKF
    start_localization_after_controllers = TimerAction(
        period=12.0,
        actions=[robot_localization_node]
    )
    
    return [
        ign_resource_path,
        gz_resource_path,
        robot_state_publisher_node,
        spawn_robot_after_rsp,
        spawn_controllers_after_robot,
        start_localization_after_controllers,
    ]


def generate_launch_description():
    """
    Launch file for robot control with proper namespace isolation.
    Uses OpaqueFunction to resolve LaunchConfiguration values before
    creating nodes, ensuring each robot instance gets unique nodes.
    """
    package_name = 'swarm_description'
    pkg_path = get_package_share_directory(package_name)
    world_pkg_path = get_package_share_directory('swarm_bringup')
    rviz_config_path = os.path.join(pkg_path, 'config', 'jackal_nav2.rviz')
    
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
    
    declare_pose_x_cmd = DeclareLaunchArgument(
        'pose_x',
        default_value='0.0',
        description='Initial X position of the robot'
    )
    
    declare_pose_y_cmd = DeclareLaunchArgument(
        'pose_y',
        default_value='-3.0',
        description='Initial Y position of the robot'
    )
    
    declare_pose_z_cmd = DeclareLaunchArgument(
        'pose_z',
        default_value='0.22',
        description='Initial Z position of the robot'
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config_cmd,
        declare_world_file_cmd,
        declare_robot_namespace_cmd,
        declare_pose_x_cmd,
        declare_pose_y_cmd,
        declare_pose_z_cmd,
        OpaqueFunction(function=launch_setup),
    ])
