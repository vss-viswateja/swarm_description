import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    """
    Launch file for demo robot visualization in RViz2.
    
    This launch file starts:
    - robot_state_publisher: publishes robot transforms
    - joint_state_publisher_gui: GUI for controlling joint states
    - rviz2: visualization tool with pre-configured view
    """
    package_name = 'swarm_description'
    
    # Get package directories
    pkg_path = get_package_share_directory(package_name)
    world_pkg_path = get_package_share_directory('swarm_bringup')
    xacro_path = os.path.join(pkg_path, 'urdf', 'jackal.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'jackal_nav2.rviz')
    controller_param = os.path.join(pkg_path, 'config', 'jackal_control.yaml')  
    localization_param = os.path.join(pkg_path, 'config', 'jackal_ekf.yaml') 
    
    # Check if required files exist
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Xacro file not found: {xacro_path}")
    
    if not os.path.exists(rviz_config_path):
        raise FileNotFoundError(f"RViz config file not found: {rviz_config_path}")
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    world_file = LaunchConfiguration('world_file')
    robot_namespace = LaunchConfiguration('robot_namespace')
    
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
    
    # --- FIX: Read the URDF file content directly with namespace parameter ---
    robot_description = ParameterValue(
        Command(['xacro ', xacro_path, ' robot_namespace:=', robot_namespace]), 
        value_type=str
    )

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=robot_namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # --- FIX: Pass the string content directly ---
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'frame_prefix': 'jackal/'
        }],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': [world_file, ' -r'],
        }.items(),
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

    spawn_jackal = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_namespace,
        arguments=[
            '-name', 'jackal',
            '-topic', 'robot_description',  # Use relative topic name
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
                    period=3.0,  # Wait 3 seconds after robot_state_publisher starts
                    actions=[spawn_jackal]
                )
            ]
        )
    )
    
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_param],
        remappings=[('/controller_manager/robot_description', '/robot_description')]
    
    )

    # Delay controller spawners to ensure Gazebo and ros2_control are fully loaded
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
    
   

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=['arm_controller'],
    )

    # Add delays to controller spawners
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=13.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_diff_drive_controller_spawner = TimerAction(
        period=19.0,
        actions=[diff_drive_controller_spawner]
    )

    
    
    
    # Create RViz2 node with configuration
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

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'jackal_bridge.yaml')
        }]
        
    )

    

    static_tf_publisher_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'jackal_odom'],
        output='screen'
    )

    robot_localization_node = Node(
    package='robot_localization',
    executable='ekf_node',
    namespace=robot_namespace,
    name='ekf_node',
    output='screen',
    parameters=[localization_param, {'use_sim_time': use_sim_time}]
)
    
    # Removed unused imports for clarity
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config_cmd,
        declare_world_file_cmd,
        declare_robot_namespace_cmd,
        ign_resource_path,
        gz_resource_path,
        gazebo,
        robot_state_publisher_node,
        spawn_robot_after_rsp,  # Event-based spawn
        #joint_state_publisher_gui,
        rviz2_node,
        #controller_manager_node,  # Not needed - gz_ros2_control plugin handles this
        delayed_joint_state_broadcaster_spawner,
        delayed_diff_drive_controller_spawner,
        #delayed_arm_controller_spawner,
        bridge,
        #static_tf_publisher_map,
        robot_localization_node,
    ])

