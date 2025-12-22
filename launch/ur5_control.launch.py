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
    Launch file for UR5 robot simulation in Gazebo IGN.
    
    This launch file starts:
    - robot_state_publisher: publishes robot transforms
    - gazebo: Simulation environment
    - Spawn_UR5: Spawns the UR5 robot in the simulation
    - Controller Spawner: Spawns the necessary controllers
    - rviz2: visualization tool with pre-configured view
    """
    package_name = 'swarm_description'
    
    # Get package directories
    pkg_path = get_package_share_directory(package_name)
    world_pkg_path = get_package_share_directory('swarm_bringup')
    xacro_path = os.path.join(pkg_path, 'urdf', 'ur5_assembly.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'ur5.rviz')
    controller_param = os.path.join(pkg_path, 'config', 'ur5_control.yaml') 
    
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
        default_value=os.path.join(world_pkg_path, 'worlds', 'test_world_v1.sdf'),
        description='Path to the world file to load'
    )
    
    declare_robot_namespace_cmd = DeclareLaunchArgument(
        'robot_namespace',
        default_value='ur5',
        description='Namespace for the robot (e.g., "robot1_", "robot2_"). Leave empty for single robot.'
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
    resource_path = os.path.join(robot_desc_pkg_prefix, 'share')
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    spawn_ur5 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_namespace,
        arguments=[
            '-name', 'ur5',
            '-topic', ['/ur5/robot_description'],
            '-x', '4.8',
            '-y', '6.8',
            '-z', '0.761',
        ],
        output='screen',
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

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=['arm_controller'],
    )

    # Add delays to controller spawners - MUCH LONGER to allow robot to fully spawn
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=15.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_arm_controller_spawner = TimerAction(
        period=20.0,
        actions=[arm_controller_spawner]
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
            'config_file': os.path.join(pkg_path, 'config', 'ur5_bridge.yaml')
        }]
    )
    static_tf_publisher_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['4.8', '6.8', '0.761', '0', '0', '0', 'world', 'ur5_base_link'],
        output='screen'
    )

    
    # Removed unused imports for clarity
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config_cmd,
        declare_world_file_cmd,
        declare_robot_namespace_cmd,
        ign_resource_path,
        gz_resource_path,
        robot_state_publisher_node,
        #gazebo,     # Un-coment this to launch gazebo sim with this file alone. 
        spawn_ur5,
        #joint_state_publisher_gui,
        #rviz2_node,
        #controller_manager_node,  # Not needed - gz_ros2_control plugin handles this
        delayed_joint_state_broadcaster_spawner,
        delayed_arm_controller_spawner,
        #static_tf_publisher_world,
        #bridge,
    ])

