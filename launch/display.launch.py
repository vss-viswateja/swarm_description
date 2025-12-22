import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

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
    xacro_path = os.path.join(pkg_path, 'urdf', 'ur5_assembly.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'jackal.rviz')
    controller_param = os.path.join(pkg_path, 'config', 'ros2_control.yaml') 
    
    # Check if required files exist
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Xacro file not found: {xacro_path}")
    
    if not os.path.exists(rviz_config_path):
        raise FileNotFoundError(f"RViz config file not found: {rviz_config_path}")
    
    
    # --- FIX: Read the URDF file content directly ---
    robot_description = ParameterValue(
    Command(['xacro ', xacro_path]), 
    value_type=str
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_path,
        description='Path to RViz configuration file'
    )
    

    # Create robot_state_publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # --- FIX: Pass the string content directly ---
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
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

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
    )
    
    
    # Create RViz2 node with configuration
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Removed unused imports for clarity
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_rviz_config_cmd,
        robot_state_publisher_node,
        joint_state_publisher_gui,
        rviz2_node,
        #controller_manager_node,
        #joint_state_broadcaster_spawner,
        #diff_drive_controller_spawner,
        #arm_controller_spawner,
    ])

