"""
Combined Launch File: UR5 Control + MoveIt2

This launch file merges ur5_control.launch.py with MoveIt2 motion planning:
1. Gazebo simulation with UR5 robot
2. Robot state publisher with TF frame_prefix
3. ros2_control controllers (joint_state_broadcaster, arm_controller)
4. MoveIt2 move_group for motion planning
5. RViz with MoveIt plugin (optional)

Timing sequence:
- 0s:   robot_state_publisher, spawn robot
- 15s:  joint_state_broadcaster controller
- 20s:  arm_controller
- 25s:  MoveIt2 move_group
- 27s:  RViz (if enabled)

Usage:
    # Default launch
    ros2 launch swarm_description ur5_moveit.launch.py
    
    # Without RViz
    ros2 launch swarm_description ur5_moveit.launch.py use_rviz:=false
    
    # Custom namespace
    ros2 launch swarm_description ur5_moveit.launch.py robot_namespace:=robot1
    
    # Custom world file
    ros2 launch swarm_description ur5_moveit.launch.py world_file:=/path/to/world.sdf
"""

import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.actions import (
    DeclareLaunchArgument, 
    SetEnvironmentVariable, 
    IncludeLaunchDescription, 
    TimerAction,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Generate launch description for UR5 + MoveIt2 simulation."""
    
    # ======================= Package Paths =======================
    package_name = 'swarm_description'
    pkg_path = get_package_share_directory(package_name)
    world_pkg_path = get_package_share_directory('swarm_bringup')
    ur5_moveit_config_pkg = get_package_share_directory('ur5_moveit_config')
    
    xacro_path = os.path.join(pkg_path, 'urdf', 'ur5_assembly.xacro')
    rviz_config_path = os.path.join(pkg_path, 'config', 'ur5.rviz')
    controller_param = os.path.join(pkg_path, 'config', 'ur5_control.yaml')
    
    # Check if required files exist
    if not os.path.exists(xacro_path):
        raise FileNotFoundError(f"Xacro file not found: {xacro_path}")

    # ======================= Launch Arguments =======================
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_namespace',
            default_value='ur5',
            description='Namespace for the robot (e.g., "ur5", "robot1")'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'world_file',
            default_value=os.path.join(world_pkg_path, 'worlds', 'test_world_v2.sdf'),
            description='Path to the Gazebo world file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with MoveIt plugin'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_path,
            description='Path to RViz configuration file'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'spawn_x',
            default_value='-1.0',
            description='X position for robot spawn'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'spawn_y',
            default_value='-3.0',
            description='Y position for robot spawn'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'spawn_z',
            default_value='0.125',
            description='Z position for robot spawn'
        )
    )

    # ======================= Get Configurations =======================
    robot_namespace = LaunchConfiguration('robot_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_file = LaunchConfiguration('world_file')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')

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
    spawn_ur5 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=robot_namespace,
        arguments=[
            '-name', 'ur5',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
        ],
        output='screen',
    )

    # ======================= Gazebo-ROS Bridge =======================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_path, 'config', 'ur5_bridge.yaml')
        }]
    )

    # ======================= Static TF Publisher =======================
    static_tf_publisher_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world',
        arguments=['-1.0', '0', '0', '0', '0', '0', 'map', 'ur5/world'],
        output='screen'
    )

    # ======================= Controllers (Delayed) =======================
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=15.0,
        actions=[joint_state_broadcaster_spawner]
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        arguments=['arm_controller'],
        output='screen',
    )
    
    delayed_arm_controller_spawner = TimerAction(
        period=20.0,
        actions=[arm_controller_spawner]
    )

    # ======================= MoveIt2 Configuration =======================
    moveit_config = MoveItConfigsBuilder(
        "ur5_assembly", 
        package_name="ur5_moveit_config"
    ).to_moveit_configs()

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "use_sim_time": use_sim_time,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # MoveGroup node - delay 25s (after controllers are ready)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        namespace=robot_namespace,
        output="screen",
        parameters=move_group_params,
        remappings=[
            ("joint_states", "joint_states"),
            ("robot_description", "robot_description"),
        ],
    )
    
    delayed_move_group = TimerAction(
        period=25.0,
        actions=[move_group_node]
    )

    # ======================= RViz with MoveIt (Delayed) =======================
    moveit_rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ur5_moveit_config"),
        "config",
        "moveit.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", moveit_rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),
    )
    
    delayed_rviz = TimerAction(
        period=27.0,
        actions=[rviz_node]
    )

    # ======================= Launch Description =======================
    return LaunchDescription(
        declared_arguments + [
            # Environment
            ign_resource_path,
            gz_resource_path,
            
            # Phase 1: Simulation setup (0s)
            robot_state_publisher_node,
            spawn_ur5,
            static_tf_publisher_world,
            # bridge,  # Uncomment if needed
            
            # Phase 2: Controllers (15s, 20s)
            delayed_joint_state_broadcaster_spawner,
            delayed_arm_controller_spawner,
            
            # Phase 3: MoveIt2 (25s)
            delayed_move_group,
            
            # Phase 4: Visualization (27s)
            delayed_rviz,
        ]
    )
