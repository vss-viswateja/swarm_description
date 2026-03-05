import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    pkg_swarm_description = get_package_share_directory('swarm_description')

    # Nodes managed by the lifecycle manager
    lifecycle_nodes = ['collision_monitor']
    autostart = True

    # ---------------------------------------------------------------------------
    # Launch arguments
    # ---------------------------------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Build the default params filename dynamically: <namespace>_collision_params.yaml
    default_params_file = PathJoinSubstitution([
        pkg_swarm_description, 'config',
        [namespace, '_collision_params.yaml']
    ])

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace for the robot (e.g. agent1, agent2)'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the collision monitor params YAML file'
    )

    # ---------------------------------------------------------------------------
    # RewrittenYaml: wraps the YAML under the namespace key so the namespaced
    # node can resolve its parameters correctly.
    # e.g.  collision_monitor: ros__parameters: ...
    #   --> agent1: collision_monitor: ros__parameters: ...
    # ---------------------------------------------------------------------------
    param_substitutions = {
        'use_sim_time': use_sim_time
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    # ---------------------------------------------------------------------------
    # Nodes
    # Both nodes MUST share the same namespace so the lifecycle manager can
    # reach the collision monitor via its relative name 'collision_monitor'.
    # ---------------------------------------------------------------------------
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='collision_lifecycle_manager',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ]
    )

    start_collision_monitor_cmd = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        namespace=namespace,
        output='screen',
        emulate_tty=True,
        parameters=[configured_params]
    )

    # ---------------------------------------------------------------------------
    # Launch Description
    # ---------------------------------------------------------------------------
    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_collision_monitor_cmd)

    return ld
