import os
import yaml
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, TimerAction, OpaqueFunction
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
 
    # Launch the navigation file via an OpaqueFunction to dynamically modify parameters
    def launch_setup(context, *args, **kwargs):
        ns = LaunchConfiguration('robot_namespace').perform(context)
        map_path_evaluated = LaunchConfiguration('map_path').perform(context)
        params_path = LaunchConfiguration('nav2_params_path').perform(context)
        
        # Load fleet config
        fleet_yaml_path = os.path.join(get_package_share_directory('nav2_proximity_wait'), 'config', 'fleet_config.yaml')
        with open(fleet_yaml_path, 'r') as f:
            fleet_yaml = yaml.safe_load(f)
            
        fleet = fleet_yaml.get('fleet_namespaces', [])
        suffix = fleet_yaml.get('frame_suffix', 'base_link')
        safety_radius = fleet_yaml.get('safety_radius', 1.0)
        clear_radius = fleet_yaml.get('clear_radius', 1.4)
        
        # Compute other_robot_frames
        other_frames = [f"{r}/{suffix}" for r in fleet if str(r) != str(ns)]
        other_frames_str = ";".join(other_frames)
        
        # Log the derived string for troubleshooting
        print(f"[mobman_nav2] Derived other_robot_frames for {ns}: '{other_frames_str}'")
        
        # Load default params for this robot
        with open(params_path, 'r') as f:
            params = yaml.safe_load(f)
            
        # Load bt_navigator overrides
        bt_nav_yaml_path = os.path.join(get_package_share_directory('nav2_proximity_wait'), 'config', 'bt_navigator_proximity.yaml')
        with open(bt_nav_yaml_path, 'r') as f:
            bt_overrides = yaml.safe_load(f)
            
        if 'bt_navigator' not in params:
            params['bt_navigator'] = {'ros__parameters': {}}
            
        # Merge overrides into the main params dict
        params['bt_navigator']['ros__parameters'].update(bt_overrides.get('bt_navigator', {}).get('ros__parameters', {}))
        
        # Set the dynamic values
        bt_xml_path = os.path.join(get_package_share_directory('nav2_proximity_wait'), 'bt_xml', 'navigate_w_proximity_wait.xml')
        params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml_path
        params['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'] = bt_xml_path # Optional safety
        params['bt_navigator']['ros__parameters']['other_robot_frames'] = other_frames_str
        params['bt_navigator']['ros__parameters']['safety_radius'] = float(safety_radius)
        params['bt_navigator']['ros__parameters']['clear_radius'] = float(clear_radius)
        
        # Write merged params to temp file (persists across the launch due to delete=False)
        temp_file = tempfile.NamedTemporaryFile(mode='w', prefix=f"{ns}_nav2_params_", suffix='.yaml', delete=False)
        yaml.dump(params, temp_file)
        temp_file.close()

        navigation = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('swarm_description'), 'launch', 'bringup.launch.py')]),
            launch_arguments={
                'use_sim_time': 'true',
                'map': map_path_evaluated,
                'params_file': temp_file.name,
                'use_namespace': 'true',
                'namespace': ns,
            }.items(),
        )
        return [navigation]

    nav_setup = OpaqueFunction(function=launch_setup)
    
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
        nav_setup,
        delayed_map_tf,        
    ])

