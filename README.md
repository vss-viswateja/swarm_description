# swarm_description

URDF/Xacro robot description package for multi-robot swarm systems featuring Jackal UGV, UR5 robotic arm, and composite mobile manipulator platforms. This package provides complete robot models, simulation configurations, and launch infrastructure for Gazebo (Ignition) simulation with ROS 2 Humble.

> **Note**: This package is part of the work-in-progress CHARS (Collaborative Heterogeneous Autonomous Robot Swarm) architecture.

## Features

- **Jackal UGV Model**: Complete URDF with differential drive, IMU, LiDAR, and camera sensors
- **UR5 Robotic Arm**: 6-DOF manipulator with ros2_control integration
- **Mobile Manipulator**: Composite platform combining Jackal base + UR5 arm
- **Multi-Robot Namespacing**: TF frame prefixing for swarm compatibility
- **ros2_control Integration**: GazeboSimSystem hardware interface for all platforms
- **Nav2 Ready**: Pre-configured navigation stack for autonomous navigation
- **MoveIt2 Support**: Motion planning configurations for UR5 arm
- **Sensor Suite**: IMU, 2D LiDAR, RGB camera with Gazebo plugins
- **RViz Configurations**: Custom visualization configs for each robot type

## Package Contents

```
swarm_description/
├── urdf/
│   ├── jackal.urdf.xacro               # Main Jackal UGV model
│   ├── jackal_accessories.urdf.xacro   # Sensor and accessory mounts
│   ├── jackal_ros2_control.xacro       # Jackal ros2_control config
│   ├── jackal.gazebo                   # Gazebo plugins for Jackal
│   ├── ur5_assembly.xacro              # UR5 arm standalone model
│   ├── ur5_ros2_control.xacro          # UR5 ros2_control config
│   ├── ur5_gazebo.xacro                # Gazebo plugins for UR5
│   ├── mobile_manipulator.urdf.xacro   # Jackal + UR5 composite
│   └── materials.xacro                 # Material definitions
├── launch/
│   ├── display.launch.py               # RViz visualization (standalone)
│   ├── jackal_control.launch.py        # Jackal spawn + control
│   ├── jackal_nav2.launch.py           # Jackal with Nav2 stack
│   ├── ur5_control.launch.py           # UR5 spawn + control
│   ├── mobile_manipulator.launch.py    # Mobile manipulator spawn
│   ├── mobman_nav2.launch.py           # Mobile manipulator with Nav2
│   └── bringup.launch.py               # Generic Nav2 launcher
├── config/
│   ├── jackal_control.yaml             # Jackal controller params
│   ├── jackal_ekf.yaml                 # EKF localization config
│   ├── jackal_nav2_params.yaml         # Nav2 parameters for Jackal
│   ├── jackal_bridge.yaml              # Gazebo-ROS bridge for Jackal
│   ├── jackal.rviz                     # RViz config for Jackal
│   ├── ur5_control.yaml                # UR5 controller params
│   ├── ur5_bridge.yaml                 # Gazebo-ROS bridge for UR5
│   ├── ur5.rviz                        # RViz config for UR5
│   ├── mobman_control.yaml             # Mobile manipulator controllers
│   ├── mobman_ekf.yaml                 # Mobile manipulator EKF
│   ├── mobman_nav2_params.yaml         # Nav2 params for mobile manipulator
│   ├── mobman_bridge.yaml              # Bridge for mobile manipulator
│   └── mobman.rviz                     # RViz config for mobile manipulator
├── meshes/
│   ├── jackal-wheel.stl                # Jackal wheel mesh
│   ├── base_link.stl                   # UR5 base mesh
│   ├── link*_*.stl                     # UR5 link meshes
│   └── ...
└── map/
    ├── empty_world_map.pgm             # Nav2 occupancy grid
    └── empty_world_map.yaml            # Map metadata
```

## Robot Specifications

### Jackal UGV
- **Drivetrain**: 4-wheel differential drive (skid-steer)
- **Dimensions**: 0.42m (L) × 0.31m (W) × 0.184m (H)
- **Wheelbase**: 0.262m
- **Track Width**: 0.376m
- **Wheel Radius**: 0.098m
- **Sensors**:
  - IMU (Gazebo IMU sensor)
  - 2D LiDAR (270° FOV, 30m range)
  - RGB Camera (640×480, 60° HFOV)

### UR5 Robotic Arm
- **DOF**: 6 revolute joints
- **Reach**: ~850mm
- **Payload**: ~5kg
- **Joint Limits**: Configured in URDF
- **Control Mode**: Position control via ros2_control

### Mobile Manipulator
- **Base**: Jackal UGV
- **Manipulator**: UR5 arm mounted on chassis center
- **Mount Height**: 0.01m above mid_mount
- **Combined Control**: Separate controllers for base (diff_drive) and arm (position)

## Dependencies

### ROS 2 Packages
```bash
# Core
ros-humble-robot-state-publisher
ros-humble-joint-state-publisher
ros-humble-joint-state-publisher-gui
ros-humble-xacro

# Simulation
ros-humble-ros-gz-sim
ros-humble-ros-gz-bridge

# Control
ros-humble-ros2-control
ros-humble-ros2-controllers
ros-humble-controller-manager
ros-humble-diff-drive-controller
ros-humble-joint-state-broadcaster

# Navigation
ros-humble-nav2-bringup
ros-humble-robot-localization

# Visualization
ros-humble-rviz2
```

### Installation

1. **Install ROS 2 Humble** (if not already installed):
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

2. **Install dependencies**:
```bash
# Robot description and visualization
sudo apt install ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher \
                 ros-humble-joint-state-publisher-gui \
                 ros-humble-xacro \
                 ros-humble-rviz2

# Simulation
sudo apt install ros-humble-ros-gz \
                 ros-humble-ros-gz-sim \
                 ros-humble-ros-gz-bridge

# Control
sudo apt install ros-humble-ros2-control \
                 ros-humble-ros2-controllers \
                 ros-humble-controller-manager \
                 ros-humble-diff-drive-controller \
                 ros-humble-joint-state-broadcaster \
                 ros-humble-joint-trajectory-controller

# Navigation
sudo apt install ros-humble-nav2-bringup \
                 ros-humble-navigation2 \
                 ros-humble-robot-localization

# SLAM (optional)
sudo apt install ros-humble-slam-toolbox
```

3. **Clone and build workspace**:
```bash
cd ~/swarm_ws/src
# Clone your repository here

cd ~/swarm_ws
colcon build --packages-select swarm_description
source install/setup.bash
```

## Usage

### Display Robot Models in RViz

Visualize robot models without simulation:

```bash
# UR5 arm only
ros2 launch swarm_description display.launch.py

# Jackal UGV (requires running display with Jackal model)
ros2 launch swarm_description display.launch.py
```

**Interactive Joint Control**: Use the `joint_state_publisher_gui` window to move joints in real-time.

### Jackal UGV Simulation

#### Basic Spawn with Control
```bash
# Single Jackal with default namespace
ros2 launch swarm_description jackal_control.launch.py

# Custom namespace for multi-robot
ros2 launch swarm_description jackal_control.launch.py \
    robot_namespace:=robot1

# Custom spawn position
ros2 launch swarm_description jackal_control.launch.py \
    robot_namespace:=robot1 \
    pose_x:=1.0 \
    pose_y:=2.0 \
    pose_z:=0.22
```

**Default Topics** (with namespace `robot1`):
- `/robot1/cmd_vel` - Velocity commands
- `/robot1/odom` - Odometry
- `/robot1/scan` - LiDAR data
- `/robot1/imu` - IMU data
- `/robot1/camera/image` - Camera feed

#### Jackal with Nav2 Navigation
```bash
# Launch with navigation stack
ros2 launch swarm_description jackal_nav2.launch.py \
    robot_namespace:=robot1 \
    map_path:=/path/to/map.yaml

# Test navigation with goal
ros2 topic pub /robot1/goal_pose geometry_msgs/msg/PoseStamped \
    '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}}}'
```

**Launch Arguments**:
- `robot_namespace` - Namespace for TF frames and topics (default: `jackal`)
- `map_path` - Path to Nav2 map YAML file
- `nav2_params_path` - Custom Nav2 parameter file
- `pose_x/y/z` - Initial spawn position

### UR5 Arm Simulation

```bash
# Spawn UR5 with ros2_control
ros2 launch swarm_description ur5_control.launch.py

# Custom namespace
ros2 launch swarm_description ur5_control.launch.py \
    robot_namespace:=ur5_robot1

# Custom position
ros2 launch swarm_description ur5_control.launch.py \
    spawn_x:=1.0 \
    spawn_y:=0.0 \
    spawn_z:=0.125
```

**Control the arm**:
```bash
# List controllers
ros2 control list_controllers --controller-manager /ur5/controller_manager

# Send joint commands via action (requires joint_trajectory_controller)
ros2 action send_goal /ur5/arm_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory ...
```

### Mobile Manipulator Simulation

```bash
# Spawn mobile manipulator (Jackal + UR5)
ros2 launch swarm_description mobile_manipulator.launch.py

# With custom namespace
ros2 launch swarm_description mobile_manipulator.launch.py \
    robot_namespace:=mobman1
```

**Separate Controller Namespaces**:
- Base: `/mobman1/diff_drive_controller`
- Arm: `/mobman1/arm_controller`

```bash
# Control base movement
ros2 topic pub /mobman1/cmd_vel geometry_msgs/msg/Twist \
    '{linear: {x: 0.5}, angular: {z: 0.0}}'

# Check arm joint states
ros2 topic echo /mobman1/joint_states
```

### Multi-Robot Spawning

```bash
# Terminal 1: Robot 1
ros2 launch swarm_description jackal_control.launch.py \
    robot_namespace:=robot1 pose_x:=0.0 pose_y:=-3.0

# Terminal 2: Robot 2
ros2 launch swarm_description jackal_control.launch.py \
    robot_namespace:=robot2 pose_x:=0.0 pose_y:=-2.0

# Terminal 3: Robot 3
ros2 launch swarm_description jackal_control.launch.py \
    robot_namespace:=robot3 pose_x:=0.0 pose_y:=-1.0
```

**TF Tree**: Each robot has isolated frames:
- `robot1/base_link`, `robot1/odom`, `robot1/laser_link`, etc.
- `robot2/base_link`, `robot2/odom`, `robot2/laser_link`, etc.

## Architecture Details

### Multi-Robot Namespace Strategy

All URDF files support `robot_namespace` argument for TF isolation:

```xml
<xacro:arg name="robot_namespace" default="" />
<xacro:property name="ns_prefix" value="" />

<!-- Links automatically prefixed -->
<link name="${ns_prefix}base_link">...</link>
<joint name="${ns_prefix}wheel_joint">...</joint>
```

**Launch Parameter**:
```python
robot_description = ParameterValue(
    Command(['xacro ', xacro_path, ' robot_namespace:=', robot_namespace]), 
    value_type=str
)
```

### ros2_control Configuration

#### Jackal (Differential Drive)
```yaml
controller_manager:
  diff_drive_controller:
    type: diff_drive_controller/DiffDriveController
    left_wheel_names: ["front_left_wheel", "rear_left_wheel"]
    right_wheel_names: ["front_right_wheel", "rear_right_wheel"]
    wheel_separation: 0.31
    wheel_radius: 0.098
```

**Hardware Interface**: `gz_ros2_control/GazeboSimSystem`

#### UR5 (Position Control)
```yaml
controller_manager:
  arm_controller:
    type: joint_trajectory_controller/JointTrajectoryController
    joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

### Sensor Plugins (Gazebo)

**IMU Sensor**:
- Topic: `/robot_namespace/imu`
- Rate: 100 Hz
- Outputs: Orientation, angular velocity, linear acceleration

**2D LiDAR**:
- Topic: `/robot_namespace/scan`
- FOV: 270° (4.7 rad)
- Range: 0.1m - 30m
- Resolution: 0.25° (720 samples)

**Camera**:
- Topic: `/robot_namespace/camera/image`
- Resolution: 640×480
- HFOV: 1.047 rad (60°)
- Format: RGB8

### EKF Sensor Fusion

Robot localization via Extended Kalman Filter (EKF):

```yaml
# Fuse wheel odometry + IMU
odom0: /robot1/odom
imu0: /robot1/imu

# Output fused odometry
odom_frame: robot1/odom
base_link_frame: robot1/base_link
world_frame: robot1/odom
```

Configuration files: [jackal_ekf.yaml](config/jackal_ekf.yaml), [mobman_ekf.yaml](config/mobman_ekf.yaml)

## Configuration Files

### Controller Configurations
- **jackal_control.yaml**: Diff drive controller for Jackal UGV
- **ur5_control.yaml**: Joint trajectory controller for UR5 arm
- **mobman_control.yaml**: Combined controllers for mobile manipulator

### Navigation Parameters
- **jackal_nav2_params.yaml**: Nav2 stack tuned for Jackal (costmaps, planners, controllers)
- **mobman_nav2_params.yaml**: Nav2 for mobile manipulator platform

### Gazebo Bridges
- **jackal_bridge.yaml**: Clock, IMU, LiDAR, camera topics
- **ur5_bridge.yaml**: Clock, joint states, arm-specific sensors
- **mobman_bridge.yaml**: Combined bridge for mobile manipulator

### RViz Configurations
- **jackal.rviz**: Jackal visualization with TF tree, laser scans, camera
- **ur5.rviz**: UR5 arm with joint states and planning scene
- **mobman.rviz**: Mobile manipulator composite view
- **jackal_nav2.rviz**: Navigation-specific displays (costmaps, paths, goals)

## Testing Commands

### Check Robot Description
```bash
# Generate URDF from Xacro
ros2 run xacro xacro ~/swarm_ws/src/swarm_description/urdf/jackal.urdf.xacro \
    robot_namespace:=robot1 > /tmp/robot1.urdf

# Check URDF validity
check_urdf /tmp/robot1.urdf

# View TF tree
ros2 run tf2_tools view_frames
evince frames.pdf
```

### Controller Management
```bash
# List available controllers
ros2 control list_controllers --controller-manager /robot1/controller_manager

# Load controller manually
ros2 control load_controller diff_drive_controller \
    --controller-manager /robot1/controller_manager

# Activate controller
ros2 control set_controller_state diff_drive_controller active \
    --controller-manager /robot1/controller_manager

# Check controller status
ros2 control list_hardware_interfaces --controller-manager /robot1/controller_manager
```

### Navigation Testing
```bash
# Check map
ros2 topic echo /robot1/map --once

# Monitor AMCL pose
ros2 topic echo /robot1/amcl_pose

# Visualize costmaps
ros2 run nav2_costmap_2d nav2_costmap_2d_markers \
    voxel_grid:=/robot1/global_costmap/voxel_grid \
    visualization_marker:=/robot1/global_costmap_marker

# Send simple goal
ros2 action send_goal /robot1/navigate_to_pose \
    nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}}}}"
```

### Sensor Data Inspection
```bash
# LiDAR scan
ros2 topic echo /robot1/scan --once

# IMU data
ros2 topic echo /robot1/imu

# Camera feed (view in RViz or rqt_image_view)
ros2 run rqt_image_view rqt_image_view /robot1/camera/image

# Joint states (for UR5)
ros2 topic echo /ur5/joint_states
```

## Troubleshooting

### Models Not Loading in Gazebo

**Issue**: Missing mesh files or resource path errors.

**Solution**:
```bash
# Verify mesh files exist
ls $(ros2 pkg prefix swarm_description)/share/swarm_description/meshes/

# Check resource path
echo $GZ_SIM_RESOURCE_PATH
echo $IGN_GAZEBO_RESOURCE_PATH

# Rebuild with install
cd ~/swarm_ws
colcon build --packages-select swarm_description --symlink-install
source install/setup.bash
```

### Controllers Not Starting

**Issue**: `[ERROR] Controller 'diff_drive_controller' not loaded`.

**Solution**:
```bash
# Check controller manager running
ros2 node list | grep controller_manager

# Verify controller params loaded
ros2 param list /robot1/controller_manager

# Check hardware interface
ros2 control list_hardware_interfaces --controller-manager /robot1/controller_manager

# Manually spawn controller
ros2 run controller_manager spawner diff_drive_controller \
    --controller-manager /robot1/controller_manager
```

### TF Frame Errors

**Issue**: `Frame [robot1/base_link] does not exist`.

**Solution**:
```bash
# Check robot_state_publisher running
ros2 node list | grep robot_state_publisher

# Verify robot_description parameter
ros2 param get /robot1/robot_state_publisher robot_description

# Check TF tree
ros2 run tf2_ros tf2_echo map robot1/base_link

# List all frames
ros2 run tf2_tools view_frames
```

### Navigation Failures

**Issue**: Robot not planning paths or getting stuck.

**Solution**:
1. **Check localization**:
   ```bash
   ros2 topic echo /robot1/amcl_pose
   ```

2. **Verify map loaded**:
   ```bash
   ros2 topic echo /robot1/map --once
   ```

3. **Inspect costmaps** in RViz:
   - Add "Map" display for `/robot1/local_costmap/costmap`
   - Check for inflation layer issues

4. **Tune Nav2 parameters**: Edit `jackal_nav2_params.yaml`:
   - Reduce `inflation_radius` if robot too conservative
   - Increase `transform_tolerance` if TF errors
   - Adjust `goal_checker` thresholds

### Joint Limits Exceeded (UR5)

**Issue**: MoveIt planning fails with joint limit violations.

**Solution**:
```bash
# Check current joint states
ros2 topic echo /ur5/joint_states

# Verify limits in URDF
ros2 run xacro xacro $(ros2 pkg prefix swarm_description)/share/swarm_description/urdf/ur5_assembly.xacro | grep limit

# Reset to home position
ros2 service call /ur5/controller_manager/switch_controller \
    controller_manager_msgs/srv/SwitchController ...
```

## Development Roadmap (CHARS Architecture)

- [ ] Add Velodyne 3D LiDAR sensor model
- [ ] Implement gripper attachment for UR5 arm
- [ ] Create custom sensor suite configurations
- [ ] Add battery simulation and power management
- [ ] Develop swarm-specific communication models
- [ ] Integrate semantic segmentation cameras
- [ ] Create modular accessory mounting system
- [ ] Add thermal and depth camera sensors

## Best Practices

1. **Namespacing**: Always use unique `robot_namespace` for multi-robot scenarios
2. **Frame Prefixes**: Ensure `frame_prefix` parameter set in `robot_state_publisher`
3. **Controller Scoping**: Nest controller params under `/**:` for namespace-agnostic configs
4. **Mesh Paths**: Use `file://$(find package)` format for Gazebo compatibility
5. **TF Isolation**: Verify each robot has isolated TF tree with `view_frames`

## License

Apache License 2.0

## Maintainer

**Viswa Teja Bottu**  
Email: vss.viswatejabottu@gmail.com

## Contributing

This package is part of ongoing research in heterogeneous multi-robot systems. For questions or collaboration opportunities, please contact the maintainer.

## See Also

- [swarm_bringup](../swarm_bringup/) - Multi-robot launch orchestration
- [ur5_moveit_config](../ur5_moveit_config/) - MoveIt2 configuration for UR5
- [plansys2_turtlesim_example](../plansys2_turtlesim_example/) - VLM-powered planning
- [ROS 2 URDF Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [ros2_control Documentation](https://control.ros.org/humble/index.html)
- [Nav2 Documentation](https://navigation.ros.org/)
