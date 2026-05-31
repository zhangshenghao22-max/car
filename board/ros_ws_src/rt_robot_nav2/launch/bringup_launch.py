# Main bringup launch file for rt_robot_nav2
# Integrates with existing slam_gmapping and chassis_controller system

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    # Get the launch directory
    rt_robot_nav2_dir = get_package_share_directory('rt_robot_nav2')
    robot_urdf_dir = get_package_share_directory('robot_urdf')
    chassis_controller_dir = get_package_share_directory('chassis_controller')
    imu_dir = get_package_share_directory('imu_ros2_device')
    lslidar_dir = get_package_share_directory('lslidar_driver')
    launch_dir = os.path.join(rt_robot_nav2_dir, 'launch')
    param_dir = os.path.join(rt_robot_nav2_dir, 'param')
    param_file = 'rt_robot_params.yaml'
    bt_file = os.path.join(param_dir, 'navigate_w_replanning_time.xml')
    map_dir = os.path.join(rt_robot_nav2_dir, 'map')
    map_file = 'rt_robot_map.yaml'
    
    # URDF file path (from robot_urdf package)
    # Default to mini_mec_robot.urdf for mecanum wheel robot
    urdf_file_name = LaunchConfiguration('urdf_file')
    # Default URDF file path
    default_urdf_file = os.path.join(robot_urdf_dir, 'urdf', 'mini_mec_robot.urdf')
    
    # Load default URDF file at launch time
    # For custom URDF file, user should specify urdf_file parameter
    try:
        with open(default_urdf_file, 'r') as f:
            default_urdf_content = f.read()
    except FileNotFoundError:
        default_urdf_content = ''
    
    # Parameter files for sensors
    # Use unified chassis_controller.yaml (mode will be overridden to "navigation" for navigation mode)
    config_file = os.path.join(chassis_controller_dir, 'config', 'chassis_controller.yaml')
    imu_filter_params_file = os.path.join(imu_dir, 'config', 'imu_filter_param.yaml')
    lslidar_params_file = os.path.join(lslidar_dir, 'params', 'lsn10p.yaml')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam', default='false')
    use_chassis_controller = LaunchConfiguration('use_chassis_controller')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    open_rviz = LaunchConfiguration('open_rviz')
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(map_dir, map_file),
        description='Full path to map yaml file to load (or filename relative to map/ directory)')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='false',
        description='Whether run a SLAM (use slam_gmapping)')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(param_dir, param_file),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=bt_file,
        description='Full path to the behavior tree xml file to use')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack')

    declare_open_rviz_cmd = DeclareLaunchArgument(
        'open_rviz',
        default_value='false',
        description='Launch Rviz?')
    
    declare_urdf_file_cmd = DeclareLaunchArgument(
        'urdf_file',
        default_value='mini_mec_robot.urdf',
        description='URDF file name from robot_urdf package (e.g., mini_mec_robot.urdf, senior_mec_robot.urdf)')

    # Optional: automatically publish initial pose from map yaml (when not using SLAM)
    declare_use_auto_initialpose_cmd = DeclareLaunchArgument(
        'use_auto_initialpose',
        default_value='true',
        description='Automatically publish initial pose from map yaml when using saved map (use_slam=false)')
    
    declare_use_chassis_controller_cmd = DeclareLaunchArgument(
        'use_chassis_controller',
        default_value='false',
        description='Enable chassis controller (odometry from RT-Thread via /odom topic)')

    # Specify the actions
    bringup_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        # Run Localization only when we don't use SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'localization_launch.py')),
            condition=UnlessCondition(use_slam),
            launch_arguments={'namespace': namespace,
                              'map': map_yaml_file,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        # Always run navigation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': bt_file,
                              'map_subscribe_transient_local': 'true'}.items()),
    ])
    
    # Sensor nodes and TF publishers (required for navigation)
    # Only launch when not using SLAM (SLAM launch already includes these)
    # Sensor nodes group
    sensor_nodes = GroupAction([
        # Joint State Publisher (publishes joint states for continuous joints)
        # Required for robot_state_publisher to publish TF for wheel links
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time
            }],
            condition=UnlessCondition(use_slam)
        ),
        
        # Robot State Publisher (publishes robot model to /robot_description)
        # Read URDF file from robot_urdf package
        # Uses default_urdf_content loaded at launch time
        # For custom URDF, user can specify urdf_file parameter (requires launch file modification)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': default_urdf_content,
                'use_sim_time': use_sim_time
            }],
            condition=UnlessCondition(use_slam)
        ),
        
        # Yahboom IMU serial driver (/dev/myimu -> /imu/data_raw)
        Node(
            package='imu_ros2_device',
            executable='ybimu_driver',
            name='ybimu_node',
            output='screen',
            condition=UnlessCondition(use_slam)
        ),

        # Fuse Yahboom IMU raw data into /imu/data for odom_fusion/Nav2.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[imu_filter_params_file],
            condition=UnlessCondition(use_slam)
        ),

        # RPLIDAR A1 driver
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            output='screen',
            emulate_tty=True,
            namespace='',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/serial/by-path/platform-xhci-hcd.11.auto-usb-0:1.4:1.0-port0',
                'serial_baudrate': 115200,
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Sensitivity',
            }],
            condition=UnlessCondition(use_slam)
        ),

        # Static TF: base_footprint -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_base_link_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen',
            condition=UnlessCondition(use_slam)
        ),

        # Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
            condition=UnlessCondition(use_slam)
        ),

        # Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0', '0', '0', '3.141592653589793', '0', '0', 'base_link', 'imu_link'],
            output='screen',
            condition=UnlessCondition(use_slam)
        ),

        # Odom fusion is disabled when using chassis /odom from F103.
        # F103 publishes /odom; odometry_to_tf below converts it to odom -> base_footprint.
        Node(
            package='chassis_controller',
            executable='odom_fusion',
            name='odom_fusion',
            output='screen',
            parameters=[
                config_file,
                {'odom_fusion.mode': 'navigation',
                 'odom_fusion.imu_yaw_weight': 0.7,
                 'odom_fusion.slam_yaw_weight': 0.7,
                 'odom_fusion.max_yaw_rate': 8.0,
                 'odom_fusion.use_velocity_integration': True,
                 'odom_fusion.cmd_vel_topic': '/cmd_vel'}
            ],
            condition=IfCondition(PythonExpression([
                "'", use_chassis_controller, "' != 'true' and '",
                use_slam, "' != 'true'"
            ]))
        ),

        # Obstacle avoidance node (DISABLED in navigation mode)
        # IMPORTANT: In navigation mode, Nav2's local_costmap + DWB controller handles obstacle avoidance
        # This node interferes with Nav2's control by blocking all commands when obstacles are detected
        # In SLAM mode (separate launch file), this node can be enabled for safety during mapping
        # Node(
        #     package='chassis_controller',
        #     executable='obstacle_avoidance',
        #     name='obstacle_avoidance',
        #     output='screen',
        #     parameters=[config_file],
        #     condition=UnlessCondition(use_slam)
        # ),

        # Goal stop publisher: sends stop command when navigation goal is reached
        # This ensures the robot comes to a complete stop after reaching the goal
        # Monitors the navigate_to_pose action status and publishes zero velocity on goal completion
        Node(
            package='chassis_controller',
            executable='goal_stop_publisher',
            name='goal_stop_publisher',
            output='screen',
            parameters=[
                config_file,
                {'cmd_vel_topic': '/cmd_vel',  # Override: Direct to chassis (obstacle_avoidance disabled)
                 'stop_duration': 0.5,  # Send stop for 0.5 seconds
                 'stop_repeats': 3,  # Repeat 3 times to ensure delivery
                 'action_status_topic': '/navigate_to_pose/_action/status',  # Monitor this action status
                 'min_goal_duration': 2.0}  # Only send stop if goal was active for >= 2 seconds
            ],
            condition=UnlessCondition(use_slam)
        ),

        # Odometry to TF converter: converts /odom topic to TF transform (odom -> base_footprint)
        # This is an alternative option when use_chassis_controller=true
        # If /odom topic exists, it provides position and orientation directly
        # NOTE: This is mutually exclusive with odom_fusion to avoid TF conflicts
        # CRITICAL: Only enable odometry_to_tf when BOTH:
        # 1. use_chassis_controller=true (RT-Thread provides /odom topic)
        # 2. NOT using SLAM (odom_fusion is only for SLAM mode in this configuration)
        Node(
            package='chassis_controller',
            executable='odometry_to_tf',
            name='odometry_to_tf',
            output='screen',
            parameters=[{
                'odom_topic': '/odom',
                'odom_frame': 'odom',
                'base_frame': 'base_footprint',
                'publish_tf': True,
                'publish_rate': 50.0,
                'publish_initial_tf': True,
            }],
            # Only enable when use_chassis_controller=true AND not using SLAM
            # This prevents TF conflicts with odom_fusion
            # NOTE: By default, use_chassis_controller is false, so this node won't launch
            # If you need odometry_to_tf, set use_chassis_controller:=true AND ensure odom_fusion is disabled
            condition=IfCondition(PythonExpression([
                "'", use_chassis_controller, "' == 'true' and '",
                use_slam, "' != 'true'"
            ]))
        ),

        # Auto initial pose: publish /initialpose once from map yaml when using saved map
        Node(
            package='rt_robot_nav2',
            executable='auto_initialpose.py',
            name='auto_initialpose',
            output='screen',
            parameters=[{
                'map_yaml': map_yaml_file,
                'frame_id': 'map',
                'publish_delay_sec': 5.0,  # Increased from 2.0 to 5.0 to allow AMCL to stabilize
            }],
            condition=UnlessCondition(use_slam)
        ),
    ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_open_rviz_cmd)
    ld.add_action(declare_use_auto_initialpose_cmd)
    ld.add_action(declare_urdf_file_cmd)
    ld.add_action(declare_use_chassis_controller_cmd)

    # Add sensor nodes (before navigation nodes to ensure TF tree is ready)
    ld.add_action(sensor_nodes)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd_group)

    return ld

