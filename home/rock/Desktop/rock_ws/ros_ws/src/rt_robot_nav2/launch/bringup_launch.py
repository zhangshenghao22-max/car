# Main bringup launch file for rt_robot_nav2
# Integrates with existing slam_gmapping and chassis_controller system

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, TimerAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import PushRosNamespace, Node


def generate_launch_description():
    # Get the launch directory
    rt_robot_nav2_dir = get_package_share_directory('rt_robot_nav2')
    robot_urdf_dir = get_package_share_directory('robot_urdf')
    chassis_controller_dir = get_package_share_directory('chassis_controller')
    imu_dir = get_package_share_directory('imu_ros2_device')
    launch_dir = os.path.join(rt_robot_nav2_dir, 'launch')
    param_dir = os.path.join(rt_robot_nav2_dir, 'param')
    config_dir = os.path.join(rt_robot_nav2_dir, 'config')
    param_file = 'rt_robot_params.yaml'
    bt_file = os.path.join(param_dir, 'navigate_w_replanning_time.xml')
    map_dir = os.path.join(rt_robot_nav2_dir, 'map')
    map_file = 'rt_robot_map.yaml'

    # URDF file path (from robot_urdf package)
    urdf_file_name = LaunchConfiguration('urdf_file')
    urdf_file = PathJoinSubstitution([robot_urdf_dir, 'urdf', urdf_file_name])

    # Parameter files for sensors
    # Use unified chassis_controller.yaml (mode will be overridden to "navigation" for navigation mode)
    config_file = os.path.join(chassis_controller_dir, 'config', 'chassis_controller.yaml')
    ekf_params_file = os.path.join(config_dir, 'f103_ekf.yaml')
    imu_filter_params_file = os.path.join(imu_dir, 'config', 'imu_filter_param.yaml')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_slam = LaunchConfiguration('use_slam', default='false')
    use_chassis_controller = LaunchConfiguration('use_chassis_controller')
    use_ekf = LaunchConfiguration('use_ekf')
    use_auto_initialpose = LaunchConfiguration('use_auto_initialpose')
    params_file = LaunchConfiguration('params_file')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    autostart = LaunchConfiguration('autostart')
    open_rviz = LaunchConfiguration('open_rviz')
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')
    rmw_envvar = SetEnvironmentVariable(
        'RMW_IMPLEMENTATION',
        EnvironmentVariable('RMW_IMPLEMENTATION', default_value='rmw_fastrtps_cpp'))
    fastdds_profile_envvar = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        EnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            default_value='/home/rock/Desktop/rock_ws/fastdds_no_shm.xml'))

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

    declare_use_ekf_cmd = DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='Fuse /odom_raw and /imu/data into /odom with robot_localization')

    declare_use_maze_turn_manager_cmd = DeclareLaunchArgument(
        'use_maze_turn_manager',
        default_value='true',
        description='Compatibility argument only; SDK1 navigation does not use maze turn manager')

    localization_cmd_group = GroupAction([
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
    ])

    navigation_cmd_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

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
        # Read the requested URDF file from robot_urdf package.
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(Command(['cat ', urdf_file]), value_type=str),
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

        # Fuse Yahboom IMU raw data into /imu/data for diagnostics and yaw-rate calibration.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[imu_filter_params_file],
            condition=UnlessCondition(use_slam)
        ),

        # Keep raw /imu/data unchanged; EKF consumes calibrated yaw rate only.
        Node(
            package='rt_robot_nav2',
            executable='imu_z_calibrator.py',
            name='imu_z_calibrator',
            output='screen',
            parameters=[{
                'input_topic': '/imu/data',
                'output_topic': '/imu/data_calibrated',
                'imu_z_scale': 1.05502,
                'imu_z_bias': 0.0,
                'imu_yaw_rate_variance': 0.0025,
            }],
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

        # Costmaps use filtered scans to ignore near self-echoes; AMCL still uses raw /scan.
        Node(
            package='rt_robot_nav2',
            executable='costmap_scan_range_filter.py',
            name='costmap_scan_range_filter',
            output='screen',
            parameters=[{
                'input_topic': '/scan',
                'output_topic': '/scan_costmap',
                'min_obstacle_range': 0.18,
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

        # Static TF: base_link -> laser (C26 lidar is on the +x front end)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=['0.13', '0', '0.12', '0', '0', '0', 'base_link', 'laser'],
            output='screen',
            condition=UnlessCondition(use_slam)
        ),

        # Static TF: base_link -> imu_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
            output='screen',
            condition=UnlessCondition(use_slam)
        ),

        # EKF publishes /odom and odom -> base_footprint from F103 /odom_raw + calibrated IMU yaw rate.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odom')],
            condition=IfCondition(PythonExpression([
                "'", use_slam, "' != 'true' and '", use_ekf, "' == 'true'"
            ]))
        ),

        # Legacy odom_fusion is disabled when EKF is enabled.
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
                use_slam, "' != 'true' and '",
                use_ekf, "' != 'true'"
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
                'tf_time_offset_s': 0.0,
            }],
            # Only enable when use_chassis_controller=true AND not using SLAM
            # This prevents TF conflicts with odom_fusion
            # NOTE: By default, use_chassis_controller is false, so this node won't launch
            # If you need odometry_to_tf, set use_chassis_controller:=true AND ensure odom_fusion is disabled
            condition=IfCondition(PythonExpression([
                "'", use_chassis_controller, "' == 'true' and '",
                use_slam, "' != 'true' and '",
                use_ekf, "' != 'true'"
            ]))
        ),

    ])

    auto_initialpose_node = Node(
        package='rt_robot_nav2',
        executable='auto_initialpose.py',
        name='auto_initialpose',
        output='screen',
        parameters=[{
            'map_yaml': map_yaml_file,
            'frame_id': 'map',
            'publish_delay_sec': 2.0,
            'publish_repeats': 3,
            'repeat_interval_sec': 0.5,
            'tf_wait_timeout_sec': 10.0,
        }],
        condition=IfCondition(PythonExpression([
            "'", use_slam, "' != 'true' and '", use_auto_initialpose, "' == 'true'"
        ]))
    )

    delayed_localization = TimerAction(
        period=3.0,
        actions=[localization_cmd_group]
    )

    delayed_auto_initialpose = TimerAction(
        period=6.0,
        actions=[auto_initialpose_node]
    )

    delayed_navigation = TimerAction(
        period=18.0,
        actions=[navigation_cmd_group]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(rmw_envvar)
    ld.add_action(fastdds_profile_envvar)

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
    ld.add_action(declare_use_ekf_cmd)
    ld.add_action(declare_use_maze_turn_manager_cmd)

    # Add sensor nodes (before navigation nodes to ensure TF tree is ready)
    ld.add_action(sensor_nodes)

    # Start localization first, publish initial pose, then start navigation servers.
    ld.add_action(delayed_localization)
    ld.add_action(delayed_auto_initialpose)
    ld.add_action(delayed_navigation)

    return ld
