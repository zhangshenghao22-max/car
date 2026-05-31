from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression
import launch.actions
import launch.conditions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_chassis_controller = LaunchConfiguration('use_chassis_controller', default='false')
    use_odom_fusion = LaunchConfiguration('use_odom_fusion', default='true')
    open_rviz = LaunchConfiguration('open_rviz', default='true')
    # Get package directories
    chassis_pkg_share = get_package_share_directory('chassis_controller')
    rt_robot_nav2_pkg_share = get_package_share_directory('rt_robot_nav2')
    imu_pkg_share = get_package_share_directory('dm_imu')
    lslidar_pkg_share = get_package_share_directory('lslidar_driver')
    robot_urdf_pkg_share = get_package_share_directory('robot_urdf')

    # Configuration files
    # Use unified velocity_limits.yaml for speed limits
    velocity_limits_file = os.path.join(rt_robot_nav2_pkg_share, 'config', 'velocity_limits.yaml')
    default_config_file = os.path.join(chassis_pkg_share, 'config', 'chassis_controller.yaml')
    config_file = LaunchConfiguration('config_file', default=default_config_file)
    
    # Parameter files
    imu_params_file = os.path.join(imu_pkg_share, 'config', 'params.yaml')
    lslidar_params_file = os.path.join(lslidar_pkg_share, 'params', 'lsn10p.yaml')
    
    # URDF file path for robot model (from robot_urdf package)
    # Default to mini_mec_robot.urdf for mecanum wheel robot
    urdf_file = os.path.join(robot_urdf_pkg_share, 'urdf', 'mini_mec_robot.urdf')
    
    # Joint State Publisher (publishes joint states for continuous joints)
    # Required for robot_state_publisher to publish TF for wheel links
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }]
    )
    
    # Robot State Publisher (publishes robot model to /robot_description)
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read() if os.path.exists(urdf_file) else '',
            'use_sim_time': use_sim_time
        }]
    )
    
    # Launch IMU node
    imu_node = launch_ros.actions.Node(
        package='dm_imu',
        executable='dm_imu_node',
        name='dm_imu',
        output='screen',
        parameters=[imu_params_file]
    )
    
    # Launch LiDAR driver node (regular Node, not LifecycleNode)
    lslidar_node = launch_ros.actions.Node(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[lslidar_params_file],
    )
    
    # Standard TF tree: map -> odom -> base_footprint -> base_link -> laser
    # base_footprint: located at ground level (z=0), used for navigation
    # base_link: located at robot center (may have height), used for physical model
    
    # Static TF publisher: base_footprint -> base_link (if base_link is at ground, use identity transform)
    # If robot center is at ground, this is (0, 0, 0, 0, 0, 0, 1)
    # If robot center is elevated, adjust z value accordingly
    base_footprint_to_base_link_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        output='screen'
    )
    
    # Static TF publisher: base_link -> laser (laser mounted 0.2m above base)
    base_to_laser_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.2', '0', '0', '0', 'base_link', 'laser'],
        output='screen'
    )
    
    # Static TF publisher: base_link -> imu_link (optional, for IMU visualization)
    base_to_imu_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )
    
    # Odometry fusion node: publishes odom->base_footprint TF
    # - IMU provides yaw angle with low-pass filtering
    # - Velocity integration calculates position (to avoid circular dependency with SLAM)
    # - SLAM provides map->odom transform for global positioning
    # - Reserved: Encoder odometry interface for future use
    # Override mode to "slam" for SLAM mode
    odom_fusion_node = launch_ros.actions.Node(
        package='chassis_controller',
        executable='odom_fusion',
        name='odom_fusion',
        output='screen',
        parameters=[
            config_file,
            {'odom_fusion.mode': 'slam'}  # Override mode to "slam" for SLAM mode
                                            # Use default max_yaw_rate (8.0 rad/s) from config file
        ],
        condition=launch.conditions.IfCondition(use_odom_fusion)
    )
    
    # Obstacle avoidance node (highest priority safety mechanism)
    # This node monitors laser scan data and immediately stops the robot if obstacles are detected
    # It publishes stop commands to /cmd_vel with highest priority
    obstacle_avoidance_node = launch_ros.actions.Node(
        package='chassis_controller',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[
            config_file,
            {'obstacle_avoidance.enable_in_slam': True}  # Enable in SLAM mode
        ]
    )
    
    # Fallback: Static TF publisher (only used if fusion and chassis controller are both disabled)
    # In ROS2 Foxy, we use PythonExpression to combine conditions (AND logic)
    # Static TF is only used when both use_odom_fusion and use_chassis_controller are false
    odom_to_base_tf_static_condition = PythonExpression([
        "'", use_odom_fusion, "' == 'false' and '",
        use_chassis_controller, "' == 'false'"
    ])
    odom_to_base_footprint_tf_static = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint_tf_static',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint'],
        output='screen',
        condition=launch.conditions.IfCondition(odom_to_base_tf_static_condition)
    )
    
    # Chassis controller nodes (for micro-ROS communication with RT-Thread)
    # Disabled by default - only enable when RT-Thread is connected
    # Odometry to TF converter: converts /odom topic to TF transform (odom -> base_footprint)
    # Note: z coordinate should be 0 for base_footprint (ground level)
    odometry_to_tf_node = launch_ros.actions.Node(
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
        condition=launch.conditions.IfCondition(use_chassis_controller)
    )
    
    # Odometry subscriber (optional, for monitoring)
    odometry_subscriber_node = launch_ros.actions.Node(
        package='chassis_controller',
        executable='odometry_subscriber',
        name='odometry_subscriber',
        output='screen',
        parameters=[{
            'odom_topic': '/odom',
            'verbose': False,
        }],
        condition=launch.conditions.IfCondition(use_chassis_controller)
    )
    
    # Keyboard teleop node (for controlling the chassis)
    # Publishes to /cmd_vel_cmd which goes through obstacle_avoidance node
    # Loads velocity limits from unified velocity_limits.yaml
    keyboard_teleop_node = launch_ros.actions.Node(
        package='chassis_controller',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[
            velocity_limits_file,  # Load unified velocity limits FIRST
            {  # Override with specific settings
                'chassis_controller.cmd_vel_topic': '/cmd_vel_cmd',  # Changed: publish to obstacle_avoidance input
            }
        ],
        condition=launch.conditions.IfCondition(use_chassis_controller)
    )
    
    # Group TF publishers together to ensure they start before other nodes
    # This helps prevent "Unknown frame" errors in RViz
    tf_group = GroupAction([
        base_footprint_to_base_link_tf,  # base_footprint -> base_link
        base_to_laser_tf,  # base_link -> laser
        base_to_imu_tf,  # base_link -> imu_link
        odom_to_base_footprint_tf_static,  # Fallback static TF (only if fusion disabled)
    ])
    
    # SLAM gmapping node
    slam_gmapping_node = launch_ros.actions.Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'scan_topic': '/scan',
            'base_frame': 'base_footprint',
            'map_frame': 'map',
            'odom_frame': 'odom',
            'transform_publish_period': 0.05,  # 20 Hz for map->odom TF updates
            'map_update_interval': 2.0,  # Update map every 5 seconds
            'linearUpdate': 0.2,  # Process scan after moving 0.5m
            'angularUpdate': 0.2,  # Process scan after rotating 0.5 rad (~28 degrees)
            'temporalUpdate': -1.0,  # Disable temporal updates
            'particles': 80,  # Reduced particles for faster processing
            'delta': 0.05,  # Map resolution (meters per pixel)
            'llsampleGain': 0.5,  # Likelihood sampling gain
            'lsigma': 0.05,  # Likelihood sigma for scan matching
            'ogain': 3.0,  # Optimization gain
            'lstep': 0.05,  # Linear search step
            'astep': 0.05,  # Angular search step
            'iterations': 10,  # Scan matching iterations
            'srr': 0.1,  # Translation noise
            'srt': 0.2,  # Rotation noise from translation
            'str': 0.2,  # Translation noise from rotation
            'stt': 0.1  # Rotation noise
        }]
    )
    
    # RViz2 for visualization
    slam_gmapping_pkg_share = get_package_share_directory('slam_gmapping')
    rviz_config_file = os.path.join(
        slam_gmapping_pkg_share,
        'rviz',
        'slam_gmapping.rviz'
    )
    # Fallback to lslidar config if slam_gmapping config doesn't exist
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(
            get_package_share_directory('lslidar_driver'),
            'rviz',
            'lslidar.rviz'
        )
    
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=launch.conditions.IfCondition(open_rviz)
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time'),
        DeclareLaunchArgument('use_chassis_controller', default_value='false', 
                            description='Enable chassis controller (odometry from RT-Thread)'),
        DeclareLaunchArgument('use_odom_fusion', default_value='true',
                            description='Enable odometry fusion (IMU + SLAM)'),
        DeclareLaunchArgument('config_file', default_value=default_config_file,
                            description='Unified parameter file (odom fusion, encoder开关等)'),
        DeclareLaunchArgument('open_rviz', default_value='true',
                            description='Launch RViz'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # Start joint state publisher first (required for continuous joints)
        joint_state_publisher_node,
        # Start robot state publisher (for robot model visualization and TF publishing)
        robot_state_publisher_node,
        # Start sensor nodes
        imu_node,
        lslidar_node,
        # Start TF publishers before nodes that depend on them
        # This ensures TF tree is established before RViz and SLAM try to use it
        # Static TF publishers (base_link->laser, base_link->imu_link)
        tf_group,
        # Odometry fusion node (fuses IMU + SLAM, with encoder interface reserved)
        # Must start BEFORE SLAM to provide odom->base_link TF that SLAM needs
        # Fusion node publishes initial identity transform immediately on startup
        odom_fusion_node,
        # Start SLAM node (needs odom->base_link TF from fusion node)
        slam_gmapping_node,
        # Chassis controller nodes (optional, for RT-Thread communication)
        odometry_to_tf_node,
        odometry_subscriber_node,
        keyboard_teleop_node,
        # Start RViz last (depends on all other nodes and TF tree)
        rviz_node,
    ])
