# Complete launch file for rt_robot_nav2
# Integrates SLAM, navigation, and all required nodes
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable, LogInfo)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    rt_robot_nav2_dir = get_package_share_directory('rt_robot_nav2')
    slam_gmapping_dir = get_package_share_directory('slam_gmapping')
    chassis_controller_dir = get_package_share_directory('chassis_controller')
    imu_dir = get_package_share_directory('dm_imu')
    lslidar_dir = get_package_share_directory('lslidar_driver')
    
    # Map directory for resolving relative paths
    map_dir = os.path.join(rt_robot_nav2_dir, 'map')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_chassis_controller = LaunchConfiguration('use_chassis_controller', default='false')
    use_odom_fusion = LaunchConfiguration('use_odom_fusion', default='true')
    use_slam = LaunchConfiguration('use_slam', default='true')
    use_nav = LaunchConfiguration('use_nav', default='false')
    map_file = LaunchConfiguration('map_file', default='')
    open_rviz = LaunchConfiguration('open_rviz', default='true')
    
    # Helper function to resolve map file path
    # This will be evaluated at launch time, not at Python execution time
    # We use PythonExpression to handle path resolution dynamically
    
    # Parameter files
    # Use unified chassis_controller.yaml (mode will be overridden to "navigation" for navigation mode)
    config_file = os.path.join(chassis_controller_dir, 'config', 'chassis_controller.yaml')
    imu_params_file = os.path.join(imu_dir, 'config', 'params.yaml')
    lslidar_params_file = os.path.join(lslidar_dir, 'params', 'lsn10p.yaml')
    
    # RViz config
    rviz_config_file = os.path.join(rt_robot_nav2_dir, 'rviz', 'rt_robot_nav2.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.join(slam_gmapping_dir, 'rviz', 'slam_gmapping.rviz')
    
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false',
                            description='Use simulation time'),
        DeclareLaunchArgument('use_chassis_controller', default_value='false',
                            description='Enable chassis controller (odometry from RT-Thread)'),
        DeclareLaunchArgument('use_odom_fusion', default_value='true',
                            description='Enable odometry fusion (IMU + SLAM)'),
        DeclareLaunchArgument('use_slam', default_value='true',
                            description='Enable SLAM (slam_gmapping)'),
        DeclareLaunchArgument('use_nav', default_value='false',
                            description='Enable Navigation2 (requires saved map if use_slam=false)'),
        DeclareLaunchArgument('map_file', default_value='',
                            description='Map file name (e.g., my_map.yaml) or full path. If relative, assumed to be in map/ directory'),
        DeclareLaunchArgument('open_rviz', default_value='true',
                            description='Launch RViz'),
        
        # Note: Sensor nodes and TF publishers are now managed by bringup_launch.py
        # This prevents duplicate node launches when use_nav=true
        
        # Warning: If both use_slam and use_nav are true, SLAM will run but navigation will use saved map
        # This is not recommended as it can cause conflicts. Navigation will force use_slam=false internally.
        LogInfo(
            msg='[WARNING] Both use_slam and use_nav are enabled. This may cause conflicts. Navigation will use saved map (use_slam=false internally).',
            condition=IfCondition(PythonExpression([
                "'", use_slam, "' == \"true\" and '", use_nav, "' == \"true\""
            ]))
        ),

        # Include SLAM launch (when use_slam=true)
        # Disable RViz in SLAM launch to avoid duplicate RViz windows
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_gmapping_dir, 'launch', 'slam_gmapping.launch.py')
            ),
            condition=IfCondition(use_slam),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_chassis_controller': use_chassis_controller,
                'use_odom_fusion': use_odom_fusion,
                # Use RViz from SLAM only when nav is disabled
                # Foxy: PythonExpression expects Python syntax, compare strings to 'true'/'false'
                'open_rviz': PythonExpression([
                    "'", open_rviz, "' == \"true\" and '", use_nav, "' == \"false\""
                ]),
            }.items()
        ),
        
        # Include Navigation launch (when use_nav=true)
        # Resolve map file path: if map_file is provided and not absolute, join with map_dir
        # Create a helper function to resolve map path at launch time
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rt_robot_nav2_dir, 'launch', 'bringup_launch.py')
            ),
            condition=IfCondition(use_nav),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_slam': 'false',  # Navigation uses saved map, not SLAM
                # 如果 map_file 为空，则回退到默认地图（rt_robot_nav2/map/rt_robot_map.yaml）
                # 若提供相对路径，则自动拼接包内 map 目录；绝对路径则直接使用
                'map': PythonExpression([
                    # 为空则使用默认地图
                    "'", map_dir.replace('\\', '/'), "/rt_robot_map.yaml' if '", map_file, "' == '' else ",
                    # 绝对路径（Linux / 或 Windows 盘符）直接用
                    "'", map_file, "' if ('", map_file, "'.startswith('/') or ('", map_file, "'.find(':') > 0 and '", map_file, "'.find(':') < 3)) else ",
                    # 否则视为相对路径，拼到包内 map 目录
                    "'", map_dir.replace('\\', '/'), "/' + '", map_file, "'"
                ]),
                'autostart': 'true',
            }.items()
        ),
        
        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
            # 只在导航场景下启动本地RViz，SLAM场景使用slam_gmapping内部的RViz
            condition=IfCondition(PythonExpression([
                "'", open_rviz, "' == \"true\" and '", use_nav, "' == \"true\""
            ]))
        ),

        # # 🔧 NEW: Node Health Monitor - 自动监控Nav2节点崩溃和资源使用
        # Node(
        #     package='rt_robot_nav2',
        #     executable='node_health_monitor.py',
        #     name='node_health_monitor',
        #     output='screen',
        #     # 只在导航模式下启动
        #     condition=IfCondition(use_nav),
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #     }]
        # ),

        # # 🔧 NEW: TF Health Monitor - 自动监控TF漂移和异常
        # Node(
        #     package='rt_robot_nav2',
        #     executable='tf_health_monitor.py',
        #     name='tf_health_monitor',
        #     output='screen',
        #     # 只在导航模式下启动
        #     condition=IfCondition(use_nav),
        #     parameters=[{
        #         'use_sim_time': use_sim_time,
        #     }]
        # ),
    ])

