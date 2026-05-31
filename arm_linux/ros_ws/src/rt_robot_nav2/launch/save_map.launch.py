# Map saving launch file for rt_robot_nav2
# Usage: ros2 launch rt_robot_nav2 save_map.launch.py map_name:=my_map
# 
# Note: This requires nav2_map_server to be installed.
# If not installed, use: sudo apt install ros-foxy-navigation2
# Or run directly: ros2 run nav2_map_server map_saver_cli -f <path>/<map_name>
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    rt_robot_nav2_pkg_share = get_package_share_directory('rt_robot_nav2')
    map_dir = os.path.join(rt_robot_nav2_pkg_share, 'map')
    
    # Launch arguments
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='rt_robot_map',
        description='Name of the map to save (without extension)'
    )
    
    map_path_arg = DeclareLaunchArgument(
        'map_path',
        default_value=map_dir,
        description='Directory to save the map'
    )
    
    map_name = LaunchConfiguration('map_name')
    map_path = LaunchConfiguration('map_path')
    
    # Build the full map file path
    map_file_path = PathJoinSubstitution([map_path, map_name])

    # Save map using map_saver_cli
    # Note: map_subscribe_transient_local is required to receive map from map_server
    # which uses TRANSIENT_LOCAL QoS durability
    map_saver = ExecuteProcess(
        cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
             '-f', map_file_path,
             '--ros-args',
             '-p', 'map_subscribe_transient_local:=true'],
        output='screen',
        name='map_saver',
        shell=True,  # use shell so ros2 is found in PATH
    )

    # After map is saved, save initial pose to map yaml
    # This reads the current robot pose in map frame and writes it to map yaml
    # Use ExecuteProcess instead of Node to ensure proper exit detection
    save_initialpose_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rt_robot_nav2', 'save_initialpose_to_map.py',
            '--ros-args',
            '-r', '__node:=save_initialpose_to_map',
            '-p', PythonExpression(['"', 'map_base_path:=', '" + "', map_file_path, '"']),
            '-p', 'map_frame:=map',
            '-p', 'base_frame:=base_footprint',
            '-p', 'timeout_sec:=5.0'
        ],
        output='screen',
        name='save_initialpose_to_map',
        shell=True,
    )

    # Shutdown launch when save_initialpose_node exits (after saving initial pose)
    shutdown_on_initialpose_saved = RegisterEventHandler(
        OnProcessExit(
            target_action=save_initialpose_node,
            on_exit=[Shutdown(reason='initial pose saved to map yaml')],
        )
    )

    # Start save_initialpose_node after map_saver completes
    start_initialpose_saver = RegisterEventHandler(
        OnProcessExit(
            target_action=map_saver,
            on_exit=[save_initialpose_node],
        )
    )

    warn_msg = LogInfo(
        msg="[WARNING] nav2_map_server not found. Please install: sudo apt install ros-foxy-navigation2",
        condition=IfCondition(PythonExpression(["'", "${find-pkg nav2_map_server}", "' == ''"]))
    )
    
    return LaunchDescription([
        map_name_arg,
        map_path_arg,
        warn_msg,
        map_saver,
        start_initialpose_saver,  # Start save_initialpose_node after map_saver
        shutdown_on_initialpose_saved,  # Shutdown after initial pose is saved
    ])

