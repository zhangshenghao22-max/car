from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directories
    chassis_controller_dir = get_package_share_directory('chassis_controller')
    rt_robot_nav2_dir = get_package_share_directory('rt_robot_nav2')

    # Configuration files
    # 优先从统一的速度限制配置文件加载，然后加载chassis_controller特定配置
    chassis_config_file = os.path.join(chassis_controller_dir, 'config', 'chassis_controller.yaml')
    velocity_limits_file = os.path.join(rt_robot_nav2_dir, 'config', 'velocity_limits.yaml')
    # Declare launch arguments
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Topic name for odometry messages from RT-Thread'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic name for velocity command messages to RT-Thread'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='True',
        description='Enable verbose output for odometry subscriber'
    )
    
    linear_step_arg = DeclareLaunchArgument(
        'linear_velocity_step',
        default_value='0.1',
        description='Step size for linear velocity changes'
    )
    
    angular_step_arg = DeclareLaunchArgument(
        'angular_velocity_step',
        default_value='0.1',
        description='Step size for angular velocity changes'
    )
    
    max_linear_x_arg = DeclareLaunchArgument(
        'max_linear_x',
        default_value='0.3',  # LIMITED to 0.3 m/s for stability - REDUCED from 0.4
        description='Maximum linear velocity in X direction (m/s): -0.3~0 (backward), 0~0.3 (forward), LIMIT: 0.3 m/s!'
    )

    max_linear_y_arg = DeclareLaunchArgument(
        'max_linear_y',
        default_value='0.3',  # LIMITED to 0.3 m/s for stability - REDUCED from 0.4
        description='Maximum linear velocity in Y direction (m/s): -0.3~0 (right), 0~0.3 (left), LIMIT: 0.3 m/s!'
    )
    
    max_angular_z_arg = DeclareLaunchArgument(
        'max_angular_z',
        default_value='0.5',  # LIMITED to 0.5 to prevent TF drift and instability
        description='Maximum angular velocity in Z direction (rad/s): -0.5~0 (right), 0~0.5 (left), recommended 0.3 or -0.3. LIMIT: Do not exceed 0.5!'
    )
    
    # Odometry subscriber node
    # Can use config file or launch arguments (launch arguments take precedence)
    odometry_subscriber_node = Node(
        package='chassis_controller',
        executable='odometry_subscriber',
        name='odometry_subscriber',
        output='screen',
        parameters=[
            config_file,  # Load from config file first
            {  # Override with launch arguments if provided
                'chassis_controller.odom_topic': LaunchConfiguration('odom_topic'),
                'chassis_controller.verbose': LaunchConfiguration('verbose'),
            }
        ]
    )
    
    # Keyboard teleop node
    # Load from unified velocity_limits.yaml first, then chassis_config, then launch arguments
    keyboard_teleop_node = Node(
        package='chassis_controller',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        parameters=[
            velocity_limits_file,  # Load unified velocity limits FIRST
            chassis_config_file,   # Then chassis-specific config
            {  # Override with launch arguments if provided (highest priority)
                'chassis_controller.cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'chassis_controller.linear_velocity_step': LaunchConfiguration('linear_velocity_step'),
                'chassis_controller.angular_velocity_step': LaunchConfiguration('angular_velocity_step'),
                'chassis_controller.max_linear_x': LaunchConfiguration('max_linear_x'),
                'chassis_controller.max_linear_y': LaunchConfiguration('max_linear_y'),
                'chassis_controller.max_angular_z': LaunchConfiguration('max_angular_z'),
            }
        ]
    )
    
    return LaunchDescription([
        odom_topic_arg,
        cmd_vel_topic_arg,
        verbose_arg,
        linear_step_arg,
        angular_step_arg,
        max_linear_x_arg,
        max_linear_y_arg,
        max_angular_z_arg,
        odometry_subscriber_node,
        keyboard_teleop_node,
    ])

