cmake_minimum_required(VERSION 3.5)
project(chassis_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
# tf2_geometry_msgs not needed for Transform conversions, use tf2/convert.h instead
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)

# Odometry subscriber node
add_executable(odometry_subscriber src/odometry_subscriber.cpp)
ament_target_dependencies(odometry_subscriber
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
)

# Keyboard teleop node
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
ament_target_dependencies(keyboard_teleop
  rclcpp
  geometry_msgs
  std_msgs
)

# Odometry to TF converter node
add_executable(odometry_to_tf src/odometry_to_tf.cpp)
ament_target_dependencies(odometry_to_tf
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
)

# IMU to TF converter node
add_executable(imu_to_tf src/imu_to_tf.cpp)
ament_target_dependencies(imu_to_tf
  rclcpp
  sensor_msgs
  tf2_ros
  tf2
  geometry_msgs
)

# Odometry fusion node (fuses IMU + SLAM, with encoder interface reserved)
add_executable(odom_fusion src/odom_fusion.cpp)
ament_target_dependencies(odom_fusion
  rclcpp
  sensor_msgs
  nav_msgs
  tf2_ros
  tf2
  geometry_msgs
)

# Obstacle avoidance node (highest priority safety mechanism)
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2
)

# Goal stop publisher node (sends stop command when navigation goal is reached)
add_executable(goal_stop_publisher src/goal_stop_publisher.cpp)
ament_target_dependencies(goal_stop_publisher
  rclcpp
  geometry_msgs
  nav2_msgs
  rclcpp_action
)

# Install executables
install(TARGETS
  odometry_subscriber
  keyboard_teleop
  odometry_to_tf
  imu_to_tf
  odom_fusion
  obstacle_avoidance
  goal_stop_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()

