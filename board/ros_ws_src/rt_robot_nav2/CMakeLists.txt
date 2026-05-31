cmake_minimum_required(VERSION 3.8)
project(rt_robot_nav2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/auto_initialpose.py
  scripts/save_initialpose_to_map.py
  scripts/verify_localization.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install parameter files
install(DIRECTORY param
  DESTINATION share/${PROJECT_NAME}/
)

# Install config directory (contains unified velocity_limits.yaml)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install map directory
install(DIRECTORY map
  DESTINATION share/${PROJECT_NAME}/
)

# Install rviz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

