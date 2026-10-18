cmake_minimum_required(VERSION 3.5)
project(car2_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveArmTarget.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
