cmake_minimum_required(VERSION 3.8)
project(traj_utils)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PolyTraj.msg"
  "msg/MINCOTraj.msg"
  "msg/DataDisp.msg"
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
