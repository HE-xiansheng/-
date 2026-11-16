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
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
set(CMAKE_CXX_STANDARD 17)
add_library(planning_visualization SHARED src/planning_visualization.cpp)
target_include_directories(planning_visualization PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> ${EIGEN3_INCLUDE_DIRS})
ament_target_dependencies(planning_visualization rclcpp visualization_msgs)
install(TARGETS planning_visualization EXPORT export_planning_visualization LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_targets(export_planning_visualization HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp visualization_msgs Eigen3)
ament_package()
