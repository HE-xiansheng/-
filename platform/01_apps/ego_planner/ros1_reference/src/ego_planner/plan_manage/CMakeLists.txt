cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  mavros_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)


add_executable(planner_node
  src/planner_node.cpp 
  src/replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(planner_node 
  ${catkin_LIBRARIES}
  )

add_executable(candidate_preview_worker
  src/candidate_preview_worker.cpp
  src/planner_manager.cpp
  )
target_link_libraries(candidate_preview_worker ${catkin_LIBRARIES})
# 机载测试与正式工作空间位置不同，程序随其devel/lib定位依赖。
set_target_properties(planner_node candidate_preview_worker PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE INSTALL_RPATH "$ORIGIN/..:/opt/ros/noetic/lib")
#add_dependencies(planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES})
# add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})
# add_executable(monitor_node src/monitor_node.cpp)
# target_link_libraries(monitor_node ${catkin_LIBRARIES})

