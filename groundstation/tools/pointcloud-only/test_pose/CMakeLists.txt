cmake_minimum_required(VERSION 3.21)
project(pose_receiver_check LANGUAGES CXX)
set(CMAKE_AUTOMOC ON)
set(CMAKE_CXX_STANDARD 17)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Qml Quick3D Network)
get_filename_component(GCS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
add_executable(check_pose check_pose.cpp
    ${GCS_SRC}/udptransport.cpp ${GCS_SRC}/udptransport.h
    ${GCS_SRC}/pointcloudprotocol.cpp
    ${GCS_SRC}/pointcloudgeometry.cpp ${GCS_SRC}/pointcloudgeometry.h
    ${GCS_SRC}/routegeometry.cpp ${GCS_SRC}/routegeometry.h)
target_include_directories(check_pose PRIVATE ${GCS_SRC})
target_link_libraries(check_pose PRIVATE Qt6::Core Qt6::Gui Qt6::Qml Qt6::Quick3D Qt6::Network)
