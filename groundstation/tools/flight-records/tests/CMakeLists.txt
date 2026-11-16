cmake_minimum_required(VERSION 3.21)
project(flight_records_checks LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Quick3D Network Concurrent ShaderTools)
qt_standard_project_setup(REQUIRES 6.8)
get_filename_component(GCS_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
qt_add_executable(check_records check_records.cpp)
target_link_libraries(check_records PRIVATE Qt6::Core Qt6::Gui Qt6::Quick Qt6::Quick3D Qt6::Network Qt6::Concurrent lz4 bz2)
target_include_directories(check_records PRIVATE ${GCS_SRC})
target_compile_definitions(check_records PRIVATE GCS_HAVE_LZ4 GCS_HAVE_BZIP2 GCS_SRC="${GCS_SRC}")
foreach(name FlightRecordWindow.qml StudioButton.qml BrandBadge.qml)
    set_source_files_properties(${GCS_SRC}/${name} PROPERTIES QT_RESOURCE_ALIAS ${name})
endforeach()
qt_add_qml_module(check_records URI CustomGeometryExample VERSION 1.0
    QML_FILES ${GCS_SRC}/FlightRecordWindow.qml ${GCS_SRC}/StudioButton.qml ${GCS_SRC}/BrandBadge.qml
    SOURCES ${GCS_SRC}/pointcloudfileloader.h ${GCS_SRC}/pointcloudfileloader.cpp
        ${GCS_SRC}/pointcloudgeometry.h ${GCS_SRC}/pointcloudgeometry.cpp
        ${GCS_SRC}/routegeometry.h ${GCS_SRC}/routegeometry.cpp
        ${GCS_SRC}/flightrecordcontroller.h ${GCS_SRC}/flightrecordcontroller.cpp
)
# 使用与正式程序一致的资源路径，测试二进制不连接真实飞控。
qt_add_resources(check_records "helpers" PREFIX "/qt/qml/CustomGeometryExample" BASE ${GCS_SRC}
    FILES ${GCS_SRC}/tools/flight-records/remote_records.py ${GCS_SRC}/assets/tianyi-logo.jpg)
qt_add_shaders(check_records "brand_shader" PREFIX "/" BASE ${GCS_SRC} FILES ${GCS_SRC}/shaders/brand.frag)

find_package(SQLite3 REQUIRED)
if(SQLite3_FOUND)
    target_link_libraries(check_records PRIVATE SQLite::SQLite3)
    target_compile_definitions(check_records PRIVATE GCS_HAVE_SQLITE3)
endif()
qt_add_executable(check_ros2_records check_ros2_records.cpp
    ${GCS_SRC}/pointcloudfileloader.h ${GCS_SRC}/pointcloudfileloader.cpp
    ${GCS_SRC}/pointcloudgeometry.h ${GCS_SRC}/pointcloudgeometry.cpp)
target_include_directories(check_ros2_records PRIVATE ${GCS_SRC})
target_link_libraries(check_ros2_records PRIVATE Qt6::Core Qt6::Gui Qt6::Quick Qt6::Quick3D Qt6::Network Qt6::Concurrent lz4 bz2 SQLite::SQLite3)
target_compile_definitions(check_ros2_records PRIVATE GCS_HAVE_LZ4 GCS_HAVE_BZIP2 GCS_HAVE_SQLITE3)
qt_add_executable(check_ros2_transfer check_ros2_transfer.cpp
    ${GCS_SRC}/pointcloudfileloader.h ${GCS_SRC}/pointcloudfileloader.cpp
    ${GCS_SRC}/pointcloudgeometry.h ${GCS_SRC}/pointcloudgeometry.cpp
    ${GCS_SRC}/flightrecordcontroller.h ${GCS_SRC}/flightrecordcontroller.cpp)
target_include_directories(check_ros2_transfer PRIVATE ${GCS_SRC})
target_link_libraries(check_ros2_transfer PRIVATE Qt6::Core Qt6::Gui Qt6::Quick Qt6::Quick3D Qt6::Network Qt6::Concurrent lz4 bz2 SQLite::SQLite3)
target_compile_definitions(check_ros2_transfer PRIVATE GCS_HAVE_LZ4 GCS_HAVE_BZIP2 GCS_HAVE_SQLITE3)
qt_add_resources(check_ros2_transfer "helpers" PREFIX "/qt/qml/CustomGeometryExample" BASE ${GCS_SRC}
    FILES ${GCS_SRC}/tools/flight-records/remote_records.py)
qt_add_executable(check_recording_ui check_recording_ui.cpp ${GCS_SRC}/recordingcontroller.cpp ${GCS_SRC}/recordingcontroller.h)
target_include_directories(check_recording_ui PRIVATE ${GCS_SRC})
target_link_libraries(check_recording_ui PRIVATE Qt6::Core Qt6::Qml)
