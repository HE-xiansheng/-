# Linux独立安装只包含当前入口实际依赖；历史ROS1安装器与测试留在源码仓库。
set(gcs_ros2_runtime_tools
    communication/manage_link.py
    communication/runtime_paths.py
    communication/ssh_session.py
    pointcloud-only/manage.py
    pointcloud-only/cloud_sender.py
    preflight/bridge.py
    preflight/planner_display.py
    preflight/planner_geometry.py
    preflight/preflight_status.py
    preflight/stream_reader.py
    preflight/telemetry_reader.py
)
foreach(gcs_tool IN LISTS gcs_ros2_runtime_tools)
    get_filename_component(gcs_tool_directory "${gcs_tool}" DIRECTORY)
    install(FILES "${CMAKE_CURRENT_LIST_DIR}/../tools/${gcs_tool}"
        DESTINATION "tools/${gcs_tool_directory}" COMPONENT ros2_tools)
endforeach()
install(PROGRAMS "${CMAKE_CURRENT_LIST_DIR}/../tools/communication/gcs-askpass"
    DESTINATION tools/communication COMPONENT ros2_tools)
unset(gcs_tool)
unset(gcs_tool_directory)
unset(gcs_ros2_runtime_tools)
