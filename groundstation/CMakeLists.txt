# Copyright (C) 2022 The Qt Company Ltd.
# SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

cmake_minimum_required(VERSION 3.16)
project(customgeometry LANGUAGES CXX)

set(CMAKE_AUTOMOC ON)
include(GNUInstallDirs)

find_package(Qt6 REQUIRED COMPONENTS Core Gui Quick Quick3D Network Concurrent ShaderTools)
qt_standard_project_setup(REQUIRES 6.8)

qt_add_executable(customgeometry
    main.cpp
    applogger.cpp applogger.h
)

if(ANDROID)
    # Android 需要自定义 Manifest 来声明 UDP 网络权限；该属性只在 Android 打包时生效，
    # 不会影响 Windows 下的 exe、windeployqt 或 package_runtime。
    set_target_properties(customgeometry PROPERTIES
        QT_ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android"
        QT_ANDROID_PACKAGE_NAME "com.skyhub.customgeometry"
    )
endif()

set_target_properties(customgeometry PROPERTIES
    WIN32_EXECUTABLE TRUE
    MACOSX_BUNDLE TRUE
)

target_compile_definitions(customgeometry PRIVATE GCS_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

target_link_libraries(customgeometry PUBLIC
    Qt::Core
    Qt::Gui
    Qt::Quick
    Qt::Quick3D
    Qt::Network
    Qt::Concurrent
)

qt_add_qml_module(customgeometry
    URI CustomGeometryExample
    VERSION 1.0
    QML_FILES
        Main.qml
        StartupWindow.qml
        PreflightPanel.qml
        PlannerPanel.qml
        RecordingPanel.qml
        MissionControls.qml
        LocalizationPanel.qml
        SectionHeader.qml
        StudioButton.qml
        ConnectionPanel.qml
        TorusMesh.qml
        VideoPanel.qml
        SensorsPanel.qml
        BrandBadge.qml
        FlightRecordWindow.qml
    SOURCES
        examplegeometry.cpp examplegeometry.h
        pointcloudgeometry.cpp pointcloudgeometry.h
        pointcloudfileloader.cpp pointcloudfileloader.h
        pointcloudprotocol.cpp pointcloudprotocol.h
        routegeometry.cpp routegeometry.h
        udptransport.cpp udptransport.h
        preflightreceiver.cpp preflightreceiver.h
        connectioncontroller.cpp connectioncontroller.h sshsession.h
        missioncontroller.cpp missioncontroller.h
        plannercontroller.cpp plannercontroller.h
        recordingcontroller.cpp recordingcontroller.h
        flightrecordcontroller.cpp flightrecordcontroller.h
        localizationcontroller.cpp localizationcontroller.h
        videocontroller.cpp videocontroller.h
    RESOURCES tools/missions/task_manager.py qt_logo_rect.png assets/tianyi-logo.jpg assets/icons/research-flight-gcs-v1.png tools/planner/stop_preview.py tools/localization/manage_stack.py tools/video/ros_image_bridge.py tools/flight-records/remote_records.py
    IMPORTS
        QtQuick3D
)

if(NOT ANDROID)
    # radarsimulator 是桌面/上位机侧的 UDP 点云发送器，Android 包只需要地面站本体。
    # 保持 Windows 平台继续生成 customgeometry.exe + radarsimulator.exe 两个程序。
    qt_add_executable(radarsimulator
        radarsimulator.cpp
        pointcloudprotocol.cpp pointcloudprotocol.h
        applogger.cpp applogger.h
    )

    target_link_libraries(radarsimulator PUBLIC
        Qt::Core
        Qt::Gui
        Qt::Network
    )
endif()

if(ANDROID)
    # Android 只安装主应用库；模拟器不进入 APK。
    install(TARGETS customgeometry
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
else()
    # Windows/桌面平台保持原有交付方式：地面站和模拟器一起安装、一起打包。
    install(TARGETS customgeometry radarsimulator
        BUNDLE  DESTINATION .
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    )
endif()

install(FILES README.md DESTINATION . OPTIONAL)

qt_generate_deploy_qml_app_script(
    TARGET customgeometry
    OUTPUT_SCRIPT deploy_script
    MACOS_BUNDLE_POST_BUILD
    NO_UNSUPPORTED_PLATFORM_ERROR
    DEPLOY_USER_QML_MODULES_ON_UNSUPPORTED_PLATFORM
)
install(SCRIPT ${deploy_script})

if(WIN32)
    configure_file(tools/packaging/windows_icon.rc.in windows_icon.rc @ONLY)
    target_sources(customgeometry PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/windows_icon.rc")
    # 密码助手不依赖 Qt，供随包 OpenSSH 在无控制台环境读取会话密码。
    add_executable(ssh-askpass WIN32 tools/packaging/ssh_askpass.cpp)
    install(TARGETS ssh-askpass RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    set(RUNTIME_PACKAGE_DIR "${CMAKE_BINARY_DIR}/package")
    add_custom_target(package_runtime
        COMMAND ${CMAKE_COMMAND}
                "-DPROJECT_BINARY_DIR=${CMAKE_BINARY_DIR}"
                "-DPACKAGE_DIR=${RUNTIME_PACKAGE_DIR}"
                "-DCONFIG=$<CONFIG>"
                -P "${CMAKE_CURRENT_SOURCE_DIR}/cmake/PackageRuntime.cmake"
        DEPENDS customgeometry radarsimulator
        COMMENT "生成可直接运行的 Windows 打包目录：${RUNTIME_PACKAGE_DIR}"
        VERBATIM
    )
endif()

qt_add_shaders(customgeometry "brand_shader" PREFIX "/" FILES shaders/brand.frag)
# 机载录包采用LZ4；缺少可选解码库的平台仍保留原有未压缩bag能力。
find_path(GCS_LZ4_INCLUDE_DIR lz4frame.h)
find_library(GCS_LZ4_LIBRARY NAMES lz4)
if(GCS_LZ4_INCLUDE_DIR AND GCS_LZ4_LIBRARY)
    target_include_directories(customgeometry PRIVATE ${GCS_LZ4_INCLUDE_DIR})
    target_link_libraries(customgeometry PRIVATE ${GCS_LZ4_LIBRARY})
    target_compile_definitions(customgeometry PRIVATE GCS_HAVE_LZ4)
endif()
find_package(BZip2 QUIET)
if(BZIP2_FOUND)
    target_link_libraries(customgeometry PRIVATE BZip2::BZip2)
    target_compile_definitions(customgeometry PRIVATE GCS_HAVE_BZIP2)
endif()

# Linux独立安装布局bin/../tools；Windows打包保留原有规则。
if(UNIX AND NOT ANDROID AND NOT APPLE)
    install(DIRECTORY tools/ DESTINATION tools FILES_MATCHING PATTERN "*.py" PATTERN "gcs-askpass" PATTERN "__pycache__" EXCLUDE)
endif()

find_package(SQLite3 QUIET)
if(SQLite3_FOUND)
    target_link_libraries(customgeometry PRIVATE SQLite::SQLite3)
    target_compile_definitions(customgeometry PRIVATE GCS_HAVE_SQLITE3)
endif()
