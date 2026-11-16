#pragma once
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>

// ROS2试验版运行工具和本机状态位置；源码构建与独立安装共用，不接管旧地面站会话。
namespace RuntimePaths {
inline QString stateDirectory() {
    const auto custom=qEnvironmentVariable("GCS_STATE_DIR");
    return custom.isEmpty()?QStandardPaths::writableLocation(QStandardPaths::GenericDataLocation)+"/drone-gcs-ros2":custom;
}
inline QString tool(const QString &relative) {
    const auto installed=QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("../"+relative);
    if(QFileInfo::exists(installed))return installed;
#ifdef GCS_SOURCE_DIR
    const auto source=QDir(QStringLiteral(GCS_SOURCE_DIR)).filePath(relative);
    if(QFileInfo::exists(source))return source;
#endif
    return {};
}
}
