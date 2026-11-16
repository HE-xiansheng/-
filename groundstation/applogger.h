// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef APPLOGGER_H
#define APPLOGGER_H

#include <QString>

// 应用日志初始化工具。
//
// 该模块负责给地面站和雷达模拟器安装同一套 Qt 消息处理器：
// - main.cpp / radarsimulator.cpp 在 QApplication/QCoreApplication 创建后调用 initialize()。
// - 项目中的 qInfo/qWarning/qCritical 会被统一写入 exe 同级 logs 目录下的 .log 文件。
// - 控制台程序仍会把日志回显到 stderr，便于开发阶段直接观察。
//
// 日志模块不依赖业务类，业务链路只需要使用 Qt 自带日志宏，避免每个类各自管理文件句柄。
namespace AppLogger {

// 初始化日志文件并安装 Qt message handler。
// appName 用于生成文件名，例如 customgeometry_20260615_101530.log。
// 返回实际日志文件路径；如果文件无法创建则返回空字符串，此时 Qt 默认日志仍可继续工作。
QString initialize(const QString &appName);

// 返回当前进程正在写入的日志路径，尚未初始化时为空。
QString logFilePath();

} // namespace AppLogger

#endif // APPLOGGER_H
