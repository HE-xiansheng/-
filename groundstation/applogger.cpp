// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "applogger.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QMutex>
#include <QMutexLocker>
#include <QStandardPaths>
#include <QTextStream>

#include <cstdlib>
#include <cstdio>

namespace {

QMutex s_logMutex;
QFile *s_logFile = nullptr;
QString s_logFilePath;
QtMessageHandler s_previousHandler = nullptr;

QString messageTypeName(QtMsgType type)
{
    switch (type) {
    case QtDebugMsg:
        return QStringLiteral("DEBUG");
    case QtInfoMsg:
        return QStringLiteral("INFO ");
    case QtWarningMsg:
        return QStringLiteral("WARN ");
    case QtCriticalMsg:
        return QStringLiteral("ERROR");
    case QtFatalMsg:
        return QStringLiteral("FATAL");
    }
    return QStringLiteral("LOG  ");
}

QString sanitizedFileName(QString name)
{
    if (name.trimmed().isEmpty())
        name = QStringLiteral("application");

    for (QChar &ch : name) {
        if (!ch.isLetterOrNumber() && ch != QLatin1Char('_') && ch != QLatin1Char('-'))
            ch = QLatin1Char('_');
    }
    return name;
}

void writeToStderr(const QString &line)
{
    const QByteArray bytes = line.toLocal8Bit();
    std::fprintf(stderr, "%s\n", bytes.constData());
    std::fflush(stderr);
}

void fileMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &message)
{
    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm:ss.zzz"));
    QString source;
    if (context.file && context.line > 0) {
        source = QStringLiteral(" [%1:%2]")
                     .arg(QString::fromLocal8Bit(context.file))
                     .arg(context.line);
    }

    const QString line = QStringLiteral("%1 [%2] %3%4")
                             .arg(timestamp,
                                  messageTypeName(type),
                                  message,
                                  source);

    {
        QMutexLocker locker(&s_logMutex);
        if (s_logFile && s_logFile->isOpen()) {
            QTextStream out(s_logFile);
            out << line << Qt::endl;
            out.flush();
        }
    }

    writeToStderr(line);

    if (type == QtFatalMsg) {
        if (s_previousHandler)
            s_previousHandler(type, context, message);
        std::abort();
    }
}

QString createLogFilePath(const QString &appName)
{
    QString logRoot;
#ifdef Q_OS_ANDROID
    // Android 的 applicationDirPath() 通常位于 APK/安装目录，不能当成普通可写目录使用。
    // 日志放到 AppDataLocation，后续通过 adb run-as 或应用导出逻辑可以取回。
    logRoot = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
#endif
    if (logRoot.isEmpty())
        logRoot = QCoreApplication::applicationDirPath();

    const QString logDirPath = QDir(logRoot).filePath(QStringLiteral("logs"));
    QDir logDir;
    if (!logDir.mkpath(logDirPath))
        return {};

    const QString timestamp = QDateTime::currentDateTime().toString(QStringLiteral("yyyyMMdd_HHmmss"));
    return QDir(logDirPath).filePath(QStringLiteral("%1_%2.log")
                                         .arg(sanitizedFileName(appName), timestamp));
}

} // namespace

namespace AppLogger {

QString initialize(const QString &appName)
{
    QMutexLocker locker(&s_logMutex);
    if (s_logFile && s_logFile->isOpen())
        return s_logFilePath;

    const QString path = createLogFilePath(appName);
    if (path.isEmpty())
        return {};

    auto *file = new QFile(path);
    if (!file->open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        delete file;
        return {};
    }

    s_logFile = file;
    s_logFilePath = QFileInfo(path).absoluteFilePath();
    s_previousHandler = qInstallMessageHandler(fileMessageHandler);

    QTextStream out(s_logFile);
    out << QStringLiteral("===== 日志开始 %1 =====")
               .arg(QDateTime::currentDateTime().toString(QStringLiteral("yyyy-MM-dd hh:mm:ss.zzz")))
        << Qt::endl;
    out.flush();

    return s_logFilePath;
}

QString logFilePath()
{
    QMutexLocker locker(&s_logMutex);
    return s_logFilePath;
}

} // namespace AppLogger
