// Copyright (C) 2020 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "applogger.h"
#include "videocontroller.h"

#include <QGuiApplication>
#include <QFont>
#include <QIcon>
#include <QDir>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QUrl>
#include <QQuickWindow>
#include <QElapsedTimer>
#include <QTimer>
#include <QPointer>
#include <QSettings>
#include <QRegularExpression>
#include <algorithm>
#include <QtQuick3D/qquick3d.h>

int main(int argc, char *argv[])
{
    QGuiApplication app(argc, argv);
#ifdef Q_OS_WIN
    // 随包工具只用于地面站子进程，不修改用户或系统 PATH。
    const QDir appDir(QCoreApplication::applicationDirPath());
    const QString runtime = appDir.absoluteFilePath("../runtime");
    qputenv("PATH", (runtime + "/openssh;" + runtime + "/ffmpeg;" + qEnvironmentVariable("PATH")).toLocal8Bit());
#endif
    // 统一工作台字体，中文使用无衬线回退，避免各面板采用不同的系统默认字体。
    QFont interfaceFont;
    interfaceFont.setFamilies({QStringLiteral("Inter"), QStringLiteral("Noto Sans"),
        QStringLiteral("Noto Sans CJK SC"), QStringLiteral("Microsoft YaHei UI"),
        QStringLiteral("PingFang SC"), QStringLiteral("sans-serif")});
    interfaceFont.setPixelSize(12);
    interfaceFont.setWeight(QFont::Normal);
    interfaceFont.setStyleStrategy(QFont::PreferAntialias);
    app.setFont(interfaceFont);
    QCoreApplication::setApplicationName(QStringLiteral("customgeometry-ros2-staging"));
    QCoreApplication::setApplicationVersion(QStringLiteral("1.0-ros2-staging"));
    QGuiApplication::setApplicationDisplayName(QStringLiteral("科研飞行平台-地面站"));
    QGuiApplication::setWindowIcon(QIcon(QStringLiteral(":/qt/qml/CustomGeometryExample/assets/icons/research-flight-gcs-v1.png")));
    QGuiApplication::setDesktopFileName(QStringLiteral("research-flight-groundstation"));

    const QString logPath = AppLogger::initialize(QStringLiteral("customgeometry"));
    if (!logPath.isEmpty())
        qInfo().noquote() << QStringLiteral("日志文件：%1").arg(logPath);

    QSurfaceFormat::setDefaultFormat(QQuick3D::idealSurfaceFormat());
    qInfo() << "Qt Quick 3D surface format initialized";

    QUrl autoLoadPointCloudUrl;
    int startupPointCloudPort =
#ifdef Q_OS_WIN
        9000;
#else
        56000;
#endif
    int startupPosePort = 9001;
    bool startupPoseActive = false;
    int startupPreflightPort = 9003;
    bool startupPreflightActive = false;
#ifdef Q_OS_WIN
    startupPoseActive = true;
    startupPreflightActive = true;
#endif
    const QStringList arguments = app.arguments();
    // 启动即统一视频与机载控制地址；未重新认证时仍不启动机载任务。
    QString startupOnboardHost = QSettings().value("onboard/host", "192.168.1.224").toString();
    const QRegularExpression validHost(QStringLiteral("^[a-zA-Z0-9][a-zA-Z0-9.-]{0,252}$"));
    for (int i = 1; i < arguments.size(); ++i) {
        if (arguments.at(i) == QStringLiteral("--host") || arguments.at(i).startsWith(QStringLiteral("--host="))) {
            const auto argument = arguments.at(i);
            const QString host = argument == QStringLiteral("--host")
                ? (i + 1 < arguments.size() ? arguments.at(++i) : QString())
                : argument.mid(7);
            if (!validHost.match(host).hasMatch()) {
                qCritical() << "--host requires a valid onboard IP or hostname";
                return 2;
            }
            startupOnboardHost = host;
        } else if (arguments.at(i) == QStringLiteral("--auto-load-pointcloud") && i + 1 < arguments.size()) {
            autoLoadPointCloudUrl = QUrl::fromLocalFile(arguments.at(i + 1));
            qInfo().noquote() << QStringLiteral("命令行自动加载点云：%1").arg(autoLoadPointCloudUrl.toLocalFile());
            ++i;
        } else if (arguments.at(i) == QStringLiteral("--cloud-port")) {
            bool valid = false;
            const int port = i + 1 < arguments.size() ? arguments.at(++i).toInt(&valid) : 0;
            if (!valid || port < 1 || port > 65535) {
                qCritical() << "--cloud-port requires a port in 1..65535";
                return 2;
            }
            startupPointCloudPort = port;
        } else if (arguments.at(i) == QStringLiteral("--pose-port")) {
            bool valid = false;
            const int port = i + 1 < arguments.size() ? arguments.at(++i).toInt(&valid) : 0;
            if (!valid || port < 1 || port > 65535) {
                qCritical() << "--pose-port requires a port in 1..65535";
                return 2;
            }
            startupPosePort = port;
            startupPoseActive = true;
        } else if (arguments.at(i) == QStringLiteral("--preflight-port")) {
            bool valid=false;
            const int port=i+1<arguments.size() ? arguments.at(++i).toInt(&valid) : 0;
            if (!valid || port<1 || port>65535) return 2;
            startupPreflightPort=port;startupPreflightActive=true;
        }
    }

    app.setProperty("startupOnboardHost", startupOnboardHost);
    VideoController videoController;
    videoController.setHost(startupOnboardHost);
    qInfo().noquote() << QStringLiteral("启动机载地址（视频与通信）：%1").arg(startupOnboardHost);
    QQmlApplicationEngine engine;
    engine.addImageProvider(QStringLiteral("livevideo"), new VideoImageProvider(&videoController));
    engine.rootContext()->setContextProperty(QStringLiteral("videoController"), &videoController);
    engine.rootContext()->setContextProperty(QStringLiteral("autoLoadPointCloudUrl"),
                                             autoLoadPointCloudUrl);
    engine.rootContext()->setContextProperty(QStringLiteral("startupPointCloudPort"), startupPointCloudPort);
    engine.rootContext()->setContextProperty(QStringLiteral("startupPosePort"), startupPosePort);
    engine.rootContext()->setContextProperty(QStringLiteral("startupPoseActive"), startupPoseActive);
    engine.rootContext()->setContextProperty(QStringLiteral("startupPreflightPort"),startupPreflightPort);
    engine.rootContext()->setContextProperty(QStringLiteral("startupPreflightActive"),startupPreflightActive);
    QElapsedTimer startupClock;startupClock.start();
    QPointer<QQuickWindow> splash;
    bool mainScheduled=false,mainFrameReady=false;
    const bool animated=!arguments.contains(QStringLiteral("--no-splash"));
    QObject::connect(&engine,&QQmlApplicationEngine::objectCreationFailed,&app,[&] {
        qCritical()<<"QML root object creation failed";
        if(splash){splash->setProperty("failed",true);splash->setProperty("message",QStringLiteral("界面加载失败，请查看程序日志"));}
        else app.exit(-1);
    },Qt::QueuedConnection);
    auto loadMain=[&] {
        if(mainScheduled)return;
        mainScheduled=true;
        if(splash)splash->setProperty("message",QStringLiteral("正在初始化三维界面…"));
        engine.loadFromModule("CustomGeometryExample","Main");
        qInfo()<<"QML module loaded: CustomGeometryExample/Main";
        if(engine.rootObjects().isEmpty())return;
        QPointer<QQuickWindow> mainWindow=qobject_cast<QQuickWindow*>(engine.rootObjects().last());
        if(!mainWindow||mainWindow==splash)return;
        // 收到真正的主界面首帧后再结束启动动画，机载连接不参与此条件。
        QObject::connect(mainWindow,&QQuickWindow::frameSwapped,&app,[&,mainWindow] {
            if(mainFrameReady)return;
            mainFrameReady=true;
            qInfo()<<"地面站首帧就绪，启动耗时(ms)"<<startupClock.elapsed();
            const int remaining=splash?std::max(0,650-int(startupClock.elapsed())):0;
            QTimer::singleShot(remaining,&app,[&,mainWindow] {
                if(splash){splash->setProperty("message",QStringLiteral("准备就绪"));splash->setProperty("ready",true);}
                if(mainWindow){mainWindow->raise();mainWindow->requestActivate();}
            });
        },Qt::QueuedConnection);
    };
    if(animated){
        engine.loadFromModule("CustomGeometryExample","StartupWindow");
        if(!engine.rootObjects().isEmpty())splash=qobject_cast<QQuickWindow*>(engine.rootObjects().first());
        if(splash){
            qInfo()<<"启动动画已显示";
            // 先绘制启动页，再创建主界面；旋转动画由渲染线程执行。
            QObject::connect(splash,&QQuickWindow::frameSwapped,&app,[&] {if(!mainScheduled)QTimer::singleShot(0,&app,loadMain);},Qt::QueuedConnection);
            QTimer::singleShot(15000,&app,[&] {
                if(!mainFrameReady&&splash){splash->setProperty("failed",true);splash->setProperty("message",QStringLiteral("界面初始化超时，请查看程序日志"));}
            });
        }
    }else loadMain();

    return app.exec();
}
