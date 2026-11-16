#include <QObject>
#include <QProcess>
#include <QImage>
#include <QMutex>
#include <QTimer>
#include <QElapsedTimer>
#include <QQuickImageProvider>
#include <QCoreApplication>
#include <QGuiApplication>
#include <QQmlEngine>
#include <QQmlContext>
#include <QQmlComponent>
#include <QQuickItem>
#include <QTemporaryDir>
#include <QFile>
#include <QThread>
#include <QBuffer>
#include <QtEndian>
#include <QDebug>
#include <functional>
#define private public
#include "videocontroller.h"
#undef private
#include "connectioncontroller.h"

static void wait(int ms) {
    QElapsedTimer timer; timer.start();
    while (timer.elapsed() < ms) { QCoreApplication::processEvents(); QThread::msleep(5); }
}
static void require(bool ok, const char *message) {
    if (!ok) qFatal("FAIL %s", message);
    qInfo() << "PASS" << message;
}
int main(int argc, char **argv) {
    QGuiApplication app(argc, argv);
    if (app.arguments().contains("--real-ssh")) {
        VideoController c;c.setHost("192.168.1.224");c.setStreamEnabled(0,true);
        QElapsedTimer timer;timer.start();
        auto until=[&](const std::function<bool()> &predicate,int limit=12000){timer.restart();while(timer.elapsed()<limit){QCoreApplication::processEvents();if(predicate())return true;QThread::msleep(10);}return false;};
        require(until([&]{return c.stereoFresh();}),"真实SSH ROS2压缩图像进入Qt");
        require(c.image(0).width()==640,"真实SSH图像尺寸符合显示限制");
        c.setTrackingMode(true);
        require(until([&]{return c.stereoFresh();}),"真实SSH跟踪标注图进入Qt");
        c.setGimbalAruco(true);c.setStreamEnabled(1,true);
        require(until([&]{return c.gimbalFresh();}),"真实SSH ArUco标注图进入Qt");
        VideoImageProvider provider(&c);QSize size;
        require(!provider.requestImage("0",&size,QSize()).isNull(),"真实SSH图像通过Qt provider");
        c.stop();require(!c.stereoFresh()&&!c.gimbalFresh(),"真实SSH视频停止后状态失效");
        require(until([&]{return c.m_stereo.state()==QProcess::NotRunning&&c.m_gimbal.state()==QProcess::NotRunning;},5000),"真实SSH视频接收进程回收");
        return 0;
    }
    app.setProperty("startupOnboardHost", QStringLiteral("192.168.1.13"));
    ConnectionController connection;
    require(connection.host() == QStringLiteral("192.168.1.13") && !connection.authenticated(),
            "通信使用启动地址且不自动授权");
    QTemporaryDir directory;
    QFile ssh(directory.filePath("ssh"));
    require(ssh.open(QIODevice::WriteOnly), "创建隔离SSH替身");
    ssh.write("#!/bin/sh\nexit 255\n"); ssh.close();
    ssh.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner);
    qputenv("PATH", directory.path().toUtf8() + ":" + qgetenv("PATH"));
    QFile gst(directory.filePath("gst-launch-1.0"));
    require(gst.open(QIODevice::WriteOnly), "创建隔离GStreamer替身");
    gst.write("#!/bin/sh\nexit 255\n"); gst.close();
    gst.setPermissions(QFileDevice::ReadOwner | QFileDevice::WriteOwner | QFileDevice::ExeOwner);
    VideoController controller;
    controller.setHost(QStringLiteral("192.168.1.13"));
    controller.start(); wait(350);
    require(controller.m_stereoRetry.isActive() && controller.m_gimbalRetry.isActive(), "双路接收退出均安排重连");
    wait(2200);
    require(controller.m_gimbalRetryDelayMs >= 8000 && controller.m_stereoRetryDelayMs >= 8000, "双路实际执行重试并退避");
    controller.stop(); wait(100);
    require(!controller.m_stereoRetry.isActive() && !controller.m_gimbalRetry.isActive(), "停止取消两路重试");
    ssh.open(QIODevice::WriteOnly | QIODevice::Truncate);
    ssh.write("#!/bin/sh\nexec /bin/sleep 30\n"); ssh.close();
    gst.open(QIODevice::WriteOnly | QIODevice::Truncate);
    gst.write("#!/bin/sh\nexec /bin/sleep 30\n"); gst.close();
    controller.start(); wait(100);
    wait(20500);
    require(controller.m_stereoRetry.isActive() && controller.m_gimbalRetry.isActive(), "存活无帧的双路接收均自动恢复");
    QImage frame(4, 4, QImage::Format_RGB32); frame.fill(Qt::green);
    QByteArray jpeg; QBuffer buffer(&jpeg); buffer.open(QIODevice::WriteOnly); frame.save(&buffer, "JPEG");
    controller.acceptFrame(0, jpeg); controller.acceptFrame(1, jpeg);
    require(controller.stereoFresh() && controller.gimbalFresh() && controller.status().contains(QStringLiteral("正常")), "新帧恢复后清除故障提示");
    controller.stop(); wait(2200);
    QFile rgb(directory.filePath("frames.rgb"));
    require(rgb.open(QIODevice::WriteOnly), "创建原始帧");
    QByteArray red(640 * 360 * 3, '\0'), blue(red.size(), '\0');
    for (qsizetype i = 0; i < red.size(); i += 3) { red[i] = char(255); blue[i + 2] = char(255); }
    rgb.write(red); rgb.write(blue); rgb.close();
    gst.open(QIODevice::WriteOnly | QIODevice::Truncate);
    gst.write(("#!/bin/sh\n/bin/cat '" + rgb.fileName().toUtf8() + "'\nexec /bin/sleep 30\n")); gst.close();
    controller.start(); wait(600);
    require(controller.gimbalFresh() && controller.image(1).pixelColor(1, 1).blue() == 255,
            "本地解码 RGB 帧进入地面站并显示最新画面");
    require(controller.m_gimbal.arguments().contains(QStringLiteral("location=rtsp://192.168.1.13:8554/gimbal"))
            && controller.m_stereo.arguments().contains(QStringLiteral("orangepi@192.168.1.13")),
            "双目与云台均使用更新后的机载地址");
    controller.stop(); wait(100);
    require(controller.m_stereo.state() == QProcess::NotRunning && controller.m_gimbal.state() == QProcess::NotRunning, "停止后不会留下接收进程或复活");
    gst.open(QIODevice::WriteOnly | QIODevice::Truncate); gst.write("#!/bin/sh\nexec /bin/sleep 30\n"); gst.close();
    QQmlEngine engine; engine.rootContext()->setContextProperty("videoController", &controller);
    engine.addImageProvider(QStringLiteral("livevideo"),new VideoImageProvider(&controller));
    QQmlComponent panel(&engine, QUrl::fromLocalFile(QStringLiteral(GCS_VIDEO_ROOT "/VideoPanel.qml")));
    QObject *root=panel.create(); if (!root) qFatal("QML create failed: %s", qPrintable(panel.errorString()));
    QObject *stereo=root->findChild<QObject *>(QStringLiteral("toggleStereoVideo"));
    QObject *gimbal=root->findChild<QObject *>(QStringLiteral("toggleGimbalVideo"));
    require(stereo && gimbal, "视频面板有两个独立按钮");
    QMetaObject::invokeMethod(stereo, "clicked"); wait(200);
    require(controller.stereoEnabled() && !controller.gimbalEnabled() && controller.m_stereo.state()==QProcess::Running && controller.m_gimbal.state()==QProcess::NotRunning, "只打开双目，不启动云台");
    QMetaObject::invokeMethod(gimbal, "clicked"); wait(200);
    require(controller.stereoEnabled() && controller.gimbalEnabled() && controller.m_gimbal.state()==QProcess::Running, "两路可同时打开");
    QMetaObject::invokeMethod(stereo, "clicked"); wait(200);
    require(!controller.stereoEnabled() && controller.gimbalEnabled() && controller.m_stereo.state()==QProcess::NotRunning && controller.m_gimbal.state()==QProcess::Running && !controller.m_stereoRetry.isActive(), "关闭双目不影响云台，也不重连双目");
    QMetaObject::invokeMethod(gimbal, "clicked"); wait(200);
    require(!controller.active() && !controller.gimbalEnabled() && controller.m_gimbal.state()==QProcess::NotRunning && !controller.m_gimbalRetry.isActive(), "关闭云台后停止接收");
    QMetaObject::invokeMethod(gimbal, "clicked"); wait(200);
    require(!controller.stereoEnabled() && controller.gimbalEnabled() && controller.m_stereo.state()==QProcess::NotRunning, "可单独打开云台");
    QQmlComponent mockTask(&engine);
    mockTask.setData("import QtQml; QtObject { property bool fresh: true; property var state: ({running: false, task: ''}); signal changed() }", QUrl());
    QObject *task=mockTask.create(); require(task, "创建任务状态替身");
    root->setProperty("taskController", QVariant::fromValue(task));
    QFile arucoFrame(directory.filePath("aruco.bin"));
    require(arucoFrame.open(QIODevice::WriteOnly), "构造识别图像帧");
    quint32 length=qToBigEndian<quint32>(quint32(jpeg.size()));
    arucoFrame.write("VID1",4); arucoFrame.write(reinterpret_cast<const char *>(&length),4); arucoFrame.write(jpeg); arucoFrame.close();
    ssh.open(QIODevice::WriteOnly | QIODevice::Truncate);
    ssh.write(("#!/bin/sh\n/bin/cat '" + arucoFrame.fileName().toUtf8() + "'\nexec /bin/sleep 30\n")); ssh.close();
    task->setProperty("state", QVariantMap{{"running",true},{"task","aruco"}});
    QMetaObject::invokeMethod(task,"changed"); wait(600);
    require(controller.gimbalAruco() && controller.gimbalFresh() && controller.image(1).pixelColor(1,1).green()>200, "ArUco 任务自动显示识别画面");
    std::function<bool(QQuickItem*)> readyImage=[&](QQuickItem *item) {
        if(item->property("source").toUrl().toString().startsWith(QStringLiteral("image://livevideo/gimbal/")) && item->property("status").toInt()==1)return true;
        for(auto child:item->childItems())if(readyImage(child))return true;
        return false;
    };
    require(readyImage(qobject_cast<QQuickItem*>(root)),"实际VideoPanel Image通过真实provider加载ArUco帧");
    require(controller.m_gimbal.program().contains("ssh") && controller.m_gimbal.arguments().last().contains("--source aruco") && !controller.stereoEnabled(), "识别模式只订阅机载图像，不启动双目");
    task->setProperty("state", QVariantMap{{"running",true},{"task","tracking"}});
    QMetaObject::invokeMethod(task,"changed"); wait(300);
    require(!controller.gimbalAruco() && !controller.gimbalFresh() && controller.m_gimbal.program().contains("gst-launch") && controller.trackingMode(), "切换人物任务时云台恢复原画，双目选择跟踪识别源");
    QMetaObject::invokeMethod(stereo,"clicked"); wait(400);
    require(controller.stereoEnabled() && controller.m_stereo.arguments().last().contains("--source tracking") && controller.stereoFresh(), "开启双目后自动显示人物跟踪识别画面");
    task->setProperty("state", QVariantMap{{"running",false},{"task","tracking"}});
    QMetaObject::invokeMethod(task,"changed"); wait(300);
    require(!controller.trackingMode() && controller.m_stereo.arguments().last().contains("--source raw"), "任务结束后双目恢复原画");
    controller.stop(); delete root; delete task;
}
