#include "sshsession.h"
#include "videocontroller.h"
#include <QFile>
#include <QDebug>
#include <QRegularExpression>
#include <QtEndian>
#include <QUrl>
#include <algorithm>

VideoController::VideoController(QObject *parent) : QObject(parent) {
    m_clock.start();
    connect(&m_stereo, &QProcess::readyReadStandardOutput, this, &VideoController::consumeStereo);
    connect(&m_gimbal, &QProcess::readyReadStandardOutput, this, &VideoController::consumeGimbal);
    for (QProcess *p : {&m_stereo, &m_gimbal}) {
        connect(p, &QProcess::readyReadStandardError, this, [this, p]() {
            const QByteArray error = p->readAllStandardError();
            if (!error.trimmed().isEmpty()) qWarning().noquote() << "视频:" << QString::fromUtf8(error).trimmed().left(1000);
            if (p != &m_stereo || m_switchingStereo || !m_personMode) return;
            m_stereoErrorBuffer += error;
            qsizetype newline = -1;
            while ((newline = m_stereoErrorBuffer.indexOf('\n')) >= 0) {
                const QByteArray line = m_stereoErrorBuffer.left(newline).trimmed();
                m_stereoErrorBuffer.remove(0, newline + 1);
                const QByteArray prefix("GCS_PERSON_ERROR: ");
                if (line.startsWith(prefix)) {
                    m_personError = QString::fromUtf8(line.mid(prefix.size())).left(300);
                    m_stereoSwitchFrame = false;
                    m_stereoRetry.stop(); m_stereoRecovering = false;
                    m_status = m_personError;
                    emit changed();
                }
            }
            if (m_stereoErrorBuffer.size() > 4096) m_stereoErrorBuffer.clear();
        });
        connect(p, &QProcess::errorOccurred, this, [this, p](QProcess::ProcessError) {
            if (m_active && m_enabled[p == &m_stereo ? 0 : 1] && !(p == &m_stereo && m_switchingStereo) && !(p == &m_gimbal && m_switchingGimbal)) {
                if (p == &m_stereo) {
                    m_received[0] = -1; m_stereoSwitchFrame = false;
                    if (m_personError.isEmpty()) scheduleStereoReconnect();
                    else m_status = m_personError;
                } else scheduleGimbalReconnect();
                qWarning() << m_status; emit changed();
            }
        });
        connect(p, qOverload<int, QProcess::ExitStatus>(&QProcess::finished), this, [this, p](int code, QProcess::ExitStatus) {
            if (m_active && m_enabled[p == &m_stereo ? 0 : 1] && !(p == &m_stereo && m_switchingStereo) && !(p == &m_gimbal && m_switchingGimbal)) {
                if (p == &m_stereo) {
                    m_received[0] = -1; m_stereoSwitchFrame = false;
                    if (m_personError.isEmpty()) scheduleStereoReconnect();
                    else m_status = m_personError;
                } else {
                    qWarning() << "云台接收进程退出" << code;
                    scheduleGimbalReconnect();
                }
                qWarning() << m_status; emit changed();
            }
        });
    }
    m_stereoRetry.setSingleShot(true);
    connect(&m_stereoRetry, &QTimer::timeout, this, [this]() {
        if (!m_active || !m_enabled[0] || m_switchingStereo) return;
        if (m_stereo.state() != QProcess::NotRunning) { scheduleStereoReconnect(); return; }
        m_status = tr("正在重新连接双目视频…");
        if (!startStereo()) m_stereoRecovering = false;
        emit changed();
    });
    m_gimbalRetry.setSingleShot(true);
    connect(&m_gimbalRetry, &QTimer::timeout, this, [this]() {
        if (!m_active || !m_enabled[1] || m_switchingGimbal) return;
        if (m_gimbal.state() != QProcess::NotRunning) { scheduleGimbalReconnect(); return; }
        startGimbal(); emit changed();
    });
    m_timer.setInterval(500);
    connect(&m_timer, &QTimer::timeout, this, [this]() {
        // SSH存活不等于持续出图；只重建本窗口的只读接收，不重启相机或ROS服务。
        for (int i = 0; i < 2; ++i) {
            QProcess &process = i == 0 ? m_stereo : m_gimbal;
            const qint64 last = m_received[i] >= 0 ? m_received[i] : m_started[i];
            const int deadline = i == 0 && m_personMode ? 45000 : 20000;
            if (!m_active || !m_enabled[i] || last < 0 || process.state() != QProcess::Running
                    || (i == 0 && (m_switchingStereo || !m_personError.isEmpty()))) continue;
            if (m_clock.elapsed() - last > deadline) {
                m_started[i] = m_clock.elapsed();
                qWarning() << (i == 0 ? "双目" : "云台") << "长时间无可解码视频帧，重建只读连接";
                process.kill(); // finished回调安排退避重连，不阻塞界面。
            }
        }
        emit changed();
    });
}
VideoController::~VideoController() { stop(); }
bool VideoController::fresh(int i) const { return m_active && m_enabled[i] && m_received[i] >= 0 && m_clock.elapsed() - m_received[i] < 3000; }
bool VideoController::stereoSwitchFrameVisible() const {
    // 保留的只是上一帧，最多显示到它实际接收后 3 秒。
    return m_active && m_enabled[0] && m_stereoSwitchFrame && m_stereoImageAt >= 0
            && m_clock.elapsed() - m_stereoImageAt < 3000;
}
QImage VideoController::image(int i) const { QMutexLocker lock(&m_mutex); return m_images[i]; }
void VideoController::setHost(const QString &host) { if (!m_active) m_host = host.trimmed(); }
void VideoController::scheduleStereoReconnect() {
    if (!m_active || !m_enabled[0] || m_switchingStereo || !m_personError.isEmpty() || m_stereoRetry.isActive()) return;
    m_stereoRecovering = true;
    m_status = tr("双目视频断流，%1 秒后重连").arg(m_stereoRetryDelayMs / 1000);
    m_stereoRetry.start(m_stereoRetryDelayMs);
    m_stereoRetryDelayMs = std::min(m_stereoRetryDelayMs * 2, 15000);
}
void VideoController::scheduleGimbalReconnect() {
    if (!m_active || !m_enabled[1] || m_switchingGimbal || m_gimbalRetry.isActive()) return;
    m_status = tr("云台连接中断，%1 秒后自动重连").arg(m_gimbalRetryDelayMs / 1000);
    m_gimbalRetry.start(m_gimbalRetryDelayMs);
    m_gimbalRetryDelayMs = std::min(m_gimbalRetryDelayMs * 2, 15000);
}
bool VideoController::startStereo() {
    m_personError.clear(); m_stereoErrorBuffer.clear();
    m_stereoBuffer.clear(); m_received[0] = -1; m_started[0] = m_clock.elapsed();
    QFile script(QStringLiteral(":/qt/qml/CustomGeometryExample/tools/video/ros_image_bridge.py"));
    if (!script.open(QIODevice::ReadOnly)) { m_status = tr("无法读取视频桥接脚本"); emit changed(); return false; }
    const QByteArray encoded = script.readAll().toBase64();
    const QString command = QStringLiteral("source /opt/ros/noetic/setup.bash; exec python3 -u -c 'exec(__import__(\"base64\").b64decode(\"%1\"))' --source %2")
            .arg(QString::fromLatin1(encoded), m_trackingMode ? QStringLiteral("tracking") : (m_personMode ? QStringLiteral("person") : QStringLiteral("raw")));
    SshSession::apply(m_stereo);
    SshSession::start(m_stereo, {QStringLiteral("-T"), QStringLiteral("-o"), SshSession::batchOption(), QStringLiteral("-o"), QStringLiteral("ConnectTimeout=5"), QStringLiteral("-o"), QStringLiteral("ServerAliveInterval=5"), QStringLiteral("-o"), QStringLiteral("ServerAliveCountMax=2"), QStringLiteral("orangepi@") + m_host, command});
    return true;
}
void VideoController::setPersonMode(bool enabled) {
    if (m_personMode == enabled) return;
    m_stereoRetry.stop(); m_stereoRecovering = false; m_stereoRetryDelayMs = 2000;
    const bool keepPreviousFrame = stereoFresh() || stereoSwitchFrameVisible();
    m_switchingStereo = true;
    if (m_stereo.state() != QProcess::NotRunning) { m_stereo.kill(); m_stereo.waitForFinished(1000); }
    // 短暂保留上一来源的末帧，仅用于显示明确标记的切换提示。
    m_stereoBuffer.clear(); m_received[0] = -1; ++m_revision[0];
    m_stereoSwitchFrame = keepPreviousFrame;
    if (!keepPreviousFrame) { QMutexLocker lock(&m_mutex); m_images[0] = QImage(); m_stereoImageAt = -1; }
    m_personMode = enabled;
    if (m_stereo.state() != QProcess::NotRunning) {
        // 旧进程未退出时继续丢弃其帧，避免旧来源被标成新来源。
        m_stereoSwitchFrame = false;
        m_status = tr("切换失败：旧视频进程未退出，请停止后重试");
        emit changed();
        return;
    }
    m_switchingStereo = false;
    if (m_active && m_enabled[0]) {
        m_status = tr("正在接收；未出画时请检查来源");
        if (!startStereo()) m_stereoSwitchFrame = false;
    }
    qInfo() << "双目视频来源切换" << (m_personMode ? "人物识别" : "原画面");
    emit changed();
}
void VideoController::setTrackingMode(bool enabled) {
    if (m_trackingMode == enabled) return;
    m_switchingStereo = true; m_stereoRetry.stop(); m_stereoRetryDelayMs = 2000;
    if (m_stereo.state() != QProcess::NotRunning) { m_stereo.kill(); m_stereo.waitForFinished(1000); }
    m_stereoBuffer.clear(); m_stereoErrorBuffer.clear(); m_personError.clear(); m_received[0] = -1;
    m_stereoImageAt = -1; m_stereoSwitchFrame = false; m_stereoRecovering = false;
    { QMutexLocker lock(&m_mutex); m_images[0] = QImage(); }
    m_trackingMode = enabled; ++m_revision[0];
    if (m_stereo.state() != QProcess::NotRunning) {
        m_switchingStereo = false; m_status = tr("切换失败：旧双目视频进程未退出"); emit changed(); return;
    }
    m_switchingStereo = false;
    if (m_enabled[0]) {
        m_status = enabled ? tr("等待人物跟踪识别画面") : tr("正在连接双目视频");
        startStereo();
    }
    emit changed();
}
void VideoController::setGimbalAruco(bool enabled) {
    if (m_gimbalAruco == enabled) return;
    m_switchingGimbal = true; m_gimbalRetry.stop(); m_gimbalRetryDelayMs = 2000;
    if (m_gimbal.state() != QProcess::NotRunning) { m_gimbal.kill(); m_gimbal.waitForFinished(1000); }
    m_gimbalBuffer.clear(); m_received[1] = -1; m_started[1] = -1;
    { QMutexLocker lock(&m_mutex); m_images[1] = QImage(); }
    m_gimbalAruco = enabled; ++m_revision[1];
    if (m_gimbal.state() != QProcess::NotRunning) {
        m_switchingGimbal = false; m_status = tr("切换失败：旧云台视频进程未退出"); emit changed(); return;
    }
    m_switchingGimbal = false;
    if (m_enabled[1]) {
        m_status = enabled ? tr("等待 ArUco 识别画面") : tr("正在连接云台原画");
        startGimbal();
    }
    emit changed();
}
void VideoController::start() {
    setStreamEnabled(0, true);
    setStreamEnabled(1, true);
}
void VideoController::setStreamEnabled(int index, bool enabled) {
    if (index < 0 || index > 1 || m_enabled[index] == enabled) return;
    if (enabled && !QRegularExpression(QStringLiteral("^[A-Za-z0-9.:-]+$")).match(m_host).hasMatch()) {
        m_status = tr("机载地址无效"); emit changed(); return;
    }
    QProcess &process = index == 0 ? m_stereo : m_gimbal;
    QTimer &retry = index == 0 ? m_stereoRetry : m_gimbalRetry;
    if (enabled && process.state() != QProcess::NotRunning) {
        m_status = tr("视频进程仍在退出，请稍后重试"); emit changed(); return;
    }
    // 先改变开关，再关闭进程，防止退出信号触发重连。
    m_enabled[index] = enabled;
    m_active = m_enabled[0] || m_enabled[1];
    retry.stop();
    if (index == 0) {
        m_stereoRetryDelayMs = 2000; m_stereoRecovering = false;
        m_switchingStereo = false; m_stereoSwitchFrame = false; m_stereoImageAt = -1;
        m_stereoBuffer.clear(); m_personError.clear();
    } else {
        m_gimbalRetryDelayMs = 2000; m_gimbalBuffer.clear();
    }
    m_received[index] = -1; m_started[index] = -1;
    if (enabled) {
        m_timer.start();
        m_status = index == 0 ? tr("正在接收双目视频") : tr("正在接收云台视频");
        if (index == 0) {
            if (!startStereo()) { m_enabled[0] = false; m_active = m_enabled[1]; if (!m_active) m_timer.stop(); }
        } else startGimbal();
    } else {
        if (process.state() != QProcess::NotRunning) { process.kill(); process.waitForFinished(1000); }
        { QMutexLocker lock(&m_mutex); m_images[index] = QImage(); }
        if (!m_active) m_timer.stop();
        m_status = index == 0 ? tr("双目视频已关闭") : tr("云台视频已关闭");
    }
    ++m_revision[index]; emit changed();
}
void VideoController::startGimbal() {
    m_gimbalBuffer.clear(); m_received[1] = -1; m_started[1] = m_clock.elapsed();
    if (m_gimbalAruco) {
        QFile script(QStringLiteral(":/qt/qml/CustomGeometryExample/tools/video/ros_image_bridge.py"));
        if (!script.open(QIODevice::ReadOnly)) { m_status = tr("无法读取识别画面桥接脚本"); emit changed(); return; }
        const QString command = QStringLiteral("source /opt/ros/noetic/setup.bash; exec python3 -u -c 'exec(__import__(\"base64\").b64decode(\"%1\"))' --source aruco")
                .arg(QString::fromLatin1(script.readAll().toBase64()));
        SshSession::apply(m_gimbal);
        SshSession::start(m_gimbal, {QStringLiteral("-T"), QStringLiteral("-o"), SshSession::batchOption(), QStringLiteral("-o"), QStringLiteral("ConnectTimeout=5"), QStringLiteral("-o"), QStringLiteral("ServerAliveInterval=5"), QStringLiteral("-o"), QStringLiteral("ServerAliveCountMax=2"), QStringLiteral("orangepi@") + m_host, command});
        return;
    }
    QUrl url;
    url.setScheme(QStringLiteral("rtsp"));
    url.setHost(m_host);
    url.setPort(8554);
    url.setPath(QStringLiteral("/gimbal"));
#ifdef Q_OS_WIN
    // Windows 随包 FFmpeg 输出同样的 640×360 RGB 原始帧，沿用现有帧处理。
    m_gimbal.start(QStringLiteral("ffmpeg"), {
        "-hide_banner", "-loglevel", "error", "-nostdin", "-rtsp_transport", "tcp",
        "-rw_timeout", "5000000", "-fflags", "nobuffer", "-flags", "low_delay",
        "-i", url.toString(), "-an", "-sn", "-vf",
        "scale=640:360:force_original_aspect_ratio=decrease,pad=640:360:(ow-iw)/2:(oh-ih)/2",
        "-pix_fmt", "rgb24", "-f", "rawvideo", "pipe:1"});
#else
    // H.264 直接经 RTSP 到本机解码；队列只保留最新画面。
    m_gimbal.start(QStringLiteral("gst-launch-1.0"), {
        QStringLiteral("-q"), QStringLiteral("rtspsrc"),
        QStringLiteral("location=") + url.toString(), QStringLiteral("protocols=tcp"),
        QStringLiteral("latency=100"), QStringLiteral("drop-on-latency=true"),
        QStringLiteral("tcp-timeout=5000000"), QStringLiteral("!"),
        QStringLiteral("rtph264depay"), QStringLiteral("!"), QStringLiteral("h264parse"),
        QStringLiteral("!"), QStringLiteral("avdec_h264"), QStringLiteral("!"),
        QStringLiteral("queue"), QStringLiteral("max-size-buffers=1"),
        QStringLiteral("max-size-bytes=0"), QStringLiteral("max-size-time=0"),
        QStringLiteral("leaky=downstream"), QStringLiteral("!"),
        QStringLiteral("videoconvert"), QStringLiteral("!"), QStringLiteral("videoscale"),
        QStringLiteral("add-borders=true"), QStringLiteral("!"),
        QStringLiteral("video/x-raw,format=RGB,width=640,height=360,pixel-aspect-ratio=1/1"),
        QStringLiteral("!"), QStringLiteral("fdsink"), QStringLiteral("fd=1"),
        QStringLiteral("sync=false")});
#endif
}
void VideoController::stop() {
    m_enabled[0] = m_enabled[1] = false; m_active = false; m_timer.stop(); m_stereoRetry.stop(); m_gimbalRetry.stop(); m_stereoRecovering = false;
    for (QProcess *p : {&m_stereo, &m_gimbal}) { if (p->state() != QProcess::NotRunning) { p->kill(); p->waitForFinished(1000); } }
    m_received[0] = m_received[1] = -1; m_stereoImageAt = -1; m_stereoSwitchFrame = false; m_stereoBuffer.clear(); m_gimbalBuffer.clear();
    { QMutexLocker lock(&m_mutex); m_images[0] = QImage(); m_images[1] = QImage(); }
    m_status = tr("视频已停止"); emit changed();
}
void VideoController::acceptFrame(int i, const QByteArray &data) {
    QImage img = QImage::fromData(data);
    if (img.isNull()) return;
    { QMutexLocker lock(&m_mutex); m_images[i] = std::move(img); }
    m_received[i] = m_clock.elapsed();
    if (i == 0) {
        m_stereoImageAt = m_received[i]; m_stereoImagePersonMode = m_personMode; m_stereoSwitchFrame = false;
        if (m_stereoRecovering) m_status = tr("双目视频已恢复");
        m_stereoRecovering = false; m_stereoRetryDelayMs = 2000;
    }
    if (i == 1) {
        if (m_gimbalRetryDelayMs > 2000) m_status = tr("云台视频已恢复");
        m_gimbalRetryDelayMs = 2000; m_gimbalRetry.stop();
    }
    if (fresh(0) && fresh(1)) m_status = tr("双目、云台视频正常");
    else if (i == 0 && !m_enabled[1]) m_status = tr("双目视频正常");
    else if (i == 1 && !m_enabled[0]) m_status = tr("云台视频正常");
    ++m_revision[i]; emit changed();
}
void VideoController::consumeStereo() {
    if (!m_enabled[0] || m_switchingStereo) { m_stereo.readAllStandardOutput(); return; }
    m_stereoBuffer += m_stereo.readAllStandardOutput();
    while (true) {
        int at = m_stereoBuffer.indexOf("VID1");
        if (at < 0) { if (m_stereoBuffer.size() > 3) m_stereoBuffer = m_stereoBuffer.right(3); return; }
        if (at) m_stereoBuffer.remove(0, at);
        if (m_stereoBuffer.size() < 8) return;
        quint32 size = qFromBigEndian<quint32>(reinterpret_cast<const uchar *>(m_stereoBuffer.constData() + 4));
        if (!size || size > 8 * 1024 * 1024) { m_stereoBuffer.remove(0, 4); continue; }
        if (m_stereoBuffer.size() < 8 + qsizetype(size)) return;
        acceptFrame(0, m_stereoBuffer.mid(8, size)); m_stereoBuffer.remove(0, 8 + size);
    }
}
void VideoController::consumeGimbal() {
    if (!m_enabled[1] || m_switchingGimbal) { m_gimbal.readAllStandardOutput(); return; }
    if (m_gimbalAruco) {
        m_gimbalBuffer += m_gimbal.readAllStandardOutput();
        while (true) {
            int at = m_gimbalBuffer.indexOf("VID1");
            if (at < 0) { if (m_gimbalBuffer.size() > 3) m_gimbalBuffer = m_gimbalBuffer.right(3); return; }
            if (at) m_gimbalBuffer.remove(0, at);
            if (m_gimbalBuffer.size() < 8) return;
            quint32 size = qFromBigEndian<quint32>(reinterpret_cast<const uchar *>(m_gimbalBuffer.constData() + 4));
            if (!size || size > 8 * 1024 * 1024) { m_gimbalBuffer.remove(0, 4); continue; }
            if (m_gimbalBuffer.size() < 8 + qsizetype(size)) return;
            acceptFrame(1, m_gimbalBuffer.mid(8, size)); m_gimbalBuffer.remove(0, 8 + size);
        }
    }
    // 固定大小 RGB 帧：丢弃完整旧帧，只展示本次收到的最新帧。
    constexpr qsizetype frameBytes = 640 * 360 * 3;
    m_gimbalBuffer += m_gimbal.readAllStandardOutput();
    const qsizetype complete = m_gimbalBuffer.size() / frameBytes;
    if (!complete) return;
    const qsizetype last = (complete - 1) * frameBytes;
    QImage frame(reinterpret_cast<const uchar *>(m_gimbalBuffer.constData() + last),
                 640, 360, 640 * 3, QImage::Format_RGB888);
    { QMutexLocker lock(&m_mutex); m_images[1] = frame.copy(); }
    m_gimbalBuffer.remove(0, complete * frameBytes);
    m_received[1] = m_clock.elapsed();
    if (m_gimbalRetryDelayMs > 2000) m_status = tr("云台视频已恢复");
    m_gimbalRetryDelayMs = 2000; m_gimbalRetry.stop();
    if (fresh(0) && fresh(1)) m_status = tr("双目、云台视频正常");
    else if (!m_enabled[0]) m_status = tr("云台视频正常");
    ++m_revision[1]; emit changed();
}
QImage VideoImageProvider::requestImage(const QString &id, QSize *size, const QSize &) {
    QImage img = m_controller->image(id.startsWith(QStringLiteral("stereo")) ? 0 : 1);
    if (size) *size = img.size(); return img;
}
