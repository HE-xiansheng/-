#pragma once
#include <QObject>
#include <QProcess>
#include <QImage>
#include <QMutex>
#include <QTimer>
#include <QElapsedTimer>
#include <QQuickImageProvider>

// 上路可切换原画或人物识别，只管理本程序启动的检测节点；下路为现有云台 RTSP。
class VideoController : public QObject {
    Q_OBJECT
    Q_PROPERTY(bool active READ active NOTIFY changed)
    Q_PROPERTY(bool stereoEnabled READ stereoEnabled NOTIFY changed)
    Q_PROPERTY(bool gimbalEnabled READ gimbalEnabled NOTIFY changed)
    Q_PROPERTY(int stereoRevision READ stereoRevision NOTIFY changed)
    Q_PROPERTY(int gimbalRevision READ gimbalRevision NOTIFY changed)
    Q_PROPERTY(bool stereoFresh READ stereoFresh NOTIFY changed)
    Q_PROPERTY(bool gimbalFresh READ gimbalFresh NOTIFY changed)
    Q_PROPERTY(bool stereoSwitchFrameVisible READ stereoSwitchFrameVisible NOTIFY changed)
    Q_PROPERTY(bool stereoImagePersonMode READ stereoImagePersonMode NOTIFY changed)
    Q_PROPERTY(bool personMode READ personMode NOTIFY changed)
    Q_PROPERTY(bool gimbalAruco READ gimbalAruco NOTIFY changed)
    Q_PROPERTY(bool trackingMode READ trackingMode NOTIFY changed)
    Q_PROPERTY(QString status READ status NOTIFY changed)
public:
    explicit VideoController(QObject *parent = nullptr);
    ~VideoController() override;
    bool active() const { return m_active; }
    bool stereoEnabled() const { return m_enabled[0]; }
    bool gimbalEnabled() const { return m_enabled[1]; }
    int stereoRevision() const { return m_revision[0]; }
    int gimbalRevision() const { return m_revision[1]; }
    bool stereoFresh() const { return fresh(0); }
    bool gimbalFresh() const { return fresh(1); }
    bool stereoSwitchFrameVisible() const;
    bool stereoImagePersonMode() const { return m_stereoImagePersonMode; }
    bool personMode() const { return m_personMode; }
    bool gimbalAruco() const { return m_gimbalAruco; }
    bool trackingMode() const { return m_trackingMode; }
    QString status() const { return m_status; }
    QImage image(int index) const;
    Q_INVOKABLE void start();
    Q_INVOKABLE void stop();
    Q_INVOKABLE void setHost(const QString &host);
    Q_INVOKABLE void setPersonMode(bool enabled);
    Q_INVOKABLE void setGimbalAruco(bool enabled);
    Q_INVOKABLE void setTrackingMode(bool enabled);
    Q_INVOKABLE void setStreamEnabled(int index, bool enabled);
signals:
    void changed();
private:
    bool fresh(int i) const;
    bool startStereo();
    void scheduleStereoReconnect();
    void startGimbal();
    void scheduleGimbalReconnect();
    void consumeStereo();
    void consumeGimbal();
    void acceptFrame(int i, const QByteArray &data);
    QProcess m_stereo, m_gimbal;
    QByteArray m_stereoBuffer, m_gimbalBuffer;
    QByteArray m_stereoErrorBuffer;
    mutable QMutex m_mutex;
    QImage m_images[2];
    int m_revision[2] = {0, 0};
    qint64 m_received[2] = {-1, -1};
    qint64 m_stereoImageAt = -1;
    QElapsedTimer m_clock;
    QTimer m_timer;
    QTimer m_stereoRetry, m_gimbalRetry;
    qint64 m_started[2] = {-1, -1};
    int m_gimbalRetryDelayMs = 2000;
    bool m_active = false;
    bool m_enabled[2] = {false, false};
    bool m_personMode = false;
    bool m_switchingStereo = false;
    bool m_switchingGimbal = false;
    bool m_gimbalAruco = false;
    bool m_trackingMode = false;
    bool m_stereoSwitchFrame = false;
    bool m_stereoImagePersonMode = false;
    bool m_stereoRecovering = false;
    int m_stereoRetryDelayMs = 2000;
    QString m_host = QStringLiteral("192.168.1.235");
    QString m_status;
    QString m_personError;
};

// provider 与控制器共享最近解码帧；渲染线程读取时受互斥锁保护。
class VideoImageProvider : public QQuickImageProvider {
public:
    explicit VideoImageProvider(VideoController *controller) : QQuickImageProvider(QQuickImageProvider::Image), m_controller(controller) {}
    QImage requestImage(const QString &id, QSize *size, const QSize &requestedSize) override;
private:
    VideoController *m_controller;
};
