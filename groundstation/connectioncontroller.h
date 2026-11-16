#pragma once
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QtQml/qqmlregistration.h>

// 统一机载地址：先验证SSH，再复用现有只读点云/监视桥接；不启动定位或飞控执行。
class ConnectionController:public QObject {
    Q_OBJECT
    QML_NAMED_ELEMENT(ConnectionController)
    Q_PROPERTY(QString host READ host NOTIFY changed)
    Q_PROPERTY(bool busy READ busy NOTIFY changed)
    Q_PROPERTY(bool authenticated READ authenticated NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
public:
    explicit ConnectionController(QObject *parent=nullptr);
    ~ConnectionController() override;
    QString host() const{return m_host;}
    bool busy() const{return m_process.state()!=QProcess::NotRunning;}
    bool authenticated() const{return m_authenticated;}
    QString message() const{return m_message;}
    Q_INVOKABLE void connectTo(const QString &host,const QString &password,int cloudPort,int posePort,int preflightPort);
signals:
    void changed();
    void endpointChanging();
private:
    void finished(int,QProcess::ExitStatus);
    QProcess m_process;
#ifdef Q_OS_WIN
    QProcess m_bridge;
#endif
    QTimer m_timeout;
    QString m_host,m_pendingHost,m_message,m_output,m_stage;
    QByteArray m_pendingPassword;
    int m_cloudPort=9000,m_posePort=9001,m_preflightPort=9003;
    bool m_authenticated=false;
};
