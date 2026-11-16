#ifndef RECORDINGCONTROLLER_H
#define RECORDINGCONTROLLER_H
#include <QObject>
#include <QProcess>
#include <QElapsedTimer>
#include <QTimer>
#include <QVariantMap>
#include <QtQml/qqmlregistration.h>

// 录制管理与飞行操作使用独立进程；只请求原录包管理器，不发布飞控命令。
class RecordingController:public QObject {
    Q_OBJECT
    QML_NAMED_ELEMENT(RecordingController)
    Q_PROPERTY(QString host MEMBER m_host NOTIFY changed)
    Q_PROPERTY(bool autoEnabled READ autoEnabled NOTIFY changed)
    Q_PROPERTY(bool autoReady READ autoReady NOTIFY changed)
    Q_PROPERTY(QString autoText READ autoText NOTIFY changed)
    Q_PROPERTY(bool busy READ busy NOTIFY changed)
    Q_PROPERTY(bool canStart READ canStart NOTIFY changed)
    Q_PROPERTY(bool canStop READ canStop NOTIFY changed)
    Q_PROPERTY(bool recording READ recording NOTIFY changed)
    Q_PROPERTY(bool fresh READ fresh NOTIFY changed)
    Q_PROPERTY(QString phaseText READ phaseText NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
    Q_PROPERTY(QString outputDirectory READ outputDirectory NOTIFY changed)
    Q_PROPERTY(double elapsedSeconds READ elapsedSeconds NOTIFY clockChanged)
    Q_PROPERTY(double bagMegabytes READ bagMegabytes NOTIFY changed)
public:
    explicit RecordingController(QObject *parent=nullptr);
    bool autoEnabled() const{return m_value.value("auto_enabled").toBool();}
    bool autoReady() const{return fresh()&&m_value.value("auto_running").toBool()&&m_value.value("auto_fresh").toBool()&&m_value.value("auto_phase").toString()!="ERROR";}
    QString autoText() const;
    bool busy() const{return m_process.state()!=QProcess::NotRunning;}
    bool fresh() const{return m_fresh && m_value.value("status_fresh").toBool();}
    bool recording() const{return fresh() && phase()=="RECORDING";}
    bool canStart() const;
    bool canStop() const;
    QString phaseText() const;
    QString message() const{return m_message;}
    QString outputDirectory() const{return m_value.value("run").toString();}
    double elapsedSeconds() const;
    double bagMegabytes() const{return m_value.value("bag_bytes").toDouble()/1e6;}
    Q_INVOKABLE void observe(const QVariantMap &value,bool fresh);
    Q_INVOKABLE void setAuto(bool enabled){request(enabled ? "auto-on" : "auto-off");}
    Q_INVOKABLE void start(){request("start");}
    Q_INVOKABLE void stop(){request("stop");}
signals:
    void changed();
    void clockChanged();
private:
    QString phase() const{return m_value.value("phase").toString();}
    void request(const QString &action);
    static QString quote(QString value);
    QProcess m_process;
    QTimer m_timeout,m_clock;
    QElapsedTimer m_received;
    QVariantMap m_value;
    bool m_fresh=false;
    double m_frozenElapsed=0.;
    QString m_host="192.168.1.235",m_action,m_pendingAction,m_pendingId,m_output,m_message;
};
#endif
