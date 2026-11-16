#ifndef LOCALIZATIONCONTROLLER_H
#define LOCALIZATIONCONTROLLER_H
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QVariantMap>
#include <QVariantList>
#include <QtQml/qqmlregistration.h>

// 左侧定位管理：复用既有 FAST-LIO / 原厂 Pose 转发，状态来自只读监视。
// 操作由机载守卫再次核实未解锁、来源身份及预览/执行器占用；不提供飞控控制接口。
class LocalizationController:public QObject {
    Q_OBJECT
    QML_NAMED_ELEMENT(LocalizationController)
    Q_PROPERTY(QString host MEMBER m_host NOTIFY changed)
    Q_PROPERTY(bool busy READ busy NOTIFY changed)
    Q_PROPERTY(bool fresh READ fresh NOTIFY changed)
    Q_PROPERTY(bool lioReady READ lioReady NOTIFY changed)
    Q_PROPERTY(bool relayReady READ relayReady NOTIFY changed)
    Q_PROPERTY(QString lioText READ lioText NOTIFY changed)
    Q_PROPERTY(QString relayText READ relayText NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
public:
    explicit LocalizationController(QObject *parent=nullptr);
    bool busy() const{return m_process.state()!=QProcess::NotRunning;}
    bool fresh() const{return m_fresh;}
    bool lioReady() const{return m_fresh&&m_lio.value("ok").toBool();}
    bool relayReady() const{return m_fresh&&m_relay.value("ok").toBool();}
    QString lioText() const{return text(m_lio,"lio_running");}
    QString relayText() const{return text(m_relay,"relay_running");}
    QString message() const{return m_message;}
    Q_INVOKABLE void observe(const QVariantMap &value,const QVariantList &rows,bool fresh);
    Q_INVOKABLE void start(){request("start");}
    Q_INVOKABLE void stop(){request("stop");}
signals:
    void changed();
    void sessionReset();
private:
    QString text(const QVariantMap &row,const QString &runningKey) const;
    void request(const QString &action);
    static QString quote(QString value);
    QProcess m_process;
    QTimer m_timeout;
    QVariantMap m_state,m_lio,m_relay;
    bool m_fresh=false,m_timedOut=false;
    QString m_host="192.168.1.235",m_action,m_output,m_message,m_session;
};
#endif
