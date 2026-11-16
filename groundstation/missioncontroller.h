#pragma once
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QVariantMap>
#include <QElapsedTimer>
#include <QtQml/qqmlregistration.h>

// 三项机载任务的SSH操作与只读监视。界面不运行控制循环，退出界面不结束飞行任务。
class MissionController : public QObject {
    Q_OBJECT
    QML_NAMED_ELEMENT(MissionController)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY changed)
    Q_PROPERTY(bool busy READ busy NOTIFY changed)
    Q_PROPERTY(bool fresh READ fresh NOTIFY changed)
    Q_PROPERTY(QVariantMap state READ state NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
    Q_PROPERTY(QVariantList waypoints READ waypoints NOTIFY waypointsChanged)
public:
    explicit MissionController(QObject *parent=nullptr);
    ~MissionController() override;
    QString host() const {return m_host;}
    void setHost(const QString &host);
    bool busy() const {return m_command.state()!=QProcess::NotRunning;}
    bool fresh() const {return m_observing && m_received>=0 && m_clock.elapsed()-m_received<4000;}
    QVariantMap state() const {return m_state;}
    QString message() const {return m_message;}
    QVariantList waypoints() const {return m_waypoints;}
    Q_INVOKABLE void addWaypoint(double x,double y,double z);
    Q_INVOKABLE void editWaypoint(int index,double z,bool photo);
    Q_INVOKABLE void removeWaypoint(int index);
    Q_INVOKABLE void clearWaypoints();
    Q_INVOKABLE void executeMapMission();
    Q_INVOKABLE void watch();
    Q_INVOKABLE void disconnectMonitor();
    Q_INVOKABLE void prepare(const QString &kind,const QString &missionFile=QString());
    Q_INVOKABLE void stopTask();
    Q_INVOKABLE void validateMission(const QString &missionFile);
signals:
    void changed();
    void waypointsChanged();
private:
    void invoke(const QStringList &arguments,const QByteArray &input=QByteArray());
    void startMonitor();
    void startSsh(QProcess &process,const QStringList &arguments,const QByteArray &input=QByteArray());
    static QString quote(QString text);
    QProcess m_command,m_monitor;
    QTimer m_timeout,m_retry,m_tick;
    QElapsedTimer m_clock;
    QByteArray m_output,m_monitorBuffer;
    QVariantMap m_state;
    QVariantList m_waypoints;
    QVariantMap m_mapSession;
    QString m_host=QStringLiteral("192.168.1.224"),m_message;
    qint64 m_received=-1,m_watchStarted=-1;
    bool m_observing=false;
};
