#ifndef PLANNERCONTROLLER_H
#define PLANNERCONTROLLER_H
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QVariantMap>
#include <QVector3D>
#include <QtQml/qqmlregistration.h>

// Ego操作适配：调用既有机载会话、自动目标及执行器管理入口。
// 与三项任务入口协作，避免同一界面同时请求不同飞行控制器。
class PlannerController: public QObject {
    Q_OBJECT
    QML_NAMED_ELEMENT(PlannerController)
    Q_PROPERTY(bool externalControlActive READ externalControlActive WRITE setExternalControlActive NOTIFY changed)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY changed)
    Q_PROPERTY(bool starting READ starting NOTIFY changed)
    Q_PROPERTY(bool busy READ busy NOTIFY changed)
    Q_PROPERTY(bool canPreview READ canPreview NOTIFY changed)
    Q_PROPERTY(bool canConfirm READ canConfirm NOTIFY changed)
    Q_PROPERTY(bool stopPending READ stopPending NOTIFY changed)
    Q_PROPERTY(bool stopComplete READ stopComplete NOTIFY changed)
    Q_PROPERTY(bool sessionInvalid READ sessionInvalid NOTIFY changed)
    Q_PROPERTY(bool previewSuppressed READ previewSuppressed NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
public:
    explicit PlannerController(QObject *parent=nullptr);
    bool externalControlActive() const{return m_externalControlActive;}
    void setExternalControlActive(bool value){if(value!=m_externalControlActive){m_externalControlActive=value;emit changed();}}
    QString host() const{return m_host;}
    void setHost(const QString &host);
    bool starting() const{return busy()&&m_stage=="start";}
    bool busy() const{return m_process.state()!=QProcess::NotRunning;}
    bool canPreview() const;
    bool canConfirm() const;
    bool stopPending() const{return m_stopPending;}
    bool stopComplete() const{return m_stopComplete;}
    bool sessionInvalid() const{return m_sessionInvalid;}
    bool previewSuppressed() const{return m_previewSuppressed;}
    QString message() const{return m_message;}
    Q_INVOKABLE void observe(const QVariantMap &planning,bool fresh);
    Q_INVOKABLE void invalidateSession();
    Q_INVOKABLE void start();
    Q_INVOKABLE void stop();
    Q_INVOKABLE void preview(const QVector3D &xyz);
    Q_INVOKABLE void confirm();
signals:
    void changed();
    void toastRequested(const QString &message);
private:
    void command(const QString &stage,const QString &remote,const QByteArray &input={});
    void checkStop();
    void finished(int code,QProcess::ExitStatus status);
    static QString quote(QString value);
    static QString navigation();
    QProcess m_process;
    QTimer m_timeout;
    QTimer m_stopPoll;
    QString m_host=QStringLiteral("192.168.1.235"),m_stage,m_output,m_token,m_confirmingToken;
    QString m_message;
    QVariantMap m_planning;
    bool m_externalControlActive=false;
    bool m_fresh=false;
    bool m_requestedCandidate=false,m_confirmingCandidate=false;
    double m_tokenSnapshot=0.;
    bool m_stopPending=false,m_stopComplete=false,m_previewSuppressed=false,m_sessionInvalid=false;
    QVector3D m_requested;
};
#endif
