#ifndef PREFLIGHTRECEIVER_H
#define PREFLIGHTRECEIVER_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QElapsedTimer>
#include <QVariantList>
#include <QtQml/qqmlregistration.h>

// 接收本机只读监视桥的采样结果；仅显示，不具备ROS或飞控控制接口。
class PreflightReceiver : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(PreflightReceiver)
    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY statusChanged)
    Q_PROPERTY(int listenPort READ listenPort WRITE setListenPort NOTIFY statusChanged)
    Q_PROPERTY(bool fresh READ fresh NOTIFY statusChanged)
    Q_PROPERTY(QVariantMap sensors READ sensors NOTIFY rowsChanged)
    Q_PROPERTY(QVariantMap localization READ localization NOTIFY rowsChanged)
    Q_PROPERTY(QVariantMap recording READ recording NOTIFY rowsChanged)
    Q_PROPERTY(QVariantMap planning READ planning NOTIFY rowsChanged)
    Q_PROPERTY(QVariantList rows READ rows NOTIFY rowsChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(QString sourceHost READ sourceHost NOTIFY statusChanged)
    Q_PROPERTY(QString detail READ detail NOTIFY detailChanged)
    Q_PROPERTY(QString expectedHost MEMBER m_expectedHost)
    Q_PROPERTY(double monitorAge READ monitorAge NOTIFY ageChanged)
public:
    explicit PreflightReceiver(QObject *parent=nullptr);
    bool active() const { return m_active; }
    int listenPort() const { return m_port; }
    bool fresh() const { return m_fresh; }
    QVariantMap sensors() const {return m_sensors;}
    QVariantMap localization() const {return m_localization;}
    QVariantMap recording() const {return m_recording;}
    QVariantMap planning() const {return m_planning;}
    QVariantList rows() const { return m_rows; }
    QString status() const { return m_status; }
    QString sourceHost() const { return m_host; }
    QString detail() const { return m_detail; }
    double monitorAge() const { return m_clock.isValid() ? (m_clock.elapsed()+m_transportMs)/1000.0 : -1.; }
    void setActive(bool active);
    void setListenPort(int port);
    Q_INVOKABLE void selectRow(int index);
signals:
    void rowsChanged();
    void statusChanged();
    void ageChanged();
    void detailChanged();
private:
    void bind();
    void read();
    void updateDetail();
    QUdpSocket m_socket;
    QTimer m_timer;
    QElapsedTimer m_clock;
    bool m_active=false, m_connected=false, m_fresh=false;
    int m_port=9003, m_selected=-1,m_transportMs=0;
    QVariantList m_rows;
    QVariantMap m_planning,m_recording,m_localization,m_sensors;
    QString m_status=QStringLiteral("未连接监视"), m_host;
    QString m_expectedHost;
    QString m_detail=QStringLiteral("点击一行查看话题与发布者");
};
#endif
