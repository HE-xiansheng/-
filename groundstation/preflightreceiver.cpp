#include "preflightreceiver.h"
#include <QJsonDocument>
#include <QJsonArray>
#include <QJsonObject>
#include <QNetworkDatagram>
#include <QSet>
#include <QDebug>
#include <cmath>

PreflightReceiver::PreflightReceiver(QObject *parent):QObject(parent)
{
    connect(&m_socket,&QUdpSocket::readyRead,this,&PreflightReceiver::read);
    connect(&m_timer,&QTimer::timeout,this,[this]() {
        const bool now=m_connected && m_clock.isValid() && m_clock.elapsed()+m_transportMs<=2500;
        if (now!=m_fresh) {
            m_fresh=now;
            if (!now && m_connected) m_status=tr("监视断流，旧采样不作为实时状态");
            emit statusChanged();
        }
        emit ageChanged();
    });
    m_timer.start(500);
}

void PreflightReceiver::setActive(bool value)
{
    if (m_active==value) return;
    m_active=value;
    bind();
}

void PreflightReceiver::setListenPort(int value)
{
    if (value<1 || value>65535 || value==m_port) return;
    m_port=value;
    bind();
}

void PreflightReceiver::bind()
{
    m_socket.close();m_clock.invalidate();m_connected=false;m_fresh=false;
    if (!m_active) m_status=tr("未连接监视");
    else if (m_socket.bind(QHostAddress::LocalHost,quint16(m_port))) {
        m_status=tr("等待机载只读采样");
        qInfo()<<"飞前状态仅监听本机127.0.0.1:"<<m_port;
    } else m_status=tr("本机监视端口%1绑定失败：%2").arg(m_port).arg(m_socket.errorString());
    emit statusChanged();emit ageChanged();
}

void PreflightReceiver::read()
{
    const QSet<QString> expected={"lidar","imu","lio","cloud","relay","fc_odom","fc","landed","ekf","rc","battery"};
    while (m_socket.hasPendingDatagrams()) {
        const auto raw=m_socket.receiveDatagram().data();
        if (raw.size()>60000) continue;
        QJsonParseError error;
        const auto doc=QJsonDocument::fromJson(raw,&error);
        if (error.error!=QJsonParseError::NoError || !doc.isObject()) continue;
        const auto obj=doc.object();
        if (obj.value("type").toString()!="PFS1" || obj.value("schema").toInt()!=1) continue;
        if (!m_expectedHost.isEmpty() && obj.value("source_host").toString()!=m_expectedHost) continue;
        if (!obj.value("connected").toBool()) {
            m_connected=false;m_fresh=false;
            m_host=obj.value("source_host").toString();
            m_status=obj.value("error").toString(tr("监视未连接"));
            emit statusChanged();continue;
        }
        const auto incoming=obj.value("rows").toArray();
        if (incoming.size()!=expected.size()) continue;
        QSet<QString> seen;
        bool valid=true;
        for (const auto &item:incoming) {
            const auto row=item.toObject();const auto key=row.value("key").toString();
            if (!expected.contains(key) || seen.contains(key) || !row.value("state").isString()) {valid=false;break;}
            seen.insert(key);
        }
        if (!valid) continue;
        const double transport=obj.value("transport_delay_s").toDouble(0.);
        if(!std::isfinite(transport)||transport<0||transport>2.5)continue;
        m_transportMs=qRound(transport*1000.);
        m_sensors=obj.value("sensors").toObject().toVariantMap();
        m_localization=obj.value("localization").toObject().toVariantMap();
        m_recording=obj.value("recording").toObject().toVariantMap();
        m_planning=obj.value("planning").toObject().toVariantMap();
        m_rows=incoming.toVariantList();m_host=obj.value("source_host").toString();
        m_connected=true;m_fresh=true;m_clock.restart();
        m_status=tr("机载最近采样 · 5 Hz状态快照");
        updateDetail();
        emit rowsChanged();emit statusChanged();emit ageChanged();
    }
}

void PreflightReceiver::selectRow(int index)
{
    m_selected=index;updateDetail();
}

void PreflightReceiver::updateDetail()
{
    if (m_selected<0 || m_selected>=m_rows.size()) return;
    const auto row=m_rows[m_selected].toMap();
    m_detail=row.value("topic").toString()+tr("\n发布者：")+row.value("publishers").toString();
    emit detailChanged();
}
