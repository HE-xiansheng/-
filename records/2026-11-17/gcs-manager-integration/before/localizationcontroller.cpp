#include "remoteros2.h"
#include "sshsession.h"
#include "localizationcontroller.h"
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QDebug>

LocalizationController::LocalizationController(QObject *parent):QObject(parent){
    m_process.setProcessChannelMode(QProcess::MergedChannels);
    connect(&m_process,&QProcess::readyReadStandardOutput,this,[this](){m_output+=QString::fromUtf8(m_process.readAllStandardOutput());if(m_output.size()>16000)m_output=m_output.right(16000);});
    m_timeout.setSingleShot(true);m_timeout.setInterval(70000);
    connect(&m_timeout,&QTimer::timeout,this,[this](){m_timedOut=true;m_process.kill();});
    connect(&m_process,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error){
        if(error==QProcess::FailedToStart){m_timeout.stop();m_message=tr("定位管理无法启动：")+m_process.errorString();emit changed();}
    });
    connect(&m_process,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,[this](int code,QProcess::ExitStatus status){
        m_timeout.stop();m_output+=QString::fromUtf8(m_process.readAllStandardOutput());
        qInfo().noquote()<<"定位与转发返回"<<m_action<<code<<m_output.trimmed();
        QJsonObject result;
        for(const auto &line:m_output.split('\n')){const auto doc=QJsonDocument::fromJson(line.toUtf8());if(doc.isObject())result=doc.object();}
        if(m_timedOut)m_message=tr("定位请求超时，实际结果未知，请查看实时状态。");
        else if(code||status!=QProcess::NormalExit||!result.value("ok").toBool())
            m_message=result.value("message").toString(tr("定位请求结果未知：")+m_output.trimmed().right(500));
        else m_message=m_action=="start" ? tr("保持机体静止，等待定位和转发数据。") : QString();
        emit changed();
    });
}
QString LocalizationController::quote(QString value){return "'"+value.replace("'","'\\''")+"'";}
QString LocalizationController::text(const QVariantMap &row,const QString &key) const{
    if(!m_fresh)return tr("状态未知 / 采样过期");
    if(row.value("ok").toBool())return tr("正常");
    const QString state=row.value("state").toString();
    if(state!="缺失/过期"&&!state.isEmpty())return state;
    return m_state.value(key).toBool() ? tr("运行中 / 等待数据") : tr("无数据 / 未运行");
}
void LocalizationController::observe(const QVariantMap &value,const QVariantList &rows,bool freshValue){
    m_state=value;m_fresh=freshValue&&value.contains("lio_running");m_lio.clear();m_relay.clear();
    for(const auto &item:rows){const auto row=item.toMap();if(row.value("key")=="lio")m_lio=row;if(row.value("key")=="relay")m_relay=row;}
    const QString session=value.value("session_id").toString();
    if(m_fresh&&!session.isEmpty()&&session!=m_session){const bool changed=!m_session.isEmpty();m_session=session;if(changed)emit sessionReset();}
    if(lioReady()&&relayReady()&&m_message==tr("保持机体静止，等待定位和转发数据。"))m_message.clear();
    emit changed();
}
void LocalizationController::request(const QString &action){
    if(busy()||!m_fresh)return;
    static const QRegularExpression valid("^[a-zA-Z0-9][a-zA-Z0-9.-]{0,252}$");
    if(!valid.match(m_host).hasMatch())return;
    if(action=="start"&&!m_state.value("lio_running").toBool())emit sessionReset();
    m_action=action;m_timedOut=false;m_output.clear();m_message=action=="start" ? tr("正在准备定位与转发…") : tr("正在停止定位与转发…");
    const QString remote=RemoteRos2::manager("localization",{action});
    qInfo()<<"定位与转发请求"<<action<<m_host;
    SshSession::apply(m_process);
    SshSession::start(m_process,{"-T","-o",SshSession::batchOption(),"-o","ConnectTimeout=5","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2",RemoteRos2::user()+"@"+m_host,remote});
    m_process.closeWriteChannel();m_timeout.start();emit changed();
}
