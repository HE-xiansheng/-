#include "sshsession.h"
#include "recordingcontroller.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QRegularExpression>
#include <QDebug>
#include <algorithm>
#include <cmath>

RecordingController::RecordingController(QObject *parent):QObject(parent){
    m_process.setProcessChannelMode(QProcess::MergedChannels);
    connect(&m_process,&QProcess::readyReadStandardOutput,this,[this](){m_output+=QString::fromUtf8(m_process.readAllStandardOutput());});
    m_timeout.setSingleShot(true);m_timeout.setInterval(15000);
    connect(&m_timeout,&QTimer::timeout,this,[this](){m_process.kill();});
    connect(&m_process,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error){
        if(error==QProcess::FailedToStart){m_timeout.stop();m_pendingAction.clear();m_message=tr("录制请求无法启动：")+m_process.errorString();emit changed();}
    });
    connect(&m_process,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,[this](int code,QProcess::ExitStatus state){
        m_timeout.stop();m_output+=QString::fromUtf8(m_process.readAllStandardOutput());
        qInfo().noquote()<<"录制管理返回"<<m_action<<code<<m_output.trimmed();
        QJsonObject result;
        for(const auto &line:m_output.split('\n')){const auto doc=QJsonDocument::fromJson(line.toUtf8());if(doc.isObject())result=doc.object();}
        if(code || state!=QProcess::NormalExit || !result.value("available").toBool()){
            m_pendingAction.clear();m_message=tr("录制请求结果未知：")+m_output.trimmed().right(500);
        }else if(m_action.startsWith("auto-")){
            m_pendingAction.clear();m_pendingId.clear();m_message.clear();
            m_value["auto_enabled"]=result.value("auto_enabled").toBool();
        }else{
            m_pendingId=result.value("request_id").toString();
            m_message=m_action=="start" ? tr("开始请求已提交，等待录制状态确认。") : tr("停止请求已提交，正在保存录包文件。");
            if(!result.contains("requested")){m_pendingAction.clear();m_message=m_action=="stop" ? tr("当前没有正在进行的录制。"):tr("录制已在进行中。");}
        }
        emit changed();emit clockChanged();
    });
    m_clock.setInterval(200);connect(&m_clock,&QTimer::timeout,this,&RecordingController::clockChanged);m_clock.start();
}
QString RecordingController::quote(QString s){return "'"+s.replace("'","'\\''")+"'";}
bool RecordingController::canStart() const{
    const auto p=phase();
    return !busy()&&fresh()&&m_value.value("available").toBool()&&m_pendingAction.isEmpty()
        &&m_value.value("pending_action").toString().isEmpty()&&m_value.value("active_bags_count").toInt()==0&&m_value.value("bag_pid").toInt()==0
        &&(p=="WAIT_MANUAL"||p=="STOPPED"||p=="NOT_RECORDING"||p=="ERROR"||p=="PREPARING");
}
bool RecordingController::canStop() const{
    const auto p=phase();
    return !busy()&&m_value.value("available").toBool()&&m_pendingAction!="stop"&&m_value.value("pending_action").toString()!="stop"
        &&(p=="RECORDING"||p=="STARTING_BAG"||p=="PREPARING"||(p=="ERROR"&&m_value.value("bag_pid").toInt()>0)||m_pendingAction=="start"||m_value.value("pending_action").toString()=="start");
}
void RecordingController::observe(const QVariantMap &value,bool freshValue){
    const double previous=elapsedSeconds();
    m_value=value;m_fresh=freshValue;m_received.restart();
    if(!fresh())m_frozenElapsed=previous;
    const auto p=phase();
    if((!m_pendingId.isEmpty()&&m_value.value("request_id").toString()==m_pendingId)||p=="ERROR")m_pendingAction.clear();
    if(p=="ERROR")m_message=m_value.value("error").toString();
    else if(p=="RECORDING"&&m_pendingAction.isEmpty())m_message.clear();
    else if(p=="STOPPED"&&m_pendingAction.isEmpty())m_message.clear();
    emit changed();emit clockChanged();
}
double RecordingController::elapsedSeconds() const{
    if(!fresh())return m_frozenElapsed;
    if(m_pendingAction=="start"&&phase()!="RECORDING")return 0;
    double elapsed=m_value.value("duration_s").toDouble();
    if(!std::isfinite(elapsed)||elapsed<0)elapsed=0;
    if(recording()&&m_received.isValid())elapsed+=std::clamp(m_value.value("status_age_s").toDouble(),0.,5.)+std::min(m_received.elapsed()/1000.,2.5);
    return elapsed;
}
QString RecordingController::autoText() const{
    if(!autoEnabled())return QString();
    if(!fresh())return tr("自动录制状态未知");
    if(!autoReady())return m_value.value("auto_error").toString().isEmpty() ? tr("自动录制未待命，请重新勾选") : m_value.value("auto_error").toString();
    return m_value.value("auto_phase").toString()=="WAIT_LANDING" ? tr("落地并锁桨后自动停止") : tr("等待 OFFBOARD 自动开始");
}
QString RecordingController::phaseText() const{
    if(!m_value.value("available").toBool())return tr("等待机载录制状态");
    if(!fresh())return tr("录制状态未知 / 采样过期");
    if(m_pendingAction=="stop"||m_value.value("pending_action").toString()=="stop")return tr("正在停止并保存…");
    if(m_pendingAction=="start"||m_value.value("pending_action").toString()=="start")return tr("正在准备开始录制…");
    const auto p=phase();
    if(p=="PREPARING")return tr("正在准备数据快照…");
    if(p=="STARTING_BAG")return tr("正在连接录包话题…");
    if(p=="RECORDING")return tr("正在录制");
    if(p=="STOPPING")return tr("正在保存…");
    if(p=="STOPPED")return m_value.value("bag_bytes").toDouble()>0 ? tr("已停止并保存") : tr("已停止 / 未生成录包");
    if(p=="ERROR")return tr("录制异常，请查看原因");
    return tr("未录制");
}
void RecordingController::request(const QString &action){
    if(action.startsWith("auto-") && (busy()||!fresh()))return;
    if((action=="start"&&!canStart())||(action=="stop"&&!canStop()))return;
    static const QRegularExpression hostPattern("^[a-zA-Z0-9][a-zA-Z0-9.-]{0,252}$");
    if(!hostPattern.match(m_host).hasMatch())return;
    m_action=action;m_output.clear();m_pendingAction=action.startsWith("auto-") ? QString() : action;m_pendingId.clear();m_message.clear();
    const QString remote="source /opt/ros/noetic/setup.bash; source ~/drone_platform/01_apps/ego_planner/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 PYTHONNOUSERSITE=1; exec /usr/bin/python3 -B ~/drone_platform/01_apps/ego_planner/recording_manage.py "+action;
    qInfo()<<"录制管理请求"<<action<<m_host;
    SshSession::apply(m_process);
    SshSession::start(m_process,{"-T","-o",SshSession::batchOption(),"-o","ConnectTimeout=5","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2","orangepi@"+m_host,"bash -lc "+quote(remote)});
    m_process.closeWriteChannel();m_timeout.start();emit changed();emit clockChanged();
}
