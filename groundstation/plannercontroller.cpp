#include "sshsession.h"
#include "plannercontroller.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QRegularExpression>
#include <QDebug>
#include <QFile>
#include <cmath>

PlannerController::PlannerController(QObject *parent):QObject(parent) {
    m_process.setProcessChannelMode(QProcess::MergedChannels);
    connect(&m_process,&QProcess::readyReadStandardOutput,this,[this](){
        m_output+=QString::fromUtf8(m_process.readAllStandardOutput());
        if(m_output.size()>20000)m_output=m_output.right(20000);
    });
    connect(&m_process,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,&PlannerController::finished);
    connect(&m_process,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error){
        if(error==QProcess::FailedToStart){m_timeout.stop();m_stopPending=false;m_message=tr("SSH无法启动：")+m_process.errorString();emit changed();}
    });
    m_timeout.setSingleShot(true);m_timeout.setInterval(30000);
    connect(&m_timeout,&QTimer::timeout,this,[this](){m_stage="timeout";m_process.kill();});
    m_stopPoll.setSingleShot(true);m_stopPoll.setInterval(1000);
    connect(&m_stopPoll,&QTimer::timeout,this,&PlannerController::checkStop);
}
QString PlannerController::quote(QString v){return "'"+v.replace("'","'\\''")+"'";}
QString PlannerController::navigation(){return "source /opt/ros/noetic/setup.bash; source ~/drone_platform/01_apps/ego_planner/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 PYTHONNOUSERSITE=1; /usr/bin/python3 -B ~/drone_platform/01_apps/ego_planner/map_navigation.py ";}
void PlannerController::setHost(const QString &v){
    static const QRegularExpression valid("^[a-zA-Z0-9][a-zA-Z0-9.-]{0,252}$");
    if(!busy() && valid.match(v).hasMatch() && v!=m_host){m_host=v;m_token.clear();emit changed();}
}
bool PlannerController::canPreview() const {
    const auto s=m_planning.value("status").toMap();
    return !busy() && !m_stopPending && !m_externalControlActive;
}
bool PlannerController::canConfirm() const { return false; }

void PlannerController::observe(const QVariantMap &p,bool fresh){
    m_planning=p;m_fresh=fresh;
    const auto s=p.value("status").toMap();
    emit changed();
}
void PlannerController::command(const QString &stage,const QString &remote,const QByteArray &input){
    if(busy())return;
    if(m_externalControlActive && (stage=="start" || stage=="preview" || stage=="execution_status" || stage=="enable")) {
        m_message=tr("请先停止当前航点、降落或跟踪任务，再使用Ego控制入口。");emit changed();return;
    }
    m_stage=stage;m_output.clear();
    if(stage=="start")m_message=tr("正在启动静态预览…");
    else if(stage=="preview")m_message=tr("正在生成目标预览…");
    else if(stage=="confirm")m_message=tr("正在确认当前目标…");
    else if(stage=="stop")m_message=tr("正在请求停止避障…");
    else if(stage=="stop_guard")m_message=tr("正在核对停止状态…");
    else if(stage=="enable")m_message=tr("正在启用PX4接入…");
    else m_message=tr("正在核对执行状态…");
    qInfo().noquote()<<"EGO-Planner操作"<<stage<<"host"<<m_host;
    SshSession::apply(m_process);
    SshSession::start(m_process,{"-T","-o",SshSession::batchOption(),"-o","ConnectTimeout=6","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2","orangepi@"+m_host,"bash -lc "+quote(remote)});
    if(!input.isEmpty())m_process.write(input);
    m_process.closeWriteChannel();
    m_timeout.start(stage=="start" ? 90000 : 30000);emit changed();
}
void PlannerController::invalidateSession(){
    m_token.clear();m_planning.clear();m_fresh=false;m_previewSuppressed=true;m_sessionInvalid=true;emit changed();
}
void PlannerController::start(){if(!m_stopPending)command("start","bash ~/drone_platform/01_apps/ego_planner/run_board.sh");}
void PlannerController::stop(){
    if(busy() || m_stopPending)return;
    m_token.clear();
    m_stopPending=true;m_stopComplete=false;m_previewSuppressed=true;
    // 既有stop-request在OFFBOARD内保持，遥控切POSCTL后结束，不杀规划器/执行器。
    command("stop","bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh stop");
}
void PlannerController::checkStop(){
    if(!m_stopPending || busy())return;
    QFile helper(":/qt/qml/CustomGeometryExample/tools/planner/stop_preview.py");
    if(!helper.open(QIODevice::ReadOnly)){
        m_stopPending=false;m_message=tr("无法加载预览停止检查；实际停止结果未确认。");emit changed();return;
    }
    const QString remote="source /opt/ros/noetic/setup.bash; source ~/drone_platform/01_apps/ego_planner/devel/setup.bash; export ROS_MASTER_URI=http://127.0.0.1:11311 ROS_IP=127.0.0.1 PYTHONNOUSERSITE=1; /usr/bin/python3 -B - # gcs_stop_preview_guard";
    command("stop_guard",remote,helper.readAll());
}
void PlannerController::preview(const QVector3D &p){
    if(!canPreview() || !std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())){
        m_message=tr("当前操作尚未完成，请稍后选点。");emit changed();return;
    }
    m_requested=p;m_requestedCandidate=m_planning.value("status").toMap().value("planning_mode").toString()=="upstream_fsm";
    m_token.clear();m_tokenSnapshot=0.;
    command("preview",navigation()+"preview "+QString::number(p.x(),'g',9)+" "+QString::number(p.y(),'g',9)+" "+QString::number(p.z(),'g',9));
}
void PlannerController::confirm(){
    // 兼容旧调用；自动目标入口不发送人工确认或令牌。
    if(!busy())command("execution_status","bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh status");
}

void PlannerController::finished(int code,QProcess::ExitStatus exit){
    m_timeout.stop();m_output+=QString::fromUtf8(m_process.readAllStandardOutput());
    const auto stage=m_stage;
    qInfo().noquote()<<"EGO-Planner返回"<<stage<<code<<m_output.trimmed();
    if(code!=0 || exit!=QProcess::NormalExit || stage=="timeout"){
        if(m_stopPending){m_stopPending=false;m_stopPoll.stop();}
        m_message=stage=="timeout" ? tr("请求超时，状态未知，请查看实时监视。") : tr("请求未完成：")+m_output.trimmed().right(1500);
        if(stage=="preview")m_token.clear();
        emit changed();return;
    }
    QJsonObject result;
    for(const auto &line:m_output.split('\n')){
        const auto doc=QJsonDocument::fromJson(line.toUtf8());if(doc.isObject())result=doc.object();
    }
    if(stage=="start"){
        m_previewSuppressed=false;m_sessionInvalid=false;m_stopComplete=false;m_message.clear();
        emit toastRequested(tr("静态预览已运行。"));
    }else if(stage=="preview"){
        const auto goal=result.value("goal").toArray();
        if(goal.size()==3 && std::abs(goal[0].toDouble()-m_requested.x())<1e-5
            && std::abs(goal[1].toDouble()-m_requested.y())<1e-5 && std::abs(goal[2].toDouble()-m_requested.z())<1e-5){
            m_message=tr("目标已交给原生规划器，正在准备PX4接入。");
            command("execution_status","bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh status");return;
        }else m_message=tr("目标更新结果未知，请查看实时状态。");
    }else if(stage=="confirm"){
        command("execution_status","bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh status");return;
    }else if(stage=="execution_status"){
        if(!result.value("running").isBool()){m_message=tr("无法核对执行器状态，未启用新接入。");emit changed();return;}
        if(!result.value("running").toBool()){
            command("enable","bash ~/drone_platform/01_apps/ego_planner/run_px4_execution.sh enable");return;
        }
        m_message=tr("目标已更新，已有PX4接入会话继续待命。手动起飞、遥控切OFFBOARD后才执行。");
    }else if(stage=="stop"){
        m_message=tr("已请求执行器受控停止；正在核对状态后结束规划预览。");
        emit changed();checkStop();return;
    }else if(stage=="stop_guard"){
        const auto resultStatus=result.value("status").toString();
        if(resultStatus=="stopped"){
            m_stopPending=false;m_stopComplete=true;
            m_message=tr("避障已停止：执行器已退出，原规划预览会话已结束。点云、定位和坐标轴继续显示。");
        }else if(resultStatus=="waiting"){
            m_message=tr("停止处理中：")+result.value("reason").toString();m_stopPoll.start();
        }else{
            m_stopPending=false;m_message=tr("预览曲线已清除，但停止结果未确认：")+result.value("reason").toString(tr("回执无效"));
        }
    }
    else if(stage=="enable")m_message=tr("已请求既有PX4接入。手动POSCTL起飞、遥控切OFFBOARD执行；切回POSCTL接管。");
    else m_message=tr("请求已完成，请查看实时状态。");
    emit changed();
}
