#include "missioncontroller.h"
#include "sshsession.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <cmath>
#include <QRegularExpression>
#include <QDebug>

MissionController::MissionController(QObject *parent):QObject(parent) {
    m_clock.start();m_timeout.setSingleShot(true);m_retry.setSingleShot(true);m_retry.setInterval(3000);
    connect(&m_timeout,&QTimer::timeout,this,[this]{m_message=tr("任务请求超时，结果未知；请查看机载状态。");m_command.kill();emit changed();});
    connect(&m_command,&QProcess::readyReadStandardOutput,this,[this]{m_output+=m_command.readAllStandardOutput();if(m_output.size()>65536)m_output=m_output.right(65536);});
    connect(&m_command,&QProcess::readyReadStandardError,this,[this]{const auto error=m_command.readAllStandardError();if(!error.trimmed().isEmpty())qWarning().noquote()<<"任务SSH:"<<QString::fromUtf8(error).left(1200);});
    connect(&m_command,&QProcess::errorOccurred,this,[this](QProcess::ProcessError e){if(e==QProcess::FailedToStart){m_timeout.stop();m_message=tr("无法启动任务SSH：")+m_command.errorString();emit changed();}});
    connect(&m_command,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,[this](int code,QProcess::ExitStatus exit){
        m_timeout.stop();m_output+=m_command.readAllStandardOutput();QJsonObject result;
        for(const auto &line:m_output.split('\n')){const auto d=QJsonDocument::fromJson(line);if(d.isObject())result=d.object();}
        if(code==0&&exit==QProcess::NormalExit&&result.value("ok").toBool()){
            m_message=result.value("message").toString(tr("操作已完成"));
            // 状态新鲜度仅由监视流推进，操作回执不能冒充持续遥测。
        }else if(!result.isEmpty())m_message=result.value("message").toString(tr("任务请求失败"));
        else if(m_message!=tr("任务请求超时，结果未知；请查看机载状态。"))m_message=tr("任务请求失败，未收到有效回执；请检查连接和机载管理程序。");
        qInfo().noquote()<<"机载任务返回"<<code<<m_message;emit changed();
    });
    connect(&m_monitor,&QProcess::readyReadStandardOutput,this,[this]{
        m_monitorBuffer+=m_monitor.readAllStandardOutput();int end;
        while((end=m_monitorBuffer.indexOf('\n'))>=0){
            const auto line=m_monitorBuffer.left(end);m_monitorBuffer.remove(0,end+1);
            const auto d=QJsonDocument::fromJson(line);if(!d.isObject())continue;const auto o=d.object();
            if(o.value("ok").toBool()&&o.value("state").isObject()){
                const auto next=o.value("state").toObject().toVariantMap();
                const auto before=m_state.value("aruco").toMap().value("landing").toMap();
                const auto landing=next.value("aruco").toMap().value("landing").toMap();
                // 仅在阶段/原因改变时写日志，保留现场取消证据而不逐帧刷屏。
                if(!landing.isEmpty() && (landing.value("phase")!=before.value("phase") ||
                    landing.value("reason")!=before.value("reason") ||
                    landing.value("start_reject_reason")!=before.value("start_reject_reason")))
                    qInfo().noquote()<<"ArUco降落状态"<<landing.value("phase").toString()
                        <<landing.value("reason").toString()<<landing.value("start_reject_reason").toString()
                        <<landing.value("rc_cancel_detail").toString();
                m_state=next;m_received=m_clock.elapsed();emit changed();
            }
        }
        if(m_monitorBuffer.size()>512*1024)m_monitorBuffer.clear();
    });
    connect(&m_monitor,&QProcess::readyReadStandardError,this,[this]{const auto e=m_monitor.readAllStandardError();if(!e.trimmed().isEmpty())qWarning().noquote()<<"任务监视:"<<QString::fromUtf8(e).left(1000);});
    auto retry=[this]{if(m_observing){m_received=-1;m_retry.start();emit changed();}};
    connect(&m_monitor,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,[retry](int,QProcess::ExitStatus){retry();});
    connect(&m_monitor,&QProcess::errorOccurred,this,[retry](QProcess::ProcessError){retry();});
    connect(&m_retry,&QTimer::timeout,this,&MissionController::startMonitor);
    m_tick.setInterval(1000);connect(&m_tick,&QTimer::timeout,this,[this]{
        const auto last=m_received>=0?m_received:m_watchStarted;
        if(m_observing && last>=0 && m_monitor.state()==QProcess::Running && m_clock.elapsed()-last>15000)m_monitor.kill();
        emit changed();
    });
}
MissionController::~MissionController(){disconnectMonitor();if(busy()){m_command.kill();m_command.waitForFinished(1000);}}
QString MissionController::quote(QString text){return "'"+text.replace("'","'\\''")+"'";}
void MissionController::setHost(const QString &host){
    static QRegularExpression valid("^[A-Za-z0-9][A-Za-z0-9.-]{0,252}$");
    if(host==m_host||!valid.match(host).hasMatch()||busy())return;
    const bool restart=m_observing;disconnectMonitor();m_host=host;m_state.clear();m_message.clear();clearWaypoints();if(restart)watch();emit changed();
}
void MissionController::startSsh(QProcess &process,const QStringList &args,const QByteArray &input){
    // ROS2运行环境由机载统一入口加载；加载失败时不继续执行任务管理器。
    QString command="source ~/drone_platform/env.sh && exec /usr/bin/python3 -B ~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py";
    for(const auto &arg:args)command+=" "+quote(arg);
    SshSession::start(process,{"-T","-o",SshSession::batchOption(),"-o","ConnectTimeout=6","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2","orangepi@"+m_host,"bash -lc "+quote(command)});
    if(!input.isEmpty())process.write(input);
    process.closeWriteChannel();
}
void MissionController::watch(){m_observing=true;m_tick.start();startMonitor();}
void MissionController::startMonitor(){if(!m_observing||m_monitor.state()!=QProcess::NotRunning)return;m_monitorBuffer.clear();m_received=-1;m_watchStarted=m_clock.elapsed();startSsh(m_monitor,{"watch"});}
void MissionController::disconnectMonitor(){m_observing=false;m_received=-1;m_tick.stop();m_retry.stop();if(m_monitor.state()!=QProcess::NotRunning){m_monitor.kill();m_monitor.waitForFinished(1000);}emit changed();}
void MissionController::invoke(const QStringList &args,const QByteArray &input){if(busy())return;m_output.clear();m_message=tr("正在处理机载任务…");qInfo()<<"机载任务请求"<<args.value(0)<<args.value(1)<<"host"<<m_host;startSsh(m_command,args,input);m_timeout.start(30000);watch();emit changed();}
void MissionController::prepare(const QString &kind,const QString &missionFile){if(kind!="waypoint"&&kind!="aruco"&&kind!="tracking")return;invoke({"start","--kind",kind,"--mission-file",missionFile});}
void MissionController::stopTask(){invoke({"stop"});}
void MissionController::validateMission(const QString &missionFile){invoke({"validate","--mission-file",missionFile});}
void MissionController::addWaypoint(double x,double y,double z){
    if(busy()||!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(z))return;
    const auto session=m_state.value("map_session").toMap();
    if(!fresh()||session.isEmpty()){m_message=tr("请先连接机载并启动FAST-LIO，取得地图定位会话。");emit changed();return;}
    if(m_waypoints.isEmpty())m_mapSession=session;
    if(session!=m_mapSession){m_message=tr("定位会话已变化，请先清空旧航点。");emit changed();return;}
    m_waypoints.append(QVariantMap{{"x",x},{"y",y},{"z",z},{"photo",false}});emit waypointsChanged();
}
void MissionController::editWaypoint(int i,double z,bool photo){
    if(busy()||i<0||i>=m_waypoints.size()||!std::isfinite(z))return;
    auto p=m_waypoints[i].toMap();p["z"]=z;p["photo"]=photo;m_waypoints[i]=p;emit waypointsChanged();
}
void MissionController::removeWaypoint(int i){if(busy()||i<0||i>=m_waypoints.size())return;m_waypoints.removeAt(i);emit waypointsChanged();}
void MissionController::clearWaypoints(){m_mapSession.clear();if(m_waypoints.isEmpty())return;m_waypoints.clear();emit waypointsChanged();}
void MissionController::executeMapMission(){
    if(m_waypoints.isEmpty()||busy())return;
    QJsonObject payload{{"version",4},{"frame","fastlio_world"},{"map_session",QJsonObject::fromVariantMap(m_mapSession)},{"waypoints",QJsonArray::fromVariantList(m_waypoints)}};
    invoke({"start-map"},QJsonDocument(payload).toJson(QJsonDocument::Compact));
}
