#include "missioncontroller.h"
#include "plannercontroller.h"
#include <QGuiApplication>
#include <QQmlEngine>
#include <QQmlComponent>
#include <QQuickItem>
#include <QQuickWindow>
#include <QTemporaryDir>
#include <QFile>
#include <QJsonObject>
#include <QJsonDocument>
#include <QJsonArray>
#include <QThread>
#include <QDebug>
#include <functional>
#include <memory>
static void require(bool value,const char *why){if(!value)qFatal("FAIL %s",why);qInfo()<<"PASS"<<why;}
static bool wait(const std::function<bool()> &predicate,int ms=5000){QElapsedTimer t;t.start();while(t.elapsed()<ms){QCoreApplication::processEvents();if(predicate())return true;QThread::msleep(10);}return false;}
static void save(const QString &path,const QJsonObject &state){QFile f(path);require(f.open(QIODevice::WriteOnly|QIODevice::Truncate),"写模拟监视状态");f.write(QJsonDocument(state).toJson(QJsonDocument::Compact));}
int main(int argc,char **argv){
 QGuiApplication app(argc,argv);
 if(app.arguments().contains("--live-readonly")) {
     MissionController live;live.setHost("192.168.1.224");live.watch();
     require(wait([&]{return live.fresh() && live.state().contains("fc");},10000),"真实ROS2机载SSH只读监视进入Qt控制器");
     require(!live.state().value("running").toBool(),"隔离域没有启动真实任务");
     require(!live.busy(),"只读验证未发送任务操作");
     qInfo().noquote()<<QJsonDocument(QJsonObject::fromVariantMap(live.state())).toJson(QJsonDocument::Compact);
     live.disconnectMonitor();return 0;
 }
 QTemporaryDir dir;require(dir.isValid(),"隔离目录");
 const auto stateFile=dir.path()+"/state.json",callsFile=dir.path()+"/calls.jsonl";
 save(stateFile,{{"running",false},{"task",""},{"aruco",QJsonObject{}},{"tracking",QJsonObject{}}});
 qputenv("MISSION_TEST_PAYLOAD",(dir.path()+"/payload.json").toUtf8());
 qputenv("MISSION_TEST_STATE",stateFile.toUtf8());qputenv("MISSION_TEST_CALLS",callsFile.toUtf8());
 QFile ssh(dir.path()+"/ssh");require(ssh.open(QIODevice::WriteOnly),"假SSH");
 ssh.write(R"PY(#!/usr/bin/python3
import os,sys,json,time,shlex
outer=shlex.split(sys.argv[-1]);assert 'source ~/drone_platform/env.sh && exec ' in outer[-1] and '/opt/ros/noetic' not in outer[-1];args=shlex.split(outer[-1]);index=args.index('~/drone_platform/01_apps/mission_demo/tools/gcs_task_manager.py');args=args[index+1:]
with open(os.environ['MISSION_TEST_CALLS'],'a') as f:f.write(json.dumps(args)+'\n')
path=os.environ['MISSION_TEST_STATE']
def read():
 try:return json.load(open(path))
 except (ValueError,OSError):return {}
if args[0]=='watch':
 while True:
  state=read()
  if state.get('offline'):sys.exit(255)
  print(json.dumps({'ok':True,'state':state}),flush=True);time.sleep(.1)
else:
 state=read();action=args[0]
 if action=='start-map':
  payload=json.load(sys.stdin)
  with open(os.environ['MISSION_TEST_PAYLOAD'],'w') as f:json.dump(payload,f)
  state.update(running=True,task='waypoint',stop_pending=False)
 elif action=='start':
  kind=args[args.index('--kind')+1];state.update(running=True,task=kind,stop_pending=False)
  if kind=='aruco':state['aruco']={'valid':True,'id':231,'offset':{'x':.12,'y':-.05},'reason':'selected_pose_valid','source_stamp_known':True,'landing_fresh':True,'landing':{'phase':'IDLE','marker_reason':'accepted'}}
  if kind=='tracking':state['tracking']={'id':42,'distance_m':2.5,'distance_valid':1}
 elif action=='stop':state['stop_pending']=True
 with open(path,'w') as f:json.dump(state,f)
 print(json.dumps({'ok':True,'message':'操作已完成','state':state}),flush=True)
)PY");ssh.close();ssh.setPermissions(QFileDevice::ReadOwner|QFileDevice::WriteOwner|QFileDevice::ExeOwner);
 qputenv("PATH",dir.path().toUtf8()+":"+qgetenv("PATH"));
 PlannerController planner;require(planner.canPreview(),"Ego原选点入口保持可用");
 planner.setExternalControlActive(true);require(!planner.canPreview(),"三项任务接管时阻止Ego同时接入");planner.start();require(!planner.busy(),"冲突时不发送Ego启动命令");planner.setExternalControlActive(false);require(planner.canPreview(),"结束任务后Ego入口恢复");
 MissionController controller;QQmlEngine engine;
 QQmlComponent component(&engine,QUrl::fromLocalFile(QStringLiteral(GCS_ROOT "/MissionControls.qml")));
 std::unique_ptr<QObject> root(component.createWithInitialProperties({{"controller",QVariant::fromValue<QObject*>(&controller)}}));
 if(!root)qCritical()<<component.errors();require(bool(root),"实际三项控制栏QML创建");
 auto item=qobject_cast<QQuickItem*>(root.get());QQuickWindow window;item->setParentItem(window.contentItem());item->setWidth(390);window.setWidth(400);window.setHeight(1000);window.show();
 std::function<QObject*(QQuickItem*,const QString&)> visualFind=[&](QQuickItem *parent,const QString &name)->QObject* {
   if(parent->objectName()==name)return parent;
   for(auto child:parent->childItems())if(auto found=visualFind(child,name))return found;
   return nullptr;
 };
 auto find=[&](const char *name){QObject *o=nullptr;require(wait([&]{o=root->findChild<QObject*>(QString::fromLatin1(name));if(!o)o=visualFind(item,QString::fromLatin1(name));return o!=nullptr;}),name);return o;};
 auto click=[&](const char *name){auto o=find(name);require(o->property("enabled").toBool(),"按钮可操作");require(QMetaObject::invokeMethod(o,"clicked"),"QML点击信号");require(wait([&]{return !controller.busy();}),"SSH操作回执结束");};
 find("waypointPlanningSection")->setProperty("collapsed",false);
 require(wait([&]{return controller.fresh();}),"展开功能栏只启动监视");
 require(!controller.state().value("running").toBool(),"展开不会启动飞行任务");
 const QString mission=QStringLiteral("/tmp/mission 'quoted;$(touch BAD).yaml");controller.validateMission(mission);
 require(wait([&]{return !controller.busy();}),"含特殊字符路径校验完成");
 QFile calls(callsFile);calls.open(QIODevice::ReadOnly);bool foundPath=false;
 for(const auto &line:calls.readAll().split('\n')){auto a=QJsonDocument::fromJson(line).array();for(const auto &v:a)if(v.toString()==mission)foundPath=true;}
 require(foundPath,"含引号与shell符号的任务路径作为单个参数传递");
 auto mapState=QJsonObject::fromVariantMap(controller.state());mapState["map_session"]=QJsonObject{{"version",2},{"boot","test"},{"domain",87},{"gid","mock-dds-gid"},{"node","mock_lio"}};save(stateFile,mapState);
 require(wait([&]{return controller.state().contains("map_session");}),"地图定位会话回传");
 click("pickMissionWaypoints");require(root->property("picking").toBool(),"地图连续选点模式");
 controller.addWaypoint(1,2,1);controller.addWaypoint(3,4,1.2);
 require(controller.waypoints().size()==2,"连续添加两个地图点");
 auto heightField=find("waypointHeight_0");heightField->setProperty("text",QStringLiteral("1.80"));
 require(QMetaObject::invokeMethod(heightField,"editingFinished"),"实际QML提交航点高度");
 QCoreApplication::processEvents();auto photoField=find("waypointPhoto_0");photoField->setProperty("checked",true);
 require(QMetaObject::invokeMethod(photoField,"toggled"),"实际QML切换拍照");
 controller.removeWaypoint(1);
 require(controller.waypoints().size()==1,"逐点高度拍照编辑及删除");
 click("prepareWaypointTask");require(wait([&]{return controller.state().value("task")=="waypoint";}),"地图航点准备与监视状态");
 QFile uploaded(dir.path()+"/payload.json");require(uploaded.open(QIODevice::ReadOnly),"SSH标准输入承载地图任务JSON");
 auto payload=QJsonDocument::fromJson(uploaded.readAll()).object();auto point=payload["waypoints"].toArray().first().toObject();
 require(payload["version"].toInt()==4 && payload["map_session"].toObject()["gid"].toString()=="mock-dds-gid" && point["photo"].toBool() && point["z"].toDouble()==1.8,"地图任务包含会话、高度及拍照设置");
 require(!root->property("picking").toBool(),"提交任务后结束选点");
 click("stopWaypointTask");require(wait([&]{return controller.state().value("stop_pending").toBool();}),"停止显示等待接管");
 auto state=QJsonObject::fromVariantMap(controller.state());state["running"]=false;state["stop_pending"]=false;save(stateFile,state);require(wait([&]{return !controller.state().value("running").toBool();}),"远端停止状态回传");
 find("visualControlSection")->setProperty("collapsed",false);find("arucoLandingSection")->setProperty("collapsed",false);
 click("prepareArucoTask");require(wait([&]{return find("arucoMarkerId")->property("value").toString()=="231";}),"ArUco ID回传到实际栏位");
 require(find("arucoOffsetX")->property("value").toString()=="0.120 m","ArUco米制偏移显示");
 require(find("arucoControlTargetStatus")->property("text").toString().contains("accepted"),"区分检测成功与控制器接受目标");
 state=QJsonObject::fromVariantMap(controller.state());
 auto aruco=state["aruco"].toObject();aruco["landing"]=QJsonObject{{"phase","CANCELLED"},{"reason","rc_stamp_stale"},{"rc_cancel_detail","channel=5;pwm=1913;rssi=255;slot=6;source_age=0.7"}};state["aruco"]=aruco;save(stateFile,state);
 require(wait([&]{return find("arucoTaskStatus")->property("text").toString().contains("rc_stamp_stale");}),"取消具体原因显示，不能笼统归因拨杆");
 require(find("arucoRcDetail")->property("text").toString().contains("pwm=1913"),"显示冻结的RC取消证据");
 window.grabWindow().save(QStringLiteral("/tmp/gcs-aruco-cancel-ui.png"));
 aruco["valid"]=false;aruco["offset"]=QJsonObject{};aruco["reason"]="marker_source_stale";state["aruco"]=aruco;save(stateFile,state);
 require(wait([&]{return find("arucoMarkerId")->property("value").toString()==QStringLiteral("—");}),"监视在线时也隐藏源过期的ID和偏移");
 require(find("arucoDetectionStatus")->property("text").toString().contains("marker_source_stale"),"源过期原因可见");
 aruco["landing_fresh"]=false;aruco["landing"]=QJsonObject{};state["aruco"]=aruco;save(stateFile,state);
 require(wait([&]{return find("arucoTaskStatus")->property("text").toString().contains(QStringLiteral("已过期"));}),"监视在线不等于控制节点状态新鲜");
 aruco["landing_fresh"]=true;aruco["landing"]=QJsonObject{{"phase","IDLE"},{"start_reject_reason","manual_mode_requires_airborne_and_armed"}};state["aruco"]=aruco;save(stateFile,state);
 require(wait([&]{return find("arucoTaskStatus")->property("text").toString().contains("manual_mode_requires_airborne_and_armed");}),"触发拒绝原因回传到界面");
 state=QJsonObject::fromVariantMap(controller.state());state["running"]=false;save(stateFile,state);require(wait([&]{return !controller.state().value("running").toBool();}),"模拟降落任务结束");
 find("personTrackingSection")->setProperty("collapsed",false);click("prepareTrackingTask");require(wait([&]{return controller.state().value("task")=="tracking";}),"人物跟踪控制入口");
 state=QJsonObject::fromVariantMap(controller.state());state["offline"]=true;save(stateFile,state);require(wait([&]{return !controller.fresh();}),"SSH断开不冒充任务状态实时");
 require(find("arucoMarkerId")->property("value").toString()==QStringLiteral("—"),"过期识别数据隐藏");
 window.grabWindow().save(QStringLiteral("/tmp/gcs-mission-controls-ui.png"));
 controller.disconnectMonitor();require(!controller.fresh(),"断开监视不发送停止飞行请求");
 return 0;
}
