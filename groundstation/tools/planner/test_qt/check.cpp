#include "plannercontroller.h"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QElapsedTimer>
#include <QThread>
#include <QDebug>
#include <functional>
#include <limits>
static void require(bool ok,const char* label){if(!ok)qFatal("FAIL %s",label);qInfo()<<"PASS"<<label;}
static bool wait(const std::function<bool()> &predicate,int ms=8000){QElapsedTimer t;t.start();while(t.elapsed()<ms){QCoreApplication::processEvents();if(predicate())return true;QThread::msleep(10);}return false;}
int main(int argc,char **argv){
 QCoreApplication app(argc,argv);QTemporaryDir dir;require(dir.isValid(),"隔离目录");
 const auto calls=dir.path()+"/calls.log";qputenv("PLANNER_TEST_CALLS",calls.toUtf8());qputenv("PLANNER_TEST_STATE",(dir.path()+"/state.json").toUtf8());
 QFile ssh(dir.path()+"/ssh");require(ssh.open(QIODevice::WriteOnly),"创建SSH替身");
 ssh.write(R"PY(#!/usr/bin/python3
import sys,os,json
remote=sys.argv[-1]
assert '/opt/ros/noetic' not in remote and ('source ~/drone_platform/env.sh && ' in remote or 'bash ~/drone_platform/01_apps/ego_planner/run_' in remote),remote
with open(os.environ['PLANNER_TEST_CALLS'],'a') as f:f.write(remote+'\n')
try:state=json.load(open(os.environ['PLANNER_TEST_STATE']))
except OSError:state={'running':False,'guard':0}
if 'gcs_stop_preview_guard' in remote:
 source=sys.stdin.read();assert 'import rclpy' in source and "manager.RUNTIME/'management.lock'" in source
 state['guard']+=1;result={'status':'waiting' if state['guard']==1 else 'stopped','reason':'模拟遥控接管'}
elif 'map_navigation.py preview ' in remote:
 import shlex
 inner=shlex.split(remote)[-1];xyz=[float(v) for v in inner.split(' preview ')[1].split()];result={'goal':xyz}
elif 'run_px4_execution.sh status' in remote:result={'running':state['running']}
elif 'run_px4_execution.sh enable' in remote:state['running']=True;result={'running':True}
elif 'run_px4_execution.sh stop' in remote:state['running']=False;result={'running':False}
elif 'run_board.sh' in remote:result={'running':True}
else:raise AssertionError(remote)
json.dump(state,open(os.environ['PLANNER_TEST_STATE'],'w'));print(json.dumps(result))
)PY");ssh.close();ssh.setPermissions(QFileDevice::ReadOwner|QFileDevice::WriteOwner|QFileDevice::ExeOwner);qputenv("PATH",dir.path().toUtf8()+":"+qgetenv("PATH"));
 PlannerController c;c.setHost("192.168.1.224");require(c.host()=="192.168.1.224","目标地址");
 require(!c.canConfirm(),"直接选点流程不启用人工确认");c.start();require(wait([&]{return !c.busy();}),"ROS2规划启动SSH完成");require(c.message().isEmpty(),"规划启动成功");
 c.preview(QVector3D(1,2,1.5));require(wait([&]{return !c.busy()&&c.message().contains("已请求既有PX4接入");}),"目标回执后自动检查并准备执行器");
 c.preview(QVector3D(2,3,1.2));require(wait([&]{return !c.busy()&&c.message().contains("待命");}),"同一执行会话可直接更新目标");
 c.setExternalControlActive(true);c.preview(QVector3D(3,4,1));require(!c.busy(),"其他任务运行时不发规划目标");c.setExternalControlActive(false);
 c.stop();require(wait([&]{return c.stopPending()&&c.message().contains("停止处理中");}),"飞手尚未接管时保留停止处理中");
 require(wait([&]{return c.stopComplete()&&!c.stopPending();}),"核实退出后完成受控停止");
 QFile log(calls);require(log.open(QIODevice::ReadOnly),"读取SSH操作记录");const auto text=log.readAll();
 require(text.count("run_px4_execution.sh enable")==1,"已有执行会话不重复启用");require(text.count("map_navigation.py preview ")==2,"两个目标分别下发一次");
 require(!text.contains("confirm ")&&!text.contains("rosnode kill")&&!text.contains("killall"),"未恢复人工确认或强制杀进程流程");
 return 0;
}
