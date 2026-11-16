#include "recordingcontroller.h"
#include <QCoreApplication>
#include <QJsonDocument>
#include <QJsonObject>
#include <QThread>
#include <QFile>
#include <QDebug>
#include <functional>
void require(bool v,const char*s){if(!v)qFatal("FAIL %s",s);qInfo()<<"PASS"<<s;}
int main(int argc,char**argv){
 QCoreApplication app(argc,argv);require(argc==2,"output.json");RecordingController control;control.setProperty("host","192.168.1.224");
 auto status=[&](){QProcess p;p.start("ssh",{"-o","BatchMode=yes","orangepi@192.168.1.224","bash -c 'source /home/orangepi/drone_platform/env.sh; export ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/ros2-recording-test5/ui-logs; python3 -B /tmp/ros2-recording-test5/recording_manage.py status'"});p.closeWriteChannel();require(p.waitForFinished(8000)&&p.exitCode()==0,"readonly remote status");auto v=QJsonDocument::fromJson(p.readAllStandardOutput()).object().toVariantMap();control.observe(v,true);return v;};
 auto wait=[&](const char*phase){QElapsedTimer t;t.start();QVariantMap v;while(t.elapsed()<25000){app.processEvents();v=status();if(v.value("phase")==phase&&!control.busy())return v;QThread::msleep(100);}qFatal("phase timeout %s",phase);return v;};
 auto before=status();require(control.canStart(),"fresh completed recorder allows start");control.start();require(control.busy(),"SSH start asynchronously launched");auto active=wait("RECORDING");require(control.recording()&&control.canStop(),"native status confirms active recording");
 require(!control.canStart(),"duplicate start disabled");control.stop();auto saved=wait("STOPPED");require(saved.value("bag_bytes").toLongLong()>0,"UI stop saved SQLite bag");
 require(control.canStart()&&!control.canStop(),"UI returns to idle");control.observe(saved,false);require(!control.fresh()&&!control.canStart(),"stale telemetry disables start");
 QJsonObject report{{"passed",true},{"run",saved.value("run").toString()},{"bytes",saved.value("bag_bytes").toLongLong()}};QFile f(argv[1]);require(f.open(QIODevice::WriteOnly),"report");f.write(QJsonDocument(report).toJson());
}
