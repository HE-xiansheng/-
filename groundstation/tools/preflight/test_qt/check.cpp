#include "connectioncontroller.h"
#include "preflightreceiver.h"
#include "runtimepaths.h"
#include <QCoreApplication>
#include <QThread>
#include <QJsonDocument>
#include <QJsonObject>
#include <QElapsedTimer>
#include <QDebug>
#include <functional>

// 实际SSH和机载合成ROS2消息经过原数据桥抵达Qt；只允许遥测和身份认证命令。
static void require(bool value,const char *message){if(!value)qFatal("FAIL %s",message);qInfo()<<"PASS"<<message;}
static bool wait(const std::function<bool()> &predicate,int ms=20000){QElapsedTimer timer;timer.start();while(timer.elapsed()<ms){QCoreApplication::processEvents();if(predicate())return true;QThread::msleep(10);}return false;}
static int freePort(){QUdpSocket socket;require(socket.bind(QHostAddress::LocalHost,quint16(0)),"取得独立本机测试端口");return socket.localPort();}
int main(int argc,char **argv){
 QCoreApplication app(argc,argv);app.setApplicationName("gcs-ros2-link-test");
 require(!qEnvironmentVariable("GCS_STATE_DIR").isEmpty(),"测试状态目录显式隔离");
 PreflightReceiver receiver;const int port=freePort();receiver.setListenPort(port);receiver.setProperty("expectedHost","192.168.1.224");receiver.setActive(true);
 ConnectionController connection;connection.connectTo("192.168.1.224","",freePort(),freePort(),port);
 require(wait([&]{return receiver.fresh();}),"Qt连接按钮路径经真实SSH接通只读遥测");
 require(wait([&]{return !connection.busy();}),"认证与本机桥接启动结束");
 auto healthy=[&]{for(const auto &v:receiver.rows()){const auto row=v.toMap();if(row.value("key")=="lio"&&row.value("ok").toBool())return true;}return false;};
 require(wait(healthy),"原Qt接收器收到有效FAST-LIO状态");
 require(receiver.sourceHost()=="192.168.1.224","接收来源是目标机载地址");
 require(wait([&]{return receiver.sensors().value("lidar").toMap().value("values").toMap().value("point_count").toInt()==20300;}),"自定义雷达消息小字段跨机传入Qt");
 require(wait([&]{return receiver.planning().value("path_valid").toBool();}),"原生camera_init规划轨迹跨机进入Qt");
 require(wait([&]{return receiver.planning().value("grid_valid").toBool();}),"栅格参数与独立TLG1通道跨机进入Qt");
 qInfo().noquote()<<QJsonDocument(QJsonObject{{"sensors",QJsonObject::fromVariantMap(receiver.sensors())},{"planning",QJsonObject::fromVariantMap(receiver.planning())}}).toJson(QJsonDocument::Compact);
 require(QProcess::execute("/usr/bin/python3",{RuntimePaths::tool("tools/pointcloud-only/manage.py"),"stop"})==0,"停止测试拥有的点云桥");
 require(QProcess::execute("/usr/bin/python3",{RuntimePaths::tool("tools/preflight/bridge.py"),"stop"})==0,"停止测试拥有的监视桥");
 require(wait([&]{return !receiver.fresh();},6000),"断流后Qt不把旧采样标为实时");
 receiver.setActive(false);return 0;
}
