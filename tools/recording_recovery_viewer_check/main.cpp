// 使用真实地面站读取器检查中断录包副本，不连接ROS或飞控。
#include "pointcloudfileloader.h"
#include "pointcloudgeometry.h"
#include <QGuiApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QUrl>
#include <QVector3D>
#include <QDebug>
static void require(bool value,const char *name) { if(!value)qFatal("FAIL %s",name);qInfo()<<"PASS"<<name; }
int main(int argc,char **argv) {
 QGuiApplication app(argc,argv);require(argc==3,"expectations.json output.json");
 QFile input(argv[1]);require(input.open(QIODevice::ReadOnly),"read independent SQL expectations");
 const auto expect=QJsonDocument::fromJson(input.readAll()).object();
 PointCloudGeometry geometry;PointCloudFileLoader loader;loader.setGeometry(&geometry);loader.setMaxPoints(70000);
 bool done=false,ok=false;QObject::connect(&loader,&PointCloudFileLoader::loadFinished,[&](bool value){done=true;ok=value;});
 QVariantList urls;const auto parts=expect["files"].toArray();
 for(qsizetype i=parts.size();i>0;--i)urls.append(QUrl::fromLocalFile(parts[i-1].toString()));
 require(loader.loadRecordingFiles(urls),"open all recovered split files in reverse order");
 QElapsedTimer timer;timer.start();while(!done&&timer.elapsed()<20000){app.processEvents();QThread::msleep(5);}
 require(done&&ok,"actual Qt reader parses recovered bag");
 require(loader.playbackPoseCount()==expect["pose_count"].toInt(),"all recovered poses present");
 const auto poses=loader.flightTrajectory();require(!poses.empty(),"trajectory available");
 auto vec=[](const QJsonValue &value){auto a=value.toArray();return QVector3D(a[0].toDouble(),a[1].toDouble(),a[2].toDouble());};
 require((poses.first().value<QVector3D>()-vec(expect["first_xyz"])).length()<1e-5,"first recovered pose matches CDR");
 require((poses.last().value<QVector3D>()-vec(expect["last_xyz"])).length()<1e-5,"last recovered pose matches CDR");
 require(qAbs(loader.playbackDurationMs()-expect["duration_ms"].toInt())<=3,"source time span matches CDR");
 require(geometry.visiblePointCount()>1000,"recovered world cloud available");
 require(loader.recordingSources().contains("/factory_fastlio/odometry")&&loader.recordingSources().contains("/factory_fastlio/cloud_world"),"correct world sources selected");
 loader.setPlaybackPositionMs(loader.playbackDurationMs()/2);require(loader.hasPlaybackPose(),"timeline cursor selects recovered pose");
 QJsonObject report{{"passed",true},{"poses",loader.playbackPoseCount()},{"duration_ms",loader.playbackDurationMs()},{"points",geometry.visiblePointCount()},{"sources",loader.recordingSources()},{"parts",parts.size()}};
 QFile output(argv[2]);require(output.open(QIODevice::WriteOnly),"save evidence on computer");output.write(QJsonDocument(report).toJson());
}
