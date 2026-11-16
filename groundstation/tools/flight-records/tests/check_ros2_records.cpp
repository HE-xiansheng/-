#include "pointcloudfileloader.h"
#include "pointcloudgeometry.h"
#include <QGuiApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFile>
#include <QUrl>
#include <QDebug>
#include <functional>
void require(bool ok,const char *label){if(!ok)qFatal("FAIL: %s",label);qInfo()<<"PASS"<<label;}
int main(int argc,char **argv){
 QGuiApplication app(argc,argv);require(argc==4||argc==5,"args: part0.db3 part1.db3 output.json [reject]");
 PointCloudGeometry geometry;PointCloudFileLoader loader;loader.setGeometry(&geometry);loader.setMaxPoints(70000);
 bool done=false,ok=false;QObject::connect(&loader,&PointCloudFileLoader::loadFinished,[&](bool v){done=true;ok=v;});
 auto wait=[&](){QElapsedTimer t;t.start();while(!done&&t.elapsed()<15000){app.processEvents();QThread::msleep(5);}require(done&&ok,"ROS2 decode completed");};
 if(argc==5){
  require(loader.loadRecordingFiles({QUrl::fromLocalFile(argv[1])}),"start invalid input check");QElapsedTimer t;t.start();while(!done&&t.elapsed()<15000){app.processEvents();QThread::msleep(5);}require(done&&!ok,"malformed recording rejected");return 0;
 }
 require(loader.loadRecordingFiles({QUrl::fromLocalFile(argv[2]),QUrl::fromLocalFile(argv[1])}),"load reversed split");wait();
 require(loader.playbackPoseCount()==240,"exact source odometry count");
 require(qAbs(loader.playbackDurationMs()-4780)<=1,"absolute source timeline merge");
 require(loader.recordingSources().contains("/factory_fastlio/odometry")&&!loader.recordingSources().contains("/mavros/local_position"),"exclude mismatched MAVROS local trajectory");
 auto poses=loader.flightTrajectory();auto first=poses.first().value<QVector3D>(),last=poses.last().value<QVector3D>();
 require(first.length()<1e-5&&(last-QVector3D(2.39f,0,0)).length()<1e-5,"exact CDR alignment and positions");
 require(geometry.visiblePointCount()>1000,"world point cloud decoded");
 loader.setPlaybackPositionMs(2400);require(loader.hasPlaybackPose(),"timeline pose cursor");
 QJsonObject report{{"passed",true},{"poses",loader.playbackPoseCount()},{"duration_ms",loader.playbackDurationMs()},{"points",geometry.visiblePointCount()},{"sources",loader.recordingSources()}};
 QFile out(argv[3]);require(out.open(QIODevice::WriteOnly),"evidence output");out.write(QJsonDocument(report).toJson());return 0;
}
