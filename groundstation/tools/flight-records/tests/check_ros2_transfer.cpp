#include "flightrecordcontroller.h"
#include "pointcloudfileloader.h"
#include "pointcloudgeometry.h"
#include <QGuiApplication>
#include <QElapsedTimer>
#include <QThread>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QCryptographicHash>
#include <QDebug>
#include <functional>
void require(bool ok,const char *label){if(!ok)qFatal("FAIL: %s",label);qInfo()<<"PASS"<<label;}
int main(int argc,char **argv){
 QGuiApplication app(argc,argv);QCoreApplication::setApplicationName("ros2-recording-transfer-check");require(argc==3,"args: host report.json");
 auto wait=[&](std::function<bool()> f){QElapsedTimer t;t.start();while(!f()&&t.elapsed()<30000){app.processEvents();QThread::msleep(5);}require(f(),"asynchronous operation completes");};
 FlightRecordController controller;controller.setHost(argv[1]);controller.refresh();wait([&]{return !controller.busy();});require(!controller.records().isEmpty(),"native recorder runs listed over real SSH");
 int index=-1;for(int i=0;i<controller.records().size();++i)if(controller.records()[i].toMap().value("parts").toInt()>1){index=i;break;}
 require(index>=0,"split capture listed");QVariantList files;QObject::connect(&controller,&FlightRecordController::recordingReady,[&](const QVariantList &v,const QString &){files=v;});
 controller.readRecord(index);wait([&]{return !controller.busy();});require(files.size()>1,"all split files transferred");
 QJsonArray manifest;
 for(const auto &v:files){QFile f(v.toUrl().toLocalFile());require(f.open(QIODevice::ReadOnly),"downloaded file available");QCryptographicHash hash(QCryptographicHash::Sha256);require(hash.addData(&f),"hash downloaded bytes");manifest.append(QJsonObject{{"path",f.fileName()},{"sha256",QString::fromLatin1(hash.result().toHex())}});}
 PointCloudGeometry geometry;PointCloudFileLoader loader;loader.setGeometry(&geometry);bool done=false,success=false;
 QObject::connect(&loader,&PointCloudFileLoader::loadFinished,[&](bool v){done=true;success=v;});require(loader.loadRecordingFiles(files),"open downloaded actual recording");wait([&]{return done;});require(success,"downloaded rosbag2 decodes");require(loader.playbackPoseCount()>0&&geometry.visiblePointCount()>0,"recorded trajectory and clouds usable");
 QJsonObject report{{"passed",true},{"files",manifest},{"record",controller.records()[index].toMap().value("name").toString()},{"poses",loader.playbackPoseCount()},{"points",geometry.visiblePointCount()}};QFile out(argv[2]);require(out.open(QIODevice::WriteOnly),"evidence output");out.write(QJsonDocument(report).toJson());return 0;
}
