#include "flightrecordcontroller.h"
#include "pointcloudfileloader.h"
#include "routegeometry.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QTemporaryDir>
#include <QElapsedTimer>
#include <QFile>
#include <QDir>
#include <QStandardPaths>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QThread>
#include <QDebug>
#include <functional>
static void require(bool ok,const QString &why){if(!ok)qFatal("FAIL: %s",qPrintable(why));}
static void waitFor(std::function<bool()> ready,const QString &why,int ms=60000){QElapsedTimer t;t.start();while(!ready()&&t.elapsed()<ms){QCoreApplication::processEvents();QThread::msleep(10);}require(ready(),why);}
int main(int argc,char **argv){
    QGuiApplication app(argc,argv);
    if(argc>=4 && QString(argv[1])=="--view-local"){
        QCoreApplication::setApplicationName("gcs-local-flight-viewer");
        const QString evidence=argv[2];QDir().mkpath(evidence);QVariantList files;
        for(int i=3;i<argc;++i)files.append(QUrl::fromLocalFile(QString::fromLocal8Bit(argv[i])));
        FlightRecordController backend;QQmlApplicationEngine engine;
        engine.setInitialProperties({{"backend",QVariant::fromValue(&backend)}});
        engine.loadData("import QtQuick\nimport CustomGeometryExample\nFlightRecordWindow { visible:true }");
        require(!engine.rootObjects().isEmpty(),"local viewer opens");
        auto *window=qobject_cast<QQuickWindow*>(engine.rootObjects().first());
        auto *loader=window->findChild<PointCloudFileLoader*>("flightRecordLoader");
        require(window&&loader,"local viewer components");
        const QString name=QFileInfo(QString::fromLocal8Bit(argv[3])).dir().dirName();
        QObject::connect(loader,&PointCloudFileLoader::loadFinished,window,[&,window,loader,name](bool ok){
            QJsonObject result{{"local_record",name},{"parts",files.size()},{"success",ok},{"status",loader->status()},{"sources",loader->recordingSources()},{"pose_count",loader->playbackPoseCount()},{"duration_ms",loader->playbackDurationMs()}};
            QFile report(evidence+"/local-record.json");report.open(QIODevice::WriteOnly);report.write(QJsonDocument(result).toJson());
            qInfo().noquote()<<QJsonDocument(result).toJson(QJsonDocument::Compact);
            if(ok)QTimer::singleShot(1500,window,[window,evidence]{window->grabWindow().save(evidence+"/local-flight-viewer.png");});
        });
        require(QMetaObject::invokeMethod(window,"parse",Q_ARG(QVariant,QVariant(files)),Q_ARG(QVariant,QVariant(name+" · 本地机载归档"))),"load local archived flight");
        return app.exec();
    }
    QCoreApplication::setApplicationName("gcs-flight-record-check");QStandardPaths::setTestModeEnabled(true);
    require(argc>=3,"usage: check_records real.bag output-directory");const QString bag=argv[1],out=argv[2];QDir().mkpath(out);
    PointCloudGeometry cloud;PointCloudFileLoader loader;loader.setGeometry(&cloud);loader.setMaxPoints(70000);
    bool done=false,success=false;QObject::connect(&loader,&PointCloudFileLoader::loadFinished,[&](bool v){done=true;success=v;});
    require(loader.loadRecordingFiles({QUrl::fromLocalFile(bag)}),"start real archived bag");waitFor([&]{return done;},"real bag decode completed");require(success,loader.status());
    require(cloud.visiblePointCount()>1000,"real cloud rendered data");require(loader.playbackPoseCount()>100,"FAST-LIO odometry decoded");
    require(loader.recordingSources().contains("/factory_fastlio/odometry")&&!loader.recordingSources().contains("/mavros/local_position"),"select correct world-frame trajectory");
    const auto trajectory=loader.flightTrajectory();require(trajectory.size()>100,"full flight trajectory");
    QJsonObject report{{"real_bag",bag},{"cloud_points",cloud.visiblePointCount()},{"pose_count",loader.playbackPoseCount()},{"duration_ms",loader.playbackDurationMs()},{"sources",loader.recordingSources()}};
    loader.setPlaybackPositionMs(loader.playbackDurationMs()/2);require(loader.hasPlaybackPose(),"time cursor has odometry pose");loader.showAccumulatedPointCloud();
    if(argc>3){
        QFile expectedFile(QString(argv[3])+"/expected.json");require(expectedFile.open(QIODevice::ReadOnly),"fixture baseline");const auto expected=QJsonDocument::fromJson(expectedFile.readAll()).object();
        done=false;success=false;
        require(loader.loadRecordingFiles({QUrl::fromLocalFile(QString(argv[3])+"/part-1.bag"),QUrl::fromLocalFile(QString(argv[3])+"/part-0.bag")}),"start reversed split bags LZ4 and BZ2");
        waitFor([&]{return done;},"multi-part decode");require(success,loader.status());
        require(loader.playbackPoseCount()==expected["pose_count"].toInt(),"split odometry count exact, wrong MAVROS source excluded");
        require(qAbs(loader.playbackDurationMs()-expected["duration_ms"].toInt())<=2,"split absolute timeline preserved");
        const auto points=loader.flightTrajectory();const auto first=points.first().value<QVector3D>(),last=points.last().value<QVector3D>();
        const auto ef=expected["first"].toArray(),el=expected["last"].toArray();
        require((first-QVector3D(ef[0].toDouble(),ef[1].toDouble(),ef[2].toDouble())).length()<1e-4,"split first position exact");
        require((last-QVector3D(el[0].toDouble(),el[1].toDouble(),el[2].toDouble())).length()<1e-4,"split last position exact");
        require(loader.status().contains(QString::number(expected["point_records"].toInt())),"only chosen world point records counted");
        report["reversed_lz4_bz2_split_merge"]=true;report["split_pose_count"]=loader.playbackPoseCount();
    }
    // 真实归档通过生产SSH控制器的二进制流路径读回；仅替换SSH执行端，不启动ROS或飞控。
    QTemporaryDir fixture;require(fixture.isValid(),"fixture");QDir().mkpath(fixture.path()+"/records/flight-a");
    require(QFile::copy(bag,fixture.path()+"/records/flight-a/inputs_0.bag"),"copy real archive fixture");
    QDir().mkpath(fixture.path()+"/records/active-flight");QFile active(fixture.path()+"/records/active-flight/inputs_0.bag.active");active.open(QIODevice::WriteOnly);active.write("active");active.close();
    QFile ssh(fixture.path()+"/ssh");require(ssh.open(QIODevice::WriteOnly),"fake SSH");
    ssh.write(R"PY(#!/usr/bin/python3
import os,sys,shlex,time
if os.environ.get("GCS_RECORDS_TEST_DELAY"):time.sleep(.5)
code=sys.stdin.read().replace("Path.home()/'drone_platform/01_apps/ego_planner/flight-records'","Path(os.environ['GCS_RECORDS_FIXTURE'])")
sys.argv=['remote_records.py']+shlex.split(sys.argv[-1])[3:]
exec(compile(code,'remote-records-read-only-test','exec'))
)PY");ssh.close();ssh.setPermissions(QFile::ReadOwner|QFile::WriteOwner|QFile::ExeOwner);
    qputenv("GCS_RECORDS_FIXTURE",(fixture.path()+"/records").toUtf8());qputenv("PATH",fixture.path().toUtf8()+":"+qgetenv("PATH"));
    FlightRecordController reader;reader.setHost("fixture-host");reader.refresh();waitFor([&]{return !reader.busy();},"record list response");
    require(reader.records().size()==1,"active recording excluded from history");
    QVariantList downloaded;QObject::connect(&reader,&FlightRecordController::recordingReady,[&](const QVariantList &f,const QString &){downloaded=f;});
    reader.readRecord(0);waitFor([&]{return !reader.busy();},"binary SSH transfer finished");require(downloaded.size()==1,reader.message());
    const QString downloadedFile=downloaded.first().toUrl().toLocalFile();require(QFileInfo(downloadedFile).size()==QFileInfo(bag).size(),"download size exact");
    QFile original(bag),copy(downloadedFile);original.open(QIODevice::ReadOnly);copy.open(QIODevice::ReadOnly);
    while(!original.atEnd())require(original.read(256*1024)==copy.read(256*1024),"download binary bytes unchanged");
    report["list_download_byte_check"]=true;
    const auto firstCache=reader.cacheDirectory();
    if(argc>3){
        QDir().mkpath(fixture.path()+"/records/flight-split");
        for(const auto &name:{"part-0.bag","part-1.bag"})require(QFile::copy(QString(argv[3])+"/"+name,fixture.path()+"/records/flight-split/"+name),"split download fixture");
        reader.refresh();waitFor([&]{return !reader.busy();},"split list");require(reader.records().size()==2,"only closed runs listed");
        int selected=-1;for(int i=0;i<reader.records().size();++i)if(reader.records()[i].toMap().value("name")=="flight-split")selected=i;
        require(selected>=0,"split run listed");downloaded.clear();reader.readRecord(selected);waitFor([&]{return !reader.busy();},"all split transfers");
        require(downloaded.size()==2,"all recording parts downloaded");report["all_parts_downloaded"]=true;
    }
    const auto finalDownloads=downloaded;
    qputenv("GCS_RECORDS_TEST_DELAY","1");reader.readRecord(0);reader.cancel();
    waitFor([&]{return !reader.busy();},"cancel read settles");
    require(QFileInfo(finalDownloads.first().toUrl().toLocalFile()).isFile(),"cancel does not delete completed cache");
    qunsetenv("GCS_RECORDS_TEST_DELAY");
    qputenv("GCS_RECORDS_FIXTURE",(fixture.path()+"/missing").toUtf8());
    reader.refresh();waitFor([&]{return !reader.busy();},"missing directory response");
    require(reader.records().isEmpty()&&reader.message().contains("flight-records"),"remote read error visible");
    qputenv("GCS_RECORDS_FIXTURE",(fixture.path()+"/records").toUtf8());
    report["cancel_and_read_error"]=true;
    reader.refresh();waitFor([&]{return !reader.busy();},"UI record catalogue");
    // 独立窗口实际加载下载结果并绘制；截图用于检查3D点云与轨迹以及深灰品牌徽章。
    QQmlApplicationEngine engine;
    engine.setInitialProperties({{"backend",QVariant::fromValue(&reader)}});
    engine.loadData(R"QML(import QtQuick
import CustomGeometryExample
FlightRecordWindow { visible:true }
)QML");
    require(!engine.rootObjects().isEmpty(),"history QML window loads");auto *window=qobject_cast<QQuickWindow*>(engine.rootObjects().first());require(window,"history window");
    auto *uiLoader=window->findChild<PointCloudFileLoader*>("flightRecordLoader");require(uiLoader,"history loader");
    bool uiDone=false;QObject::connect(uiLoader,&PointCloudFileLoader::loadFinished,[&](bool v){require(v,uiLoader->status());uiDone=true;});
    auto *readButton=window->findChild<QObject*>("readFlightRecord");require(readButton&&readButton->property("enabled").toBool(),"UI record selection ready");
    require(QMetaObject::invokeMethod(readButton,"clicked"),"click read and plot");
    waitFor([&]{return uiDone;},"UI parses real flight record");
    auto *route=window->findChild<RouteGeometry*>("flightRecordPath");require(route&&route->waypointCount()>100,"3D trajectory geometry built");
    require(downloaded.size()==1,"UI downloads only selected bag part");
    require(uiLoader->playbackPositionMs()==uiLoader->playbackDurationMs(),"initial overview at recording end");
    auto *uiCloud=window->findChild<PointCloudGeometry*>("flightRecordCloud");
    auto *timeline=window->findChild<QObject*>("flightRecordTimeline");require(uiCloud&&timeline,"playback controls");
    const auto accumulated=uiCloud->vertexData();const int fullCount=route->waypointCount();
    timeline->setProperty("value",uiLoader->playbackDurationMs()*.25);require(QMetaObject::invokeMethod(timeline,"moved"),"scrub quarter");
    const int quarterCount=route->waypointCount();require(quarterCount>10&&quarterCount<fullCount/2,"scrubbing shortens trajectory to elapsed segment");
    require(uiCloud->vertexData()==accumulated&&!uiLoader->showingPlaybackFrame(),"cloud remains accumulated while scrubbing");
    require((route->waypointVector().last()-uiLoader->playbackPosePositionMeters()).length()<1e-5,"cursor follows visible trajectory endpoint");
    QElapsedTimer quarterFrames;quarterFrames.start();waitFor([&]{return quarterFrames.elapsed()>600;},"quarter frames",2000);
    require(window->grabWindow().save(out+"/trajectory-quarter.png"),"quarter trajectory screenshot");
    timeline->setProperty("value",uiLoader->playbackDurationMs()*.75);QMetaObject::invokeMethod(timeline,"moved");
    require(route->waypointCount()>quarterCount&&route->waypointCount()<fullCount,"forward scrub grows trajectory");
    timeline->setProperty("value",0);QMetaObject::invokeMethod(timeline,"moved");require(route->waypointCount()<quarterCount,"backward scrub removes future trajectory");
    timeline->setProperty("value",uiLoader->playbackDurationMs());QMetaObject::invokeMethod(timeline,"moved");
    require(route->waypointCount()==fullCount&&uiCloud->vertexData()==accumulated,"end restores full single-part trajectory without cloud change");
    report["single_part_only"]=true;report["timeline_changes_trajectory"]=true;report["pointcloud_stays_accumulated"]=true;report["quarter_trajectory_points"]=quarterCount;
    QElapsedTimer frames;frames.start();waitFor([&]{return frames.elapsed()>2000;},"render frames",5000);
    const auto shot=window->grabWindow();require(!shot.isNull()&&shot.save(out+"/flight-record-viewer.png"),"save real 3D visualization");
    report["ui_button_full_pipeline"]=true;report["ui_trajectory_vertices"]=route->waypointCount();report["window_screenshot"]=out+"/flight-record-viewer.png";
    QFile json(out+"/real-record-validation.json");json.open(QIODevice::WriteOnly);json.write(QJsonDocument(report).toJson());
    qInfo().noquote()<<QJsonDocument(report).toJson(QJsonDocument::Compact);
    window->close();
    QQmlApplicationEngine badgeEngine;
    badgeEngine.loadData(R"QML(import QtQuick
import CustomGeometryExample
Window {visible:true;width:370;height:94;color:"#202125"
    BrandBadge {x:16;y:24}
    Text {x:74;y:34;text:"科研飞行平台-地面站";font.pixelSize:18;color:"#eeeef1"}
}
)QML");
    require(!badgeEngine.rootObjects().isEmpty(),"brand badge QML");
    auto *brand=qobject_cast<QQuickWindow*>(badgeEngine.rootObjects().first());
    QElapsedTimer brandFrames;brandFrames.start();waitFor([&]{return brandFrames.elapsed()>1000;},"brand render",3000);
    require(brand->grabWindow().save(out+"/brand-badge.png"),"brand screenshot");brand->close();
    QDir(firstCache).removeRecursively();QDir(reader.cacheDirectory()).removeRecursively();return 0;
}
