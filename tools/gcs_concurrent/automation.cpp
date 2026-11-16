// 仅给独立测试二进制加入自动化；使用正式main、Main.qml和原控制器。
#include <QGuiApplication>
#include <QQuickWindow>
#include <QQuickItem>
#include <QQmlContext>
#include <QTimer>
#include <QThread>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFile>
#include <QProcess>
#include <functional>
#include "connectioncontroller.h"
#include <QQuickImageProvider>
#include <QMutex>
// 仅测试探针读取既有字段；不改产品控制器源码或数据处理。
#define private public
#include "videocontroller.h"
#undef private
#include "preflightreceiver.h"
#include "udptransport.h"
#include "recordingcontroller.h"
#include "missioncontroller.h"
#include "runtimepaths.h"
namespace {
QJsonObject checks;
void require(bool ok,const char *name){checks[name]=ok;qInfo()<< (ok?"PASS":"FAIL") <<name;if(!ok)throw QString(name);}
bool until(const std::function<bool()> &f,int timeout=30000){
 if(f())return true;
 QEventLoop loop;QTimer poll,deadline;bool ready=false;deadline.setSingleShot(true);
 QObject::connect(&poll,&QTimer::timeout,&loop,[&]{if(f()){ready=true;loop.quit();}});
 QObject::connect(&deadline,&QTimer::timeout,&loop,&QEventLoop::quit);
 poll.start(10);deadline.start(timeout);loop.exec();return ready;
}
void run(){
 QQuickWindow *window=nullptr;VideoController *video=nullptr;MissionController *mission=nullptr;
 bool passed=false;QString error;QJsonObject result;
 qint64 rawCallbacks=0,rawBytes=0;
 try{
  require(until([&]{for(auto w:QGuiApplication::allWindows()){auto q=qobject_cast<QQuickWindow*>(w);if(q&&q->findChild<ConnectionController*>()){window=q;return true;}}return false;}),"full_main_window_created");
  auto connection=window->findChild<ConnectionController*>();auto cloud=window->findChild<UdpPointCloudReceiver*>();auto pose=window->findChild<UdpPoseReceiver*>();auto monitor=window->findChild<PreflightReceiver*>();auto recorder=window->findChild<RecordingController*>();mission=window->findChild<MissionController*>();
  video=qobject_cast<VideoController*>(qmlContext(window)->contextProperty("videoController").value<QObject*>());
  require(connection&&cloud&&pose&&monitor&&recorder&&video&&mission,"production_controllers_found");
  if(qEnvironmentVariableIsSet("GCS_VIDEO_PIPE_PROBE")){
   video->setHost("192.168.1.224");video->setStreamEnabled(1,true);
   QEventLoop loop;QTimer tick,end;QJsonArray rows;QElapsedTimer clock;clock.start();end.setSingleShot(true);
   QObject::connect(&tick,&QTimer::timeout,&loop,[&]{rows.append(QJsonObject{{"ms",clock.elapsed()},{"revision",video->gimbalRevision()},{"buffer",video->m_gimbalBuffer.size()},{"pending",video->m_gimbal.bytesAvailable()},{"fresh",video->gimbalFresh()}});});
   QObject::connect(&end,&QTimer::timeout,&loop,&QEventLoop::quit);tick.start(1000);end.start(25000);loop.exec();
   result["rows"]=rows;result["scope"]="Full production main/window/controller, local synthetic raw GStreamer only; pipe diagnosis, no SSH/ROS/flight";
   result["frames"]=video->gimbalRevision()-1;result["passed"]=video->gimbalRevision()>300;
   video->stop();QFile file(qEnvironmentVariable("GCS_CONCURRENT_RESULT"));if(file.open(QIODevice::WriteOnly))file.write(QJsonDocument(result).toJson());QCoreApplication::exit(result["passed"].toBool()?0:1);return;
  }
  auto click=[&](const char *name){auto b=window->findChild<QObject*>(name);require(b,name);require(b->property("enabled").toBool(),"button_enabled");require(QMetaObject::invokeMethod(b,"clicked"),"actual_qml_clicked_handler");};
  click("onboardConnect");
  require(until([&]{return connection->authenticated()&&!connection->busy()&&monitor->fresh();},45000),"real_ssh_connection_and_telemetry");
  require(until([&]{return cloud->framesReceived()>4&&pose->packetsReceived()>20;}),"actual_lio_cloud_pose_cross_host_udp");
  require(cloud->lastPointCount()>0,"cloud_points_available");
  QObject::connect(&video->m_gimbal,&QProcess::readyReadStandardOutput,window,[&]{
   ++rawCallbacks;rawBytes=qint64(video->gimbalRevision()-1)*640*360*3+video->m_gimbalBuffer.size();
  });
  click("toggleStereoVideo");click("toggleGimbalVideo");
  require(until([&]{return video->stereoFresh()&&video->gimbalFresh();},30000),"both_live_video_streams");
  std::function<int(QQuickItem*)> loadedImages=[&](QQuickItem *item){int count=0;if(item->property("source").toUrl().toString().startsWith("image://livevideo/")&&item->property("status").toInt()==1)++count;for(auto child:item->childItems())count+=loadedImages(child);return count;};
  require(until([&]{return loadedImages(window->contentItem())>=2;}),"both_actual_qml_images_loaded");
  require(until([&]{return recorder->canStart();}),"recording_state_arrives_through_production_telemetry");
  click("recordingStart");require(until([&]{return recorder->recording()&&recorder->canStop();},45000),"qml_record_start_confirmed_by_remote_state");
  const int frames=cloud->framesReceived(),poses=pose->packetsReceived(),stereo=video->stereoRevision(),gimbal=video->gimbalRevision();
  QElapsedTimer t;t.start();int samples=0;bool allFresh=true;QJsonArray sampleRows;
  QEventLoop sampleLoop;QTimer sampler,finish;finish.setSingleShot(true);
  QObject::connect(&sampler,&QTimer::timeout,&sampleLoop,[&]{++samples;sampleRows.append(QJsonObject{{"elapsed_ms",t.elapsed()},{"stereo_fresh",video->stereoFresh()},{"gimbal_fresh",video->gimbalFresh()},{"gimbal_revision",video->gimbalRevision()},{"raw_callbacks",rawCallbacks},{"raw_bytes_estimate",rawBytes},{"raw_pending_bytes",video->m_gimbal.bytesAvailable()},{"raw_buffer_bytes",video->m_gimbalBuffer.size()},{"video_clock_ms",video->m_clock.elapsed()},{"last_gimbal_frame_ms",video->m_received[1]},{"stereo_revision",video->stereoRevision()},{"cloud_frames",cloud->framesReceived()},{"telemetry_fresh",monitor->fresh()},{"monitor_age",monitor->monitorAge()},{"recording",recorder->recording()},{"recording_fresh",recorder->fresh()},{"video_status",video->status()}});allFresh=allFresh&&video->stereoFresh()&&video->gimbalFresh()&&monitor->fresh()&&recorder->recording();});
  QObject::connect(&finish,&QTimer::timeout,&sampleLoop,&QEventLoop::quit);
  sampler.start(1000);finish.start(30000);sampleLoop.exec();
  result["concurrent_samples"]=sampleRows;result["sample_count"]=samples;result["recording_elapsed"]=recorder->elapsedSeconds();result["bag_mb"]=recorder->bagMegabytes();
  require(allFresh&&samples>=29,"simultaneous_streams_and_recording_remain_fresh_30s");
  require(cloud->framesReceived()>frames+20&&pose->packetsReceived()>poses+150,"live_cloud_pose_progress_while_recording");
  require(video->stereoRevision()>stereo+100&&video->gimbalRevision()>gimbal+300,"both_video_streams_progress_while_recording");
  require(recorder->elapsedSeconds()>25&&recorder->bagMegabytes()>1,"recording_elapsed_and_size_update_in_main_ui");
  require(!window->grabWindow().isNull(),"full_main_scene_rendered");
  click("recordingStop");require(until([&]{return !recorder->recording()&&recorder->canStart()&&recorder->bagMegabytes()>1;},45000),"qml_record_stop_normal_finalization");
  result["cloud_frames"]=cloud->framesReceived();result["pose_packets"]=pose->packetsReceived();result["stereo_revision"]=video->stereoRevision();result["gimbal_revision"]=video->gimbalRevision();result["bag_mb"]=recorder->bagMegabytes();result["run"]=recorder->outputDirectory();
  click("toggleStereoVideo");require(!video->stereoEnabled()&&video->gimbalEnabled()&&video->gimbalFresh(),"closing_stereo_keeps_gimbal_live");click("toggleGimbalVideo");require(!video->active(),"both_video_buttons_stop_receivers");
  passed=true;
 }catch(const QString &s){error=s;}
 if(video)video->stop();if(mission)mission->disconnectMonitor();
 for(const auto &name:{"tools/pointcloud-only/manage.py","tools/preflight/bridge.py"})QProcess::execute("/usr/bin/python3",{RuntimePaths::tool(name),"stop"});
 result["passed"]=passed;result["error"]=error;result["checks"]=checks;result["scope"]="Actual main/QML/controllers; actual camera/NPU/LIO replay; synthetic gimbal RTSP on independent port; record writes redirected to isolated copy; no flight commands";
 QFile file(qEnvironmentVariable("GCS_CONCURRENT_RESULT"));if(file.open(QIODevice::WriteOnly))file.write(QJsonDocument(result).toJson());QCoreApplication::exit(passed?0:1);
}
void schedule(){if(!qEnvironmentVariable("GCS_CONCURRENT_RESULT").isEmpty())QTimer::singleShot(1500,run);}
}
Q_COREAPP_STARTUP_FUNCTION(schedule)
