#include <QCoreApplication>
#include <QProcess>
#include <QTimer>
#include <QElapsedTimer>
#include <QDebug>
// 仅测 QProcess 读取吞吐；不加载平台/ROS/网络。
int main(int argc,char **argv){
 QCoreApplication app(argc,argv);QProcess source;QElapsedTimer clock;clock.start();qint64 bytes=0,calls=0;
 QObject::connect(&source,&QProcess::readyReadStandardOutput,&app,[&]{bytes+=source.readAllStandardOutput().size();++calls;});
 QObject::connect(&source,&QProcess::readyReadStandardError,&app,[&]{qWarning().noquote()<<source.readAllStandardError();});
 source.start("/usr/bin/gst-launch-1.0",{"-q","videotestsrc","is-live=true","!","video/x-raw,format=RGB,width=640,height=360,framerate=30/1","!","fdsink","fd=1","sync=false"});
 QTimer tick;QObject::connect(&tick,&QTimer::timeout,&app,[&]{qInfo()<<"ms"<<clock.elapsed()<<"bytes"<<bytes<<"frames"<<bytes/(640*360*3)<<"callbacks"<<calls;});tick.start(1000);
 QTimer::singleShot(10000,&app,[&]{source.kill();source.waitForFinished(1000);app.exit(bytes>qint64(640*360*3)*200?0:1);});return app.exec();
}
