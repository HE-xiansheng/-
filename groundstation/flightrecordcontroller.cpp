#include "flightrecordcontroller.h"
#include "sshsession.h"
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QStorageInfo>
#include <QDir>
#include <QUrl>
#include <QDebug>
namespace {QString quoted(QString s){return "'"+s.replace("'","'\\''")+"'";}}
FlightRecordController::FlightRecordController(QObject *parent):QObject(parent){
    connect(&m_process,&QProcess::readyReadStandardOutput,this,&FlightRecordController::consume);
    connect(&m_process,&QProcess::readyReadStandardError,this,[this]{m_error+=m_process.readAllStandardError();m_error=m_error.right(4096);});
    connect(&m_process,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,&FlightRecordController::finish);
    connect(&m_process,&QProcess::errorOccurred,this,[this](QProcess::ProcessError e){if(e==QProcess::FailedToStart)fail(tr("SSH启动失败：")+m_process.errorString());});
    m_timeout.setSingleShot(true);m_timeout.setInterval(30000);
    connect(&m_timeout,&QTimer::timeout,this,[this]{m_error="读取超时，请检查机载通信";m_process.kill();});
}
FlightRecordController::~FlightRecordController(){m_cancelled=true;m_process.kill();m_process.waitForFinished(1000);}
void FlightRecordController::setHost(const QString &v){static const QRegularExpression valid("^[a-zA-Z0-9][a-zA-Z0-9.-]{0,252}$");if(v==m_host||!valid.match(v).hasMatch())return;cancel();m_host=v;m_records.clear();m_message.clear();emit changed();}
void FlightRecordController::request(const QString &action,const QStringList &args){
    static const QRegularExpression hostPattern("^[a-zA-Z0-9][a-zA-Z0-9.-]{0,252}$");
    if(!hostPattern.match(m_host).hasMatch()){fail(tr("机载IP无效"));return;}
    QFile helper(":/qt/qml/CustomGeometryExample/tools/flight-records/remote_records.py");
    if(!helper.open(QIODevice::ReadOnly)){fail(tr("无法读取飞行记录工具"));return;}
    m_hasError=false;m_action=action;m_output.clear();m_error.clear();m_cancelled=false;m_busy=true;
    QString remote="exec /usr/bin/python3 - "+action;
    for(const auto &arg:args)remote+=" "+quoted(arg);
    SshSession::start(m_process,{"-T","-o",SshSession::batchOption(),"-o","ConnectTimeout=5","-o","ServerAliveInterval=5","-o","ServerAliveCountMax=2","orangepi@"+m_host,remote});
    m_process.write(helper.readAll());m_process.closeWriteChannel();m_timeout.start();emit changed();
}
void FlightRecordController::refresh(){if(m_busy)return;m_records.clear();m_total=m_done=m_received=0;m_message=tr("正在读取机载飞行记录…");request("list");}
void FlightRecordController::consume(){
    const auto data=m_process.readAllStandardOutput();
    if(m_action=="read"&&m_file){
        if(m_file->write(data)!=data.size()){m_error="本地缓存写入失败";m_process.kill();return;}
        m_received+=data.size();m_timeout.start();
    }else{m_output+=data;if(m_output.size()>4*1024*1024){m_error="机载记录列表过大";m_process.kill();}}
    emit changed();
}
void FlightRecordController::fail(const QString &reason){m_hasError=true;m_timeout.stop();m_file.reset();m_busy=false;m_message=reason;qWarning().noquote()<<"读取飞行记录失败"<<m_host<<reason;emit changed();}
void FlightRecordController::finish(int code,QProcess::ExitStatus state){
    consume();m_timeout.stop();m_error+=m_process.readAllStandardError();
    if(m_cancelled){m_file.reset();m_busy=false;m_message=tr("已取消读取");emit changed();return;}
    if(code||state!=QProcess::NormalExit){fail(m_error.isEmpty()?tr("读取失败，请检查机载通信"):QString::fromUtf8(m_error).right(600));return;}
    if(m_action=="list"){
        QJsonParseError error;const auto doc=QJsonDocument::fromJson(m_output,&error);
        if(error.error!=QJsonParseError::NoError||!doc.object().value("records").isArray()){fail(tr("机载记录列表格式无效"));return;}
        m_records=doc.object().value("records").toArray().toVariantList();m_busy=false;
        m_message=m_records.isEmpty()?tr("暂无已结束的飞行记录"):tr("找到 %1 次飞行记录").arg(m_records.size());emit changed();return;
    }
    const auto info=m_files[m_index].toMap();
    if(m_received!=info.value("size").toLongLong()||!m_file||!m_file->commit()){fail(tr("录包大小校验或保存失败，请重新读取"));return;}
    m_urls.append(QUrl::fromLocalFile(m_directory+"/"+info.value("name").toString()));
    m_done+=m_received;m_received=0;m_file.reset();++m_index;
    QTimer::singleShot(0,this,&FlightRecordController::nextFile);
}
void FlightRecordController::readRecord(int index){
    if(m_busy||index<0||index>=m_records.size())return;
    const auto row=m_records[index].toMap();beginRead(row,row.value("files").toList());
}
void FlightRecordController::readRecordPart(int recordIndex,int partIndex){
    if(m_busy||recordIndex<0||recordIndex>=m_records.size())return;
    const auto row=m_records[recordIndex].toMap();const auto files=row.value("files").toList();
    if(partIndex<0||partIndex>=files.size())return;
    beginRead(row,{files[partIndex]});
}
void FlightRecordController::beginRead(const QVariantMap &row,const QVariantList &files){
    m_files=files;m_name=row.value("name").toString();
    static const QRegularExpression safe("^[a-zA-Z0-9_.-]+$");
    if(m_files.isEmpty()||!safe.match(m_name).hasMatch()){fail(tr("飞行记录名称无效"));return;}
    m_directory=QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)+"/flight-records/"+m_host+"/"+m_name;
    if(!QDir().mkpath(m_directory)){fail(tr("无法创建本地飞行记录缓存"));return;}
    m_total=m_done=m_received=0;m_index=0;m_urls.clear();m_cancelled=false;
    for(const auto &f:m_files){const auto v=f.toMap();if(!safe.match(v.value("name").toString()).hasMatch()||v.value("size").toLongLong()<=0){fail(tr("录包文件名或大小无效"));return;}m_total+=v.value("size").toLongLong();}
    const QStorageInfo storage(m_directory);if(storage.isValid()&&storage.isReady()&&storage.bytesAvailable()<m_total+64*1024*1024){fail(tr("本地磁盘空间不足"));return;}
    m_busy=true;nextFile();
}
void FlightRecordController::nextFile(){
    if(m_cancelled){m_busy=false;emit changed();return;}
    if(m_index>=m_files.size()){
        m_busy=false;m_message=tr("录包已读取到本机");emit changed();
        qInfo()<<"机载飞行记录读取完成"<<m_host<<m_name<<m_files.size()<<m_done;
        emit recordingReady(m_urls,m_name);return;
    }
    const auto info=m_files[m_index].toMap();m_received=0;
    m_file=std::make_unique<QSaveFile>(m_directory+"/"+info.value("name").toString());
    if(!m_file->open(QIODevice::WriteOnly)){fail(tr("无法保存录包到本地"));return;}
    m_message=tr("读取 %1（%2/%3）").arg(m_name).arg(m_index+1).arg(m_files.size());
    const auto token=info.value("path").toString().toUtf8().toBase64(QByteArray::Base64UrlEncoding);
    request("read",{"--path",QString::fromLatin1(token),"--size",info.value("size").toString(),"--mtime",info.value("mtime_ns").toString()});
}
void FlightRecordController::cancel(){m_cancelled=true;m_timeout.stop();if(m_process.state()!=QProcess::NotRunning)m_process.kill();else if(m_busy){m_busy=false;m_file.reset();m_message=tr("已取消读取");emit changed();}}
