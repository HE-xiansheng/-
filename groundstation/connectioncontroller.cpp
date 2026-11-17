#include "connectioncontroller.h"
#include "sshsession.h"
#include "runtimepaths.h"
#include <QSettings>
#include <QRegularExpression>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QJsonDocument>
#include <QJsonObject>

ConnectionController::ConnectionController(QObject *parent):QObject(parent) {
#ifdef Q_OS_LINUX
    // 本机后台遥测跟随所属窗口退出；不改变机载传感器或定位会话。
    qputenv("GCS_GUI_OWNER_PID", QByteArray::number(QCoreApplication::applicationPid()));
#endif
    // 与启动时的视频地址一致；命令行地址仅在认证成功后持久保存。
    const auto startupHost=QCoreApplication::instance()->property("startupOnboardHost");
    m_host=startupHost.isValid()?startupHost.toString():QSettings().value("onboard/host","192.168.1.224").toString();
    m_message=tr("尚未检查连接");
    m_process.setProcessChannelMode(QProcess::MergedChannels);
    connect(&m_process,&QProcess::readyReadStandardOutput,this,[this](){m_output+=QString::fromUtf8(m_process.readAllStandardOutput());m_output=m_output.right(8000);});
    connect(&m_process,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,&ConnectionController::finished);
    connect(&m_process,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error){if(error==QProcess::FailedToStart){m_timeout.stop();m_message=tr("无法启动连接程序");m_pendingPassword.clear();emit changed();}});
    m_timeout.setSingleShot(true);m_timeout.setInterval(45000);
    connect(&m_timeout,&QTimer::timeout,this,[this](){m_stage="timeout";m_process.kill();});
#ifdef Q_OS_WIN
    connect(&m_bridge,&QProcess::readyReadStandardOutput,this,[this](){
        const auto output=m_bridge.readAllStandardOutput();
        if(output.contains("GCS_BRIDGE_READY")){m_message=tr("SSH已连接，等待机载数据");emit changed();}
    });
    connect(&m_bridge,&QProcess::readyReadStandardError,this,[this](){
        const auto output=m_bridge.readAllStandardError();
        if(!output.trimmed().isEmpty())qWarning().noquote()<<"Windows遥测桥接:"<<QString::fromUtf8(output).left(600);
    });
    connect(&m_bridge,&QProcess::errorOccurred,this,[this](QProcess::ProcessError){
        m_message=tr("SSH已连接，遥测桥接启动失败，请检查安装是否完整");emit changed();
    });
    connect(&m_bridge,qOverload<int,QProcess::ExitStatus>(&QProcess::finished),this,[this](int,QProcess::ExitStatus){
        m_message=tr("遥测桥接已退出，请重新应用连接");emit changed();
    });
#endif
}
ConnectionController::~ConnectionController(){
#ifdef Q_OS_WIN
    m_bridge.kill();m_bridge.waitForFinished(3000);
#endif
}
void ConnectionController::connectTo(const QString &host,const QString &password,int cloudPort,int posePort,int preflightPort) {
    if(busy())return;
    static const QRegularExpression valid("^[a-zA-Z0-9][a-zA-Z0-9.-]{0,252}$");
    const auto candidate=host.trimmed();
    if(!valid.match(candidate).hasMatch() || cloudPort<1 || cloudPort>65535 || posePort<1 || posePort>65535 || preflightPort<1 || preflightPort>65535){m_message=tr("IP或端口无效");emit changed();return;}
    m_pendingHost=candidate;m_pendingPassword=password.toUtf8();m_cloudPort=cloudPort;m_posePort=posePort;m_preflightPort=preflightPort;
    m_stage="authenticate";m_output.clear();m_message=tr("正在连接 %1…").arg(candidate);
    SshSession::apply(m_process,m_pendingPassword);
    SshSession::start(m_process,{"-T","-o",SshSession::batchOption(m_pendingPassword),"-o","NumberOfPasswordPrompts=1","-o","ConnectTimeout=6","orangepi@"+candidate,"printf GCS_SSH_OK"},m_pendingPassword);
    m_process.closeWriteChannel();m_timeout.start();emit changed();
}
void ConnectionController::finished(int code,QProcess::ExitStatus status) {
    m_timeout.stop();m_output+=QString::fromUtf8(m_process.readAllStandardOutput());
    if(code!=0 || status!=QProcess::NormalExit || m_stage=="timeout") {
        if(m_stage=="link")m_message=tr("SSH已连接，数据桥接未启动，请检查本地桥接日志");
        else if(m_stage=="timeout")m_message=tr("连接超时，请检查IP和设备供电");
        else if(m_output.contains("REMOTE HOST IDENTIFICATION HAS CHANGED"))m_message=tr("SSH主机指纹已变化，请核实设备身份后更新指纹");
        else if(m_output.contains("Host key verification failed"))m_message=tr("SSH主机指纹校验失败");
        else if(m_output.contains("Permission denied") || m_output.contains("Authentication failed"))m_message=tr("SSH认证失败，请检查用户名和密码；密码留空时使用密钥");
        else if(m_output.contains("Connection refused"))m_message=tr("设备拒绝SSH连接，请检查SSH服务是否运行");
        else if(m_output.contains("No route to host") || m_output.contains("Connection timed out"))m_message=tr("设备不可达，请检查IP和连接状态");
        else m_message=tr("SSH连接未完成，请检查地址和认证信息");
        m_pendingPassword.clear();emit changed();return;
    }
    if(m_stage=="authenticate") {
        if(!m_output.contains("GCS_SSH_OK")){m_message=tr("未收到SSH连接回执");m_pendingPassword.clear();emit changed();return;}
        emit endpointChanging();
        SshSession::password=m_pendingPassword;m_pendingPassword.clear();m_host=m_pendingHost;m_authenticated=true;
        QSettings().setValue("onboard/host",m_host);
        const auto stateDir=RuntimePaths::stateDirectory();
        if(QDir().mkpath(stateDir)) {
            QSaveFile saved(QDir(stateDir).filePath("connection.json"));
            if(saved.open(QIODevice::WriteOnly)) {
                saved.write(QJsonDocument(QJsonObject{{"host",m_host},{"cloud_port",m_cloudPort},{"pose_port",m_posePort}}).toJson());
                saved.commit();
            }
        }
#ifdef Q_OS_WIN
        const QDir appDir(QCoreApplication::applicationDirPath());
        const auto helper=appDir.absoluteFilePath("../tools/communication/windows_bridge.py");
        const auto python=appDir.absoluteFilePath("../runtime/python/python.exe");
        if(!QFileInfo::exists(helper)||!QFileInfo::exists(python)){m_message=tr("SSH已连接，安装目录缺少Windows遥测工具");emit changed();return;}
        m_bridge.kill();m_bridge.waitForFinished(3000);
        SshSession::apply(m_bridge);
        m_message=tr("SSH已连接，正在接通点云和话题监视…");
        m_bridge.start(python,{"-u",helper,"--host",m_host,"--cloud-port",QString::number(m_cloudPort),"--pose-port",QString::number(m_posePort),"--preflight-port",QString::number(m_preflightPort)});
        emit changed();return;
#else
        const auto helper=RuntimePaths::tool("tools/communication/manage_link.py");
        if(!QFileInfo::exists(helper)){m_message=tr("SSH已连接；此安装目录未包含自动数据桥接工具");emit changed();return;}
        m_stage="link";m_message=tr("SSH已连接，正在接通点云和话题监视…");m_output.clear();
        SshSession::apply(m_process);
        m_process.start("/usr/bin/python3",{helper,"--host",m_host,"--cloud-port",QString::number(m_cloudPort),"--pose-port",QString::number(m_posePort),"--preflight-port",QString::number(m_preflightPort)});
        m_process.closeWriteChannel();m_timeout.start();emit changed();return;
#endif
    }
    m_message=tr("SSH已连接，等待机载数据");emit changed();
}
