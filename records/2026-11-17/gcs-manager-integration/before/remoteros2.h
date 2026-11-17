#pragma once
#include <QFile>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QProcessEnvironment>
#include <QRegularExpression>
#include <stdexcept>

// 所有Qt远程入口使用同一ROS2配置；与本地Python桥共享profile.json。
namespace RemoteRos2 {
inline QString quote(QString v) {return "'"+v.replace("'","'\\''")+"'";}
inline QJsonObject profile() {
    QFile f(qEnvironmentVariable("GCS_ROS2_PROFILE",":/qt/qml/CustomGeometryExample/tools/ros2/profile.json"));
    if(!f.open(QIODevice::ReadOnly))throw std::runtime_error("无法读取ROS2配置");
    QJsonParseError e;const auto d=QJsonDocument::fromJson(f.readAll(),&e);const auto p=d.object();
    if(e.error!=QJsonParseError::NoError || p.value("schema").toInt()!=1 || p.value("backend").toString()!="ros2")throw std::runtime_error("ROS2配置无效");
    const auto domain=p.value("domain_id");
    if(!domain.isDouble() || domain.toDouble()!=domain.toInt(-1) || domain.toInt(-1)<0 || domain.toInt()>232)
        throw std::runtime_error("ROS_DOMAIN_ID需为0至232整数");
    const QRegularExpression identifier("^[A-Za-z0-9_][A-Za-z0-9_.-]*$");
    for(const auto &key:QStringList{"user","host","rmw"})
        if(!p.value(key).isString() || !identifier.match(p.value(key).toString()).hasMatch())
            throw std::runtime_error("ROS2配置user/host/rmw无效");
    if(p.contains("overlays") && !p.value("overlays").isArray())throw std::runtime_error("overlays必须是数组");
    auto paths=p.value("overlays").toArray();paths.append(p.value("remote_env"));paths.append(p.value("app_root"));
    if(p.contains("records_root"))paths.append(p.value("records_root"));
    for(const auto &path:paths) {
        const auto v=path.toString();
        if(!path.isString() || !v.startsWith('/') || v.contains(QChar(0)) || v.contains('\n'))
            throw std::runtime_error("ROS2环境及数据目录必须是绝对路径");
    }
    return p;
}
inline QString user(){return profile().value("user").toString("orangepi");}
inline QString host(){return profile().value("host").toString();}
inline QString command(const QString &program) {
    const auto p=profile();QStringList parts{"unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME", "export PYTHONNOUSERSITE=1",
        "source "+quote(p.value("remote_env").toString())+" || exit 78"};
    for(const auto &v:p.value("overlays").toArray())parts<<"source "+quote(v.toString())+" || exit 78";
    const auto config=QJsonDocument(p).toJson(QJsonDocument::Compact).toBase64();
    parts<<"export ROS_DOMAIN_ID="+QString::number(p.value("domain_id").toInt())+" RMW_IMPLEMENTATION="+quote(p.value("rmw").toString())+" GCS_ROS2_CONFIG="+quote(QString::fromLatin1(config));
    parts<<program;return "bash -c "+quote(parts.join("; "));
}
inline QByteArray payload(const QByteArray &source) {
    QByteArray code="import sys,types,base64\n";
    for(const auto &name:QStringList{"runtime","recording"}) {
        QFile f(":/qt/qml/CustomGeometryExample/tools/ros2/"+name+".py");
        if(!f.open(QIODevice::ReadOnly))throw std::runtime_error("ROS2运行时资源缺失");
        const auto module="gcs_ros2_"+name.toUtf8();
        code+="_gcs_module=types.ModuleType('"+module+"');sys.modules['"+module+"']=_gcs_module\n";
        code+="_gcs_module.__gcs_source__=base64.b64decode('"+f.readAll().toBase64()+"')\nexec(_gcs_module.__gcs_source__,_gcs_module.__dict__)\n";
    }
    return code+source;
}

inline QString manager(const QString &role,const QStringList &args) {
    QFile f(":/qt/qml/CustomGeometryExample/tools/ros2/manager.py");
    if(!f.open(QIODevice::ReadOnly))throw std::runtime_error("ROS2管理接口资源缺失");
    const auto encoded=payload(f.readAll()).toBase64();
    QString program="exec /usr/bin/python3 -u -c "+quote("exec(__import__('base64').b64decode('"+QString::fromLatin1(encoded)+"'))")+" "+quote(role);
    for(const auto &arg:args)program+=" "+quote(arg);
    return command(program);
}

}
