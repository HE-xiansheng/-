#include "localizationcontroller.h"
#include <QGuiApplication>
#include <QQmlComponent>
#include <QQmlEngine>
#include <QTemporaryDir>
#include <QFile>
#include <QEventLoop>
#include <QTimer>
#include <QDebug>
#include <memory>
void require(bool ok,const char *why){if(!ok)qFatal("FAIL: %s",why);qInfo()<<"PASS"<<why;}
void wait(int ms){QEventLoop l;QTimer::singleShot(ms,&l,&QEventLoop::quit);l.exec();}
void idle(LocalizationController &c){for(int i=0;c.busy()&&i<200;++i)wait(10);require(!c.busy(),"SSH completes");}
int main(int argc,char **argv){
 QGuiApplication app(argc,argv);QTemporaryDir dir;
 QFile ssh(dir.path()+"/ssh");require(ssh.open(QIODevice::WriteOnly),"create fake SSH");
 ssh.write(R"SH(#!/bin/sh
for last do :; done
printf '%s\n' "$last" >> "$GCS_LOC_CALLS"
cat > "$GCS_LOC_SOURCE"
case "$last" in
*' - start '*) echo '{"ok":true,"action":"start"}';;
*' - stop '*) echo '{"ok":false,"message":"请先停止避障预览"}'; exit 1;;
*) exit 9;;
esac
)SH");ssh.close();ssh.setPermissions(QFileDevice::ReadOwner|QFileDevice::WriteOwner|QFileDevice::ExeOwner);
 qputenv("PATH",dir.path().toUtf8()+":"+qgetenv("PATH"));qputenv("GCS_LOC_CALLS",(dir.path()+"/calls").toUtf8());qputenv("GCS_LOC_SOURCE",(dir.path()+"/helper").toUtf8());
 LocalizationController c;c.setProperty("host","192.168.1.224");int resets=0;QObject::connect(&c,&LocalizationController::sessionReset,[&](){++resets;});
 QQmlEngine engine;QQmlComponent qml(&engine,QUrl::fromLocalFile(QStringLiteral(GCS_QML_DIR "/LocalizationPanel.qml")));
 if(qml.isError()){qCritical()<<qml.errors();return 1;}
 std::unique_ptr<QObject> panel(qml.createWithInitialProperties({{"controller",QVariant::fromValue(&c)}}));require(bool(panel),"real panel loads");
 auto *start=panel->findChild<QObject*>("localizationStart"),*stop=panel->findChild<QObject*>("localizationStop");require(start&&stop,"real buttons exist");
 require(!start->property("enabled").toBool()&&!QFile::exists(dir.path()+"/calls"),"no automatic localization start");
 QVariantMap state{{"lio_running",false},{"relay_running",false},{"session_id",""}};
 QVariantMap lio{{"key","lio"},{"ok",false},{"state","缺失/过期"}},relay{{"key","relay"},{"ok",false},{"state","缺失/过期"}};
 c.observe(state,{lio,relay},true);QMetaObject::invokeMethod(start,"clicked");idle(c);require(resets==1&&!c.lioReady()&&!c.relayReady(),"request clears old display but never invents healthy data");
 state["lio_running"]=true;state["relay_running"]=true;state["session_id"]="boot:123:456";c.observe(state,{lio,relay},true);
 require(c.lioText().contains(QStringLiteral("等待数据"))&&!c.lioReady(),"live PID is not fresh localization");require(resets==1,"first observed session preserves current app binding");
 lio["ok"]=true;relay["ok"]=true;c.observe(state,{lio,relay},true);require(c.lioReady()&&c.relayReady()&&c.message().isEmpty(),"actual healthy stream clears startup hint");
 c.observe(state,{lio,relay},false);require(!start->property("enabled").toBool()&&!c.lioReady()&&!c.relayReady(),"stale monitor disables controls and healthy status");
 c.observe(state,{lio,relay},true);require(resets==1,"reconnect same origin preserves display");
 state["session_id"]="boot:124:555";c.observe(state,{lio,relay},true);require(resets==2,"new localization session clears previous origin");
 QMetaObject::invokeMethod(stop,"clicked");idle(c);require(c.message()==QStringLiteral("请先停止避障预览"),"guard failure shown without raw process metadata");
 QFile calls(dir.path()+"/calls");require(calls.open(QIODevice::ReadOnly),"calls readable");const auto commands=calls.readAll();require(commands.count("gcs_localization_stack")==2&&!commands.contains("set_mode")&&!commands.contains("arming")&&!commands.contains("ego_planner/devel/setup.bash")&&!commands.contains("noetic")&&commands.contains("source ~/drone_platform/env.sh &&"),"buttons use the original localization workspace without overriding its package path");
 QFile helper(dir.path()+"/helper");require(helper.open(QIODevice::ReadOnly)&&helper.readAll().contains("module.main()"),"packaged helper actually sent over stdin");
 qInfo()<<"PASS: localization buttons, packaged SSH helper, actual freshness, errors, session identity, reconnect, no automatic flight commands";
}
