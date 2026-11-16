#pragma once
#include <QProcess>
#include <QProcessEnvironment>
#include <QTemporaryDir>
#include <QFile>
#include <QCoreApplication>
#include <QDir>

// 全部SSH入口共享本次会话凭据；密码仅经子进程环境交给askpass，不拼入参数或日志。
namespace SshSession {
inline QByteArray password;
inline QTemporaryDir helperDirectory;
inline QString helperPath() {
#ifdef Q_OS_WIN
    return QDir(QCoreApplication::applicationDirPath()).filePath("ssh-askpass.exe");
#else
    const QString path=helperDirectory.path()+"/askpass";
    if (!QFile::exists(path)) {
        QFile file(path);
        if (!file.open(QIODevice::WriteOnly)) return {};
        file.write(R"SH(#!/bin/sh
case "$1" in *assword*) ;; *) exit 1 ;; esac
printf '%s\n' "$GCS_SSH_PASSWORD"
)SH");file.close();
        file.setPermissions(QFileDevice::ReadOwner|QFileDevice::WriteOwner|QFileDevice::ExeOwner);
    }
    return path;
#endif
}
inline QString batchOption(const QByteArray &secret=password) {return secret.isEmpty()?QStringLiteral("BatchMode=yes"):QStringLiteral("BatchMode=no");}
inline void apply(QProcess &process,const QByteArray &secret=password) {
    auto env=QProcessEnvironment::systemEnvironment();
    env.remove("GCS_SSH_PASSWORD");env.remove("SSH_ASKPASS");env.remove("SSH_ASKPASS_REQUIRE");
    if (!secret.isEmpty()) {
        env.insert("GCS_SSH_PASSWORD",QString::fromUtf8(secret));
        env.insert("SSH_ASKPASS",helperPath());env.insert("SSH_ASKPASS_REQUIRE","force");
        if(env.value("DISPLAY").isEmpty())env.insert("DISPLAY",":0");
    }
    process.setProcessEnvironment(env);
}
inline void start(QProcess &process,const QStringList &arguments,const QByteArray &secret=password) {
    apply(process,secret);
    // 新地址首次连接登记主机指纹；已登记指纹发生变化仍由SSH拒绝。
    QStringList options{QStringLiteral("-o"),QStringLiteral("StrictHostKeyChecking=accept-new")};
    options.append(arguments);
#ifndef Q_OS_WIN
    // Ubuntu 20.04 的 OpenSSH 8.2 不支持强制askpass；脱离终端后才能可靠使用密码框。
    if(!secret.isEmpty()) {
        QStringList detached{QStringLiteral("ssh")};detached.append(options);
        process.start(QStringLiteral("setsid"),detached);return;
    }
#endif
    process.start(QStringLiteral("ssh"),options);
}

}
