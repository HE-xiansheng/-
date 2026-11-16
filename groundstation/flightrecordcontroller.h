#pragma once
#include <QObject>
#include <QProcess>
#include <QTimer>
#include <QSaveFile>
#include <QVariantList>
#include <QtQml/qqmlregistration.h>
#include <memory>
// 独立的只读SSH通道：列举已结束的飞行记录，逐分包原子下载到本机缓存。
class FlightRecordController : public QObject {
    Q_OBJECT
    QML_NAMED_ELEMENT(FlightRecordController)
    Q_PROPERTY(QString host READ host WRITE setHost NOTIFY changed)
    Q_PROPERTY(QVariantList records READ records NOTIFY changed)
    Q_PROPERTY(bool busy READ busy NOTIFY changed)
    Q_PROPERTY(double progress READ progress NOTIFY changed)
    Q_PROPERTY(bool hasError READ hasError NOTIFY changed)
    Q_PROPERTY(QString message READ message NOTIFY changed)
    Q_PROPERTY(QString cacheDirectory READ cacheDirectory NOTIFY changed)
public:
    explicit FlightRecordController(QObject *parent=nullptr);
    ~FlightRecordController() override;
    QString host() const{return m_host;}
    void setHost(const QString &host);
    QVariantList records() const{return m_records;}
    bool busy() const{return m_busy;}
    double progress() const{return m_total>0 ? double(m_done+m_received)/double(m_total):0.;}
    bool hasError() const{return m_hasError;}
    QString message() const{return m_message;}
    QString cacheDirectory() const{return m_directory;}
    Q_INVOKABLE void refresh();
    Q_INVOKABLE void readRecord(int index);
    Q_INVOKABLE void readRecordPart(int recordIndex,int partIndex);
    Q_INVOKABLE void cancel();
signals:
    void changed();
    void recordingReady(const QVariantList &files,const QString &name);
private:
    void request(const QString &action,const QStringList &args={});
    void beginRead(const QVariantMap &row,const QVariantList &files);
    void nextFile();
    void consume();
    void finish(int code,QProcess::ExitStatus state);
    void fail(const QString &reason);
    QString m_host="192.168.1.235",m_message,m_directory,m_action,m_name;
    QVariantList m_records,m_files,m_urls;
    QProcess m_process;QTimer m_timeout;
    QByteArray m_output,m_error;
    std::unique_ptr<QSaveFile> m_file;
    int m_index=0;
    qint64 m_total=0,m_done=0,m_received=0;
    bool m_busy=false,m_cancelled=false,m_hasError=false;
};
