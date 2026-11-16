// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef UDPTRANSPORT_H
#define UDPTRANSPORT_H

#include "pointcloudgeometry.h"
#include "routegeometry.h"

#include <QHostAddress>
#include <QElapsedTimer>
#include <QTimer>
#include <QObject>
#include <QUdpSocket>
#include <QVector>
#include <QHash>
#include <QtQml/qqmlregistration.h>

// 地面站点云接收器。
//
// 作用：
// - 监听点云 UDP 端口，按 PointCloudProtocol 解析每个分片。
// - 根据 sequence/chunkIndex 把多个 UDP 包重组为完整点云帧。
// - 把完整帧或实时累计后的点集交给 PointCloudGeometry 显示。
//
// 协作关系：
// - radarsimulator.exe 或未来真实雷达适配器发送点云分片到 listenPort。
// - 本类只负责网络和帧重组，不直接处理 3D 顶点；渲染交给 PointCloudGeometry。
// - Main.qml 绑定统计属性，显示已收帧、丢帧、点数和实时累计状态。
//
// 异常策略：
// - 无法解析的 UDP 包会被丢弃并写入日志。
// - 新帧到来时如果上一帧分片不完整，会丢弃旧帧，避免 UI 显示半帧数据。
class UdpPointCloudReceiver : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(UdpPointCloudReceiver)
    Q_PROPERTY(PointCloudGeometry *geometry READ geometry WRITE setGeometry NOTIFY geometryChanged)
    Q_PROPERTY(int listenPort READ listenPort WRITE setListenPort NOTIFY listenPortChanged)
    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(int packetsReceived READ packetsReceived NOTIFY statsChanged)
    Q_PROPERTY(int framesReceived READ framesReceived NOTIFY statsChanged)
    Q_PROPERTY(int droppedFrames READ droppedFrames NOTIFY statsChanged)
    Q_PROPERTY(int lastPointCount READ lastPointCount NOTIFY statsChanged)
    Q_PROPERTY(int lastSequence READ lastSequence NOTIFY statsChanged)
    Q_PROPERTY(int latestFramePointCount READ latestFramePointCount NOTIFY statsChanged)
    Q_PROPERTY(int displayPointCount READ displayPointCount NOTIFY statsChanged)
    Q_PROPERTY(bool persistentMapping READ persistentMapping WRITE setPersistentMapping NOTIFY accumulationSettingsChanged)
    Q_PROPERTY(bool realtimeAccumulation READ realtimeAccumulation WRITE setRealtimeAccumulation NOTIFY accumulationSettingsChanged)
    Q_PROPERTY(int accumulationFrameWindow READ accumulationFrameWindow WRITE setAccumulationFrameWindow NOTIFY accumulationSettingsChanged)
    Q_PROPERTY(int accumulationPointLimit READ accumulationPointLimit WRITE setAccumulationPointLimit NOTIFY accumulationSettingsChanged)

public:
    // 创建接收器。默认端口 56000，需 active=true 后开始监听。
    explicit UdpPointCloudReceiver(QObject *parent = nullptr);

    // 当前绑定的点云几何体。
    PointCloudGeometry *geometry() const { return m_geometry; }
    // 绑定或替换渲染目标。通常由 QML geometry: pointCloudGeometry 完成。
    void setGeometry(PointCloudGeometry *geometry);

    // 当前监听的 UDP 端口。
    int listenPort() const { return m_listenPort; }
    // 修改监听端口；如果正在监听会自动重新绑定。
    void setListenPort(int port);

    // 是否处于监听状态。
    bool active() const { return m_active; }
    // 启动或停止 UDP 监听。
    void setActive(bool active);

    // 面向 UI 的简短状态文本。
    QString status() const { return m_status; }
    // 累计收到的 UDP 分片包数量。
    int packetsReceived() const { return m_packetsReceived; }
    // 成功重组并显示的点云帧数量。
    int framesReceived() const { return m_framesReceived; }
    // 因分片不完整或序号切换被丢弃的帧数量。
    int droppedFrames() const { return m_droppedFrames; }
    // 最近一次显示到几何体里的点数。
    int lastPointCount() const { return m_lastPointCount; }
    // 最近一次完整点云帧序号。
    int lastSequence() const { return int(m_lastSequence); }
    // 最近一帧原始点数，不含累计裁剪。
    int latestFramePointCount() const { return m_latestFramePointCount; }
    // 当前实际显示点数，实时累计开启时可能大于单帧点数。
    int displayPointCount() const { return m_displayPointCount; }

    // 是否把最近多帧合并显示。默认关闭，用户可按需要开启。
    // 仅世界坐标点云可使用持久体素累计；与机载SLAM及避障地图独立。
    bool persistentMapping() const { return m_persistentMapping; }
    void setPersistentMapping(bool enabled);
    bool realtimeAccumulation() const { return m_realtimeAccumulation; }
    // 打开或关闭实时累计显示；关闭时清理累计缓存。
    void setRealtimeAccumulation(bool enabled);

    // 实时累计最多保留多少帧。
    int accumulationFrameWindow() const { return m_accumulationFrameWindow; }
    // 设置实时累计帧窗口，范围在实现中做保护。
    void setAccumulationFrameWindow(int frameWindow);

    // 实时累计最多显示多少个点。
    int accumulationPointLimit() const { return m_accumulationPointLimit; }
    // 设置累计点数上限，超出时从最老帧开始裁剪。
    void setAccumulationPointLimit(int pointLimit);

    // 主动重启监听。UI 修改端口或用户排查网络问题时调用。
    Q_INVOKABLE void restart();
    // 定位会话原点变化时清空旧图显示及累计缓存，不重启机载数据源。
    Q_INVOKABLE void clearDisplay();

signals:
    void geometryChanged();
    void listenPortChanged();
    void activeChanged();
    void statusChanged();
    void statsChanged();
    void accumulationSettingsChanged();

private:
    // 按当前端口绑定 QUdpSocket，并写入状态/日志。
    void bindSocket();
    // 关闭 QUdpSocket，并清理未完成帧。
    void closeSocket();
    // 读取 socket 中所有待处理 UDP 包。
    void readPendingDatagrams();
    // 清理当前正在重组但还没完整的点云帧。
    void resetPendingFrame();
    // 清理香橙派 LZ4 压缩点云的重组缓存。
    void resetPendingCompressedFrame();
    // 接受一个解析后的分片，按序号放入待重组缓存。
    void acceptChunk(const PointCloudChunk &chunk);
    // 当所有分片都到齐时发布完整帧到 PointCloudGeometry。
    void publishPendingFrameIfComplete();
    // 接受一个香橙派 LZ4 压缩分片，按序号重组压缩帧。
    void acceptCompressedChunk(const RosCompressedPointCloudChunk &chunk);
    // 香橙派压缩帧完整后，执行 LZ4 解压并发布到 PointCloudGeometry。
    void publishPendingCompressedFrameIfComplete();
    // 所有点云协议最终都通过这里进入几何体，统一统计和日志。
    void publishFramePoints(const QVector<PointCloudPoint> &points,
                            quint32 sequence,
                            quint64 timestampUsec,
                            quint32 declaredPointCount,
                            const QString &protocolName);
    // 清空实时累计帧缓存。
    void clearAccumulatedFrames();
    // 根据实时累计配置决定最终显示最新帧还是多帧合并结果。
    QVector<PointCloudPoint> displayPointsForFrame(const QVector<PointCloudPoint> &framePoints);
    // 更新 UI 状态文本并发出 statusChanged。
    void setStatus(const QString &status);

    // UDP 点云 socket，监听模拟器或真实雷达适配器发来的点云分片。
    QUdpSocket m_socket;
    // 被刷新的点云渲染对象，不拥有其生命周期。
    PointCloudGeometry *m_geometry = nullptr;
    int m_listenPort = 56000;
    bool m_active = false;
    QString m_status;

    quint32 m_pendingSequence = 0;
    quint64 m_pendingTimestampUsec = 0;
    quint32 m_pendingTotalPointCount = 0;
    QVector<QVector<PointCloudPoint>> m_pendingChunks;
    QVector<bool> m_chunkReceived;
    int m_receivedChunkCount = 0;
    bool m_hasPendingFrame = false;

    quint32 m_pendingCompressedSequence = 0;
    quint32 m_pendingCompressedUncompressedBytes = 0;
    quint32 m_pendingCompressedBytes = 0;
    QVector<QByteArray> m_pendingCompressedChunks;
    QVector<bool> m_compressedChunkReceived;
    int m_receivedCompressedChunkCount = 0;
    bool m_hasPendingCompressedFrame = false;

    int m_packetsReceived = 0;
    int m_invalidDatagrams = 0;
    int m_framesReceived = 0;
    int m_droppedFrames = 0;
    int m_lastPointCount = 0;
    quint32 m_lastSequence = 0;

    bool m_realtimeAccumulation = false;
    bool m_persistentMapping = false;
    QHash<quint64,PointCloudPoint> m_mapVoxels;
    int m_accumulationFrameWindow = 5;
    int m_accumulationPointLimit = 220000;
    QVector<QVector<PointCloudPoint>> m_accumulatedFrames;
    int m_accumulatedPointCount = 0;
    int m_latestFramePointCount = 0;
    int m_displayPointCount = 0;
};

// 香橙派 ROS 位姿接收器。
//
// 作用：
// - 监听 something1017/udp_uav_tx_node 发来的 POS1 位姿 UDP 包。
// - 保存最近一次 px/py/pz 和四元数，并暴露给 QML 在 3D 点云场景中显示飞行器/雷达实时位姿。
// - 同时生成 UI 状态文本，方便现场确认定位链路是否在线。
//
// 协作关系：
// - 与 UdpPointCloudReceiver 独立监听端口，默认 POS1 端口为 9001。
// - Main.qml 使用 positionMeters 和 yawDegrees 绘制位姿 marker；航线规划仍可在没有 POS1 时独立工作。
class UdpPoseReceiver : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(UdpPoseReceiver)
    Q_PROPERTY(int listenPort READ listenPort WRITE setListenPort NOTIFY listenPortChanged)
    Q_PROPERTY(bool active READ active WRITE setActive NOTIFY activeChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(QString poseText READ poseText NOTIFY poseChanged)
    Q_PROPERTY(bool hasPose READ hasPose NOTIFY poseChanged)
    Q_PROPERTY(QVector3D positionMeters READ positionMeters NOTIFY poseChanged)
    Q_PROPERTY(float qx READ qx NOTIFY poseChanged)
    Q_PROPERTY(float qy READ qy NOTIFY poseChanged)
    Q_PROPERTY(float qz READ qz NOTIFY poseChanged)
    Q_PROPERTY(float qw READ qw NOTIFY poseChanged)
    Q_PROPERTY(float yawDegrees READ yawDegrees NOTIFY poseChanged)
    Q_PROPERTY(int packetsReceived READ packetsReceived NOTIFY statsChanged)
    Q_PROPERTY(int lastSequence READ lastSequence NOTIFY statsChanged)

public:
    // 创建位姿接收器。默认端口 9001，active=true 后开始监听。
    explicit UdpPoseReceiver(QObject *parent = nullptr);

    int listenPort() const { return m_listenPort; }
    void setListenPort(int port);

    bool active() const { return m_active; }
    void setActive(bool active);

    QString status() const { return m_status; }
    QString poseText() const { return m_poseText; }
    bool hasPose() const { return m_hasPose; }
    QVector3D positionMeters() const { return m_positionMeters; }
    float qx() const { return m_qx; }
    float qy() const { return m_qy; }
    float qz() const { return m_qz; }
    float qw() const { return m_qw; }
    float yawDegrees() const { return m_yawDegrees; }
    int packetsReceived() const { return m_packetsReceived; }
    int lastSequence() const { return int(m_lastSequence); }

    Q_INVOKABLE void restart();

signals:
    void listenPortChanged();
    void activeChanged();
    void statusChanged();
    void poseChanged();
    void statsChanged();

private:
    void bindSocket();
    void closeSocket();
    void readPendingDatagrams();
    void setStatus(const QString &status);
    void setPoseText(const QString &poseText);

    QUdpSocket m_socket;
    int m_listenPort = 9001;
    bool m_active = false;
    QString m_status;
    QString m_poseText;
    int m_packetsReceived = 0;
    int m_invalidDatagrams = 0;
    quint32 m_lastSequence = 0;
    bool m_hasPose = false;
    QVector3D m_positionMeters;
    float m_qx = 0.0f;
    float m_qy = 0.0f;
    float m_qz = 0.0f;
    float m_qw = 1.0f;
    float m_yawDegrees = 0.0f;
    QElapsedTimer m_poseReceiveClock;
    QTimer m_poseWatchdog;
    double m_lastSourceStamp = 0.0;
};

// 航线发送器。
//
// 作用：
// - 把 RouteGeometry 中的完整航点列表编码成 PointCloudProtocol 的航线包。
// - 用户点击“上传航线”时发送一次，不在拖拽过程中自动发送。
//
// 协作关系：
// - Main.qml 负责按钮交互，并把 RouteGeometry 传给 sendRoute()。
// - radarsimulator.exe 或未来飞控/机器人端监听 targetPort，解析完整航线快照。
// - 航线包每次都包含全量航点，接收端不需要保存增量状态。
class RouteUdpSender : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(RouteUdpSender)
    Q_PROPERTY(QString targetHost READ targetHost WRITE setTargetHost NOTIFY targetChanged)
    Q_PROPERTY(int targetPort READ targetPort WRITE setTargetPort NOTIFY targetChanged)
    Q_PROPERTY(QString uploadProtocol READ uploadProtocol WRITE setUploadProtocol NOTIFY uploadProtocolChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(int routesSent READ routesSent NOTIFY routesSentChanged)

public:
    // 创建发送器。默认目标为 127.0.0.1:56001。
    explicit RouteUdpSender(QObject *parent = nullptr);

    // 当前目标主机文本。
    QString targetHost() const { return m_targetHost.toString(); }
    // 设置目标主机。解析失败时回退到本机并记录日志。
    void setTargetHost(const QString &host);

    // 当前目标 UDP 端口。
    int targetPort() const { return m_targetPort; }
    // 设置目标 UDP 端口。
    void setTargetPort(int port);

    // 当前航线上传协议：rte1 表示本 demo 完整航线，gol1 表示香橙派 ROS 单目标点协议。
    QString uploadProtocol() const;
    // 设置航线上传协议。UI 可在本机模拟器和香橙派 ROS 节点之间切换。
    void setUploadProtocol(const QString &protocol);

    // 面向 UI 的发送状态。
    QString status() const { return m_status; }
    // 成功发送的航线次数。
    int routesSent() const { return m_routesSent; }

    // 从 RouteGeometry 读取完整航点并发送，按钮“上传航线”使用此接口。
    Q_INVOKABLE bool sendRoute(RouteGeometry *route);
    // 从 QML QVariantList 直接发送航线，预留给脚本化/导入任务使用。
    Q_INVOKABLE bool sendWaypoints(const QVariantList &waypoints);
    // 只发送一个航点。用于机载端只接收单目标点的场景，不影响完整航线上传。
    Q_INVOKABLE bool sendSingleWaypoint(const QVector3D &waypoint);

signals:
    void targetChanged();
    void uploadProtocolChanged();
    void statusChanged();
    void routesSentChanged();

private:
    enum class UploadProtocol
    {
        Rte1FullRoute,
        Gol1Waypoints
    };

    // 实际编码并发送航点数组的内部入口。
    bool sendWaypointVector(const QVector<QVector3D> &waypoints);
    // 按 RTE1 发送完整航线快照，供本项目 radarsimulator 使用。
    bool sendRte1Route(const QVector<QVector3D> &waypoints);
    // 按 GOL1 把航线拆成多个单目标点，供 something1017/udp_uav_tx 使用。
    bool sendGol1Waypoints(const QVector<QVector3D> &waypoints);
    // 更新 UI 状态文本并发出 statusChanged。
    void setStatus(const QString &status);

    // UDP 发送 socket。
    QUdpSocket m_socket;
    // 航线接收端地址和端口。
    QHostAddress m_targetHost = QHostAddress::LocalHost;
    int m_targetPort = 56001;
    UploadProtocol m_uploadProtocol = UploadProtocol::Rte1FullRoute;
    // 航线包序号，每次成功发送后递增。
    quint32 m_sequence = 0;
    QString m_status;
    int m_routesSent = 0;
};

#endif // UDPTRANSPORT_H
