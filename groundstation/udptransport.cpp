// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "udptransport.h"

#include "pointcloudprotocol.h"

#include <QDateTime>
#include <QDebug>
#include <QNetworkDatagram>
#include <QtMath>

#include <algorithm>
#include <cmath>

UdpPointCloudReceiver::UdpPointCloudReceiver(QObject *parent)
    : QObject(parent)
{
    connect(&m_socket, &QUdpSocket::readyRead, this, &UdpPointCloudReceiver::readPendingDatagrams);
    setStatus(tr("未启动"));
}

void UdpPointCloudReceiver::setGeometry(PointCloudGeometry *geometry)
{
    if (m_geometry == geometry)
        return;

    m_geometry = geometry;
    qInfo().noquote() << tr("点云接收器绑定渲染几何：0x%1").arg(quintptr(m_geometry), 0, 16);
    emit geometryChanged();
}

void UdpPointCloudReceiver::setListenPort(int port)
{
    const int safePort = std::clamp(port, 1, 65535);
    if (m_listenPort == safePort)
        return;

    m_listenPort = safePort;
    qInfo().noquote() << tr("点云监听端口变更：%1").arg(m_listenPort);
    emit listenPortChanged();
    if (m_active)
        restart();
}

void UdpPointCloudReceiver::setActive(bool active)
{
    if (m_active == active)
        return;

    m_active = active;
    qInfo().noquote() << tr("点云 UDP 监听状态：%1").arg(m_active ? tr("启动") : tr("停止"));
    emit activeChanged();
    if (m_active) {
        bindSocket();
    } else {
        closeSocket();
        clearAccumulatedFrames();
    }
}

void UdpPointCloudReceiver::setRealtimeAccumulation(bool enabled)
{
    if (m_realtimeAccumulation == enabled)
        return;

    m_realtimeAccumulation = enabled;
    clearAccumulatedFrames();
    qInfo().noquote() << tr("实时累计显示：%1").arg(enabled ? tr("开启") : tr("关闭"));
    emit accumulationSettingsChanged();
}

void UdpPointCloudReceiver::setAccumulationFrameWindow(int frameWindow)
{
    const int safeWindow = std::clamp(frameWindow, 1, 30);
    if (m_accumulationFrameWindow == safeWindow)
        return;

    m_accumulationFrameWindow = safeWindow;
    clearAccumulatedFrames();
    qInfo().noquote() << tr("实时累计帧数上限：%1").arg(m_accumulationFrameWindow);
    emit accumulationSettingsChanged();
}

void UdpPointCloudReceiver::setAccumulationPointLimit(int pointLimit)
{
    const int safeLimit = std::clamp(pointLimit, 1000, 2000000);
    if (m_accumulationPointLimit == safeLimit)
        return;

    m_accumulationPointLimit = safeLimit;
    clearAccumulatedFrames();
    qInfo().noquote() << tr("实时累计点数上限：%1").arg(m_accumulationPointLimit);
    emit accumulationSettingsChanged();
}

void UdpPointCloudReceiver::clearDisplay()
{
    resetPendingFrame();resetPendingCompressedFrame();clearAccumulatedFrames();
    m_lastPointCount=0;m_latestFramePointCount=0;m_displayPointCount=0;
    if(m_geometry)m_geometry->setPoints({},0,0);
    emit statsChanged();
}

void UdpPointCloudReceiver::restart()
{
    closeSocket();
    if (m_active)
        bindSocket();
}

void UdpPointCloudReceiver::bindSocket()
{
    closeSocket();
    resetPendingFrame();
    clearAccumulatedFrames();

    const bool ok = m_socket.bind(QHostAddress::AnyIPv4,
                                  quint16(m_listenPort),
                                  QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    if (ok) {
        qInfo().noquote() << tr("点云 UDP 监听已绑定：0.0.0.0:%1").arg(m_listenPort);
        setStatus(tr("正在监听 UDP %1").arg(m_listenPort));
    } else {
        qCritical().noquote() << tr("点云 UDP 端口绑定失败：%1，原因：%2")
                                     .arg(m_listenPort)
                                     .arg(m_socket.errorString());
        setStatus(tr("端口 %1 绑定失败：%2").arg(m_listenPort).arg(m_socket.errorString()));
    }
}

void UdpPointCloudReceiver::closeSocket()
{
    if (m_socket.state() != QAbstractSocket::UnconnectedState) {
        qInfo().noquote() << tr("关闭点云 UDP 监听端口：%1").arg(m_listenPort);
        m_socket.close();
    }
}

void UdpPointCloudReceiver::readPendingDatagrams()
{
    while (m_socket.hasPendingDatagrams()) {
        const QNetworkDatagram datagram = m_socket.receiveDatagram();
        PointCloudChunk chunk;
        if (PointCloudProtocol::decodePointCloudChunk(datagram.data(), &chunk)) {
            ++m_packetsReceived;
            acceptChunk(chunk);
            continue;
        }

        RosCompressedPointCloudChunk compressedChunk;
        if (PointCloudProtocol::decodeRosCompressedPointCloudChunk(datagram.data(), &compressedChunk)) {
            ++m_packetsReceived;
            acceptCompressedChunk(compressedChunk);
            continue;
        }

        {
            ++m_invalidDatagrams;
            if (m_invalidDatagrams <= 5 || m_invalidDatagrams % 100 == 0) {
                qWarning().noquote() << tr("收到无法解析的点云 UDP 包，累计 %1 个，当前包大小 %2 字节，来源 %3:%4")
                                            .arg(m_invalidDatagrams)
                                            .arg(datagram.data().size())
                                            .arg(datagram.senderAddress().toString())
                                            .arg(datagram.senderPort());
            }
        }
    }
    emit statsChanged();
}

void UdpPointCloudReceiver::resetPendingFrame()
{
    m_pendingSequence = 0;
    m_pendingTimestampUsec = 0;
    m_pendingTotalPointCount = 0;
    m_pendingChunks.clear();
    m_chunkReceived.clear();
    m_receivedChunkCount = 0;
    m_hasPendingFrame = false;
    resetPendingCompressedFrame();
}

void UdpPointCloudReceiver::resetPendingCompressedFrame()
{
    m_pendingCompressedSequence = 0;
    m_pendingCompressedUncompressedBytes = 0;
    m_pendingCompressedBytes = 0;
    m_pendingCompressedChunks.clear();
    m_compressedChunkReceived.clear();
    m_receivedCompressedChunkCount = 0;
    m_hasPendingCompressedFrame = false;
}

void UdpPointCloudReceiver::acceptChunk(const PointCloudChunk &chunk)
{
    const bool startsNewFrame = !m_hasPendingFrame
        || chunk.sequence != m_pendingSequence
        || chunk.chunkCount != m_pendingChunks.size();

    if (startsNewFrame) {
        if (m_hasPendingFrame && m_receivedChunkCount != m_pendingChunks.size()) {
            ++m_droppedFrames;
            qWarning().noquote() << tr("点云帧 %1 分片不完整，已丢弃；收到 %2/%3 个分片")
                                        .arg(m_pendingSequence)
                                        .arg(m_receivedChunkCount)
                                        .arg(m_pendingChunks.size());
        }

        m_pendingSequence = chunk.sequence;
        m_pendingTimestampUsec = chunk.timestampUsec;
        m_pendingTotalPointCount = chunk.totalPointCount;
        m_pendingChunks = QVector<QVector<PointCloudPoint>>(chunk.chunkCount);
        m_chunkReceived = QVector<bool>(chunk.chunkCount, false);
        m_receivedChunkCount = 0;
        m_hasPendingFrame = true;
    }

    if (chunk.chunkIndex >= m_pendingChunks.size()) {
        qWarning().noquote() << tr("点云帧 %1 的分片下标越界：%2/%3")
                                    .arg(chunk.sequence)
                                    .arg(chunk.chunkIndex)
                                    .arg(chunk.chunkCount);
        return;
    }

    if (m_chunkReceived.at(chunk.chunkIndex)) {
        qWarning().noquote() << tr("点云帧 %1 收到重复分片：%2")
                                    .arg(chunk.sequence)
                                    .arg(chunk.chunkIndex);
        return;
    }

    m_pendingChunks[chunk.chunkIndex] = chunk.points;
    m_chunkReceived[chunk.chunkIndex] = true;
    ++m_receivedChunkCount;
    publishPendingFrameIfComplete();
}

void UdpPointCloudReceiver::publishPendingFrameIfComplete()
{
    if (!m_hasPendingFrame || m_receivedChunkCount != m_pendingChunks.size())
        return;

    QVector<PointCloudPoint> points;
    points.reserve(int(m_pendingTotalPointCount));
    for (const QVector<PointCloudPoint> &chunkPoints : std::as_const(m_pendingChunks))
        points += chunkPoints;

    publishFramePoints(points,
                       m_pendingSequence,
                       m_pendingTimestampUsec,
                       m_pendingTotalPointCount,
                       tr("RTE/PCD1 未压缩"));
    resetPendingFrame();
}

void UdpPointCloudReceiver::acceptCompressedChunk(const RosCompressedPointCloudChunk &chunk)
{
    const bool startsNewFrame = !m_hasPendingCompressedFrame
        || chunk.sequence != m_pendingCompressedSequence
        || chunk.chunkCount != m_pendingCompressedChunks.size();

    if (startsNewFrame) {
        if (m_hasPendingCompressedFrame
            && m_receivedCompressedChunkCount != m_pendingCompressedChunks.size()) {
            ++m_droppedFrames;
            qWarning().noquote() << tr("香橙派点云帧 %1 分片不完整，已丢弃；收到 %2/%3 个分片")
                                        .arg(m_pendingCompressedSequence)
                                        .arg(m_receivedCompressedChunkCount)
                                        .arg(m_pendingCompressedChunks.size());
        }

        m_pendingCompressedSequence = chunk.sequence;
        m_pendingCompressedUncompressedBytes = chunk.uncompressedBytes;
        m_pendingCompressedBytes = chunk.compressedBytes;
        m_pendingCompressedChunks = QVector<QByteArray>(chunk.chunkCount);
        m_compressedChunkReceived = QVector<bool>(chunk.chunkCount, false);
        m_receivedCompressedChunkCount = 0;
        m_hasPendingCompressedFrame = true;
    }

    if (chunk.uncompressedBytes != m_pendingCompressedUncompressedBytes
        || chunk.compressedBytes != m_pendingCompressedBytes
        || chunk.chunkIndex >= m_pendingCompressedChunks.size()) {
        ++m_droppedFrames;
        qWarning().noquote() << tr("香橙派点云帧 %1 分片头不一致，已丢弃当前帧")
                                    .arg(chunk.sequence);
        resetPendingCompressedFrame();
        return;
    }

    if (m_compressedChunkReceived.at(chunk.chunkIndex)) {
        qWarning().noquote() << tr("香橙派点云帧 %1 收到重复分片：%2")
                                    .arg(chunk.sequence)
                                    .arg(chunk.chunkIndex);
        return;
    }

    m_pendingCompressedChunks[chunk.chunkIndex] = chunk.payload;
    m_compressedChunkReceived[chunk.chunkIndex] = true;
    ++m_receivedCompressedChunkCount;
    publishPendingCompressedFrameIfComplete();
}

void UdpPointCloudReceiver::publishPendingCompressedFrameIfComplete()
{
    if (!m_hasPendingCompressedFrame
        || m_receivedCompressedChunkCount != m_pendingCompressedChunks.size()) {
        return;
    }

    QByteArray compressedFrame;
    compressedFrame.reserve(int(m_pendingCompressedBytes));
    for (const QByteArray &payload : std::as_const(m_pendingCompressedChunks))
        compressedFrame.append(payload);

    if (compressedFrame.size() != int(m_pendingCompressedBytes)) {
        ++m_droppedFrames;
        qWarning().noquote() << tr("香橙派点云帧 %1 压缩长度不一致：协议 %2 字节，实际 %3 字节")
                                    .arg(m_pendingCompressedSequence)
                                    .arg(m_pendingCompressedBytes)
                                    .arg(compressedFrame.size());
        resetPendingCompressedFrame();
        return;
    }

    QVector<PointCloudPoint> points;
    if (!PointCloudProtocol::decodeRosCompressedPointCloudFrame(compressedFrame,
                                                                m_pendingCompressedUncompressedBytes,
                                                                &points)) {
        ++m_droppedFrames;
        qWarning().noquote() << tr("香橙派点云帧 %1 LZ4 解压失败，压缩 %2 字节，原始 %3 字节")
                                    .arg(m_pendingCompressedSequence)
                                    .arg(m_pendingCompressedBytes)
                                    .arg(m_pendingCompressedUncompressedBytes);
        resetPendingCompressedFrame();
        return;
    }

    const quint32 declaredPointCount = m_pendingCompressedUncompressedBytes / (3 * sizeof(float));
    publishFramePoints(points,
                       m_pendingCompressedSequence,
                       quint64(QDateTime::currentMSecsSinceEpoch()) * 1000,
                       declaredPointCount,
                       tr("香橙派 PCD1/LZ4"));
    resetPendingCompressedFrame();
}

void UdpPointCloudReceiver::publishFramePoints(const QVector<PointCloudPoint> &points,
                                               quint32 sequence,
                                               quint64 timestampUsec,
                                               quint32 declaredPointCount,
                                               const QString &protocolName)
{
    if (m_geometry) {
        m_geometry->setPoints(displayPointsForFrame(points), sequence, timestampUsec);
    } else {
        qWarning() << "点云帧已接收但渲染 geometry 未绑定";
    }

    ++m_framesReceived;
    m_latestFramePointCount = points.size();
    m_displayPointCount = m_geometry ? m_geometry->pointCount() : m_latestFramePointCount;
    m_lastPointCount = m_displayPointCount;
    m_lastSequence = sequence;
    setStatus(tr("已接收 %1 帧 %2，单帧 %3 点，显示 %4 点")
                  .arg(protocolName)
                  .arg(m_lastSequence)
                  .arg(m_latestFramePointCount)
                  .arg(m_displayPointCount));
    if (m_latestFramePointCount != int(declaredPointCount)) {
        qWarning().noquote() << tr("%1 点云帧 %2 点数不一致：协议声明 %3 点，实际重组 %4 点")
                                    .arg(protocolName)
                                    .arg(m_lastSequence)
                                    .arg(declaredPointCount)
                                    .arg(m_latestFramePointCount);
    }
    if (m_framesReceived <= 3 || m_framesReceived % 30 == 0) {
        qInfo().noquote() << tr("点云帧完成：协议 %1，序号 %2，单帧 %3 点，显示 %4 点，累计收到 %5 帧，丢帧 %6")
                              .arg(protocolName)
                              .arg(m_lastSequence)
                              .arg(m_latestFramePointCount)
                              .arg(m_displayPointCount)
                              .arg(m_framesReceived)
                              .arg(m_droppedFrames);
    }
}

void UdpPointCloudReceiver::setPersistentMapping(bool enabled)
{
    if (m_persistentMapping == enabled) return;
    m_persistentMapping = enabled;
    clearAccumulatedFrames();
    qInfo() << "世界坐标累计建图显示" << enabled;
    emit accumulationSettingsChanged();
}

void UdpPointCloudReceiver::clearAccumulatedFrames()
{
    m_mapVoxels.clear();
    m_accumulatedFrames.clear();
    m_accumulatedPointCount = 0;
}

QVector<PointCloudPoint> UdpPointCloudReceiver::displayPointsForFrame(const QVector<PointCloudPoint> &framePoints)
{
    if (!m_realtimeAccumulation)
        return framePoints;

    if (m_persistentMapping) {
        // 10厘米体素保留已扫描区域；固定点数上限，防止内存无限增长。
        constexpr qint64 bias = 1 << 20;
        for (const auto &point : framePoints) {
            const auto p = point.position;
            if (!std::isfinite(p.x()) || !std::isfinite(p.y()) || !std::isfinite(p.z())) continue;
            const double x=std::floor(double(p.x())*10), y=std::floor(double(p.y())*10), z=std::floor(double(p.z())*10);
            if (x < -bias || x >= bias || y < -bias || y >= bias || z < -bias || z >= bias) continue;
            const quint64 key=quint64(qint64(x)+bias) | (quint64(qint64(y)+bias)<<21) | (quint64(qint64(z)+bias)<<42);
            if (m_mapVoxels.contains(key) || m_mapVoxels.size()<m_accumulationPointLimit)
                m_mapVoxels.insert(key,point);
        }
        QVector<PointCloudPoint> result;
        result.reserve(m_mapVoxels.size());
        for (const auto &point : std::as_const(m_mapVoxels)) result.append(point);
        return result;
    }
    m_accumulatedFrames.append(framePoints);
    m_accumulatedPointCount += framePoints.size();

    // 实时雷达是按帧流动的；地面站为了呈现建图观感，只保留最近几帧，
    // 避免无限累计导致 GPU 顶点缓冲和选点搜索越来越慢。
    while (!m_accumulatedFrames.isEmpty()
           && (m_accumulatedFrames.size() > m_accumulationFrameWindow
               || m_accumulatedPointCount > m_accumulationPointLimit)) {
        qInfo().noquote() << tr("实时累计裁剪：帧数 %1，点数 %2，上限帧 %3，上限点 %4")
                              .arg(m_accumulatedFrames.size())
                              .arg(m_accumulatedPointCount)
                              .arg(m_accumulationFrameWindow)
                              .arg(m_accumulationPointLimit);
        m_accumulatedPointCount -= m_accumulatedFrames.constFirst().size();
        m_accumulatedFrames.removeFirst();
    }

    QVector<PointCloudPoint> displayPoints;
    displayPoints.reserve(m_accumulatedPointCount);
    for (const QVector<PointCloudPoint> &frame : std::as_const(m_accumulatedFrames))
        displayPoints += frame;
    return displayPoints;
}

void UdpPointCloudReceiver::setStatus(const QString &status)
{
    if (m_status == status)
        return;

    m_status = status;
    emit statusChanged();
}

UdpPoseReceiver::UdpPoseReceiver(QObject *parent)
    : QObject(parent)
{
    connect(&m_socket, &QUdpSocket::readyRead, this, &UdpPoseReceiver::readPendingDatagrams);
    setStatus(tr("位姿未启动"));
    setPoseText(tr("暂无位姿"));
    connect(&m_poseWatchdog, &QTimer::timeout, this, [this]() {
        if (m_hasPose && m_poseReceiveClock.isValid() && m_poseReceiveClock.elapsed() > 1500) {
            m_hasPose = false;
            setStatus(tr("位姿断流，已隐藏过期位置/朝向"));
            setPoseText(tr("末次位姿已过期"));
            emit poseChanged();
        }
    });
    m_poseWatchdog.start(250);
}

void UdpPoseReceiver::setListenPort(int port)
{
    const int safePort = std::clamp(port, 1, 65535);
    if (m_listenPort == safePort)
        return;

    m_listenPort = safePort;
    qInfo().noquote() << tr("位姿监听端口变更：%1").arg(m_listenPort);
    emit listenPortChanged();
    if (m_active)
        restart();
}

void UdpPoseReceiver::setActive(bool active)
{
    if (m_active == active)
        return;

    m_active = active;
    qInfo().noquote() << tr("位姿 UDP 监听状态：%1").arg(m_active ? tr("启动") : tr("停止"));
    emit activeChanged();
    if (m_active)
        bindSocket();
    else
        closeSocket();
}

void UdpPoseReceiver::restart()
{
    closeSocket();
    if (m_active)
        bindSocket();
}

void UdpPoseReceiver::bindSocket()
{
    closeSocket();
    const bool ok = m_socket.bind(QHostAddress::AnyIPv4,
                                  quint16(m_listenPort),
                                  QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    if (ok) {
        qInfo().noquote() << tr("位姿 UDP 监听已绑定：0.0.0.0:%1").arg(m_listenPort);
        setStatus(tr("正在监听 POS1 %1").arg(m_listenPort));
    } else {
        qCritical().noquote() << tr("位姿 UDP 端口绑定失败：%1，原因：%2")
                                     .arg(m_listenPort)
                                     .arg(m_socket.errorString());
        setStatus(tr("位姿端口 %1 绑定失败：%2").arg(m_listenPort).arg(m_socket.errorString()));
    }
}

void UdpPoseReceiver::closeSocket()
{
    if (m_hasPose) {
        m_hasPose = false;
        emit poseChanged();
    }
    m_poseReceiveClock.invalidate();
    m_lastSourceStamp = 0.0;
    if (m_socket.state() != QAbstractSocket::UnconnectedState) {
        qInfo().noquote() << tr("关闭位姿 UDP 监听端口：%1").arg(m_listenPort);
        m_socket.close();
    }
}

void UdpPoseReceiver::readPendingDatagrams()
{
    while (m_socket.hasPendingDatagrams()) {
        const QNetworkDatagram datagram = m_socket.receiveDatagram();
        RosPosePacket pose;
        if (!PointCloudProtocol::decodeRosPosePacket(datagram.data(), &pose)) {
            ++m_invalidDatagrams;
            if (m_invalidDatagrams <= 5 || m_invalidDatagrams % 100 == 0) {
                qWarning().noquote() << tr("收到无法解析的 POS1 UDP 包，累计 %1 个，当前包大小 %2 字节，来源 %3:%4")
                                            .arg(m_invalidDatagrams)
                                            .arg(datagram.data().size())
                                            .arg(datagram.senderAddress().toString())
                                            .arg(datagram.senderPort());
            }
            continue;
        }

        // 不以乱序/重复旧位姿刷新标记；断流使用本机单调时钟。
        if (m_lastSourceStamp > 0.0 && pose.stampSeconds <= m_lastSourceStamp)
            continue;
        m_lastSourceStamp = pose.stampSeconds;
        m_poseReceiveClock.restart();
        ++m_packetsReceived;
        m_lastSequence = pose.sequence;
        m_hasPose = true;
        m_positionMeters = pose.position;
        m_qx = pose.qx;
        m_qy = pose.qy;
        m_qz = pose.qz;
        m_qw = pose.qw;
        // something1017 里的 POS1 四元数来自 ROS PoseStamped，默认 map 坐标系下 Z 轴为 yaw。
        // QML marker 先使用 yaw 显示水平航向；完整四元数仍通过属性和日志保留，便于后续接入更精细模型。
        const double sinyCosp = 2.0 * (double(m_qw) * double(m_qz) + double(m_qx) * double(m_qy));
        const double cosyCosp = 1.0 - 2.0 * (double(m_qy) * double(m_qy) + double(m_qz) * double(m_qz));
        m_yawDegrees = float(qRadiansToDegrees(std::atan2(sinyCosp, cosyCosp)));
        setPoseText(tr("POS1 #%1  X:%2  Y:%3  Z:%4 m  航向:%5°")
                        .arg(pose.sequence)
                        .arg(pose.position.x(), 0, 'f', 2)
                        .arg(pose.position.y(), 0, 'f', 2)
                        .arg(pose.position.z(), 0, 'f', 3)
                        .arg(m_yawDegrees, 0, 'f', 1));
        setStatus(tr("已接收 POS1 %1，累计 %2 包").arg(pose.sequence).arg(m_packetsReceived));
        if (m_packetsReceived <= 3 || m_packetsReceived % 100 == 0) {
            qInfo().noquote() << tr("POS1 位姿包：序号 %1，位置 [%2, %3, %4]，四元数 [%5, %6, %7, %8]")
                                  .arg(pose.sequence)
                                  .arg(pose.position.x(), 0, 'f', 3)
                                  .arg(pose.position.y(), 0, 'f', 3)
                                  .arg(pose.position.z(), 0, 'f', 3)
                                  .arg(pose.qx, 0, 'f', 3)
                                  .arg(pose.qy, 0, 'f', 3)
                                  .arg(pose.qz, 0, 'f', 3)
                                  .arg(pose.qw, 0, 'f', 3);
        }
    }
    emit statsChanged();
}

void UdpPoseReceiver::setStatus(const QString &status)
{
    if (m_status == status)
        return;

    m_status = status;
    emit statusChanged();
}

void UdpPoseReceiver::setPoseText(const QString &poseText)
{
    if (m_poseText == poseText)
        return;

    m_poseText = poseText;
    emit poseChanged();
}

RouteUdpSender::RouteUdpSender(QObject *parent)
    : QObject(parent)
{
    setStatus(tr("航线发送目标 127.0.0.1:%1").arg(m_targetPort));
}

void RouteUdpSender::setTargetHost(const QString &host)
{
    QHostAddress address(host);
    if (address.isNull()) {
        qWarning().noquote() << tr("航线目标地址解析失败，回退到 127.0.0.1：%1").arg(host);
        address = QHostAddress::LocalHost;
    }

    if (m_targetHost == address)
        return;

    m_targetHost = address;
    qInfo().noquote() << tr("航线目标地址变更：%1:%2").arg(m_targetHost.toString()).arg(m_targetPort);
    emit targetChanged();
    setStatus(tr("航线发送目标 %1:%2").arg(m_targetHost.toString()).arg(m_targetPort));
}

void RouteUdpSender::setTargetPort(int port)
{
    const int safePort = std::clamp(port, 1, 65535);
    if (m_targetPort == safePort)
        return;

    m_targetPort = safePort;
    qInfo().noquote() << tr("航线目标端口变更：%1:%2").arg(m_targetHost.toString()).arg(m_targetPort);
    emit targetChanged();
    setStatus(tr("航线发送目标 %1:%2").arg(m_targetHost.toString()).arg(m_targetPort));
}

QString RouteUdpSender::uploadProtocol() const
{
    return m_uploadProtocol == UploadProtocol::Gol1Waypoints
        ? QStringLiteral("gol1")
        : QStringLiteral("rte1");
}

void RouteUdpSender::setUploadProtocol(const QString &protocol)
{
    const QString normalized = protocol.trimmed().toLower();
    const UploadProtocol nextProtocol =
        (normalized == QStringLiteral("gol1")
         || normalized == QStringLiteral("orangepi")
         || normalized == QStringLiteral("orange-pi"))
        ? UploadProtocol::Gol1Waypoints
        : UploadProtocol::Rte1FullRoute;

    if (m_uploadProtocol == nextProtocol)
        return;

    m_uploadProtocol = nextProtocol;
    qInfo().noquote() << tr("航线上传协议变更：%1").arg(uploadProtocol());
    emit uploadProtocolChanged();
    setStatus(m_uploadProtocol == UploadProtocol::Gol1Waypoints
                  ? tr("航线发送目标 %1:%2，协议 GOL1 逐点")
                        .arg(m_targetHost.toString())
                        .arg(m_targetPort)
                  : tr("航线发送目标 %1:%2，协议 RTE1 完整航线")
                        .arg(m_targetHost.toString())
                        .arg(m_targetPort));
}

bool RouteUdpSender::sendRoute(RouteGeometry *route)
{
    if (!route) {
        qWarning() << "航线发送失败：RouteGeometry 未绑定";
        return false;
    }
    return sendWaypointVector(route->waypointVector());
}

bool RouteUdpSender::sendWaypoints(const QVariantList &waypoints)
{
    QVector<QVector3D> parsed;
    parsed.reserve(waypoints.size());
    for (const QVariant &waypoint : waypoints) {
        if (waypoint.canConvert<QVector3D>())
            parsed.append(waypoint.value<QVector3D>());
    }
    qInfo().noquote() << tr("准备从 QML 航点列表发送航线，航点数：%1").arg(parsed.size());
    return sendWaypointVector(parsed);
}

bool RouteUdpSender::sendSingleWaypoint(const QVector3D &waypoint)
{
    QVector<QVector3D> onePoint;
    onePoint.append(waypoint);

    if (m_uploadProtocol == UploadProtocol::Gol1Waypoints) {
        qInfo().noquote() << tr("准备按 GOL1 发送单个航点：X=%1 Y=%2 Z=%3")
                              .arg(waypoint.x(), 0, 'f', 2)
                              .arg(waypoint.y(), 0, 'f', 2)
                              .arg(waypoint.z(), 0, 'f', 2);
        return sendGol1Waypoints(onePoint);
    }

    qInfo().noquote() << tr("准备按 RTE1 发送单个航点快照：X=%1 Y=%2 Z=%3")
                          .arg(waypoint.x(), 0, 'f', 2)
                          .arg(waypoint.y(), 0, 'f', 2)
                          .arg(waypoint.z(), 0, 'f', 2);
    return sendRte1Route(onePoint);
}

bool RouteUdpSender::sendWaypointVector(const QVector<QVector3D> &waypoints)
{
    if (waypoints.size() < 2) {
        qWarning().noquote() << tr("航线发送被拒绝：航点数不足，当前 %1 个").arg(waypoints.size());
        setStatus(tr("航线至少需要两个点"));
        return false;
    }

    if (m_uploadProtocol == UploadProtocol::Gol1Waypoints)
        return sendGol1Waypoints(waypoints);

    return sendRte1Route(waypoints);
}

bool RouteUdpSender::sendRte1Route(const QVector<QVector3D> &waypoints)
{
    const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;
    const QByteArray datagram = PointCloudProtocol::encodeRoute(++m_sequence, timestampUsec, waypoints);
    const qint64 written = m_socket.writeDatagram(datagram, m_targetHost, quint16(m_targetPort));
    if (written != datagram.size()) {
        qCritical().noquote() << tr("航线 UDP 发送失败：序号 %1，目标 %2:%3，期望 %4 字节，实际 %5 字节，错误：%6")
                                    .arg(m_sequence)
                                    .arg(m_targetHost.toString())
                                    .arg(m_targetPort)
                                    .arg(datagram.size())
                                    .arg(written)
                                    .arg(m_socket.errorString());
        setStatus(tr("航线发送失败：%1").arg(m_socket.errorString()));
        return false;
    }

    ++m_routesSent;
    qInfo().noquote() << tr("航线 UDP 已发送：序号 %1，目标 %2:%3，航点 %4 个，字节 %5")
                            .arg(m_sequence)
                            .arg(m_targetHost.toString())
                            .arg(m_targetPort)
                            .arg(waypoints.size())
                            .arg(written);
    emit routesSentChanged();
    setStatus(tr("已发送航线 %1，%2 个点").arg(m_sequence).arg(waypoints.size()));
    return true;
}

bool RouteUdpSender::sendGol1Waypoints(const QVector<QVector3D> &waypoints)
{
    const double stampSeconds = double(QDateTime::currentMSecsSinceEpoch()) / 1000.0;
    int sentDatagrams = 0;
    for (const QVector3D &waypoint : waypoints) {
        const QByteArray datagram = PointCloudProtocol::encodeRosGoalPose(++m_sequence,
                                                                          stampSeconds,
                                                                          waypoint);
        const qint64 written = m_socket.writeDatagram(datagram, m_targetHost, quint16(m_targetPort));
        if (written != datagram.size()) {
            qCritical().noquote() << tr("GOL1 航点 UDP 发送失败：序号 %1，目标 %2:%3，期望 %4 字节，实际 %5 字节，错误：%6")
                                        .arg(m_sequence)
                                        .arg(m_targetHost.toString())
                                        .arg(m_targetPort)
                                        .arg(datagram.size())
                                        .arg(written)
                                        .arg(m_socket.errorString());
            setStatus(tr("GOL1 航点发送失败：%1").arg(m_socket.errorString()));
            return false;
        }
        ++sentDatagrams;
    }

    ++m_routesSent;
    qInfo().noquote() << tr("GOL1 航线已发送：目标 %1:%2，航点 %3 个，UDP 包 %4 个，最新序号 %5")
                            .arg(m_targetHost.toString())
                            .arg(m_targetPort)
                            .arg(waypoints.size())
                            .arg(sentDatagrams)
                            .arg(m_sequence);
    emit routesSentChanged();
    setStatus(tr("已按 GOL1 发送航线，%1 个目标点").arg(sentDatagrams));
    return true;
}

void RouteUdpSender::setStatus(const QString &status)
{
    if (m_status == status)
        return;

    m_status = status;
    emit statusChanged();
}
