// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef POINTCLOUDPROTOCOL_H
#define POINTCLOUDPROTOCOL_H

#include <QByteArray>
#include <QVector>
#include <QVector3D>

// 点云/航线 UDP 协议定义。
//
// 作用：
// - 提供两个 demo 共用的轻量二进制协议，避免发送端和接收端各自拼字节。
// - 点云帧会被拆成多个 UDP 分片；航线包始终包含完整航点列表。
//
// 协作关系：
// - radarsimulator.exe 调用 encodePointCloudChunk() 周期性发送室内点云。
// - UdpPointCloudReceiver 调用 decodePointCloudChunk() 解析、重组并显示点云。
// - RouteUdpSender 调用 encodeRoute() 上传完整航线。
// - radarsimulator.exe 或真实控制端调用 decodeRoute() 接收航线。
//
// 坐标和字节序：
// - 坐标单位统一为米，坐标系为 X 前、Y 左、Z 上。
// - QDataStream 固定 LittleEndian 和 SinglePrecision，Windows/Qt 之间可直接互通。
// - decode* 返回 false 表示包头、版本、长度或字段不合法，调用方负责写日志和丢弃该包。

// 单个点云点。
struct PointCloudPoint
{
    QVector3D position;   // 单个点在雷达坐标系下的位置，单位：米。
    float intensity = 1.0f; // 0.0 ~ 1.0 的反射率/强度，用于地面站着色。
};

// 点云帧的一个 UDP 分片。
struct PointCloudChunk
{
    quint32 sequence = 0;        // 点云帧序号。
    quint16 chunkIndex = 0;      // 当前分片下标，从 0 开始。
    quint16 chunkCount = 0;      // 当前帧总分片数。
    quint32 totalPointCount = 0; // 当前帧总点数。
    quint64 timestampUsec = 0;   // 发送端时间戳，单位：微秒。
    QVector<PointCloudPoint> points;
};

// 香橙派 ROS 工程使用的 LZ4 压缩点云分片。
//
// something1017/src/udp_uav_tx 会把一整帧 PointCloud2 的 xyz(float32) 先做
// LZ4 block 压缩，再按 1400 字节 UDP 包切片。本结构只保存单片载荷；
// UdpPointCloudReceiver 负责把同一 sequence 的多片拼回完整压缩帧。
struct RosCompressedPointCloudChunk
{
    quint32 sequence = 0;          // 点云帧序号，对应 ROS 发送端 frame_id。
    quint16 chunkIndex = 0;        // 当前压缩分片下标。
    quint16 chunkCount = 0;        // 当前压缩帧总分片数。
    quint32 uncompressedBytes = 0; // 解压后 xyz(float32) 总字节数。
    quint32 compressedBytes = 0;   // 整帧 LZ4 压缩数据总字节数。
    QByteArray payload;            // 当前 UDP 分片里的压缩数据片段。
};

// 香橙派 ROS 工程发来的 POS1 位姿单包。
struct RosPosePacket
{
    quint32 sequence = 0;     // 位姿包序号。
    double stampSeconds = 0;  // ROS 时间戳，单位：秒。
    QVector3D position;       // px/py/pz，单位：米。
    float qx = 0.0f;          // 姿态四元数 x。
    float qy = 0.0f;          // 姿态四元数 y。
    float qz = 0.0f;          // 姿态四元数 z。
    float qw = 1.0f;          // 姿态四元数 w。
};

// 完整航线 UDP 包。
struct RoutePacket
{
    quint32 sequence = 0;      // 航线快照序号。
    quint64 timestampUsec = 0; // 发送端时间戳，单位：微秒。
    QVector<QVector3D> waypoints;
};

namespace PointCloudProtocol {

// 当前协议版本。版本不匹配时 decode 会返回 false。
constexpr quint16 Version = 1;
// 一个点的二进制长度：float x/y/z/intensity。
constexpr qsizetype PointRecordBytes = 16; // float x/y/z/intensity。
// 一个航点的二进制长度：float x/y/z。
constexpr qsizetype RoutePointBytes = 12;  // float x/y/z。
// 单个 UDP 包最多携带的点数。900 点约 14KB，适合 demo 局域网传输。
constexpr int MaxPointsPerDatagram = 900;  // 控制在常见 MTU 分片风险较低的 UDP 包大小内。

// 香橙派 ROS 工程中的二进制协议常量。
// 注意：远端 C++ 代码把 magic 当 uint32_t 写入 socket，小端机器实际字节序不是字符串字面量。
constexpr quint32 RosPcd1Magic = 0x50434431; // 'PCD1' 的 uint32_t 值。
constexpr quint32 RosPos1Magic = 0x504F5331; // 'POS1' 的 uint32_t 值。
constexpr quint32 RosGol1Magic = 0x474F4C31; // 'GOL1' 的 uint32_t 值。
constexpr qsizetype RosCompressedPointCloudHeaderBytes = 22;
constexpr qsizetype RosPoseBytes = 44;
constexpr qsizetype RosGoalPoseBytes = 44;

// 编码点云分片。
// sequence/chunkIndex/chunkCount 用于接收端重组完整帧；
// offset/count 指定从 points 中取哪一段写入当前 UDP 包。
QByteArray encodePointCloudChunk(quint32 sequence,
                                 quint16 chunkIndex,
                                 quint16 chunkCount,
                                 quint32 totalPointCount,
                                 quint64 timestampUsec,
                                 const QVector<PointCloudPoint> &points,
                                 qsizetype offset,
                                 qsizetype count);

// 解码点云分片。失败时返回 false，chunk 内容不应继续使用。
bool decodePointCloudChunk(const QByteArray &datagram, PointCloudChunk *chunk);

// 解码 something1017/udp_uav_tx 发来的 LZ4 压缩点云分片。
// 该函数只解析 UDP 头和当前片 payload，不做整帧解压。
bool decodeRosCompressedPointCloudChunk(const QByteArray &datagram,
                                        RosCompressedPointCloudChunk *chunk);

// 解压并转换一整帧香橙派 ROS 点云。
// compressedFrame 必须是按 chunkIndex 顺序拼好的 LZ4 block 数据；
// uncompressedBytes 是协议头里的原始 xyz(float32) 字节数。
bool decodeRosCompressedPointCloudFrame(const QByteArray &compressedFrame,
                                        quint32 uncompressedBytes,
                                        QVector<PointCloudPoint> *points);

// 解码 something1017/udp_uav_tx 发来的 POS1 位姿单包。
bool decodeRosPosePacket(const QByteArray &datagram, RosPosePacket *pose);

// 编码完整航线快照。每次上传都包含所有航点，接收端以后到包为准。
QByteArray encodeRoute(quint32 sequence,
                       quint64 timestampUsec,
                       const QVector<QVector3D> &waypoints);

// 解码完整航线快照。失败时返回 false，route 内容不应继续使用。
bool decodeRoute(const QByteArray &datagram, RoutePacket *route);

// 编码 something1017/udp_uav_tx 监听的 GOL1 单目标点。
// sequence 保持远端协议里的普通发送序号语义；当前远端 PoseStamped 不携带独立航点 ID。
// 远端会把每个 GOL1 包发布成一个 geometry_msgs/PoseStamped；
// 本 demo 上传整条航线时会按航点顺序发送多个 GOL1 包。
QByteArray encodeRosGoalPose(quint32 sequence,
                             double stampSeconds,
                             const QVector3D &position);

} // namespace PointCloudProtocol

#endif // POINTCLOUDPROTOCOL_H
