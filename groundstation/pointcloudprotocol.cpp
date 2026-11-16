// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "pointcloudprotocol.h"

#include <QDataStream>
#include <QIODevice>
#include <QtEndian>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>

namespace {
constexpr char PointCloudMagic[4] = { 'P', 'C', 'D', '1' };
constexpr char RouteMagic[4] = { 'R', 'T', 'E', '1' };
constexpr quint16 PointCloudHeaderBytes = 32;
constexpr quint16 RouteHeaderBytes = 24;
constexpr qsizetype RosMaxCompressedFrameBytes = 64 * 1024 * 1024;
constexpr qsizetype RosMaxUncompressedFrameBytes = 256 * 1024 * 1024;

void setupStream(QDataStream &stream)
{
    stream.setVersion(QDataStream::Qt_6_0);
    stream.setByteOrder(QDataStream::LittleEndian);
    stream.setFloatingPointPrecision(QDataStream::SinglePrecision);
}

quint16 readLe16(const char *data)
{
    return qFromLittleEndian<quint16>(reinterpret_cast<const uchar *>(data));
}

quint32 readLe32(const char *data)
{
    return qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(data));
}

float readFloat32Le(const char *data)
{
    const quint32 bits = readLe32(data);
    float value = 0.0f;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

double readFloat64Le(const char *data)
{
    const quint64 bits = qFromLittleEndian<quint64>(reinterpret_cast<const uchar *>(data));
    double value = 0.0;
    static_assert(sizeof(value) == sizeof(bits));
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

void appendLe16(QByteArray *buffer, quint16 value)
{
    const quint16 le = qToLittleEndian(value);
    buffer->append(reinterpret_cast<const char *>(&le), sizeof(le));
}

void appendLe32(QByteArray *buffer, quint32 value)
{
    const quint32 le = qToLittleEndian(value);
    buffer->append(reinterpret_cast<const char *>(&le), sizeof(le));
}

void appendFloat32Le(QByteArray *buffer, float value)
{
    quint32 bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    appendLe32(buffer, bits);
}

void appendFloat64Le(QByteArray *buffer, double value)
{
    quint64 bits = 0;
    static_assert(sizeof(bits) == sizeof(value));
    std::memcpy(&bits, &value, sizeof(bits));
    const quint64 le = qToLittleEndian(bits);
    buffer->append(reinterpret_cast<const char *>(&le), sizeof(le));
}

bool readLz4Length(const QByteArray &source, int *offset, int *length)
{
    if (!offset || !length)
        return false;

    while (*offset < source.size()) {
        const int add = uchar(source.at(*offset));
        ++(*offset);
        if (*length > std::numeric_limits<int>::max() - add)
            return false;
        *length += add;
        if (add != 255)
            return true;
    }
    return false;
}

bool decompressLz4Block(const QByteArray &source, int expectedBytes, QByteArray *output)
{
    if (!output || expectedBytes < 0)
        return false;

    output->resize(expectedBytes);
    char *dst = output->data();
    int srcOffset = 0;
    int dstOffset = 0;

    while (srcOffset < source.size()) {
        const int token = uchar(source.at(srcOffset++));

        int literalLength = token >> 4;
        if (literalLength == 15 && !readLz4Length(source, &srcOffset, &literalLength))
            return false;
        if (literalLength < 0
            || srcOffset > source.size() - literalLength
            || dstOffset > expectedBytes - literalLength) {
            return false;
        }

        if (literalLength > 0) {
            std::memcpy(dst + dstOffset, source.constData() + srcOffset, size_t(literalLength));
            srcOffset += literalLength;
            dstOffset += literalLength;
        }

        // LZ4 block 的最后一个 sequence 可以只有 literals，没有 offset/match。
        if (srcOffset == source.size())
            break;
        if (srcOffset > source.size() - 2)
            return false;

        const int matchOffset = readLe16(source.constData() + srcOffset);
        srcOffset += 2;
        if (matchOffset <= 0 || matchOffset > dstOffset)
            return false;

        int matchLength = (token & 0x0f) + 4;
        if ((token & 0x0f) == 15 && !readLz4Length(source, &srcOffset, &matchLength))
            return false;
        if (matchLength < 0 || dstOffset > expectedBytes - matchLength)
            return false;

        // match 区间允许重叠，必须逐字节复制，不能直接 memcpy。
        for (int i = 0; i < matchLength; ++i) {
            dst[dstOffset] = dst[dstOffset - matchOffset];
            ++dstOffset;
        }
    }

    return dstOffset == expectedBytes;
}
} // namespace

QByteArray PointCloudProtocol::encodePointCloudChunk(quint32 sequence,
                                                     quint16 chunkIndex,
                                                     quint16 chunkCount,
                                                     quint32 totalPointCount,
                                                     quint64 timestampUsec,
                                                     const QVector<PointCloudPoint> &points,
                                                     qsizetype offset,
                                                     qsizetype count)
{
    const qsizetype safeOffset = std::clamp<qsizetype>(offset, 0, points.size());
    const qsizetype safeCount = std::clamp<qsizetype>(count, 0, points.size() - safeOffset);

    QByteArray datagram;
    datagram.reserve(PointCloudHeaderBytes + int(safeCount * PointRecordBytes));

    QDataStream out(&datagram, QIODevice::WriteOnly);
    setupStream(out);
    out.writeRawData(PointCloudMagic, 4);
    out << Version
        << PointCloudHeaderBytes
        << sequence
        << chunkIndex
        << chunkCount
        << totalPointCount
        << quint32(safeCount)
        << timestampUsec;

    for (qsizetype i = 0; i < safeCount; ++i) {
        const PointCloudPoint &point = points.at(safeOffset + i);
        out << point.position.x()
            << point.position.y()
            << point.position.z()
            << point.intensity;
    }

    return datagram;
}

bool PointCloudProtocol::decodePointCloudChunk(const QByteArray &datagram, PointCloudChunk *chunk)
{
    if (!chunk || datagram.size() < PointCloudHeaderBytes)
        return false;

    QDataStream in(datagram);
    setupStream(in);

    char magic[4] = {};
    if (in.readRawData(magic, 4) != 4 || std::memcmp(magic, PointCloudMagic, 4) != 0)
        return false;

    quint16 version = 0;
    quint16 headerBytes = 0;
    quint32 sequence = 0;
    quint16 chunkIndex = 0;
    quint16 chunkCount = 0;
    quint32 totalPointCount = 0;
    quint32 chunkPointCount = 0;
    quint64 timestampUsec = 0;
    in >> version
       >> headerBytes
       >> sequence
       >> chunkIndex
       >> chunkCount
       >> totalPointCount
       >> chunkPointCount
       >> timestampUsec;

    if (in.status() != QDataStream::Ok
        || version != Version
        || headerBytes != PointCloudHeaderBytes
        || chunkCount == 0
        || chunkIndex >= chunkCount)
        return false;

    const qsizetype expectedBytes = headerBytes + qsizetype(chunkPointCount) * PointRecordBytes;
    if (datagram.size() != expectedBytes)
        return false;

    QVector<PointCloudPoint> points;
    points.reserve(int(chunkPointCount));
    for (quint32 i = 0; i < chunkPointCount; ++i) {
        PointCloudPoint point;
        in >> point.position[0]
           >> point.position[1]
           >> point.position[2]
           >> point.intensity;
        if (in.status() != QDataStream::Ok)
            return false;
        points.append(point);
    }

    chunk->sequence = sequence;
    chunk->chunkIndex = chunkIndex;
    chunk->chunkCount = chunkCount;
    chunk->totalPointCount = totalPointCount;
    chunk->timestampUsec = timestampUsec;
    chunk->points = std::move(points);
    return true;
}

bool PointCloudProtocol::decodeRosCompressedPointCloudChunk(
    const QByteArray &datagram,
    RosCompressedPointCloudChunk *chunk)
{
    if (!chunk || datagram.size() < RosCompressedPointCloudHeaderBytes)
        return false;

    const char *data = datagram.constData();
    const quint32 magic = readLe32(data);
    if (magic != RosPcd1Magic)
        return false;

    const quint32 sequence = readLe32(data + 4);
    const quint16 chunkIndex = readLe16(data + 8);
    const quint16 chunkCount = readLe16(data + 10);
    const quint32 uncompressedBytes = readLe32(data + 12);
    const quint32 compressedBytes = readLe32(data + 16);
    const quint16 payloadLength = readLe16(data + 20);

    if (chunkCount == 0
        || chunkIndex >= chunkCount
        || payloadLength == 0
        || datagram.size() != RosCompressedPointCloudHeaderBytes + payloadLength
        || uncompressedBytes == 0
        || compressedBytes == 0
        || uncompressedBytes % (3 * sizeof(float)) != 0
        || uncompressedBytes > RosMaxUncompressedFrameBytes
        || compressedBytes > RosMaxCompressedFrameBytes) {
        return false;
    }

    chunk->sequence = sequence;
    chunk->chunkIndex = chunkIndex;
    chunk->chunkCount = chunkCount;
    chunk->uncompressedBytes = uncompressedBytes;
    chunk->compressedBytes = compressedBytes;
    chunk->payload = datagram.mid(RosCompressedPointCloudHeaderBytes, payloadLength);
    return true;
}

bool PointCloudProtocol::decodeRosCompressedPointCloudFrame(
    const QByteArray &compressedFrame,
    quint32 uncompressedBytes,
    QVector<PointCloudPoint> *points)
{
    if (!points
        || compressedFrame.isEmpty()
        || uncompressedBytes == 0
        || uncompressedBytes % (3 * sizeof(float)) != 0
        || uncompressedBytes > RosMaxUncompressedFrameBytes
        || compressedFrame.size() > RosMaxCompressedFrameBytes) {
        return false;
    }

    QByteArray xyzBytes;
    if (!decompressLz4Block(compressedFrame, int(uncompressedBytes), &xyzBytes))
        return false;

    const qsizetype pointCount = xyzBytes.size() / (3 * qsizetype(sizeof(float)));
    QVector<PointCloudPoint> decoded;
    decoded.reserve(int(pointCount));
    const char *raw = xyzBytes.constData();
    for (qsizetype i = 0; i < pointCount; ++i) {
        const qsizetype offset = i * 3 * qsizetype(sizeof(float));
        PointCloudPoint point;
        point.position.setX(readFloat32Le(raw + offset));
        point.position.setY(readFloat32Le(raw + offset + 4));
        point.position.setZ(readFloat32Le(raw + offset + 8));
        point.intensity = 1.0f;
        decoded.append(point);
    }

    *points = std::move(decoded);
    return true;
}

bool PointCloudProtocol::decodeRosPosePacket(const QByteArray &datagram, RosPosePacket *pose)
{
    if (!pose || datagram.size() != RosPoseBytes)
        return false;

    const char *data = datagram.constData();
    const quint32 magic = readLe32(data);
    if (magic != RosPos1Magic)
        return false;

    pose->sequence = readLe32(data + 4);
    pose->stampSeconds = readFloat64Le(data + 8);
    pose->position.setX(readFloat32Le(data + 16));
    pose->position.setY(readFloat32Le(data + 20));
    pose->position.setZ(readFloat32Le(data + 24));
    pose->qx = readFloat32Le(data + 28);
    pose->qy = readFloat32Le(data + 32);
    pose->qz = readFloat32Le(data + 36);
    pose->qw = readFloat32Le(data + 40);
    const double normSquared = double(pose->qx)*pose->qx + double(pose->qy)*pose->qy
        + double(pose->qz)*pose->qz + double(pose->qw)*pose->qw;
    if (!std::isfinite(pose->stampSeconds) || pose->stampSeconds <= 0.0
        || !std::isfinite(pose->position.x()) || !std::isfinite(pose->position.y()) || !std::isfinite(pose->position.z())
        || !std::isfinite(normSquared) || std::abs(std::sqrt(normSquared)-1.0) > 0.05)
        return false;
    return true;
}

QByteArray PointCloudProtocol::encodeRoute(quint32 sequence,
                                           quint64 timestampUsec,
                                           const QVector<QVector3D> &waypoints)
{
    QByteArray datagram;
    datagram.reserve(RouteHeaderBytes + int(waypoints.size() * RoutePointBytes));

    QDataStream out(&datagram, QIODevice::WriteOnly);
    setupStream(out);
    out.writeRawData(RouteMagic, 4);
    out << Version
        << RouteHeaderBytes
        << sequence
        << quint32(waypoints.size())
        << timestampUsec;

    for (const QVector3D &point : waypoints)
        out << point.x() << point.y() << point.z();

    return datagram;
}

bool PointCloudProtocol::decodeRoute(const QByteArray &datagram, RoutePacket *route)
{
    if (!route || datagram.size() < RouteHeaderBytes)
        return false;

    QDataStream in(datagram);
    setupStream(in);

    char magic[4] = {};
    if (in.readRawData(magic, 4) != 4 || std::memcmp(magic, RouteMagic, 4) != 0)
        return false;

    quint16 version = 0;
    quint16 headerBytes = 0;
    quint32 sequence = 0;
    quint32 waypointCount = 0;
    quint64 timestampUsec = 0;
    in >> version
       >> headerBytes
       >> sequence
       >> waypointCount
       >> timestampUsec;

    if (in.status() != QDataStream::Ok || version != Version || headerBytes != RouteHeaderBytes)
        return false;

    const qsizetype expectedBytes = headerBytes + qsizetype(waypointCount) * RoutePointBytes;
    if (datagram.size() != expectedBytes)
        return false;

    QVector<QVector3D> waypoints;
    waypoints.reserve(int(waypointCount));
    for (quint32 i = 0; i < waypointCount; ++i) {
        QVector3D point;
        in >> point[0] >> point[1] >> point[2];
        if (in.status() != QDataStream::Ok)
            return false;
        waypoints.append(point);
    }

    route->sequence = sequence;
    route->timestampUsec = timestampUsec;
    route->waypoints = std::move(waypoints);
    return true;
}

QByteArray PointCloudProtocol::encodeRosGoalPose(quint32 sequence,
                                                 double stampSeconds,
                                                 const QVector3D &position)
{
    QByteArray datagram;
    datagram.reserve(RosGoalPoseBytes);
    appendLe32(&datagram, RosGol1Magic);
    appendLe32(&datagram, sequence);
    appendFloat64Le(&datagram, stampSeconds);
    appendFloat32Le(&datagram, position.x());
    appendFloat32Le(&datagram, position.y());
    appendFloat32Le(&datagram, position.z());
    appendFloat32Le(&datagram, 0.0f);
    appendFloat32Le(&datagram, 0.0f);
    appendFloat32Le(&datagram, 0.0f);
    appendFloat32Le(&datagram, 1.0f);
    return datagram;
}
