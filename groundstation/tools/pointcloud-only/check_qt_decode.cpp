// 用地面站原始 C++ 解码器校验真实采集的发送桥数据，避免只验证自写编解码器。
#include "pointcloudprotocol.h"
#include <QCoreApplication>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <cmath>
#include <iostream>

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    if (argc != 3) return 2;
    QFile packet(argv[1]), data(argv[2]);
    if (!packet.open(QIODevice::ReadOnly) || !data.open(QIODevice::ReadOnly)) return 3;
    RosCompressedPointCloudChunk chunk;
    QVector<PointCloudPoint> points;
    if (!PointCloudProtocol::decodeRosCompressedPointCloudChunk(packet.readAll(), &chunk)) return 4;
    const auto compressed = data.readAll();
    if (compressed.size() != chunk.compressedBytes) return 5;
    if (!PointCloudProtocol::decodeRosCompressedPointCloudFrame(compressed,chunk.uncompressedBytes,&points)) return 6;
    if (points.isEmpty() || points.size()*12 != chunk.uncompressedBytes) return 7;
    for (const auto &p : points)
        if (!std::isfinite(p.position.x()) || !std::isfinite(p.position.y()) || !std::isfinite(p.position.z())) return 8;
    QJsonObject result;
    result["passed"] = true;
    result["original_qt_decoder"] = true;
    result["sequence"] = double(chunk.sequence);
    result["points"] = int(points.size());
    result["first_xyz"] = QJsonArray{points[0].position.x(),points[0].position.y(),points[0].position.z()};
    std::cout << QJsonDocument(result).toJson(QJsonDocument::Compact).constData() << std::endl;
    return 0;
}
