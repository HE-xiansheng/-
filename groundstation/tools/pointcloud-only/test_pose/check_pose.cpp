// 使用真实Qt接收器验证POS1解码、方向映射、断流隐藏和乱序拒绝；仅回环UDP。
#include "udptransport.h"
#include "pointcloudprotocol.h"
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QFile>
#include <QQuaternion>
#include <QThread>
#include <QtEndian>
#include <cmath>
#include <iostream>

bool waitFor(const std::function<bool()> &ready, int timeout)
{
    QElapsedTimer elapsed;
    elapsed.start();
    while (elapsed.elapsed() < timeout) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        if (ready()) return true;
        QThread::msleep(5);
    }
    return false;
}

int main(int argc, char **argv)
{
    QCoreApplication app(argc, argv);
    if (argc != 3) return 2;
    QFile f(argv[1]), next(argv[2]);
    if (!f.open(QIODevice::ReadOnly) || !next.open(QIODevice::ReadOnly)) return 3;
    auto packet = f.readAll();
    auto newer = next.readAll();
    RosPosePacket decoded;
    if (!PointCloudProtocol::decodeRosPosePacket(packet, &decoded)) return 4;
    if ((decoded.position-QVector3D(1.25,-2.5,.3)).length() > 1e-6) return 5;
    auto bad = packet;
    qToLittleEndian<quint32>(0x7fc00000, reinterpret_cast<uchar*>(bad.data()+16));
    if (PointCloudProtocol::decodeRosPosePacket(bad, &decoded)) return 6;
    bad = packet;
    for (int offset = 28; offset < 44; offset += 4)
        qToLittleEndian<quint32>(0, reinterpret_cast<uchar*>(bad.data()+offset));
    if (PointCloudProtocol::decodeRosPosePacket(bad, &decoded)) return 7;
    QUdpSocket portProbe;
    if (!portProbe.bind(QHostAddress(QHostAddress::LocalHost), quint16(0))) return 8;
    const auto port = portProbe.localPort();
    portProbe.close();
    UdpPoseReceiver receiver;
    receiver.setListenPort(port);
    receiver.setActive(true);
    QUdpSocket sender;
    sender.writeDatagram(packet, QHostAddress::LocalHost, port);
    if (!waitFor([&] { return receiver.hasPose(); }, 500)) return 9;
    if (std::abs(receiver.yawDegrees()-90.) > .001) return 10;
    // ROS +90°前向为世界+Y，经原metersToScene映射为Qt-X。
    const auto arrow = QQuaternion::fromEulerAngles(0,receiver.yawDegrees(),0).rotatedVector(QVector3D(0,0,-1));
    if ((arrow-QVector3D(-1,0,0)).length() > 1e-5) return 11;
    if (!waitFor([&] { return !receiver.hasPose(); }, 2000)) return 12;
    sender.writeDatagram(packet, QHostAddress::LocalHost, port);
    waitFor([] { return false; }, 100);
    if (receiver.hasPose()) return 13;
    sender.writeDatagram(newer, QHostAddress::LocalHost, port);
    if (!waitFor([&] { return receiver.hasPose(); }, 500)) return 14;
    receiver.setActive(false);
    if (receiver.hasPose()) return 15;
    std::cout << "{\"passed\":true,\"checks\":[\"python_POS1_to_original_Qt_decode\",\"finite_and_unit_quaternion\",\"ROS_90deg_to_Qt_forward\",\"stale_marker_hidden\",\"old_timestamp_rejected\",\"new_pose_recovers\",\"disable_hides_marker\"]}" << std::endl;
    return 0;
}
