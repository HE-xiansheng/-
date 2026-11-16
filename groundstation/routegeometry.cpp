// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "routegeometry.h"

#include <QDebug>
#include <QtMath>

#include <algorithm>
#include <limits>

namespace {
constexpr int RouteVertexStride = 3 * int(sizeof(float));
}

RouteGeometry::RouteGeometry(QQuick3DObject *parent)
    : QQuick3DGeometry(parent)
{
    rebuildGeometry();
}

QVariantList RouteGeometry::waypoints() const
{
    QVariantList list;
    list.reserve(m_waypoints.size());
    for (const QVector3D &point : m_waypoints)
        list.append(QVariant::fromValue(point));
    return list;
}

void RouteGeometry::setWaypoints(const QVariantList &points)
{
    QVector<QVector3D> parsed;
    parsed.reserve(points.size());
    for (const QVariant &point : points) {
        if (point.canConvert<QVector3D>())
            parsed.append(point.value<QVector3D>());
    }

    if (parsed == m_waypoints)
        return;

    qInfo().noquote() << QStringLiteral("航线整体替换：原航点 %1 个，新航点 %2 个")
                          .arg(m_waypoints.size())
                          .arg(parsed.size());
    m_waypoints = std::move(parsed);
    notifyRouteChanged();
}

QVariantList RouteGeometry::sceneWaypoints() const
{
    QVariantList list;
    list.reserve(m_waypoints.size());
    for (const QVector3D &point : m_waypoints)
        list.append(QVariant::fromValue(pointToScene(point)));
    return list;
}

void RouteGeometry::setSceneScale(float scale)
{
    const float safeScale = std::max(scale, 1.0f);
    if (qFuzzyCompare(m_sceneScale, safeScale))
        return;

    qInfo().noquote() << QStringLiteral("航线场景缩放变更：%1 -> %2").arg(m_sceneScale).arg(safeScale);
    m_sceneScale = safeScale;
    rebuildGeometry();
    update();
    emit sceneScaleChanged();
    emit waypointsChanged();
}

void RouteGeometry::clearRoute()
{
    if (m_waypoints.isEmpty())
        return;

    qInfo().noquote() << QStringLiteral("清空航线：原航点 %1 个").arg(m_waypoints.size());
    m_waypoints.clear();
    notifyRouteChanged();
}

void RouteGeometry::addWaypoint(const QVector3D &pointMeters)
{
    m_waypoints.append(pointMeters);
    qInfo().noquote() << QStringLiteral("新增航点 #%1：X=%2 Y=%3 Z=%4")
                          .arg(m_waypoints.size())
                          .arg(pointMeters.x(), 0, 'f', 2)
                          .arg(pointMeters.y(), 0, 'f', 2)
                          .arg(pointMeters.z(), 0, 'f', 2);
    notifyRouteChanged();
}

void RouteGeometry::insertWaypoint(int index, const QVector3D &pointMeters)
{
    const int safeIndex = std::clamp(index, 0, int(m_waypoints.size()));
    m_waypoints.insert(safeIndex, pointMeters);
    qInfo().noquote() << QStringLiteral("插入避障航点 index=%1：X=%2 Y=%3 Z=%4")
                          .arg(safeIndex)
                          .arg(pointMeters.x(), 0, 'f', 2)
                          .arg(pointMeters.y(), 0, 'f', 2)
                          .arg(pointMeters.z(), 0, 'f', 2);
    notifyRouteChanged();
}

void RouteGeometry::setWaypoint(int index, const QVector3D &pointMeters)
{
    if (index < 0 || index >= m_waypoints.size()) {
        qWarning().noquote() << QStringLiteral("更新航点失败：index=%1 越界，当前航点数=%2")
                                .arg(index)
                                .arg(m_waypoints.size());
        return;
    }

    if (m_waypoints[index] == pointMeters)
        return;

    m_waypoints[index] = pointMeters;
    static int waypointUpdateLogCount = 0;
    ++waypointUpdateLogCount;
    if (waypointUpdateLogCount <= 5 || waypointUpdateLogCount % 20 == 0) {
        qInfo().noquote() << QStringLiteral("更新航点 #%1：X=%2 Y=%3 Z=%4")
                              .arg(index + 1)
                              .arg(pointMeters.x(), 0, 'f', 2)
                              .arg(pointMeters.y(), 0, 'f', 2)
                              .arg(pointMeters.z(), 0, 'f', 2);
    }
    notifyRouteChanged();
}

void RouteGeometry::removeWaypoint(int index)
{
    if (index < 0 || index >= m_waypoints.size()) {
        qWarning().noquote() << QStringLiteral("删除航点失败：index=%1 越界，当前航点数=%2")
                                .arg(index)
                                .arg(m_waypoints.size());
        return;
    }

    qInfo().noquote() << QStringLiteral("删除航点 #%1").arg(index + 1);
    m_waypoints.removeAt(index);
    notifyRouteChanged();
}

QVector3D RouteGeometry::waypoint(int index) const
{
    if (index < 0 || index >= m_waypoints.size())
        return {};
    return m_waypoints.at(index);
}

QVector3D RouteGeometry::sceneWaypoint(int index) const
{
    return pointToScene(waypoint(index));
}

QVector3D RouteGeometry::toScenePosition(const QVector3D &meters) const
{
    return pointToScene(meters);
}

QVector3D RouteGeometry::fromScenePosition(const QVector3D &scenePosition) const
{
    return sceneToPoint(scenePosition);
}

void RouteGeometry::rebuildGeometry()
{
    clear();

    QByteArray vertexData;
    vertexData.resize(m_waypoints.size() * RouteVertexStride);
    float *cursor = reinterpret_cast<float *>(vertexData.data());

    QVector3D minScene(std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max());
    QVector3D maxScene(std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest());

    for (const QVector3D &point : m_waypoints) {
        const QVector3D scenePoint = pointToScene(point);
        minScene.setX(std::min(minScene.x(), scenePoint.x()));
        minScene.setY(std::min(minScene.y(), scenePoint.y()));
        minScene.setZ(std::min(minScene.z(), scenePoint.z()));
        maxScene.setX(std::max(maxScene.x(), scenePoint.x()));
        maxScene.setY(std::max(maxScene.y(), scenePoint.y()));
        maxScene.setZ(std::max(maxScene.z(), scenePoint.z()));

        *cursor++ = scenePoint.x();
        *cursor++ = scenePoint.y();
        *cursor++ = scenePoint.z();
    }

    if (m_waypoints.isEmpty()) {
        minScene = QVector3D(-1.0f, -1.0f, -1.0f);
        maxScene = QVector3D(1.0f, 1.0f, 1.0f);
    }

    setVertexData(vertexData);
    setStride(RouteVertexStride);
    setBounds(minScene, maxScene);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::LineStrip);
    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                 0,
                 QQuick3DGeometry::Attribute::F32Type);
}

QVector3D RouteGeometry::pointToScene(const QVector3D &meters) const
{
    return QVector3D(-meters.y() * m_sceneScale,
                     meters.z() * m_sceneScale,
                     -meters.x() * m_sceneScale);
}

QVector3D RouteGeometry::sceneToPoint(const QVector3D &scenePosition) const
{
    return QVector3D(-scenePosition.z() / m_sceneScale,
                     -scenePosition.x() / m_sceneScale,
                     scenePosition.y() / m_sceneScale);
}

void RouteGeometry::notifyRouteChanged()
{
    rebuildGeometry();
    update();
    emit waypointsChanged();
}
