// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "pointcloudgeometry.h"

#include <QDebug>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
constexpr int FloatsPerVertex = 7; // scene x/y/z + vertex color r/g/b/a。
constexpr int VertexStride = FloatsPerVertex * int(sizeof(float));

float clampedIntensity(float value)
{
    return std::clamp(value, 0.0f, 1.0f);
}

QVector3D mixColor(const QVector3D &a, const QVector3D &b, float t)
{
    const float safeT = std::clamp(t, 0.0f, 1.0f);
    return a * (1.0f - safeT) + b * safeT;
}

QVector3D heightColor(float normalizedHeight)
{
    const float h = std::clamp(normalizedHeight, 0.0f, 1.0f);
    const QVector3D deepBlue(0.00f, 0.18f, 0.95f);
    const QVector3D cyan(0.00f, 0.88f, 1.00f);
    const QVector3D green(0.05f, 0.95f, 0.18f);
    const QVector3D yellow(1.00f, 0.88f, 0.00f);
    const QVector3D orange(1.00f, 0.42f, 0.00f);
    const QVector3D red(1.00f, 0.00f, 0.00f);

    if (h < 0.28f)
        return mixColor(deepBlue, cyan, h / 0.28f);
    if (h < 0.52f)
        return mixColor(cyan, green, (h - 0.28f) / 0.24f);
    if (h < 0.72f)
        return mixColor(green, yellow, (h - 0.52f) / 0.20f);
    if (h < 0.88f)
        return mixColor(yellow, orange, (h - 0.72f) / 0.16f);
    return mixColor(orange, red, (h - 0.88f) / 0.12f);
}
// 类似截图的高饱和彩虹色，颜色仍由高度决定，不代表反射率或障碍类型。
QVector3D vividHeightColor(float normalizedHeight)
{
    const float h = std::clamp(normalizedHeight, 0.0f, 1.0f);
    const QVector3D colors[] = {{0.85f, 0.0f, 1.0f}, {0.0f, 0.1f, 1.0f},
                                {0.0f, 1.0f, 1.0f}, {0.0f, 1.0f, 0.0f},
                                {1.0f, 1.0f, 0.0f}, {1.0f, 0.05f, 0.0f}};
    const float position = h * 5.0f;
    const int index = std::min(4, int(position));
    return mixColor(colors[index], colors[index + 1], position - index);
}
} // namespace

PointCloudGeometry::PointCloudGeometry(QQuick3DObject *parent)
    : QQuick3DGeometry(parent)
{
    rebuildGeometry();
}

void PointCloudGeometry::setSceneScale(float scale)
{
    const float safeScale = std::max(scale, 1.0f);
    if (qFuzzyCompare(m_sceneScale, safeScale))
        return;

    qInfo().noquote() << QStringLiteral("点云场景缩放变更：%1 -> %2").arg(m_sceneScale).arg(safeScale);
    m_sceneScale = safeScale;
    rebuildGeometry();
    update();
    emit sceneScaleChanged();
    emit pointCloudChanged();
}

void PointCloudGeometry::setRainbowStyle(bool enabled)
{
    if (m_rainbowStyle == enabled) return;
    m_rainbowStyle = enabled;
    rebuildGeometry();
    update();
    emit rainbowStyleChanged();
}

void PointCloudGeometry::setCeilingClipEnabled(bool enabled)
{
    if (m_ceilingClipEnabled == enabled) return;
    m_ceilingClipEnabled = enabled;
    rebuildGeometry(); update();
    emit ceilingClipChanged(); emit pointCloudChanged();
}

void PointCloudGeometry::setCeilingHeight(float height)
{
    if (!std::isfinite(height) || qFuzzyCompare(m_ceilingHeight, height)) return;
    m_ceilingHeight = height;
    if (m_ceilingClipEnabled) { rebuildGeometry(); update(); emit pointCloudChanged(); }
    emit ceilingClipChanged();
}

void PointCloudGeometry::setPoints(const QVector<PointCloudPoint> &points,
                                   quint32 sequence,
                                   quint64 timestampUsec)
{
    static quint64 setPointsCallCount = 0;
    ++setPointsCallCount;

    m_points = points;
    m_lastSequence = sequence;
    m_lastTimestampUsec = timestampUsec;

    if (m_points.isEmpty()) {
        m_boundsMinMeters = QVector3D();
        m_boundsMaxMeters = QVector3D();
        qWarning().noquote() << QStringLiteral("点云几何收到空点集：sequence=%1").arg(sequence);
    } else {
        QVector3D minPoint(std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max(),
                           std::numeric_limits<float>::max());
        QVector3D maxPoint(std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest(),
                           std::numeric_limits<float>::lowest());
        for (const PointCloudPoint &point : m_points) {
            minPoint.setX(std::min(minPoint.x(), point.position.x()));
            minPoint.setY(std::min(minPoint.y(), point.position.y()));
            minPoint.setZ(std::min(minPoint.z(), point.position.z()));
            maxPoint.setX(std::max(maxPoint.x(), point.position.x()));
            maxPoint.setY(std::max(maxPoint.y(), point.position.y()));
            maxPoint.setZ(std::max(maxPoint.z(), point.position.z()));
        }
        m_boundsMinMeters = minPoint;
        m_boundsMaxMeters = maxPoint;
    }

    if (setPointsCallCount <= 3 || setPointsCallCount % 30 == 0 || m_points.isEmpty()) {
        qInfo().noquote() << QStringLiteral("点云几何更新：sequence=%1，点数=%2，boundsMin=(%3,%4,%5)，boundsMax=(%6,%7,%8)")
                              .arg(sequence)
                              .arg(m_points.size())
                              .arg(m_boundsMinMeters.x(), 0, 'f', 2)
                              .arg(m_boundsMinMeters.y(), 0, 'f', 2)
                              .arg(m_boundsMinMeters.z(), 0, 'f', 2)
                              .arg(m_boundsMaxMeters.x(), 0, 'f', 2)
                              .arg(m_boundsMaxMeters.y(), 0, 'f', 2)
                              .arg(m_boundsMaxMeters.z(), 0, 'f', 2);
    }

    rebuildGeometry();
    update();
    emit pointCloudChanged();
}

QVector3D PointCloudGeometry::toScenePosition(const QVector3D &meters) const
{
    return pointToScene(meters);
}

QVector3D PointCloudGeometry::fromScenePosition(const QVector3D &scenePosition) const
{
    return sceneToPoint(scenePosition);
}

QVariantMap PointCloudGeometry::nearestPointOnRay(const QVector3D &rayOriginScene,
                                                  const QVector3D &rayDirectionScene,
                                                  float maxPointDistanceScene,
                                                  float maxRayDistanceScene) const
{
    QVariantMap result;
    result.insert(QStringLiteral("hit"), false);
    result.insert(QStringLiteral("index"), -1);

    const QVector3D direction = rayDirectionScene.normalized();
    if (direction.isNull() || m_points.isEmpty()) {
        qWarning().noquote() << QStringLiteral("点云选点失败：射线无效或点云为空，点数=%1").arg(m_points.size());
        return result;
    }

    const float maxDistance = std::max(maxPointDistanceScene, 1.0f);
    const float maxRayDistance = std::max(maxRayDistanceScene, maxDistance);
    float bestDistance = maxDistance;
    float bestRayDistance = 0.0f;
    int bestIndex = -1;
    QVector3D bestScenePoint;

    for (int i = 0; i < m_points.size(); ++i) {
        const QVector3D scenePoint = pointToScene(m_points.at(i).position);
        const QVector3D originToPoint = scenePoint - rayOriginScene;
        const float rayDistance = QVector3D::dotProduct(originToPoint, direction);
        if (rayDistance < 0.0f || rayDistance > maxRayDistance)
            continue;

        const QVector3D closestPointOnRay = rayOriginScene + direction * rayDistance;
        const float distance = (scenePoint - closestPointOnRay).length();
        if (distance < bestDistance) {
            bestDistance = distance;
            bestRayDistance = rayDistance;
            bestIndex = i;
            bestScenePoint = scenePoint;
        }
    }

    if (bestIndex < 0) {
        qInfo().noquote() << QStringLiteral("点云选点未命中：候选点数=%1，最大垂距=%2，最大射线距离=%3")
                              .arg(m_points.size())
                              .arg(maxDistance, 0, 'f', 2)
                              .arg(maxRayDistance, 0, 'f', 2);
        return result;
    }

    const PointCloudPoint &point = m_points.at(bestIndex);
    qInfo().noquote() << QStringLiteral("点云选点命中：index=%1，distance=%2，rayDistance=%3，point=(%4,%5,%6)")
                          .arg(bestIndex)
                          .arg(bestDistance, 0, 'f', 2)
                          .arg(bestRayDistance, 0, 'f', 2)
                          .arg(point.position.x(), 0, 'f', 2)
                          .arg(point.position.y(), 0, 'f', 2)
                          .arg(point.position.z(), 0, 'f', 2);
    result.insert(QStringLiteral("hit"), true);
    result.insert(QStringLiteral("index"), bestIndex);
    result.insert(QStringLiteral("distance"), bestDistance);
    result.insert(QStringLiteral("rayDistance"), bestRayDistance);
    result.insert(QStringLiteral("position"), QVariant::fromValue(point.position));
    result.insert(QStringLiteral("scenePosition"), QVariant::fromValue(bestScenePoint));
    result.insert(QStringLiteral("intensity"), point.intensity);
    return result;
}

QVariantMap PointCloudGeometry::nearestObstacleAround(const QVector3D &pointMeters,
                                                      float horizontalRadiusMeters,
                                                      float verticalRadiusMeters) const
{
    QVariantMap result;
    result.insert(QStringLiteral("blocked"), false);
    result.insert(QStringLiteral("index"), -1);

    if (m_points.isEmpty()) {
        result.insert(QStringLiteral("distance"), std::numeric_limits<float>::infinity());
        return result;
    }

    const float safeHorizontalRadius = std::max(0.01f, horizontalRadiusMeters);
    const float safeVerticalRadius = std::max(0.01f, verticalRadiusMeters);
    const float horizontalRadiusSquared = safeHorizontalRadius * safeHorizontalRadius;

    int bestIndex = -1;
    float bestDistance = std::numeric_limits<float>::max();
    QVector3D bestPoint;
    for (int i = 0; i < m_points.size(); ++i) {
        const QVector3D obstacle = m_points.at(i).position;
        const float dz = qAbs(obstacle.z() - pointMeters.z());
        if (dz > safeVerticalRadius)
            continue;

        const float dx = obstacle.x() - pointMeters.x();
        const float dy = obstacle.y() - pointMeters.y();
        const float horizontalDistanceSquared = dx * dx + dy * dy;
        if (horizontalDistanceSquared > horizontalRadiusSquared)
            continue;

        const float distance = std::sqrt(horizontalDistanceSquared + dz * dz);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestIndex = i;
            bestPoint = obstacle;
        }
    }

    const bool blocked = bestIndex >= 0;
    result.insert(QStringLiteral("blocked"), blocked);
    result.insert(QStringLiteral("index"), bestIndex);
    result.insert(QStringLiteral("distance"), blocked ? bestDistance : safeHorizontalRadius);
    if (blocked)
        result.insert(QStringLiteral("position"), QVariant::fromValue(bestPoint));

    if (blocked) {
        qInfo().noquote() << QStringLiteral("航点避障检查：候选点 X=%1 Y=%2 Z=%3 附近存在障碍，最近距离=%4，障碍点=(%5,%6,%7)")
                              .arg(pointMeters.x(), 0, 'f', 2)
                              .arg(pointMeters.y(), 0, 'f', 2)
                              .arg(pointMeters.z(), 0, 'f', 2)
                              .arg(bestDistance, 0, 'f', 2)
                              .arg(bestPoint.x(), 0, 'f', 2)
                              .arg(bestPoint.y(), 0, 'f', 2)
                              .arg(bestPoint.z(), 0, 'f', 2);
    }
    return result;
}

QVariantMap PointCloudGeometry::nearestObstacleOnSegment(const QVector3D &startMeters,
                                                         const QVector3D &endMeters,
                                                         float horizontalRadiusMeters,
                                                         float verticalRadiusMeters) const
{
    QVariantMap result;
    result.insert(QStringLiteral("blocked"), false);
    result.insert(QStringLiteral("index"), -1);
    result.insert(QStringLiteral("segmentT"), 0.0f);

    if (m_points.isEmpty()) {
        result.insert(QStringLiteral("distance"), std::numeric_limits<float>::infinity());
        return result;
    }

    const float safeHorizontalRadius = std::max(0.01f, horizontalRadiusMeters);
    const float safeVerticalRadius = std::max(0.01f, verticalRadiusMeters);
    const float horizontalRadiusSquared = safeHorizontalRadius * safeHorizontalRadius;
    const float segmentX = endMeters.x() - startMeters.x();
    const float segmentY = endMeters.y() - startMeters.y();
    const float segmentZ = endMeters.z() - startMeters.z();
    const float segmentHorizontalLengthSquared = segmentX * segmentX + segmentY * segmentY;

    if (segmentHorizontalLengthSquared < 0.000001f)
        return nearestObstacleAround(startMeters, safeHorizontalRadius, safeVerticalRadius);

    const float minX = std::min(startMeters.x(), endMeters.x()) - safeHorizontalRadius;
    const float maxX = std::max(startMeters.x(), endMeters.x()) + safeHorizontalRadius;
    const float minY = std::min(startMeters.y(), endMeters.y()) - safeHorizontalRadius;
    const float maxY = std::max(startMeters.y(), endMeters.y()) + safeHorizontalRadius;
    const float minZ = std::min(startMeters.z(), endMeters.z()) - safeVerticalRadius;
    const float maxZ = std::max(startMeters.z(), endMeters.z()) + safeVerticalRadius;

    int bestIndex = -1;
    float bestDistance = std::numeric_limits<float>::max();
    float bestT = 0.0f;
    QVector3D bestObstaclePoint;
    QVector3D bestRoutePoint;

    for (int i = 0; i < m_points.size(); ++i) {
        const QVector3D obstacle = m_points.at(i).position;
        if (obstacle.x() < minX || obstacle.x() > maxX
            || obstacle.y() < minY || obstacle.y() > maxY
            || obstacle.z() < minZ || obstacle.z() > maxZ) {
            continue;
        }

        const float obstacleToStartX = obstacle.x() - startMeters.x();
        const float obstacleToStartY = obstacle.y() - startMeters.y();
        const float t = std::clamp((obstacleToStartX * segmentX + obstacleToStartY * segmentY)
                                       / segmentHorizontalLengthSquared,
                                   0.0f,
                                   1.0f);
        const QVector3D routePoint(startMeters.x() + segmentX * t,
                                   startMeters.y() + segmentY * t,
                                   startMeters.z() + segmentZ * t);
        const float dz = qAbs(obstacle.z() - routePoint.z());
        if (dz > safeVerticalRadius)
            continue;

        const float dx = obstacle.x() - routePoint.x();
        const float dy = obstacle.y() - routePoint.y();
        const float horizontalDistanceSquared = dx * dx + dy * dy;
        if (horizontalDistanceSquared > horizontalRadiusSquared)
            continue;

        const float distance = std::sqrt(horizontalDistanceSquared + dz * dz);
        if (distance < bestDistance) {
            bestDistance = distance;
            bestIndex = i;
            bestT = t;
            bestObstaclePoint = obstacle;
            bestRoutePoint = routePoint;
        }
    }

    const bool blocked = bestIndex >= 0;
    result.insert(QStringLiteral("blocked"), blocked);
    result.insert(QStringLiteral("index"), bestIndex);
    result.insert(QStringLiteral("distance"), blocked ? bestDistance : safeHorizontalRadius);
    result.insert(QStringLiteral("segmentT"), bestT);
    if (blocked) {
        result.insert(QStringLiteral("position"), QVariant::fromValue(bestObstaclePoint));
        result.insert(QStringLiteral("routePosition"), QVariant::fromValue(bestRoutePoint));
        qInfo().noquote() << QStringLiteral("航线线段避障检查：起点=(%1,%2,%3)，终点=(%4,%5,%6)，t=%7，最近障碍距离=%8，障碍点=(%9,%10,%11)")
                              .arg(startMeters.x(), 0, 'f', 2)
                              .arg(startMeters.y(), 0, 'f', 2)
                              .arg(startMeters.z(), 0, 'f', 2)
                              .arg(endMeters.x(), 0, 'f', 2)
                              .arg(endMeters.y(), 0, 'f', 2)
                              .arg(endMeters.z(), 0, 'f', 2)
                              .arg(bestT, 0, 'f', 2)
                              .arg(bestDistance, 0, 'f', 2)
                              .arg(bestObstaclePoint.x(), 0, 'f', 2)
                              .arg(bestObstaclePoint.y(), 0, 'f', 2)
                              .arg(bestObstaclePoint.z(), 0, 'f', 2);
    }

    return result;
}

void PointCloudGeometry::setVoxelSize(float size)
{
    if (!std::isfinite(size) || size<0 || qFuzzyCompare(size,m_voxelSize)) return;
    m_voxelSize=size;rebuildGeometry();update();emit sceneScaleChanged();
}

void PointCloudGeometry::rebuildGeometry()
{
    clear();
    if (m_voxelSize>0) {
        // 八顶点索引立方体：直接显示原规划器发布的格心，不另建碰撞地图。
        const float half=m_voxelSize*m_sceneScale*.5f;
        QByteArray vertices,indices;
        vertices.resize(m_points.size()*8*3*sizeof(float));
        indices.resize(m_points.size()*36*sizeof(quint32));
        auto *v=reinterpret_cast<float*>(vertices.data());
        auto *idx=reinterpret_cast<quint32*>(indices.data());
        const quint32 faces[]={0,1,3,0,3,2,4,6,7,4,7,5,0,4,5,0,5,1,2,3,7,2,7,6,0,2,6,0,6,4,1,5,7,1,7,3};
        QVector3D lo(1e20f,1e20f,1e20f),hi(-1e20f,-1e20f,-1e20f);
        int visibleCount=0;
        for(int i=0;i<m_points.size();++i) {
            // 与点云按同一世界Z上限裁剪；格心仅在显示端过滤。
            if (m_ceilingClipEnabled && m_points[i].position.z()>m_ceilingHeight) continue;
            const auto c=pointToScene(m_points[i].position);
            for(int bit=0;bit<8;++bit) {
                const QVector3D p=c+QVector3D(bit&1?half:-half,bit&2?half:-half,bit&4?half:-half);
                *v++=p.x();*v++=p.y();*v++=p.z();
                for(int axis=0;axis<3;++axis){lo[axis]=std::min(lo[axis],p[axis]);hi[axis]=std::max(hi[axis],p[axis]);}
            }
            for(auto f:faces)*idx++=8*visibleCount+f;
            ++visibleCount;
        }
        m_visiblePointCount=visibleCount;
        vertices.resize(visibleCount*8*3*sizeof(float));
        indices.resize(visibleCount*36*sizeof(quint32));
        if(visibleCount==0){lo=QVector3D(-1,-1,-1);hi=QVector3D(1,1,1);}
        setVertexData(vertices);setIndexData(indices);setStride(3*sizeof(float));setBounds(lo,hi);
        setPrimitiveType(QQuick3DGeometry::PrimitiveType::Triangles);
        addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,0,QQuick3DGeometry::Attribute::F32Type);
        addAttribute(QQuick3DGeometry::Attribute::IndexSemantic,0,QQuick3DGeometry::Attribute::U32Type);
        return;
    }

    QByteArray vertexData;
    vertexData.resize(m_points.size() * VertexStride);
    float *cursor = reinterpret_cast<float *>(vertexData.data());

    QVector3D minScene(std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max());
    QVector3D maxScene(std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest());
    const float heightRange = std::max(0.001f, m_boundsMaxMeters.z() - m_boundsMinMeters.z());

    int visibleCount = 0;
    for (const PointCloudPoint &point : m_points) {
        if (m_ceilingClipEnabled && point.position.z() > m_ceilingHeight)
            continue;
        ++visibleCount;
        const QVector3D scenePoint = pointToScene(point.position);
        minScene.setX(std::min(minScene.x(), scenePoint.x()));
        minScene.setY(std::min(minScene.y(), scenePoint.y()));
        minScene.setZ(std::min(minScene.z(), scenePoint.z()));
        maxScene.setX(std::max(maxScene.x(), scenePoint.x()));
        maxScene.setY(std::max(maxScene.y(), scenePoint.y()));
        maxScene.setZ(std::max(maxScene.z(), scenePoint.z()));

        const float intensity = clampedIntensity(point.intensity);
        const float normalizedHeight = (point.position.z() - m_boundsMinMeters.z()) / heightRange;
        const QVector3D color = (m_rainbowStyle ? vividHeightColor(normalizedHeight) : heightColor(normalizedHeight)) * (0.72f + 0.28f * intensity);

        *cursor++ = scenePoint.x();
        *cursor++ = scenePoint.y();
        *cursor++ = scenePoint.z();
        // 按高度做类似点云建图软件的彩虹伪彩色：按当前点集的高度范围着色，不能作为障碍物类别判断。
        *cursor++ = std::clamp(color.x(), 0.0f, 1.0f);
        *cursor++ = std::clamp(color.y(), 0.0f, 1.0f);
        *cursor++ = std::clamp(color.z(), 0.0f, 1.0f);
        *cursor++ = 1.0f;
    }

    m_visiblePointCount = visibleCount;
    vertexData.resize(visibleCount * VertexStride);
    if (visibleCount == 0) {
        minScene = QVector3D(-1.0f, -1.0f, -1.0f);
        maxScene = QVector3D(1.0f, 1.0f, 1.0f);
    }

    setVertexData(vertexData);
    setStride(VertexStride);
    setBounds(minScene, maxScene);
    setPrimitiveType(QQuick3DGeometry::PrimitiveType::Points);
    addAttribute(QQuick3DGeometry::Attribute::PositionSemantic,
                 0,
                 QQuick3DGeometry::Attribute::F32Type);
    addAttribute(QQuick3DGeometry::Attribute::ColorSemantic,
                 3 * int(sizeof(float)),
                 QQuick3DGeometry::Attribute::F32Type);
}

QVector3D PointCloudGeometry::pointToScene(const QVector3D &meters) const
{
    // 雷达坐标 X 前/Y 左/Z 上 -> Qt Quick 3D 场景坐标 X 右/Y 上/Z 朝屏幕外。
    // ROS/MAVROS 的 +Y 表示左侧，而 Qt 场景的 +X 在屏幕右侧，因此这里必须取 -Y。
    // 取 sceneX = -Y、sceneY = Z、sceneZ = -X，可让前方空间向屏幕深处延伸，同时避免左右镜像。
    return QVector3D(-meters.y() * m_sceneScale,
                     meters.z() * m_sceneScale,
                     -meters.x() * m_sceneScale);
}

QVector3D PointCloudGeometry::sceneToPoint(const QVector3D &scenePosition) const
{
    return QVector3D(-scenePosition.z() / m_sceneScale,
                     -scenePosition.x() / m_sceneScale,
                     scenePosition.y() / m_sceneScale);
}
