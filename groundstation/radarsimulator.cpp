// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "applogger.h"
#include "pointcloudprotocol.h"

#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QNetworkDatagram>
#include <QTimer>
#include <QUdpSocket>
#include <QVector2D>
#include <QtMath>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace {

struct BoxObstacle
{
    QVector3D minPoint;
    QVector3D maxPoint;
    float intensity = 0.5f;
};

struct RectArea
{
    float minX = 0.0f;
    float maxX = 0.0f;
    float minY = 0.0f;
    float maxY = 0.0f;
};

struct WallSegment
{
    QVector2D start;
    QVector2D end;
    float minZ = 0.0f;
    float maxZ = 0.0f;
    float intensity = 0.85f;
};

struct RoundColumn
{
    QVector2D center;
    float radius = 0.2f;
    float minZ = 0.0f;
    float maxZ = 0.0f;
    float intensity = 0.75f;
};

struct ArcWall
{
    QVector2D center;
    float radius = 1.0f;
    float startRadians = 0.0f;
    float endRadians = 0.0f;
    float minZ = 0.0f;
    float maxZ = 0.0f;
    float intensity = 0.82f;
};

struct RayHit
{
    bool hit = false;
    float distance = std::numeric_limits<float>::max();
    float intensity = 1.0f;
};

constexpr float BlindZoneMeters = 0.10f;
constexpr float RoomMinX = -9.4f;
constexpr float RoomMaxX = 8.6f;
constexpr float RoomMinY = -6.8f;
constexpr float RoomMaxY = 5.8f;
constexpr float FloorZ = -1.05f;
constexpr float CeilingZ = 2.45f;

float rectArea(const RectArea &area)
{
    return std::max(0.0f, area.maxX - area.minX) * std::max(0.0f, area.maxY - area.minY);
}

float segmentLength(const WallSegment &wall)
{
    return (wall.end - wall.start).length();
}

const QVector<BoxObstacle> &obstacles()
{
    // 这些盒体模拟参考图里分散的桌子、设备柜、货架、矮箱和临时障碍物。
    // 坐标仍然使用雷达坐标系，单位：米；Z 方向越高，在地面站里越偏黄/红。
    static const QVector<BoxObstacle> boxes = {
        { QVector3D(-7.8f, -4.8f, FloorZ), QVector3D(-6.9f, -3.9f, 0.55f), 0.48f },
        { QVector3D(-6.2f, -2.7f, FloorZ), QVector3D(-5.0f, -1.9f, 0.82f), 0.58f },
        { QVector3D(-5.8f,  0.9f, FloorZ), QVector3D(-4.8f,  1.8f, 0.70f), 0.52f },
        { QVector3D(-4.6f,  3.1f, FloorZ), QVector3D(-3.2f,  4.2f, 1.05f), 0.64f },
        { QVector3D(-2.8f, -4.3f, FloorZ), QVector3D(-1.6f, -3.2f, 0.62f), 0.45f },
        { QVector3D(-1.1f, -2.6f, FloorZ), QVector3D(-0.1f, -1.6f, 1.18f), 0.72f },
        { QVector3D( 0.4f, -0.6f, FloorZ), QVector3D( 1.4f,  0.4f, 0.78f), 0.50f },
        { QVector3D( 1.2f,  1.2f, FloorZ), QVector3D( 2.2f,  2.2f, 1.45f), 0.76f },
        { QVector3D( 2.7f, -0.8f, FloorZ), QVector3D( 3.8f,  0.1f, 1.18f), 0.68f },
        { QVector3D( 3.1f,  1.2f, FloorZ), QVector3D( 4.1f,  2.3f, 0.70f), 0.46f },
        { QVector3D( 4.7f,  0.0f, FloorZ), QVector3D( 5.8f,  0.8f, 0.86f), 0.56f },
        { QVector3D( 5.0f,  2.8f, FloorZ), QVector3D( 6.4f,  4.0f, 1.10f), 0.62f },
        { QVector3D( 6.9f, -0.2f, FloorZ), QVector3D( 7.6f,  0.9f, 0.96f), 0.70f },
        { QVector3D( 6.5f,  4.3f, FloorZ), QVector3D( 7.8f,  5.1f, 1.35f), 0.78f },
        { QVector3D( 2.6f, -3.4f, FloorZ), QVector3D( 3.6f, -2.4f, 0.66f), 0.42f },
        { QVector3D( 4.2f, -3.0f, FloorZ), QVector3D( 5.3f, -2.0f, 0.92f), 0.58f },
        { QVector3D( 6.1f, -2.6f, FloorZ), QVector3D( 7.1f, -1.6f, 0.74f), 0.50f }
    };
    return boxes;
}

const QVector<RectArea> &floorAreas()
{
    // 多矩形拼成一个类似建图软件里的室内平面：左侧大开间、中间走廊、右侧房间、下方设备间。
    // 这些区域只负责“可通行地面”采样，墙体和隔断由 wallSegments 单独采样。
    static const QVector<RectArea> areas = {
        { -8.8f, -1.2f, -5.3f,  2.2f },
        { -6.8f,  1.1f,  2.0f,  5.2f },
        { -1.4f,  2.4f, -3.5f,  2.3f },
        {  2.2f,  8.2f, -1.4f,  5.4f },
        {  2.0f,  7.8f, -4.1f, -1.4f },
        { -8.5f, -5.0f, -6.4f, -5.1f }
    };
    return areas;
}

const QVector<WallSegment> &wallSegments()
{
    // 竖直墙体按线段定义，允许故意留出门洞；采样时再拉伸到 Z 高度。
    // 参考图里最醒目的红黄边缘主要来自这些墙体、隔断和门框的高处点。
    static const QVector<WallSegment> walls = {
        { QVector2D(-8.8f, -5.3f), QVector2D(-1.2f, -5.3f), FloorZ, CeilingZ, 0.88f },
        { QVector2D(-8.8f,  2.2f), QVector2D(-5.7f,  2.2f), FloorZ, CeilingZ, 0.86f },
        { QVector2D(-4.6f,  2.2f), QVector2D(-1.2f,  2.2f), FloorZ, CeilingZ, 0.86f },
        { QVector2D(-8.8f, -5.3f), QVector2D(-8.8f, -0.8f), FloorZ, CeilingZ, 0.84f },
        { QVector2D(-8.8f,  0.5f), QVector2D(-8.8f,  2.2f), FloorZ, CeilingZ, 0.84f },
        { QVector2D(-1.2f, -5.3f), QVector2D(-1.2f, -1.8f), FloorZ, CeilingZ, 0.88f },
        { QVector2D(-1.2f, -0.7f), QVector2D(-1.2f,  2.2f), FloorZ, CeilingZ, 0.88f },

        { QVector2D(-6.8f,  5.2f), QVector2D( 1.1f,  5.2f), FloorZ, CeilingZ, 0.86f },
        { QVector2D(-6.8f,  2.0f), QVector2D(-6.8f,  5.2f), FloorZ, CeilingZ, 0.82f },
        { QVector2D( 1.1f,  2.0f), QVector2D( 1.1f,  5.2f), FloorZ, CeilingZ, 0.88f },

        { QVector2D(-1.4f, -3.5f), QVector2D( 2.4f, -3.5f), FloorZ, CeilingZ, 0.85f },
        { QVector2D(-1.4f,  2.3f), QVector2D( 0.1f,  2.3f), FloorZ, CeilingZ, 0.82f },
        { QVector2D( 1.0f,  2.3f), QVector2D( 2.4f,  2.3f), FloorZ, CeilingZ, 0.82f },
        { QVector2D( 2.4f, -3.5f), QVector2D( 2.4f, -1.5f), FloorZ, CeilingZ, 0.86f },
        { QVector2D( 2.4f, -0.3f), QVector2D( 2.4f,  2.3f), FloorZ, CeilingZ, 0.86f },

        { QVector2D( 2.2f, -1.4f), QVector2D( 8.2f, -1.4f), FloorZ, CeilingZ, 0.90f },
        { QVector2D( 2.2f,  5.4f), QVector2D( 8.2f,  5.4f), FloorZ, CeilingZ, 0.90f },
        { QVector2D( 8.2f, -1.4f), QVector2D( 8.2f,  5.4f), FloorZ, CeilingZ, 0.92f },
        { QVector2D( 2.2f, -1.4f), QVector2D( 2.2f,  0.8f), FloorZ, CeilingZ, 0.88f },
        { QVector2D( 2.2f,  1.8f), QVector2D( 2.2f,  5.4f), FloorZ, CeilingZ, 0.88f },

        { QVector2D( 2.0f, -4.1f), QVector2D( 7.8f, -4.1f), FloorZ, CeilingZ, 0.84f },
        { QVector2D( 7.8f, -4.1f), QVector2D( 7.8f, -1.4f), FloorZ, CeilingZ, 0.84f },
        { QVector2D( 3.4f, -1.4f), QVector2D( 7.8f, -1.4f), FloorZ, CeilingZ, 0.84f },

        // 两条斜向隔断用于打破“方盒子感”，让视觉上更接近参考图中右侧房间的斜墙/通道。
        { QVector2D( 0.4f, -2.9f), QVector2D( 2.5f, -0.9f), FloorZ, CeilingZ, 0.89f },
        { QVector2D( 2.9f,  0.0f), QVector2D( 5.1f,  2.5f), FloorZ, CeilingZ, 0.90f },
        { QVector2D( 4.6f, -0.9f), QVector2D( 6.8f,  1.4f), FloorZ, CeilingZ, 0.87f }
    };
    return walls;
}

const QVector<RoundColumn> &roundColumns()
{
    static const QVector<RoundColumn> columns = {
        { QVector2D(-7.3f,  1.2f), 0.22f, FloorZ, 1.85f, 0.74f },
        { QVector2D(-3.1f, -1.2f), 0.18f, FloorZ, 1.55f, 0.70f },
        { QVector2D( 1.0f,  3.3f), 0.20f, FloorZ, 1.70f, 0.76f },
        { QVector2D( 6.9f,  3.5f), 0.18f, FloorZ, 1.60f, 0.72f }
    };
    return columns;
}

const QVector<ArcWall> &arcWalls()
{
    // 左侧弧形墙/围挡用来模拟参考图中左上区域的弯曲轮廓。
    static const QVector<ArcWall> arcs = {
        { QVector2D(-7.0f, -2.7f), 1.55f, qDegreesToRadians(88.0f), qDegreesToRadians(255.0f), FloorZ, 1.75f, 0.82f },
        { QVector2D(-6.9f, -2.7f), 0.85f, qDegreesToRadians(95.0f), qDegreesToRadians(250.0f), FloorZ, 0.95f, 0.66f }
    };
    return arcs;
}

void considerHit(float distance, float intensity, RayHit *best)
{
    if (!best || distance < BlindZoneMeters || distance >= best->distance)
        return;

    best->hit = true;
    best->distance = distance;
    best->intensity = intensity;
}

void intersectPlaneX(const QVector3D &direction, float x, float maxRange, float intensity, RayHit *best)
{
    if (qAbs(direction.x()) < 0.0001f)
        return;

    const float distance = x / direction.x();
    if (distance <= BlindZoneMeters || distance > maxRange)
        return;

    const QVector3D point = direction * distance;
    if (point.y() >= RoomMinY && point.y() <= RoomMaxY && point.z() >= FloorZ && point.z() <= CeilingZ)
        considerHit(distance, intensity, best);
}

void intersectPlaneY(const QVector3D &direction, float y, float maxRange, float intensity, RayHit *best)
{
    if (qAbs(direction.y()) < 0.0001f)
        return;

    const float distance = y / direction.y();
    if (distance <= BlindZoneMeters || distance > maxRange)
        return;

    const QVector3D point = direction * distance;
    if (point.x() >= RoomMinX && point.x() <= RoomMaxX && point.z() >= FloorZ && point.z() <= CeilingZ)
        considerHit(distance, intensity, best);
}

void intersectPlaneZ(const QVector3D &direction, float z, float maxRange, float intensity, RayHit *best)
{
    if (qAbs(direction.z()) < 0.0001f)
        return;

    const float distance = z / direction.z();
    if (distance <= BlindZoneMeters || distance > maxRange)
        return;

    const QVector3D point = direction * distance;
    if (point.x() >= RoomMinX && point.x() <= RoomMaxX && point.y() >= RoomMinY && point.y() <= RoomMaxY)
        considerHit(distance, intensity, best);
}

void intersectBox(const QVector3D &direction, const BoxObstacle &box, float maxRange, RayHit *best)
{
    float tMin = BlindZoneMeters;
    float tMax = maxRange;

    for (int axis = 0; axis < 3; ++axis) {
        const float d = direction[axis];
        const float minValue = box.minPoint[axis];
        const float maxValue = box.maxPoint[axis];
        if (qAbs(d) < 0.0001f) {
            if (0.0f < minValue || 0.0f > maxValue)
                return;
            continue;
        }

        float t1 = minValue / d;
        float t2 = maxValue / d;
        if (t1 > t2)
            std::swap(t1, t2);
        tMin = std::max(tMin, t1);
        tMax = std::min(tMax, t2);
        if (tMin > tMax)
            return;
    }

    considerHit(tMin, box.intensity, best);
}

RayHit traceIndoorScene(const QVector3D &direction, float maxRange)
{
    RayHit best;

    // 墙面反射率较高、地面偏低，模拟白墙/混凝土地面的强度差异。
    intersectPlaneX(direction, RoomMinX, maxRange, 0.88f, &best);
    intersectPlaneX(direction, RoomMaxX, maxRange, 0.88f, &best);
    intersectPlaneY(direction, RoomMinY, maxRange, 0.82f, &best);
    intersectPlaneY(direction, RoomMaxY, maxRange, 0.82f, &best);
    intersectPlaneZ(direction, FloorZ, maxRange, 0.28f, &best);
    intersectPlaneZ(direction, CeilingZ, maxRange, 0.70f, &best);

    for (const BoxObstacle &box : obstacles())
        intersectBox(direction, box, maxRange, &best);

    return best;
}

float fractionalPart(float value)
{
    return value - std::floor(value);
}

float randomBetween(std::mt19937 &rng, float minValue, float maxValue)
{
    std::uniform_real_distribution<float> distribution(minValue, maxValue);
    return distribution(rng);
}

const RectArea &randomFloorArea(std::mt19937 &rng)
{
    const QVector<RectArea> &areas = floorAreas();
    float totalArea = 0.0f;
    for (const RectArea &area : areas)
        totalArea += rectArea(area);

    float cursor = randomBetween(rng, 0.0f, totalArea);
    for (const RectArea &area : areas) {
        cursor -= rectArea(area);
        if (cursor <= 0.0f)
            return area;
    }
    return areas.last();
}

const WallSegment &randomWallSegment(std::mt19937 &rng)
{
    const QVector<WallSegment> &walls = wallSegments();
    float totalLength = 0.0f;
    for (const WallSegment &wall : walls)
        totalLength += segmentLength(wall);

    float cursor = randomBetween(rng, 0.0f, totalLength);
    for (const WallSegment &wall : walls) {
        cursor -= segmentLength(wall);
        if (cursor <= 0.0f)
            return wall;
    }
    return walls.last();
}

PointCloudPoint makeSurfacePoint(const QVector3D &position,
                                 float intensity,
                                 std::mt19937 &rng,
                                 std::normal_distribution<float> &positionNoise,
                                 std::normal_distribution<float> &intensityNoise)
{
    PointCloudPoint point;
    point.position = QVector3D(position.x() + positionNoise(rng),
                               position.y() + positionNoise(rng),
                               position.z() + positionNoise(rng));
    point.intensity = std::clamp(intensity + intensityNoise(rng), 0.05f, 1.0f);
    return point;
}

QVector3D randomFloorPoint(std::mt19937 &rng)
{
    const RectArea &area = randomFloorArea(rng);
    return QVector3D(randomBetween(rng, area.minX, area.maxX),
                     randomBetween(rng, area.minY, area.maxY),
                     FloorZ);
}

QVector3D randomCeilingPoint(std::mt19937 &rng)
{
    const RectArea &area = randomFloorArea(rng);
    return QVector3D(randomBetween(rng, area.minX, area.maxX),
                     randomBetween(rng, area.minY, area.maxY),
                     CeilingZ);
}

QVector3D randomWallPoint(const WallSegment &wall, std::mt19937 &rng)
{
    const float t = randomBetween(rng, 0.0f, 1.0f);
    const QVector2D xy = wall.start * (1.0f - t) + wall.end * t;
    return QVector3D(xy.x(),
                     xy.y(),
                     randomBetween(rng, wall.minZ, wall.maxZ));
}

QVector3D randomColumnPoint(std::mt19937 &rng, float *intensity)
{
    const QVector<RoundColumn> &columns = roundColumns();
    const RoundColumn &column = columns.at(std::uniform_int_distribution<int>(0, int(columns.size()) - 1)(rng));
    const float angle = randomBetween(rng, 0.0f, 6.2831853071795864769f);
    if (intensity)
        *intensity = column.intensity;
    return QVector3D(column.center.x() + std::cos(angle) * column.radius,
                     column.center.y() + std::sin(angle) * column.radius,
                     randomBetween(rng, column.minZ, column.maxZ));
}

QVector3D randomArcWallPoint(std::mt19937 &rng, float *intensity)
{
    const QVector<ArcWall> &arcs = arcWalls();
    const ArcWall &arc = arcs.at(std::uniform_int_distribution<int>(0, int(arcs.size()) - 1)(rng));
    const float angle = randomBetween(rng, arc.startRadians, arc.endRadians);
    if (intensity)
        *intensity = arc.intensity;
    return QVector3D(arc.center.x() + std::cos(angle) * arc.radius,
                     arc.center.y() + std::sin(angle) * arc.radius,
                     randomBetween(rng, arc.minZ, arc.maxZ));
}

QVector3D randomPointOnBoxFace(const BoxObstacle &box, std::mt19937 &rng)
{
    const int face = std::uniform_int_distribution<int>(0, 5)(rng);
    const float x = randomBetween(rng, box.minPoint.x(), box.maxPoint.x());
    const float y = randomBetween(rng, box.minPoint.y(), box.maxPoint.y());
    const float z = randomBetween(rng, box.minPoint.z(), box.maxPoint.z());

    switch (face) {
    case 0:
        return QVector3D(box.minPoint.x(), y, z);
    case 1:
        return QVector3D(box.maxPoint.x(), y, z);
    case 2:
        return QVector3D(x, box.minPoint.y(), z);
    case 3:
        return QVector3D(x, box.maxPoint.y(), z);
    case 4:
        return QVector3D(x, y, box.minPoint.z());
    default:
        return QVector3D(x, y, box.maxPoint.z());
    }
}

void appendIndoorSurfaceSample(QVector<PointCloudPoint> *points,
                               std::mt19937 &rng,
                               std::normal_distribution<float> &positionNoise,
                               std::normal_distribution<float> &intensityNoise)
{
    if (!points)
        return;

    const float surfaceChoice = randomBetween(rng, 0.0f, 1.0f);
    QVector3D position;
    float intensity = 0.8f;

    if (surfaceChoice < 0.48f) {
        // 地面占比最高。地面站按高度伪彩着色后，地面会形成参考图里的蓝青色可通行区域。
        position = randomFloorPoint(rng);
        intensity = 0.38f;
    } else if (surfaceChoice < 0.70f) {
        const WallSegment &wall = randomWallSegment(rng);
        position = randomWallPoint(wall, rng);
        intensity = wall.intensity;
    } else if (surfaceChoice < 0.77f) {
        // 右侧房间、走廊附近的小立柱让地图不再只是平墙和方盒。
        position = randomColumnPoint(rng, &intensity);
    } else if (surfaceChoice < 0.83f) {
        // 弧形墙面可以形成类似参考图左侧区域的弯曲轮廓。
        position = randomArcWallPoint(rng, &intensity);
    } else if (surfaceChoice < 0.96f) {
        const QVector<BoxObstacle> &boxes = obstacles();
        const int boxIndex = std::uniform_int_distribution<int>(0, int(boxes.size()) - 1)(rng);
        const BoxObstacle &box = boxes.at(boxIndex);
        position = randomPointOnBoxFace(box, rng);
        intensity = box.intensity;
    } else {
        // 少量天花板/高处点让高度范围稳定，伪彩色能自然出现绿黄红层次。
        position = randomCeilingPoint(rng);
        intensity = 0.70f;
    }

    points->append(makeSurfacePoint(position, intensity, rng, positionNoise, intensityNoise));
}

QVector<PointCloudPoint> generateMid360LikeFrame(int requestedPoints,
                                                 quint32 frameSequence,
                                                 float maxRangeMeters)
{
    // MID-360/MID-360S 系列的关键观感：360 度水平视场、约 59 度垂直视场、
    // 非传统机械线扫的非重复覆盖。真实建图软件通常显示的是一段时间累计后的地图，
    // 而不是单个瞬时包，所以这里把“少量实时射线命中”和“大量固定室内地图抽样”结合起来。
    static std::mt19937 rng(std::random_device{}());
    static std::normal_distribution<float> rangeNoise(0.0f, 0.018f);
    static std::normal_distribution<float> surfacePositionNoise(0.0f, 0.012f);
    static std::normal_distribution<float> intensityNoise(0.0f, 0.035f);

    constexpr float TwoPi = 6.2831853071795864769f;
    constexpr float GoldenAngle = 2.39996322972865332f;
    const float minElevation = qDegreesToRadians(-7.0f);
    const float maxElevation = qDegreesToRadians(52.0f);
    const float elevationSpan = maxElevation - minElevation;
    const float framePhase = float(frameSequence) * 0.071f;
    const int rayPointBudget = std::max(1, requestedPoints * 8 / 100);

    QVector<PointCloudPoint> points;
    points.reserve(requestedPoints);
    for (int i = 0; i < rayPointBudget; ++i) {
        const float azimuth = std::fmod(framePhase + float(i) * GoldenAngle, TwoPi);
        const float verticalSeed = fractionalPart(float(i) * 0.754877666f
                                                  + float(frameSequence) * 0.019f
                                                  + 0.035f * std::sin(float(i) * 0.13f));
        const float elevation = minElevation + elevationSpan * verticalSeed;
        const float cosElevation = std::cos(elevation);
        const QVector3D direction(cosElevation * std::cos(azimuth),
                                  cosElevation * std::sin(azimuth),
                                  std::sin(elevation));

        const RayHit hit = traceIndoorScene(direction, maxRangeMeters);
        if (!hit.hit)
            continue;

        const float noisyDistance = std::max(BlindZoneMeters, hit.distance + rangeNoise(rng));
        PointCloudPoint point;
        point.position = direction * noisyDistance;
        point.intensity = std::clamp(hit.intensity + intensityNoise(rng), 0.05f, 1.0f);
        points.append(point);
    }

    // 地面站实时 UDP 模式显示的是“最新完整帧”，不是长时间自动累计地图。
    // 为了让模拟器更像参考图里的建图结果，每一帧都从同一个室内模型中抽样完整地图。
    while (points.size() < requestedPoints)
        appendIndoorSurfaceSample(&points, rng, surfacePositionNoise, intensityNoise);

    return points;
}

void sendPointCloudFrame(QUdpSocket *socket,
                         const QHostAddress &targetHost,
                         quint16 targetPort,
                         const QVector<PointCloudPoint> &points,
                         quint32 sequence)
{
    const int chunkSize = PointCloudProtocol::MaxPointsPerDatagram;
    const int pointCount = int(points.size());
    const int chunkCount = std::max(1, (pointCount + chunkSize - 1) / chunkSize);
    const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;

    for (int chunk = 0; chunk < chunkCount; ++chunk) {
        const qsizetype offset = qsizetype(chunk) * chunkSize;
        const qsizetype count = std::min<qsizetype>(chunkSize, points.size() - offset);
        const QByteArray datagram = PointCloudProtocol::encodePointCloudChunk(sequence,
                                                                              quint16(chunk),
                                                                              quint16(chunkCount),
                                                                              quint32(points.size()),
                                                                              timestampUsec,
                                                                              points,
                                                                              offset,
                                                                              count);
        const qint64 written = socket->writeDatagram(datagram, targetHost, targetPort);
        if (written != datagram.size()) {
            qCritical().noquote() << QStringLiteral("点云 UDP 分片发送失败：sequence=%1，chunk=%2/%3，目标=%4:%5，期望=%6，实际=%7，错误=%8")
                                        .arg(sequence)
                                        .arg(chunk + 1)
                                        .arg(chunkCount)
                                        .arg(targetHost.toString())
                                        .arg(targetPort)
                                        .arg(datagram.size())
                                        .arg(written)
                                        .arg(socket->errorString());
        }
    }
}

void readRouteDatagrams(QUdpSocket *routeSocket)
{
    static int invalidRouteDatagrams = 0;
    while (routeSocket->hasPendingDatagrams()) {
        const QNetworkDatagram datagram = routeSocket->receiveDatagram();
        RoutePacket route;
        if (!PointCloudProtocol::decodeRoute(datagram.data(), &route)) {
            ++invalidRouteDatagrams;
            if (invalidRouteDatagrams <= 5 || invalidRouteDatagrams % 100 == 0) {
                qWarning().noquote() << QStringLiteral("收到无法解析的航线 UDP 包：累计=%1，大小=%2，来源=%3:%4")
                                            .arg(invalidRouteDatagrams)
                                            .arg(datagram.data().size())
                                            .arg(datagram.senderAddress().toString())
                                            .arg(datagram.senderPort());
            }
            continue;
        }

        qInfo().noquote() << QStringLiteral("收到航线 %1，航点数：%2")
                             .arg(route.sequence)
                             .arg(route.waypoints.size());
        for (int i = 0; i < route.waypoints.size(); ++i) {
            const QVector3D &point = route.waypoints.at(i);
            qInfo().noquote() << QStringLiteral("  #%1  X=%2 m  Y=%3 m  Z=%4 m")
                                 .arg(i + 1)
                                 .arg(point.x(), 0, 'f', 2)
                                 .arg(point.y(), 0, 'f', 2)
                                 .arg(point.z(), 0, 'f', 2);
        }
    }
}

} // namespace

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    QCoreApplication::setApplicationName(QStringLiteral("radarsimulator"));
    QCoreApplication::setApplicationVersion(QStringLiteral("1.0"));
    const QString logPath = AppLogger::initialize(QStringLiteral("radarsimulator"));
    if (!logPath.isEmpty())
        qInfo().noquote() << QStringLiteral("日志文件：%1").arg(logPath);

    QCommandLineParser parser;
    parser.setApplicationDescription(QStringLiteral("Livox MID-360 风格室内点云 UDP 模拟器"));
    parser.addHelpOption();
    parser.addVersionOption();

    QCommandLineOption hostOption(QStringList() << QStringLiteral("H") << QStringLiteral("host"),
                                  QStringLiteral("地面站点云接收地址。"),
                                  QStringLiteral("host"),
                                  QStringLiteral("127.0.0.1"));
    QCommandLineOption pointPortOption(QStringList() << QStringLiteral("p") << QStringLiteral("point-port"),
                                       QStringLiteral("地面站点云 UDP 端口。"),
                                       QStringLiteral("port"),
                                       QStringLiteral("56000"));
    QCommandLineOption routePortOption(QStringList() << QStringLiteral("r") << QStringLiteral("route-port"),
                                       QStringLiteral("模拟器监听航线回传的 UDP 端口。"),
                                       QStringLiteral("port"),
                                       QStringLiteral("56001"));
    QCommandLineOption hzOption(QStringList() << QStringLiteral("hz"),
                                QStringLiteral("点云帧率。默认 5Hz，用更密单帧模拟建图结果；需要原始 10Hz 实时流可手动指定。"),
                                QStringLiteral("hz"),
                                QStringLiteral("5"));
    QCommandLineOption pointsOption(QStringList() << QStringLiteral("n") << QStringLiteral("points"),
                                    QStringLiteral("每帧点数。默认 40000，配合 5Hz 仍是 200000 点/秒，但画面更接近完整建图点云。"),
                                    QStringLiteral("count"),
                                    QStringLiteral("40000"));
    QCommandLineOption rangeOption(QStringList() << QStringLiteral("range"),
                                   QStringLiteral("最大模拟测距，单位米。"),
                                   QStringLiteral("meters"),
                                   QStringLiteral("18"));
    parser.addOptions({ hostOption, pointPortOption, routePortOption, hzOption, pointsOption, rangeOption });
    parser.process(app);

    QHostAddress targetHost(parser.value(hostOption));
    if (targetHost.isNull()) {
        qWarning().noquote() << QStringLiteral("点云目标地址解析失败，回退到 127.0.0.1：%1").arg(parser.value(hostOption));
        targetHost = QHostAddress::LocalHost;
    }
    const quint16 pointPort = quint16(std::clamp(parser.value(pointPortOption).toInt(), 1, 65535));
    const quint16 routePort = quint16(std::clamp(parser.value(routePortOption).toInt(), 1, 65535));
    const int hz = std::clamp(parser.value(hzOption).toInt(), 1, 30);
    const int pointsPerFrame = std::clamp(parser.value(pointsOption).toInt(), 1000, 120000);
    const float maxRangeMeters = std::clamp(parser.value(rangeOption).toFloat(), 3.0f, 70.0f);

    QUdpSocket pointSocket;
    QUdpSocket routeSocket;
    if (!routeSocket.bind(QHostAddress::AnyIPv4,
                          routePort,
                          QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        qCritical().noquote() << QStringLiteral("航线回传端口 %1 绑定失败：%2")
                                 .arg(routePort)
                                 .arg(routeSocket.errorString());
        return 1;
    }

    QObject::connect(&routeSocket, &QUdpSocket::readyRead, [&routeSocket]() {
        readRouteDatagrams(&routeSocket);
    });

    qInfo().noquote() << QStringLiteral("雷达模拟器已启动：点云 -> %1:%2，航线监听 UDP %3，%4 Hz，%5 点/帧，约 %6 点/秒")
                         .arg(targetHost.toString())
                         .arg(pointPort)
                         .arg(routePort)
                         .arg(hz)
                         .arg(pointsPerFrame)
                         .arg(pointsPerFrame * hz);

    quint32 sequence = 0;
    QTimer timer;
    timer.setTimerType(Qt::PreciseTimer);
    QObject::connect(&timer, &QTimer::timeout, [&]() {
        const QVector<PointCloudPoint> points = generateMid360LikeFrame(pointsPerFrame,
                                                                        sequence,
                                                                        maxRangeMeters);
        sendPointCloudFrame(&pointSocket, targetHost, pointPort, points, sequence);
        if (sequence % quint32(hz) == 0) {
            qInfo().noquote() << QStringLiteral("发送点云帧 %1：%2 点").arg(sequence).arg(points.size());
        }
        ++sequence;
    });
    timer.start(std::max(1, 1000 / hz));

    return app.exec();
}
