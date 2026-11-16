// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef POINTCLOUDGEOMETRY_H
#define POINTCLOUDGEOMETRY_H

#include "pointcloudprotocol.h"

#include <QQuick3DGeometry>
#include <QVariantMap>
#include <QVector3D>
#include <QtQml/qqmlregistration.h>

// 点云渲染几何体。
//
// 作用：
// - 保存当前正在显示的点云数据，并把点云转换成 Qt Quick 3D 可渲染的 GPU 顶点缓冲。
// - 对 QML 暴露点数、帧序号、包围盒和射线拾取能力，支撑 UI 里的状态显示和点击选点。
//
// 协作关系：
// - UdpPointCloudReceiver 收到实时 UDP 点云后调用 setPoints() 刷新这里的点集。
// - PointCloudFileLoader 加载 LVX2/PCD/PLY/CSV 等静态模型后也调用 setPoints() 复用同一套渲染和拾取。
// - RouteGeometry 使用相同 sceneScale，把用户选中的米制坐标显示成航线。
//
// 坐标约定：
// - 内部 m_points 始终保存雷达米制坐标：X 前、Y 左、Z 上。
// - 写入 3D 场景时通过 sceneScale 放大，并映射到 Qt Quick 3D 坐标。
// - 航线回传时仍使用米制坐标，避免 UI 缩放影响真实控制数据。
class PointCloudGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_NAMED_ELEMENT(PointCloudGeometry)
    Q_PROPERTY(float voxelSize READ voxelSize WRITE setVoxelSize NOTIFY sceneScaleChanged)
    Q_PROPERTY(int pointCount READ pointCount NOTIFY pointCloudChanged)
    Q_PROPERTY(int lastSequence READ lastSequence NOTIFY pointCloudChanged)
    Q_PROPERTY(QVector3D boundsMinMeters READ boundsMinMeters NOTIFY pointCloudChanged)
    Q_PROPERTY(QVector3D boundsMaxMeters READ boundsMaxMeters NOTIFY pointCloudChanged)
    Q_PROPERTY(float sceneScale READ sceneScale WRITE setSceneScale NOTIFY sceneScaleChanged)
    Q_PROPERTY(bool rainbowStyle READ rainbowStyle WRITE setRainbowStyle NOTIFY rainbowStyleChanged)
    Q_PROPERTY(bool ceilingClipEnabled READ ceilingClipEnabled WRITE setCeilingClipEnabled NOTIFY ceilingClipChanged)
    Q_PROPERTY(float ceilingHeight READ ceilingHeight WRITE setCeilingHeight NOTIFY ceilingClipChanged)
    Q_PROPERTY(int visiblePointCount READ visiblePointCount NOTIFY pointCloudChanged)

public:
    // 创建一个空点云几何体。QML 中通常由 PointCloudGeometry {} 直接实例化。
    explicit PointCloudGeometry(QQuick3DObject *parent = nullptr);

    // 当前可见点数，用于状态栏显示和判断是否可以选点。
    int pointCount() const { return m_points.size(); }
    int visiblePointCount() const { return m_visiblePointCount; }
    // 只裁剪渲染顶点；原点集、障碍查询和定位坐标不变。
    bool ceilingClipEnabled() const { return m_ceilingClipEnabled; }
    float ceilingHeight() const { return m_ceilingHeight; }
    void setCeilingClipEnabled(bool enabled);
    void setCeilingHeight(float height);
    // 最近一次实时 UDP 点云帧序号；静态模型通常传 0。
    int lastSequence() const { return int(m_lastSequence); }
    // 点云包围盒最小角，单位：米，用于调试和后续自动视角适配。
    QVector3D boundsMinMeters() const { return m_boundsMinMeters; }
    // 点云包围盒最大角，单位：米，用于调试和后续自动视角适配。
    QVector3D boundsMaxMeters() const { return m_boundsMaxMeters; }

    // 场景缩放比例。1 米会被渲染为 sceneScale 个 3D 场景单位。
    float sceneScale() const { return m_sceneScale; }
    // 可切换更鲜明的彩虹高度伪彩色；只改变显示，不改变点坐标。
    bool rainbowStyle() const { return m_rainbowStyle; }
    void setRainbowStyle(bool enabled);
    // 修改场景缩放，并重建 GPU 顶点缓冲，保证点云和航线比例一致。
    void setSceneScale(float scale);
    // 0保留原点云；>0以原生分辨率显示占据体素，仅用于显示。
    float voxelSize() const {return m_voxelSize;}
    void setVoxelSize(float size);

    // 替换当前点云。调用方负责完成 UDP 重组、文件抽稀或时间窗筛选。
    void setPoints(const QVector<PointCloudPoint> &points, quint32 sequence, quint64 timestampUsec);

    // 把米制雷达坐标转换为 Qt Quick 3D 场景坐标，QML 航点手柄定位会用到。
    Q_INVOKABLE QVector3D toScenePosition(const QVector3D &meters) const;
    // 把 Qt Quick 3D 场景坐标转换回米制雷达坐标，拖拽航点后回写航线会用到。
    Q_INVOKABLE QVector3D fromScenePosition(const QVector3D &scenePosition) const;
    // 在当前点云里查找距离屏幕点击射线最近的点。
    // 返回 QVariantMap，包含 hit/index/distance/rayDistance/position，便于 QML 直接判断和取坐标。
    Q_INVOKABLE QVariantMap nearestPointOnRay(const QVector3D &rayOriginScene,
                                              const QVector3D &rayDirectionScene,
                                              float maxPointDistanceScene = 22.0f,
                                              float maxRayDistanceScene = 5000.0f) const;
    // 检查候选航点附近是否有障碍物点。
    // 点云在航线规划中被视为障碍物地图；QML 在固定飞行高度平面上生成候选航点后，
    // 调用本函数判断水平半径和垂向半径内是否存在点云障碍。
    Q_INVOKABLE QVariantMap nearestObstacleAround(const QVector3D &pointMeters,
                                                  float horizontalRadiusMeters = 0.45f,
                                                  float verticalRadiusMeters = 0.45f) const;
    // 检查一段航线附近是否有障碍物点。
    // 用于“上传航线”前的最终校验：航点本身可能安全，但连接线段仍可能穿过墙体/家具。
    Q_INVOKABLE QVariantMap nearestObstacleOnSegment(const QVector3D &startMeters,
                                                     const QVector3D &endMeters,
                                                     float horizontalRadiusMeters = 0.45f,
                                                     float verticalRadiusMeters = 0.45f) const;

signals:
    void pointCloudChanged();
    void sceneScaleChanged();
    void rainbowStyleChanged();
    void ceilingClipChanged();

private:
    // 根据 m_points 重建 Qt Quick 3D 需要的顶点缓冲、属性布局和包围盒。
    void rebuildGeometry();
    // 单点米制坐标 -> 3D 场景坐标，和 RouteGeometry 的转换保持一致。
    QVector3D pointToScene(const QVector3D &meters) const;
    // 单点 3D 场景坐标 -> 米制坐标，主要给拾取和航线拖拽使用。
    QVector3D sceneToPoint(const QVector3D &scenePosition) const;

    // 当前显示点集，可能是实时最新帧、实时累计帧、静态完整地图或 LVX2 时间窗。
    QVector<PointCloudPoint> m_points;
    // 最近一次点云序号和时间戳，用于日志、状态栏和问题追踪。
    quint32 m_lastSequence = 0;
    quint64 m_lastTimestampUsec = 0;
    // 当前点云的米制包围盒。
    QVector3D m_boundsMinMeters;
    QVector3D m_boundsMaxMeters;
    // UI 场景放大倍率。
    float m_sceneScale = 45.0f;
    float m_voxelSize = 0.0f;
    bool m_rainbowStyle = true;
    bool m_ceilingClipEnabled = false;
    float m_ceilingHeight = 2.5f;
    int m_visiblePointCount = 0;
};

#endif // POINTCLOUDGEOMETRY_H
