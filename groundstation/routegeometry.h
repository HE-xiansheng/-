// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef ROUTEGEOMETRY_H
#define ROUTEGEOMETRY_H

#include <QQuick3DGeometry>
#include <QVariantList>
#include <QVector>
#include <QVector3D>
#include <QtQml/qqmlregistration.h>

// 航线几何体。
//
// 作用：
// - 保存用户规划的完整航点列表，并生成 Qt Quick 3D 可渲染的折线几何。
// - 支持 QML 追加起终点、拖动已有航点、在线段中插入避障折点。
//
// 协作关系：
// - Main.qml 在固定航点高度平面上生成自由空间航点，再调用 addWaypoint() 生成起终点。
// - Main.qml 拖拽航点或线段时调用 insertWaypoint()/setWaypoint() 调整避障路线。
// - RouteUdpSender 读取 waypointVector()，把完整米制航线一次性通过 UDP 发回模拟器或真实控制端。
//
// 坐标约定：
// - m_waypoints 保存米制雷达坐标，避免显示缩放影响控制坐标。
// - sceneWaypoints 是给 QML 航点手柄使用的场景坐标副本。
class RouteGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_NAMED_ELEMENT(RouteGeometry)
    Q_PROPERTY(QVariantList waypoints READ waypoints WRITE setWaypoints NOTIFY waypointsChanged)
    Q_PROPERTY(QVariantList sceneWaypoints READ sceneWaypoints NOTIFY waypointsChanged)
    Q_PROPERTY(int waypointCount READ waypointCount NOTIFY waypointsChanged)
    Q_PROPERTY(bool valid READ valid NOTIFY waypointsChanged)
    Q_PROPERTY(float sceneScale READ sceneScale WRITE setSceneScale NOTIFY sceneScaleChanged)

public:
    // 创建空航线几何体。通常由 QML RouteGeometry {} 实例化。
    explicit RouteGeometry(QQuick3DObject *parent = nullptr);

    // 返回 QVariantList 形式的米制航点，便于 QML 列表显示。
    QVariantList waypoints() const;
    // 用 QVariantList 整体替换航线，适合从 QML 或未来任务文件恢复航线。
    void setWaypoints(const QVariantList &points);

    // 返回场景坐标航点，专门供 QML 里的 3D 手柄和屏幕距离计算使用。
    QVariantList sceneWaypoints() const;
    // 当前航点数量。
    int waypointCount() const { return m_waypoints.size(); }
    // 至少包含两个航点时才认为航线有效，可以上传。
    bool valid() const { return m_waypoints.size() >= 2; }

    // 与 PointCloudGeometry 保持一致的场景缩放倍率。
    float sceneScale() const { return m_sceneScale; }
    // 修改场景缩放并重建折线几何，通常不需要用户手动调用。
    void setSceneScale(float scale);

    // 返回 C++ 直接可用的米制航点数组，RouteUdpSender 使用它编码 UDP。
    QVector<QVector3D> waypointVector() const { return m_waypoints; }

    // 清空整条航线，用户重新选择起终点时调用。
    Q_INVOKABLE void clearRoute();
    // 在航线末尾追加一个米制航点，用户短按空旷区域生成航点时调用。
    Q_INVOKABLE void addWaypoint(const QVector3D &pointMeters);
    // 在指定位置插入米制航点，拖动已有线段避障时调用。
    Q_INVOKABLE void insertWaypoint(int index, const QVector3D &pointMeters);
    // 修改指定航点坐标，拖动航点或新插入折点时调用。
    Q_INVOKABLE void setWaypoint(int index, const QVector3D &pointMeters);
    // 删除指定航点，预留给后续右键删除/任务编辑功能。
    Q_INVOKABLE void removeWaypoint(int index);
    // 读取指定米制航点；越界时返回空 QVector3D。
    Q_INVOKABLE QVector3D waypoint(int index) const;
    // 读取指定场景坐标航点，用于拖拽平面和屏幕命中计算。
    Q_INVOKABLE QVector3D sceneWaypoint(int index) const;
    // 米制坐标转场景坐标，保证航点贴合点云位置。
    Q_INVOKABLE QVector3D toScenePosition(const QVector3D &meters) const;
    // 场景坐标转米制坐标，拖拽完成后回写航线和 UDP 上传使用。
    Q_INVOKABLE QVector3D fromScenePosition(const QVector3D &scenePosition) const;

signals:
    void waypointsChanged();
    void sceneScaleChanged();

private:
    // 根据 m_waypoints 重建折线顶点缓冲。
    void rebuildGeometry();
    // 单点米制坐标 -> 3D 场景坐标，和 PointCloudGeometry 的转换保持一致。
    QVector3D pointToScene(const QVector3D &meters) const;
    // 单点 3D 场景坐标 -> 米制坐标。
    QVector3D sceneToPoint(const QVector3D &scenePosition) const;
    // 航线数据变化后的统一通知入口，负责重建几何并发出 QML 信号。
    void notifyRouteChanged();

    // 完整航点列表，单位：米。
    QVector<QVector3D> m_waypoints;
    // UI 场景放大倍率。
    float m_sceneScale = 45.0f;
};

#endif // ROUTEGEOMETRY_H
