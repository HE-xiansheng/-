// Copyright (C) 2020 The Qt Company Ltd.
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef EXAMPLEGEOMETRY_H
#define EXAMPLEGEOMETRY_H

#include <QQuick3DGeometry>

// Qt Quick 3D 自定义几何示例：三角形几何体。
//
// 作用：
// - 保留 Qt 示例中的最小自定义几何实现，用于验证 QQuick3DGeometry 的基本属性、
//   法线、UV 和顶点缓冲更新流程。
//
// 协作关系：
// - 当前点云地面站的主业务不依赖该类；真正的点云显示使用 PointCloudGeometry，
//   航线显示使用 RouteGeometry。
// - 如果后续确认不再需要 Qt 示例入口，可以单独移除 Main.qml 中相关演示引用和本类。
//! [triangle geometry]
class ExampleTriangleGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_NAMED_ELEMENT(ExampleTriangleGeometry)
    Q_PROPERTY(bool normals READ normals WRITE setNormals NOTIFY normalsChanged)
    Q_PROPERTY(float normalXY READ normalXY WRITE setNormalXY NOTIFY normalXYChanged)
    Q_PROPERTY(bool uv READ uv WRITE setUV NOTIFY uvChanged)
    Q_PROPERTY(float uvAdjust READ uvAdjust WRITE setUVAdjust NOTIFY uvAdjustChanged)

public:
    // 创建默认三角形几何，并初始化顶点缓冲。
    ExampleTriangleGeometry();

    // 是否写入顶点法线属性。
    bool normals() const { return m_hasNormals; }
    // 打开或关闭法线属性，并重建缓冲。
    void setNormals(bool enable);

    // 法线在 XY 平面的偏移量，用于演示材质光照变化。
    float normalXY() const { return m_normalXY; }
    // 修改法线偏移并重建缓冲。
    void setNormalXY(float xy);

    // 是否写入 UV 属性。
    bool uv() const { return m_hasUV; }
    // 打开或关闭 UV 属性，并重建缓冲。
    void setUV(bool enable);

    // UV 坐标调整量，用于演示纹理坐标变化。
    float uvAdjust() const { return m_uvAdjust; }
    // 修改 UV 坐标调整量并重建缓冲。
    void setUVAdjust(float f);

signals:
    void normalsChanged();
    void normalXYChanged();
    void uvChanged();
    void uvAdjustChanged();

private:
    // 根据 normals/uv 配置重新写入三角形顶点数据。
    void updateData();

    bool m_hasNormals = false;
    float m_normalXY = 0.0f;
    bool m_hasUV = false;
    float m_uvAdjust = 0.0f;
};
//! [triangle geometry]

// Qt Quick 3D 自定义点几何示例。
//
// 作用：
// - 演示 QQuick3DGeometry 如何渲染少量 point primitive。
//
// 协作关系：
// - 当前业务点云不使用该类；真实点云通过 PointCloudGeometry 批量写入点缓冲。
class ExamplePointGeometry : public QQuick3DGeometry
{
    Q_OBJECT
    QML_NAMED_ELEMENT(ExamplePointGeometry)

public:
    // 创建示例点几何。
    ExamplePointGeometry();

private:
    // 写入示例点数据。
    void updateData();
};

#endif
