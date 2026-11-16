// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#ifndef POINTCLOUDFILELOADER_H
#define POINTCLOUDFILELOADER_H

#include "pointcloudgeometry.h"

#include <QObject>
#include <QUrl>
#include <QVariantList>
#include <QtQml/qqmlregistration.h>

#include <atomic>
#include <memory>

// 录制文件中的位姿样本。
//
// 作用：
// - ROS bag 中的 geometry_msgs/PoseStamped 会被转成这个轻量结构。
// - PointCloudFileLoader 按播放进度选择最近样本，并通过 QML 属性驱动 3D 位姿 marker。
// - 实时 UDP POS1 仍由 UdpPoseReceiver 负责；这个结构只用于离线录制回放。
struct PointCloudPlaybackPoseSample
{
    int timeMs = 0;
    QVector3D positionMeters;
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    float yawDegrees = 0.0f;
};

// 静态点云文件加载器。
//
// 用于加载雷达已经建好的离线模型，加载后仍然复用 PointCloudGeometry 的点击拾取和航线规划能力。
// 这样实时 UDP 点云和静态地图走同一套渲染、拾取、航线编辑与上传链路，后续接真实 Livox
// 建图结果时，不需要再为“离线地图选点”写第二套交互。
//
// 坐标约定：
// - 文件中的 x/y/z 按米制雷达坐标解释：X 前、Y 左、Z 上。
// - intensity/reflectivity/RGB 会归一化到 0.0 ~ 1.0，用于渲染亮度辅助。
// - maxPoints > 0 时按上限抽稀；maxPoints == 0 时加载全部有效点。
//
// 当前支持常见的 LVX2、ROS bag、PLY、PCD、XYZ/TXT/CSV 点列表；
// 真实项目可继续扩展 LAS/LAZ/E57 适配器。
class PointCloudFileLoader : public QObject
{
    Q_OBJECT
    QML_NAMED_ELEMENT(PointCloudFileLoader)
    Q_PROPERTY(PointCloudGeometry *geometry READ geometry WRITE setGeometry NOTIFY geometryChanged)
    Q_PROPERTY(PointCloudGeometry *previewGeometry READ previewGeometry WRITE setPreviewGeometry NOTIFY previewGeometryChanged)
    Q_PROPERTY(int maxPoints READ maxPoints WRITE setMaxPoints NOTIFY maxPointsChanged)
    Q_PROPERTY(bool accumulatedPlayback MEMBER m_accumulatedPlayback NOTIFY playbackStateChanged)
    Q_PROPERTY(QVariantList playbackTrajectory READ playbackTrajectory NOTIFY playbackPositionChanged)
    Q_PROPERTY(QVariantList flightTrajectory READ flightTrajectory NOTIFY playbackStateChanged)
    Q_PROPERTY(QString recordingSources READ recordingSources NOTIFY playbackStateChanged)
    Q_PROPERTY(QString status READ status NOTIFY statusChanged)
    Q_PROPERTY(QString loadedFile READ loadedFile NOTIFY loadedFileChanged)
    Q_PROPERTY(bool loading READ loading NOTIFY loadingChanged)
    Q_PROPERTY(int progress READ progress NOTIFY progressChanged)
    Q_PROPERTY(bool playbackAvailable READ playbackAvailable NOTIFY playbackStateChanged)
    Q_PROPERTY(bool showingPlaybackFrame READ showingPlaybackFrame NOTIFY playbackStateChanged)
    Q_PROPERTY(int playbackDurationMs READ playbackDurationMs NOTIFY playbackStateChanged)
    Q_PROPERTY(int playbackPositionMs READ playbackPositionMs WRITE setPlaybackPositionMs NOTIFY playbackPositionChanged)
    Q_PROPERTY(bool interactivePreviewEnabled READ interactivePreviewEnabled NOTIFY interactivePreviewChanged)
    Q_PROPERTY(bool playbackPoseAvailable READ playbackPoseAvailable NOTIFY playbackPoseChanged)
    Q_PROPERTY(bool hasPlaybackPose READ hasPlaybackPose NOTIFY playbackPoseChanged)
    Q_PROPERTY(int playbackPoseCount READ playbackPoseCount NOTIFY playbackPoseChanged)
    Q_PROPERTY(QString playbackPoseText READ playbackPoseText NOTIFY playbackPoseChanged)
    Q_PROPERTY(QVector3D playbackPosePositionMeters READ playbackPosePositionMeters NOTIFY playbackPoseChanged)
    Q_PROPERTY(float playbackPoseYawDegrees READ playbackPoseYawDegrees NOTIFY playbackPoseChanged)

public:
    // 创建静态点云加载器。默认显示上限为 500000 点，避免一次加载超大模型卡住交互。
    explicit PointCloudFileLoader(QObject *parent = nullptr);
    // 析构时标记后台任务失效，防止窗口关闭后异步回调访问已销毁对象。
    ~PointCloudFileLoader() override;

    // 主显示几何体。加载完成后完整地图或录制时间窗写入这里。
    PointCloudGeometry *geometry() const { return m_geometry; }
    // 绑定主显示几何体。QML 中通过 geometry: pointCloudGeometry 设置。
    void setGeometry(PointCloudGeometry *geometry);
    // 交互预览几何体。拖动视角时可临时显示抽稀点云，降低全量模型旋转卡顿。
    PointCloudGeometry *previewGeometry() const { return m_previewGeometry; }
    // 绑定交互预览几何体。QML 中通过 previewGeometry: pointCloudPreviewGeometry 设置。
    void setPreviewGeometry(PointCloudGeometry *geometry);

    // 静态模型最多显示的点数；0 表示尽量加载全部有效点。
    int maxPoints() const { return m_maxPoints; }
    // 设置显示点数上限。下一次加载文件时生效。
    void setMaxPoints(int maxPoints);

    // 面向 UI 的加载状态文本，包含文件名、显示点数和错误原因。
    QString status() const { return m_status; }
    // 最近成功加载的文件路径。
    QString loadedFile() const { return m_loadedFile; }
    // 是否正在后台加载文件。
    bool loading() const { return m_loading; }
    // 加载进度百分比。大文件主要用于 UI 反馈，不保证每种格式精确到每一行。
    int progress() const { return m_progress; }
    // 当前静态文件是否包含可播放时间轴。LVX2/ROS bag 通常为 true，CSV/PLY/PCD 通常为 false。
    bool playbackAvailable() const { return m_playbackAvailable; }
    // 是否正在显示录制文件的某个短时间窗，而不是累计地图。
    bool showingPlaybackFrame() const { return m_showingPlaybackFrame; }
    // 录制文件总时长，单位：毫秒。
    int playbackDurationMs() const { return m_playbackDurationMs; }
    // 当前录制文件播放位置，单位：毫秒。
    int playbackPositionMs() const { return m_playbackPositionMs; }
    // 设置录制文件播放位置，并刷新当前时间窗点云。
    void setPlaybackPositionMs(int positionMs);
    // 是否正在使用抽稀的交互预览几何体。
    bool interactivePreviewEnabled() const { return m_interactivePreviewEnabled; }
    // 当前录制文件是否包含可随时间轴显示的位姿样本。
    bool playbackPoseAvailable() const { return !m_playbackPoseSamples.isEmpty(); }
    // 当前播放位置是否已经匹配到一个可显示位姿。
    bool hasPlaybackPose() const { return m_hasPlaybackPose; }
    // 当前录制文件中的位姿样本数量。
    int playbackPoseCount() const { return m_playbackPoseSamples.size(); }
    // 当前播放位置对应的位姿文本。
    QString playbackPoseText() const { return m_playbackPoseText; }
    // 当前播放位置对应的米制位置。
    QVector3D playbackPosePositionMeters() const { return m_playbackPosePositionMeters; }
    // 当前播放位置对应的水平航向角。
    float playbackPoseYawDegrees() const { return m_playbackPoseYawDegrees; }

    // 从本地 URL 加载静态点云。会根据后缀选择 LVX2/ROS bag/PLY/PCD/文本解析器。
    Q_INVOKABLE bool loadFromUrl(const QUrl &url);
    // 同一次飞行的所有分包，在后台按绝对时间合并世界点云与FAST-LIO轨迹。
    Q_INVOKABLE bool loadRecordingFiles(const QVariantList &urls);
    QVariantList flightTrajectory() const;
    QVariantList playbackTrajectory() const;
    QString recordingSources() const{return m_recordingSources;}
    // 从录制时间窗或交互预览恢复到完整累计地图。
    Q_INVOKABLE void showAccumulatedPointCloud();
    // 打开/关闭交互预览。打开时 QML 会隐藏完整模型并显示抽稀模型。
    Q_INVOKABLE void setInteractivePreviewEnabled(bool enabled);

signals:
    void geometryChanged();
    void previewGeometryChanged();
    void maxPointsChanged();
    void statusChanged();
    void loadedFileChanged();
    void loadingChanged();
    void progressChanged();
    void playbackStateChanged();
    void playbackPositionChanged();
    void interactivePreviewChanged();
    void playbackPoseChanged();
    void loadFinished(bool success);

private:
    // 读取 PLY 点云，支持 ascii 和 binary_little_endian 顶点点云。
    bool loadPly(const QString &filePath, QVector<PointCloudPoint> *points, QString *error) const;
    // 读取 PCD 点云，支持 ascii 和 binary，暂不支持 binary_compressed。
    bool loadPcd(const QString &filePath, QVector<PointCloudPoint> *points, QString *error) const;
    // 读取小型 XYZ/TXT/CSV 点列表；大文件会走异步流式版本。
    bool loadTextXyz(const QString &filePath, QVector<PointCloudPoint> *points, QString *error) const;
    // 按 maxPoints 对已经读入内存的点集做均匀抽稀。
    QVector<PointCloudPoint> decimated(const QVector<PointCloudPoint> &points) const;
    // 启动文本/CSV 的 QtConcurrent 后台加载，避免 UI 线程阻塞。
    bool loadTextXyzAsync(const QString &filePath);
    // 启动 LVX2 的 QtConcurrent 后台加载，解析累计地图和播放时间轴。
    bool loadLvx2Async(const QString &filePath);
    // 启动 ROS1 bag 的 QtConcurrent 后台加载，解析 PointCloud2 累计地图和播放时间轴。
    bool loadRosBagAsync(const QString &filePath);
    // 后台加载完成后回到 UI 线程统一收尾：设置几何、状态、进度和回放缓存。
    void finishAsyncLoad(quint64 token,
                         const QString &filePath,
                         bool ok,
                         const QString &error,
                         const QVector<PointCloudPoint> &points,
                         quint64 sourcePointCount,
                         const QVector<PointCloudPoint> &playbackPoints,
                         const QVector<int> &playbackTimesMs,
                         const QVector<PointCloudPlaybackPoseSample> &playbackPoseSamples,
                         int playbackDurationMs,
                         int playbackFrameDurationMs);
    // 清空录制文件/静态模型缓存和预览几何，切换文件前调用。
    void clearPlaybackData();
    // 保存累计点云、交互预览点云和录制文件时间轴数据。
    void setPlaybackData(const QVector<PointCloudPoint> &accumulatedPoints,
                         const QVector<PointCloudPoint> &playbackPoints,
                         const QVector<int> &playbackTimesMs,
                         const QVector<PointCloudPlaybackPoseSample> &playbackPoseSamples,
                         int playbackDurationMs,
                         int playbackFrameDurationMs);
    // 根据 playbackPositionMs 切出一个短时间窗并刷新主几何体。
    void updatePlaybackGeometry();
    // 根据 playbackPositionMs 选择最近的录制位姿，并刷新 QML 位姿属性。
    void updatePlaybackPose();
    // 更新状态文本并通知 QML。
    void setStatus(const QString &status);
    // 更新最近加载文件路径并通知 QML。
    void setLoadedFile(const QString &filePath);
    // 更新 loading 状态并通知 QML。
    void setLoading(bool loading);
    // 更新加载进度并通知 QML。
    void setProgress(int progress);

    // 主点云几何体，不拥有生命周期。
    PointCloudGeometry *m_geometry = nullptr;
    // 抽稀预览几何体，不拥有生命周期。
    PointCloudGeometry *m_previewGeometry = nullptr;
    // 当前加载上限，0 表示不主动抽稀。
    int m_maxPoints = 500000;
    QString m_status;
    QString m_recordingSources;
    QString m_loadedFile;
    bool m_loading = false;
    int m_progress = 0;
    // 异步加载令牌。新任务启动后旧任务结果会被忽略，避免慢任务覆盖新文件。
    quint64 m_loadToken = 0;
    // 生命周期标记，后台线程回调前会检查它，避免访问已销毁对象。
    std::shared_ptr<std::atomic_bool> m_alive;
    // 累计地图点云，静态模式下完整显示的数据源。
    QVector<PointCloudPoint> m_accumulatedPoints;
    // 抽稀后的交互预览点云，专门用于拖动视角时保持流畅。
    QVector<PointCloudPoint> m_interactivePreviewPoints;
    // LVX2/ROS bag 可播放点云，按时间戳排序。
    QVector<PointCloudPoint> m_playbackPoints;
    // 与 m_playbackPoints 一一对应的毫秒时间戳。
    QVector<int> m_playbackTimesMs;
    // ROS bag 中的 PoseStamped 位姿时间轴。
    QVector<PointCloudPlaybackPoseSample> m_playbackPoseSamples;
    bool m_hasPlaybackPose = false;
    QString m_playbackPoseText;
    QVector3D m_playbackPosePositionMeters;
    float m_playbackPoseYawDegrees = 0.0f;
    // 交互预览、回放可用性和当前显示状态。
    bool m_interactivePreviewEnabled = false;
    bool m_playbackAvailable = false;
    bool m_showingPlaybackFrame = false;
    bool m_accumulatedPlayback = false;
    // 录制文件时间轴信息。
    int m_playbackDurationMs = 0;
    int m_playbackFrameDurationMs = 50;
    int m_playbackPositionMs = 0;
};

#endif // POINTCLOUDFILELOADER_H
