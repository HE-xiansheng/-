// Copyright (C) 2026
// SPDX-License-Identifier: LicenseRef-Qt-Commercial OR BSD-3-Clause

#include "pointcloudfileloader.h"

#ifdef GCS_HAVE_SQLITE3
#include <sqlite3.h>
#endif
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QHash>
#include <QPointer>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QThread>
#include <QtConcurrent>
#include <QtEndian>
#include <QtMath>
#ifdef GCS_HAVE_LZ4
#include <lz4frame.h>
#endif
#ifdef GCS_HAVE_BZIP2
#include <bzlib.h>
#endif

#include <algorithm>
#include <atomic>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <random>

namespace {

constexpr int InteractivePreviewPointLimit = 800000;

struct FieldLayout
{
    QByteArray name;
    QByteArray type;
    int size = 0;
    int count = 1;
    qsizetype byteOffset = 0;
    int tokenOffset = 0;
};

struct PlyProperty
{
    QByteArray name;
    QByteArray type;
};

using ProgressCallback = std::function<void(int)>;

struct TextLoadResult
{
    bool ok = false;
    QString error;
    QVector<PointCloudPoint> points;
    quint64 sourcePointCount = 0;
    quint64 startStampUsec = 0;
    QString sources;
    QVector<PointCloudPoint> playbackPoints;
    QVector<int> playbackTimesMs;
    QVector<PointCloudPlaybackPoseSample> playbackPoseSamples;
    int playbackDurationMs = 0;
    int playbackFrameDurationMs = 0;
};

struct LvxPackageRange
{
    qint64 payloadOffset = 0;
    quint32 payloadLength = 0;
    quint8 dataType = 0;
    quint64 firstRawPointIndex = 0;
    int timeMs = 0;
};

struct RosBagRecord
{
    quint8 op = 0;
    QHash<QByteArray, QByteArray> fields;
    qsizetype dataOffset = 0;
    quint32 dataLength = 0;
};

struct RosBagChunk
{
    qsizetype dataOffset = 0;
    quint32 dataLength = 0;
    QString compression;
    quint32 rawSize = 0;
};

struct RosBagConnection
{
    QString topic;
    QString type;
};

struct RosPointField
{
    QByteArray name;
    quint32 offset = 0;
    quint8 datatype = 0;
    quint32 count = 0;
};

struct RosPointCloud2View
{
    quint64 stampUsec = 0;
    quint32 width = 0;
    quint32 height = 0;
    QVector<RosPointField> fields;
    bool isBigEndian = false;
    quint32 pointStep = 0;
    quint32 rowStep = 0;
    qsizetype dataOffset = 0;
    quint32 dataLength = 0;
};

struct StreamingSampler
{
    explicit StreamingSampler(int maxPointCount)
        : maxPoints(maxPointCount <= 0 ? 0 : std::max(1000, maxPointCount))
    {
        if (maxPoints > 0)
            points.reserve(maxPoints);
    }

    void append(const PointCloudPoint &point)
    {
        // Livox Viewer 导出的 CSV 里会混入少量 0,0,0 填充点，静态地图显示时应跳过。
        if (!std::isfinite(point.position.x()) || !std::isfinite(point.position.y())
            || !std::isfinite(point.position.z())) {
            return;
        }
        if (qFuzzyIsNull(point.position.x()) && qFuzzyIsNull(point.position.y())
            && qFuzzyIsNull(point.position.z())) {
            return;
        }

        ++sourcePointCount;
        if (maxPoints <= 0) {
            points.append(point);
            return;
        }

        if (points.size() < maxPoints) {
            points.append(point);
            return;
        }

        // 固定随机种子的 reservoir sampling：只保留 maxPoints 个代表点，
        // 不需要提前知道文件总行数，也不会随每次加载产生不同画面。
        std::uniform_int_distribution<quint64> distribution(0, sourcePointCount - 1);
        const quint64 index = distribution(rng);
        if (index < quint64(maxPoints))
            points[int(index)] = point;
    }

    int maxPoints = 0;
    QVector<PointCloudPoint> points;
    quint64 sourcePointCount = 0;
    std::mt19937_64 rng { 0x4d4944333630ULL };
};

float normalizedIntensity(float value);

QStringList splitNumberTokens(QByteArray line)
{
    line.replace(',', ' ');
    line.replace(';', ' ');
    line.replace('\t', ' ');
    return QString::fromLatin1(line).simplified().split(QLatin1Char(' '), Qt::SkipEmptyParts);
}

QByteArrayList splitDataTokens(QByteArray line)
{
    if (line.contains(','))
        return line.split(',');

    line.replace(';', ' ');
    line.replace('\t', ' ');
    return line.simplified().split(' ');
}

QByteArray normalizedColumnName(const QByteArray &token)
{
    QByteArray result = token.trimmed().toLower();
    result.replace("\"", "");
    result.replace(" ", "");
    result.replace("_", "");
    return result;
}

QString sanitizedCacheFileName(QString fileName)
{
    fileName = fileName.trimmed();
    if (fileName.isEmpty())
        fileName = QStringLiteral("selected_pointcloud");

    fileName.replace(QRegularExpression(QStringLiteral(R"([\\/:*?"<>|])")), QStringLiteral("_"));
    fileName.replace(QRegularExpression(QStringLiteral(R"(\s+)")), QStringLiteral("_"));
    return fileName.left(120);
}

QString suffixFromUrlOrName(const QUrl &url, const QString &fileName)
{
    QString suffix = QFileInfo(fileName).suffix().toLower();
    if (!suffix.isEmpty())
        return suffix;

    const QString decodedUrl = QUrl::fromPercentEncoding(url.toString().toUtf8());
    const QRegularExpression suffixPattern(QStringLiteral(R"(\.(lvx2|bag|db3|pcd|ply|xyz|txt|csv)(?:$|[/?#]))"),
                                           QRegularExpression::CaseInsensitiveOption);
    const QRegularExpressionMatch match = suffixPattern.match(decodedUrl);
    return match.hasMatch() ? match.captured(1).toLower() : QString();
}

#ifdef Q_OS_ANDROID
QString cachedAndroidContentFilePath(const QUrl &url, QString *error)
{
    // Android 的系统文件选择器通常返回 content:// URI，而不是 Windows 那种真实磁盘路径。
    // 现有 LVX2/Bag/CSV/PCD/PLY 解析器需要随机访问本地文件，所以先把用户选中的文件
    // 流式复制到应用缓存目录，再复用同一套解析逻辑。该函数只在 Android 编译进目标。
    QString fileName = QUrl::fromPercentEncoding(url.fileName().toUtf8());
    const QString suffix = suffixFromUrlOrName(url, fileName);
    fileName = sanitizedCacheFileName(fileName);
    if (QFileInfo(fileName).suffix().isEmpty() && !suffix.isEmpty())
        fileName += QLatin1Char('.') + suffix;

    const QString cacheRoot = QStandardPaths::writableLocation(QStandardPaths::CacheLocation);
    if (cacheRoot.isEmpty()) {
        if (error)
            *error = QStringLiteral("Android 缓存目录不可用。");
        return {};
    }

    QDir cacheDir(cacheRoot);
    if (!cacheDir.mkpath(QStringLiteral("pointcloud_imports"))) {
        if (error)
            *error = QStringLiteral("无法创建 Android 点云导入缓存目录：%1").arg(cacheDir.absoluteFilePath(QStringLiteral("pointcloud_imports")));
        return {};
    }
    cacheDir.cd(QStringLiteral("pointcloud_imports"));

    QFile source(url.toString());
    if (!source.open(QIODevice::ReadOnly)) {
        if (error)
            *error = QStringLiteral("无法读取 Android 选择的文件：%1").arg(source.errorString());
        qCritical().noquote() << QStringLiteral("Android content URI 打开失败：url=%1，error=%2")
                                    .arg(url.toString(), source.errorString());
        return {};
    }

    const QString destinationPath = cacheDir.absoluteFilePath(
        QStringLiteral("%1_%2").arg(QDateTime::currentMSecsSinceEpoch()).arg(fileName));
    QFile destination(destinationPath);
    if (!destination.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        if (error)
            *error = QStringLiteral("无法写入 Android 点云缓存文件：%1").arg(destination.errorString());
        qCritical().noquote() << QStringLiteral("Android 点云缓存写入失败：path=%1，error=%2")
                                    .arg(destinationPath, destination.errorString());
        return {};
    }

    constexpr qint64 ChunkSize = 4 * 1024 * 1024;
    qint64 copiedBytes = 0;
    while (!source.atEnd()) {
        const QByteArray chunk = source.read(ChunkSize);
        if (chunk.isEmpty() && source.error() != QFile::NoError) {
            if (error)
                *error = QStringLiteral("读取 Android 点云文件失败：%1").arg(source.errorString());
            return {};
        }
        if (destination.write(chunk) != chunk.size()) {
            if (error)
                *error = QStringLiteral("写入 Android 点云缓存失败：%1").arg(destination.errorString());
            return {};
        }
        copiedBytes += chunk.size();
    }

    qInfo().noquote() << QStringLiteral("Android 点云文件已复制到应用缓存：url=%1，cache=%2，bytes=%3")
                          .arg(url.toString(), destinationPath)
                          .arg(copiedBytes);
    return destinationPath;
}
#endif

QString pointCloudFilePathForUrl(const QUrl &url, QString *error)
{
#ifdef Q_OS_ANDROID
    // Android 专用：把 content:// 导入为临时本地文件；Windows/桌面平台不走这个分支。
    if (url.scheme() == QLatin1String("content"))
        return cachedAndroidContentFilePath(url, error);
#endif

    // 桌面平台保持原行为：本地文件 URL 转为本地路径，其他 URL 直接传给加载器。
    Q_UNUSED(error);
    return url.isLocalFile() ? url.toLocalFile() : url.toString();
}

int indexOfColumn(const QByteArrayList &columns, const QByteArrayList &names)
{
    for (int i = 0; i < columns.size(); ++i) {
        const QByteArray columnName = normalizedColumnName(columns.at(i));
        if (names.contains(columnName))
            return i;
    }
    return -1;
}

bool parseFloatToken(const QByteArray &token, float *value)
{
    if (!value)
        return false;

    const QByteArray trimmed = token.trimmed();
    if (trimmed.isEmpty())
        return false;

    bool ok = false;
    const float parsed = trimmed.toFloat(&ok);
    if (!ok || !std::isfinite(parsed))
        return false;

    *value = parsed;
    return true;
}

bool parsePointFromTokens(const QByteArrayList &tokens,
                          int xIndex,
                          int yIndex,
                          int zIndex,
                          int intensityIndex,
                          int redIndex,
                          int greenIndex,
                          int blueIndex,
                          PointCloudPoint *point)
{
    if (!point)
        return false;

    const int requiredIndex = std::max({ xIndex, yIndex, zIndex });
    if (xIndex < 0 || yIndex < 0 || zIndex < 0 || tokens.size() <= requiredIndex)
        return false;

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (!parseFloatToken(tokens.at(xIndex), &x)
        || !parseFloatToken(tokens.at(yIndex), &y)
        || !parseFloatToken(tokens.at(zIndex), &z)) {
        return false;
    }

    float intensity = 0.75f;
    if (intensityIndex >= 0 && tokens.size() > intensityIndex) {
        float parsedIntensity = 0.0f;
        if (parseFloatToken(tokens.at(intensityIndex), &parsedIntensity))
            intensity = normalizedIntensity(parsedIntensity);
    } else if (redIndex >= 0 && greenIndex >= 0 && blueIndex >= 0
               && tokens.size() > std::max({ redIndex, greenIndex, blueIndex })) {
        float r = 0.0f;
        float g = 0.0f;
        float b = 0.0f;
        if (parseFloatToken(tokens.at(redIndex), &r)
            && parseFloatToken(tokens.at(greenIndex), &g)
            && parseFloatToken(tokens.at(blueIndex), &b)) {
            intensity = normalizedIntensity((r + g + b) / 3.0f);
        }
    }

    point->position = QVector3D(x, y, z);
    point->intensity = intensity;
    return true;
}

TextLoadResult loadTextPointCloudStreaming(const QString &filePath,
                                           int maxPoints,
                                           const ProgressCallback &progressCallback)
{
    TextLoadResult result;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        result.error = file.errorString();
        qCritical().noquote() << QStringLiteral("文本点云打开失败：path=%1，error=%2").arg(filePath, result.error);
        return result;
    }

    StreamingSampler sampler(maxPoints);
    const qint64 fileSize = std::max<qint64>(1, file.size());
    qInfo().noquote() << QStringLiteral("文本点云开始读取：path=%1，size=%2，maxPoints=%3")
                          .arg(filePath)
                          .arg(fileSize)
                          .arg(maxPoints == 0 ? QStringLiteral("全部") : QString::number(maxPoints));
    int lastProgress = -1;
    auto reportProgress = [&]() {
        if (!progressCallback)
            return;

        const int progress = int(std::clamp<qint64>(file.pos() * 100 / fileSize, 0, 100));
        if (progress != lastProgress) {
            lastProgress = progress;
            progressCallback(progress);
        }
    };

    int xIndex = 0;
    int yIndex = 1;
    int zIndex = 2;
    int intensityIndex = 3;
    int redIndex = -1;
    int greenIndex = -1;
    int blueIndex = -1;
    bool firstDataLineMayNeedParsing = false;
    QByteArray firstDataLine;

    while (!file.atEnd()) {
        firstDataLine = file.readLine().trimmed();
        if (firstDataLine.isEmpty() || firstDataLine.startsWith('#') || firstDataLine.startsWith("//"))
            continue;

        const QByteArrayList tokens = splitDataTokens(firstDataLine);
        const int detectedX = indexOfColumn(tokens, { "x" });
        const int detectedY = indexOfColumn(tokens, { "y" });
        const int detectedZ = indexOfColumn(tokens, { "z" });
        if (detectedX >= 0 && detectedY >= 0 && detectedZ >= 0) {
            xIndex = detectedX;
            yIndex = detectedY;
            zIndex = detectedZ;
            intensityIndex = indexOfColumn(tokens, { "intensity", "reflectivity", "reflectance", "i" });
            redIndex = indexOfColumn(tokens, { "red", "r" });
            greenIndex = indexOfColumn(tokens, { "green", "g" });
            blueIndex = indexOfColumn(tokens, { "blue", "b" });
            qInfo().noquote() << QStringLiteral("文本点云识别到表头：x=%1，y=%2，z=%3，intensity=%4，rgb=(%5,%6,%7)")
                                  .arg(xIndex)
                                  .arg(yIndex)
                                  .arg(zIndex)
                                  .arg(intensityIndex)
                                  .arg(redIndex)
                                  .arg(greenIndex)
                                  .arg(blueIndex);
        } else {
            firstDataLineMayNeedParsing = true;
        }
        break;
    }

    if (firstDataLineMayNeedParsing) {
        PointCloudPoint point;
        if (parsePointFromTokens(splitDataTokens(firstDataLine),
                                 xIndex,
                                 yIndex,
                                 zIndex,
                                 intensityIndex,
                                 redIndex,
                                 greenIndex,
                                 blueIndex,
                                 &point)) {
            sampler.append(point);
        }
    }

    quint64 lineCounter = 0;
    while (!file.atEnd()) {
        if (QThread::currentThread()->isInterruptionRequested()) {
            result.error = QStringLiteral("加载已取消。");
            return result;
        }

        const QByteArray line = file.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#') || line.startsWith("//"))
            continue;

        PointCloudPoint point;
        if (parsePointFromTokens(splitDataTokens(line),
                                 xIndex,
                                 yIndex,
                                 zIndex,
                                 intensityIndex,
                                 redIndex,
                                 greenIndex,
                                 blueIndex,
                                 &point)) {
            sampler.append(point);
        }

        ++lineCounter;
        if (lineCounter % 20000 == 0)
            reportProgress();
    }

    reportProgress();
    result.ok = true;
    result.points = std::move(sampler.points);
    result.sourcePointCount = sampler.sourcePointCount;
    qInfo().noquote() << QStringLiteral("文本点云读取完成：path=%1，source=%2，visible=%3")
                          .arg(filePath)
                          .arg(result.sourcePointCount)
                          .arg(result.points.size());
    return result;
}

template<typename T>
bool readLittleEndianValue(QFile *file, T *value)
{
    if (!file || !value)
        return false;

    char buffer[sizeof(T)];
    if (file->read(buffer, sizeof(T)) != qsizetype(sizeof(T)))
        return false;

    *value = qFromLittleEndian<T>(reinterpret_cast<const uchar *>(buffer));
    return true;
}

template<typename T>
T readLittleEndianAt(const uchar *data)
{
    return qFromLittleEndian<T>(data);
}

bool readRosBagHeaderFields(const QByteArray &data,
                            qsizetype offset,
                            qsizetype length,
                            QHash<QByteArray, QByteArray> *fields,
                            QString *error)
{
    if (!fields || offset < 0 || length < 0 || offset > data.size() - length) {
        if (error)
            *error = QStringLiteral("ROS bag 记录头越界。");
        return false;
    }

    fields->clear();
    const qsizetype end = offset + length;
    qsizetype cursor = offset;
    while (cursor < end) {
        if (cursor > end - 4) {
            if (error)
                *error = QStringLiteral("ROS bag 记录头字段长度不完整。");
            return false;
        }

        const quint32 fieldLength = readLittleEndianAt<quint32>(
            reinterpret_cast<const uchar *>(data.constData() + cursor));
        cursor += 4;
        if (fieldLength == 0 || cursor > end - qsizetype(fieldLength)) {
            if (error)
                *error = QStringLiteral("ROS bag 记录头字段越界。");
            return false;
        }

        const QByteArray field = data.mid(cursor, int(fieldLength));
        cursor += fieldLength;
        const int separator = field.indexOf('=');
        if (separator <= 0)
            continue;

        fields->insert(field.left(separator), field.mid(separator + 1));
    }

    return cursor == end;
}

quint8 rosBagOp(const QHash<QByteArray, QByteArray> &fields)
{
    const QByteArray op = fields.value("op");
    return op.isEmpty() ? 0 : quint8(op.at(0));
}

quint32 rosBagFieldU32(const QHash<QByteArray, QByteArray> &fields,
                       const QByteArray &name,
                       quint32 fallback = 0)
{
    const QByteArray value = fields.value(name);
    if (value.size() < 4)
        return fallback;
    return qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(value.constData()));
}

quint64 rosBagTimeUsec(const QByteArray &value)
{
    if (value.size() < 8)
        return 0;

    const quint32 sec = qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(value.constData()));
    const quint32 nsec = qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(value.constData() + 4));
    return quint64(sec) * 1000000ULL + quint64(nsec / 1000U);
}

bool readRosBagRecord(const QByteArray &data,
                      qsizetype *cursor,
                      RosBagRecord *record,
                      QString *error)
{
    if (!cursor || !record || *cursor < 0 || *cursor > data.size() - 4) {
        if (error)
            *error = QStringLiteral("ROS bag 记录起始位置无效。");
        return false;
    }

    const quint32 headerLength = readLittleEndianAt<quint32>(
        reinterpret_cast<const uchar *>(data.constData() + *cursor));
    *cursor += 4;
    if (headerLength == 0 || *cursor > data.size() - qsizetype(headerLength) - 4) {
        if (error)
            *error = QStringLiteral("ROS bag 记录头长度无效。");
        return false;
    }

    if (!readRosBagHeaderFields(data, *cursor, headerLength, &record->fields, error))
        return false;
    *cursor += headerLength;

    const quint32 dataLength = readLittleEndianAt<quint32>(
        reinterpret_cast<const uchar *>(data.constData() + *cursor));
    *cursor += 4;
    if (*cursor > data.size() - qsizetype(dataLength)) {
        if (error)
            *error = QStringLiteral("ROS bag 记录数据长度越界。");
        return false;
    }

    record->op = rosBagOp(record->fields);
    record->dataOffset = *cursor;
    record->dataLength = dataLength;
    *cursor += dataLength;
    return true;
}

bool readRosU32(const QByteArray &data, qsizetype end, qsizetype *cursor, quint32 *value)
{
    if (!cursor || !value || *cursor < 0 || *cursor > end - 4 || end > data.size())
        return false;

    *value = readLittleEndianAt<quint32>(reinterpret_cast<const uchar *>(data.constData() + *cursor));
    *cursor += 4;
    return true;
}

bool readRosU8(const QByteArray &data, qsizetype end, qsizetype *cursor, quint8 *value)
{
    if (!cursor || !value || *cursor < 0 || *cursor >= end || end > data.size())
        return false;

    *value = quint8(data.at(*cursor));
    ++(*cursor);
    return true;
}

bool readRosString(const QByteArray &data, qsizetype end, qsizetype *cursor, QByteArray *value)
{
    quint32 length = 0;
    if (!value || !readRosU32(data, end, cursor, &length))
        return false;
    if (*cursor > end - qsizetype(length))
        return false;

    *value = data.mid(*cursor, int(length));
    *cursor += length;
    return true;
}

bool readRosFloat64(const QByteArray &data, qsizetype end, qsizetype *cursor, double *value)
{
    if (!cursor || !value || *cursor < 0 || *cursor > end - 8 || end > data.size())
        return false;

    const quint64 raw = readLittleEndianAt<quint64>(reinterpret_cast<const uchar *>(data.constData() + *cursor));
    std::memcpy(value, &raw, sizeof(double));
    *cursor += 8;
    return std::isfinite(*value);
}

float yawDegreesFromQuaternion(float qx, float qy, float qz, float qw)
{
    const double sinyCosp = 2.0 * (double(qw) * double(qz) + double(qx) * double(qy));
    const double cosyCosp = 1.0 - 2.0 * (double(qy) * double(qy) + double(qz) * double(qz));
    return float(qRadiansToDegrees(std::atan2(sinyCosp, cosyCosp)));
}

bool parseRosPoseStampedSample(const QByteArray &chunkData,
                               qsizetype messageOffset,
                               quint32 messageLength,
                               quint64 bagTimeUsec,
                               PointCloudPlaybackPoseSample *sample,
                               quint64 *stampUsec,
                               bool odometry = false)
{
    if (!sample || messageOffset < 0 || messageOffset > chunkData.size() - qsizetype(messageLength))
        return false;

    const qsizetype end = messageOffset + messageLength;
    qsizetype cursor = messageOffset;
    quint32 ignoredSequence = 0;
    quint32 stampSec = 0;
    quint32 stampNsec = 0;
    QByteArray ignoredFrameId;
    if (!readRosU32(chunkData, end, &cursor, &ignoredSequence)
        || !readRosU32(chunkData, end, &cursor, &stampSec)
        || !readRosU32(chunkData, end, &cursor, &stampNsec)
        || !readRosString(chunkData, end, &cursor, &ignoredFrameId)) {
        return false;
    }

    if (odometry) { QByteArray child; if (!readRosString(chunkData,end,&cursor,&child)) return false; }
    double px = 0.0;
    double py = 0.0;
    double pz = 0.0;
    double qx = 0.0;
    double qy = 0.0;
    double qz = 0.0;
    double qw = 1.0;
    if (!readRosFloat64(chunkData, end, &cursor, &px)
        || !readRosFloat64(chunkData, end, &cursor, &py)
        || !readRosFloat64(chunkData, end, &cursor, &pz)
        || !readRosFloat64(chunkData, end, &cursor, &qx)
        || !readRosFloat64(chunkData, end, &cursor, &qy)
        || !readRosFloat64(chunkData, end, &cursor, &qz)
        || !readRosFloat64(chunkData, end, &cursor, &qw)) {
        return false;
    }

    sample->positionMeters = QVector3D(float(px), float(py), float(pz));
    sample->qx = float(qx);
    sample->qy = float(qy);
    sample->qz = float(qz);
    sample->qw = float(qw);
    sample->yawDegrees = yawDegreesFromQuaternion(sample->qx, sample->qy, sample->qz, sample->qw);
    const quint64 messageStampUsec = quint64(stampSec) * 1000000ULL + quint64(stampNsec / 1000U);
    if (stampUsec)
        *stampUsec = messageStampUsec > 0 ? messageStampUsec : bagTimeUsec;
    return true;
}

int rosPointFieldIndex(const QVector<RosPointField> &fields, const QByteArrayList &names)
{
    for (int i = 0; i < fields.size(); ++i) {
        const QByteArray fieldName = fields.at(i).name.toLower();
        if (names.contains(fieldName))
            return i;
    }
    return -1;
}

bool parseRosPointCloud2View(const QByteArray &chunkData,
                             qsizetype messageOffset,
                             quint32 messageLength,
                             quint64 bagTimeUsec,
                             RosPointCloud2View *view)
{
    if (!view || messageOffset < 0 || messageOffset > chunkData.size() - qsizetype(messageLength))
        return false;

    const qsizetype end = messageOffset + messageLength;
    qsizetype cursor = messageOffset;
    quint32 ignoredU32 = 0;
    quint32 stampSec = 0;
    quint32 stampNsec = 0;
    QByteArray ignoredString;
    if (!readRosU32(chunkData, end, &cursor, &ignoredU32)
        || !readRosU32(chunkData, end, &cursor, &stampSec)
        || !readRosU32(chunkData, end, &cursor, &stampNsec)
        || !readRosString(chunkData, end, &cursor, &ignoredString)
        || !readRosU32(chunkData, end, &cursor, &view->height)
        || !readRosU32(chunkData, end, &cursor, &view->width)) {
        return false;
    }

    quint32 fieldCount = 0;
    if (!readRosU32(chunkData, end, &cursor, &fieldCount) || fieldCount > 128)
        return false;

    view->fields.clear();
    view->fields.reserve(int(fieldCount));
    for (quint32 i = 0; i < fieldCount; ++i) {
        RosPointField field;
        if (!readRosString(chunkData, end, &cursor, &field.name)
            || !readRosU32(chunkData, end, &cursor, &field.offset)
            || !readRosU8(chunkData, end, &cursor, &field.datatype)
            || !readRosU32(chunkData, end, &cursor, &field.count)) {
            return false;
        }
        view->fields.append(field);
    }

    quint8 bigEndian = 0;
    if (!readRosU8(chunkData, end, &cursor, &bigEndian)
        || !readRosU32(chunkData, end, &cursor, &view->pointStep)
        || !readRosU32(chunkData, end, &cursor, &view->rowStep)
        || !readRosU32(chunkData, end, &cursor, &view->dataLength)) {
        return false;
    }

    if (view->pointStep == 0 || view->dataLength == 0
        || cursor > end - qsizetype(view->dataLength)) {
        return false;
    }

    view->isBigEndian = bigEndian != 0;
    view->dataOffset = cursor;
    cursor += view->dataLength;
    quint8 ignoredDense = 0;
    if (!readRosU8(chunkData, end, &cursor, &ignoredDense))
        return false;

    const quint64 stampUsec = quint64(stampSec) * 1000000ULL + quint64(stampNsec / 1000U);
    view->stampUsec = stampUsec > 0 ? stampUsec : bagTimeUsec;
    return true;
}

bool readRosPointFieldScalar(const char *pointData,
                             const RosPointField &field,
                             float *value)
{
    if (!pointData || !value)
        return false;

    const char *data = pointData + field.offset;
    switch (field.datatype) {
    case 1: // INT8
        *value = float(*reinterpret_cast<const qint8 *>(data));
        return true;
    case 2: // UINT8
        *value = float(*reinterpret_cast<const quint8 *>(data));
        return true;
    case 3: // INT16
        *value = float(qFromLittleEndian<qint16>(reinterpret_cast<const uchar *>(data)));
        return true;
    case 4: // UINT16
        *value = float(qFromLittleEndian<quint16>(reinterpret_cast<const uchar *>(data)));
        return true;
    case 5: // INT32
        *value = float(qFromLittleEndian<qint32>(reinterpret_cast<const uchar *>(data)));
        return true;
    case 6: // UINT32
        *value = float(qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(data)));
        return true;
    case 7: { // FLOAT32
        quint32 raw = qFromLittleEndian<quint32>(reinterpret_cast<const uchar *>(data));
        std::memcpy(value, &raw, sizeof(float));
        return std::isfinite(*value);
    }
    case 8: { // FLOAT64
        quint64 raw = qFromLittleEndian<quint64>(reinterpret_cast<const uchar *>(data));
        double parsed = 0.0;
        std::memcpy(&parsed, &raw, sizeof(double));
        if (!std::isfinite(parsed))
            return false;
        *value = float(parsed);
        return true;
    }
    default:
        return false;
    }
}

int rosPointFieldScalarSize(quint8 datatype)
{
    switch (datatype) {
    case 1: // INT8
    case 2: // UINT8
        return 1;
    case 3: // INT16
    case 4: // UINT16
        return 2;
    case 5: // INT32
    case 6: // UINT32
    case 7: // FLOAT32
        return 4;
    case 8: // FLOAT64
        return 8;
    default:
        return 0;
    }
}

bool rosPointFieldFitsPointStep(const RosPointField &field, quint32 pointStep)
{
    const int scalarSize = rosPointFieldScalarSize(field.datatype);
    return scalarSize > 0 && field.count > 0 && field.offset <= pointStep
           && quint64(field.offset) + quint64(scalarSize) <= quint64(pointStep);
}

bool isUsablePoint(const QVector3D &position)
{
    if (!std::isfinite(position.x()) || !std::isfinite(position.y())
        || !std::isfinite(position.z())) {
        return false;
    }

    return !(qFuzzyIsNull(position.x()) && qFuzzyIsNull(position.y()) && qFuzzyIsNull(position.z()));
}

bool keepSampledRawPoint(quint64 rawIndex, quint64 totalRawPointCount, int maxPoints)
{
    if (maxPoints <= 0 || totalRawPointCount <= quint64(maxPoints))
        return true;

    const quint64 slot = rawIndex * quint64(maxPoints) / totalRawPointCount;
    const quint64 previousSlot = rawIndex == 0
                                     ? std::numeric_limits<quint64>::max()
                                     : (rawIndex - 1) * quint64(maxPoints) / totalRawPointCount;
    return slot != previousSlot;
}

QVector<PointCloudPoint> uniformlyDecimatedPointsWithTimes(const QVector<PointCloudPoint> &points,
                                                           const QVector<int> &timesMs,
                                                           int maxPoints,
                                                           QVector<int> *decimatedTimesMs)
{
    if (decimatedTimesMs)
        decimatedTimesMs->clear();

    if (maxPoints <= 0 || points.size() <= maxPoints) {
        if (decimatedTimesMs)
            *decimatedTimesMs = timesMs;
        return points;
    }

    QVector<PointCloudPoint> result;
    result.reserve(maxPoints);
    if (decimatedTimesMs)
        decimatedTimesMs->reserve(maxPoints);

    const double step = double(points.size()) / double(maxPoints);
    for (int i = 0; i < maxPoints; ++i) {
        const qsizetype sourceIndex = std::min<qsizetype>(points.size() - 1,
                                                          qsizetype(std::floor(double(i) * step)));
        result.append(points.at(sourceIndex));
        if (decimatedTimesMs)
            decimatedTimesMs->append(timesMs.value(sourceIndex, 0));
    }
    return result;
}

TextLoadResult loadLvx2PointCloudStreaming(const QString &filePath,
                                           int maxPoints,
                                           const ProgressCallback &progressCallback)
{
    TextLoadResult result;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        result.error = file.errorString();
        qCritical().noquote() << QStringLiteral("LVX2 打开失败：path=%1，error=%2").arg(filePath, result.error);
        return result;
    }

    const QByteArray fileData = file.readAll();
    const qint64 fileSize = fileData.size();
    const uchar *bytes = reinterpret_cast<const uchar *>(fileData.constData());
    if (fileSize < 29 || !fileData.startsWith("livox_tech")) {
        result.error = QStringLiteral("不是有效的 Livox LVX2 文件。");
        qCritical().noquote() << QStringLiteral("LVX2 文件头无效：path=%1，size=%2").arg(filePath).arg(fileSize);
        return result;
    }

    const quint32 frameDuration = readLittleEndianAt<quint32>(bytes + 24);
    const quint8 deviceCount = *(bytes + 28);
    const qint64 firstFrameOffset = 24 + 5 + qint64(deviceCount) * 63;
    qInfo().noquote() << QStringLiteral("LVX2 文件头：path=%1，size=%2，frameDuration=%3，deviceCount=%4，firstFrameOffset=%5")
                          .arg(filePath)
                          .arg(fileSize)
                          .arg(frameDuration)
                          .arg(deviceCount)
                          .arg(firstFrameOffset);
    if (firstFrameOffset >= fileSize) {
        result.error = QStringLiteral("LVX2 文件没有帧数据。");
        qCritical().noquote() << QStringLiteral("LVX2 文件没有帧数据：path=%1").arg(filePath);
        return result;
    }

    if (progressCallback)
        progressCallback(1);

    QVector<LvxPackageRange> packages;
    packages.reserve(180000);
    quint64 totalRawPointCount = 0;
    quint64 firstTimestampNs = 0;
    int maxPackageTimeMs = 0;
    qint64 frameOffset = firstFrameOffset;
    int lastScanProgress = -1;
    auto reportScanProgress = [&](qint64 offset) {
        if (!progressCallback || fileSize <= 0)
            return;

        const int progress = int(std::clamp<qint64>(offset * 20 / fileSize, 1, 20));
        if (progress != lastScanProgress) {
            lastScanProgress = progress;
            progressCallback(progress);
        }
    };

    while (frameOffset + 24 <= fileSize) {
        const uchar *frameHeader = bytes + frameOffset;
        qint64 currentOffset = readLittleEndianAt<qint64>(frameHeader);
        qint64 nextOffset = readLittleEndianAt<qint64>(frameHeader + 8);
        const qint64 frameIndex = readLittleEndianAt<qint64>(frameHeader + 16);
        Q_UNUSED(frameIndex);

        if (currentOffset > 0 && currentOffset < fileSize)
            frameOffset = currentOffset;

        const qint64 frameEnd = (nextOffset > frameOffset && nextOffset <= fileSize)
                                    ? nextOffset
                                    : fileSize;
        qint64 packageOffset = frameOffset + 24;

        while (packageOffset + 27 <= frameEnd) {
            const uchar *packageHeader = bytes + packageOffset;
            // LVX2 点包头共 27 字节：本 demo 当前用到的字段为
            // - +7  : 8 字节 Livox 时间戳，通常是纳秒级单调时间；
            // - +17 : 点数据类型，1/2 表示笛卡尔点；
            // - +18 : payload 字节数。
            // Livox Viewer 的播放条也是基于这些帧/包时间推进，而不是把 LVX2 当静态点列表。
            const quint64 packetTimestampNs = readLittleEndianAt<quint64>(packageHeader + 7);
            const quint8 dataType = *(packageHeader + 17);
            const quint32 payloadLength = readLittleEndianAt<quint32>(packageHeader + 18);
            const qint64 payloadOffset = packageOffset + 27;
            const qint64 nextPackageOffset = payloadOffset + payloadLength;
            if (payloadLength == 0 || nextPackageOffset > frameEnd)
                break;

            int pointStride = 0;
            if (dataType == 1) {
                pointStride = 14;
            } else if (dataType == 2) {
                pointStride = 8;
            }

            if (pointStride > 0) {
                const quint64 count = payloadLength / quint32(pointStride);
                if (firstTimestampNs == 0 && packetTimestampNs > 0)
                    firstTimestampNs = packetTimestampNs;

                int packageTimeMs = 0;
                if (firstTimestampNs > 0 && packetTimestampNs >= firstTimestampNs) {
                    packageTimeMs = int(std::min<quint64>((packetTimestampNs - firstTimestampNs) / 1000000ULL,
                                                          quint64(std::numeric_limits<int>::max())));
                } else if (frameDuration > 0 && frameIndex >= 0) {
                    packageTimeMs = int(std::min<qint64>(frameIndex * qint64(frameDuration),
                                                         qint64(std::numeric_limits<int>::max())));
                }

                maxPackageTimeMs = std::max(maxPackageTimeMs, packageTimeMs);
                packages.append({ payloadOffset,
                                  payloadLength,
                                  dataType,
                                  totalRawPointCount,
                                  packageTimeMs });
                totalRawPointCount += count;
            }

            packageOffset = nextPackageOffset;
        }

        reportScanProgress(frameEnd);
        if (nextOffset <= frameOffset || nextOffset >= fileSize)
            break;
        frameOffset = nextOffset;
    }

    if (packages.isEmpty()) {
        result.error = QStringLiteral("LVX2 文件中没有可显示的笛卡尔点云包。");
        qCritical().noquote() << QStringLiteral("LVX2 没有可显示笛卡尔点包：path=%1").arg(filePath);
        return result;
    }

    result.ok = true;
    qInfo().noquote() << QStringLiteral("LVX2 扫描完成：path=%1，packages=%2，rawPoints=%3，maxPackageTimeMs=%4")
                          .arg(filePath)
                          .arg(packages.size())
                          .arg(totalRawPointCount)
                          .arg(maxPackageTimeMs);
    const quint64 rawSampleLimit = maxPoints > 0 && totalRawPointCount > quint64(maxPoints)
                                       ? std::min(totalRawPointCount, quint64(maxPoints) * 3ULL)
                                       : totalRawPointCount;
    const int rawSamplePointLimit = maxPoints <= 0
                                        ? 0
                                        : int(std::min<quint64>(rawSampleLimit,
                                                                quint64(std::numeric_limits<int>::max())));
    const quint64 reserveEstimate = maxPoints > 0 && rawSampleLimit > quint64(maxPoints)
                                        ? std::min<quint64>(rawSampleLimit, quint64(maxPoints) * 2ULL)
                                        : rawSampleLimit;
    result.points.reserve(qsizetype(std::min<quint64>(reserveEstimate,
                                                      quint64(std::numeric_limits<int>::max()))));
    QVector<int> candidateTimesMs;
    candidateTimesMs.reserve(result.points.capacity());

    int lastDecodeProgress = 20;
    const int packageCount = int(packages.size());
    for (int packageIndex = 0; packageIndex < packageCount; ++packageIndex) {
        const LvxPackageRange &package = packages.at(packageIndex);
        const uchar *payload = bytes + package.payloadOffset;
        if (package.dataType == 1) {
            // 笛卡尔坐标，高精度：x/y/z 为 int32，单位 mm；后跟 reflectivity/tag。
            constexpr int PointStride = 14;
            const int count = int(package.payloadLength) / PointStride;
            for (int i = 0; i < count; ++i) {
                const quint64 rawIndex = package.firstRawPointIndex + quint64(i);
                if (!keepSampledRawPoint(rawIndex, totalRawPointCount, rawSamplePointLimit))
                    continue;

                const uchar *pointData = payload + i * PointStride;
                const QVector3D position(float(readLittleEndianAt<qint32>(pointData)) * 0.001f,
                                         float(readLittleEndianAt<qint32>(pointData + 4)) * 0.001f,
                                         float(readLittleEndianAt<qint32>(pointData + 8)) * 0.001f);
                if (!isUsablePoint(position))
                    continue;

                result.points.append({ position, normalizedIntensity(float(*(pointData + 12))) });
                candidateTimesMs.append(package.timeMs);
            }
        } else if (package.dataType == 2) {
            // 笛卡尔坐标，低精度：x/y/z 为 int16，单位 cm；后跟 reflectivity/tag。
            constexpr int PointStride = 8;
            const int count = int(package.payloadLength) / PointStride;
            for (int i = 0; i < count; ++i) {
                const quint64 rawIndex = package.firstRawPointIndex + quint64(i);
                if (!keepSampledRawPoint(rawIndex, totalRawPointCount, rawSamplePointLimit))
                    continue;

                const uchar *pointData = payload + i * PointStride;
                const QVector3D position(float(readLittleEndianAt<qint16>(pointData)) * 0.01f,
                                         float(readLittleEndianAt<qint16>(pointData + 2)) * 0.01f,
                                         float(readLittleEndianAt<qint16>(pointData + 4)) * 0.01f);
                if (!isUsablePoint(position))
                    continue;

                result.points.append({ position, normalizedIntensity(float(*(pointData + 6))) });
                candidateTimesMs.append(package.timeMs);
            }
        }

        if (progressCallback && packageIndex % 2048 == 0) {
            const int progress = 20 + int(double(packageIndex) * 78.0 / double(std::max(1, packageCount)));
            if (progress != lastDecodeProgress) {
                lastDecodeProgress = progress;
                progressCallback(std::clamp(progress, 20, 98));
            }
        }
    }

    result.sourcePointCount = totalRawPointCount;
    result.playbackPoints = result.points;
    result.playbackTimesMs = candidateTimesMs;
    result.points = uniformlyDecimatedPointsWithTimes(result.playbackPoints,
                                                      result.playbackTimesMs,
                                                      maxPoints,
                                                      nullptr);
    result.playbackFrameDurationMs = frameDuration > 0 ? int(frameDuration) : 50;
    result.playbackDurationMs = std::max(maxPackageTimeMs,
                                         int(packages.isEmpty()
                                                 ? 0
                                                 : packages.last().timeMs + result.playbackFrameDurationMs));
    qInfo().noquote() << QStringLiteral("LVX2 解码完成：path=%1，usablePlayback=%2，visible=%3，durationMs=%4，frameDurationMs=%5")
                          .arg(filePath)
                          .arg(result.playbackPoints.size())
                          .arg(result.points.size())
                          .arg(result.playbackDurationMs)
                          .arg(result.playbackFrameDurationMs);
    if (progressCallback)
        progressCallback(98);
    return result;
}

// 每次只保留当前解压chunk；ROS bag的LZ4采用标准LZ4 frame（含校验）。
bool decodeBagChunk(const QByteArray &fileData,const RosBagChunk &chunk,QByteArray &data,QString &error)
{
    if(chunk.rawSize>512U*1024U*1024U){error=QStringLiteral("录包chunk过大或损坏");return false;}
    const auto compressed=QByteArray::fromRawData(fileData.constData()+chunk.dataOffset,chunk.dataLength);
    if(chunk.compression=="none"){data=compressed;return true;}
#ifdef GCS_HAVE_LZ4
    if(chunk.compression=="lz4"){
        LZ4F_dctx *ctx=nullptr;
        if(LZ4F_isError(LZ4F_createDecompressionContext(&ctx,LZ4F_VERSION))){error="LZ4初始化失败";return false;}
        data.resize(chunk.rawSize);size_t src=0,dst=0,hint=1;
        while(hint && src<size_t(compressed.size())){
            size_t in=size_t(compressed.size())-src,out=size_t(data.size())-dst;
            hint=LZ4F_decompress(ctx,data.data()+dst,&out,compressed.constData()+src,&in,nullptr);
            if(LZ4F_isError(hint)||(!in&&!out)){LZ4F_freeDecompressionContext(ctx);error="LZ4录包损坏或校验失败";return false;}
            src+=in;dst+=out;
        }
        LZ4F_freeDecompressionContext(ctx);
        if(hint||dst!=chunk.rawSize||src!=size_t(compressed.size())){error="LZ4录包长度不匹配";return false;}
        return true;
    }
#endif
#ifdef GCS_HAVE_BZIP2
    if(chunk.compression=="bz2"){
        data.resize(chunk.rawSize);unsigned int size=chunk.rawSize;
        if(BZ2_bzBuffToBuffDecompress(data.data(),&size,const_cast<char*>(compressed.constData()),compressed.size(),0,0)!=BZ_OK||size!=chunk.rawSize){error="BZ2录包损坏";return false;}
        return true;
    }
#endif
    error=QStringLiteral("当前构建不支持%1压缩录包").arg(chunk.compression);return false;
}

// ROS2 sqlite3/CDR读取不依赖ROS运行时；保留ROS1读取路径。
#ifdef GCS_HAVE_SQLITE3
class CdrReader {
public:
    QByteArray data; qsizetype at=4; bool little=true; bool ok=true;
    explicit CdrReader(const QByteArray &v):data(v){
        ok=data.size()>=4 && data[0]==0 && (data[1]==0||data[1]==1);little=ok&&data[1]==1;
    }
    bool take(qsizetype n){if(!ok||n<0||at>data.size()-n){ok=false;return false;}return true;}
    void align(int n){at+=((n-(at-4)%n)%n);if(at>data.size())ok=false;}
    quint8 u8(){if(!take(1))return 0;return quint8(data[at++]);}
    quint32 u32(){align(4);if(!take(4))return 0;auto p=reinterpret_cast<const uchar*>(data.constData()+at);at+=4;return little?qFromLittleEndian<quint32>(p):qFromBigEndian<quint32>(p);}
    double f64(){align(8);if(!take(8))return 0;auto p=reinterpret_cast<const uchar*>(data.constData()+at);quint64 v=little?qFromLittleEndian<quint64>(p):qFromBigEndian<quint64>(p);at+=8;double d;std::memcpy(&d,&v,8);return d;}
    QByteArray string(){quint32 n=u32();if(n<1||n>65536||!take(n)){ok=false;return {};}auto s=data.mid(at,n-1);if(data[at+n-1]!=0)ok=false;at+=n;return s;}
    QByteArray bytes(){quint32 n=u32();if(!take(n))return {};auto s=data.mid(at,n);at+=n;return s;}
};
void ros2Append32(QByteArray &out,quint32 v){char b[4];qToLittleEndian(v,reinterpret_cast<uchar*>(b));out.append(b,4);}
void ros2AppendString(QByteArray &out,const QByteArray &v){ros2Append32(out,v.size());out+=v;}
QByteArray ros2ToRos1(const QByteArray &cdr,bool cloud,bool odometry,QString *frame){
    CdrReader r(cdr);QByteArray out;
    quint32 sec=r.u32(),ns=r.u32();auto f=r.string();*frame=QString::fromUtf8(f);
    if(ns>=1000000000U){return {};}
    ros2Append32(out,0);ros2Append32(out,sec);ros2Append32(out,ns);ros2AppendString(out,f);
    if(!cloud){
        if(odometry)ros2AppendString(out,r.string());
        for(int i=0;i<7;++i){double d=r.f64();quint64 v;std::memcpy(&v,&d,8);char b[8];qToLittleEndian(v,reinterpret_cast<uchar*>(b));out.append(b,8);}
    }else{
        auto height=r.u32(),width=r.u32(),n=r.u32();if(n>256)return {};
        ros2Append32(out,height);ros2Append32(out,width);ros2Append32(out,n);
        for(quint32 i=0;i<n;++i){ros2AppendString(out,r.string());ros2Append32(out,r.u32());out.append(char(r.u8()));ros2Append32(out,r.u32());}
        out.append(char(r.u8()));ros2Append32(out,r.u32());ros2Append32(out,r.u32());
        auto bytes=r.bytes();ros2Append32(out,bytes.size());out+=bytes;out.append(char(r.u8()));
    }
    return r.ok?out:QByteArray();
}
TextLoadResult loadRos2Bag(const QString &path,int maxPoints,const ProgressCallback &progress,bool recordingMode){
    TextLoadResult result;sqlite3 *raw=nullptr;
    const auto encoded=QFile::encodeName(path);
    if(sqlite3_open_v2(encoded.constData(),&raw,SQLITE_OPEN_READONLY,nullptr)!=SQLITE_OK){result.error="无法打开ROS2 SQLite录包";if(raw)sqlite3_close(raw);return result;}
    std::unique_ptr<sqlite3,decltype(&sqlite3_close)> db(raw,sqlite3_close);
    sqlite3_exec(raw,"BEGIN",nullptr,nullptr,nullptr);
    auto query=[&](const char *sql){sqlite3_stmt *q=nullptr;if(sqlite3_prepare_v2(raw,sql,-1,&q,nullptr)!=SQLITE_OK)return static_cast<sqlite3_stmt*>(nullptr);return q;};
    auto text=[](sqlite3_stmt *q,int i){auto p=sqlite3_column_text(q,i);return p?QString::fromUtf8(reinterpret_cast<const char*>(p)):QString();};
    auto q=query("SELECT name,type,serialization_format FROM topics");
    if(!q){result.error="ROS2录包topics表无效";return result;}
    QHash<QString,QString> topics;
    while(sqlite3_step(q)==SQLITE_ROW){if(text(q,2)=="cdr")topics.insert(text(q,0),text(q,1));}sqlite3_finalize(q);
    QString cloud,pose;
    for(const auto &s:QStringList{"/factory_fastlio/cloud_world","/cloud_registered","/laser_cloud_surround","/livox/cloud_world"})if(topics.value(s)=="sensor_msgs/msg/PointCloud2"){cloud=s;break;}
    for(const auto &s:QStringList{"/factory_fastlio/odometry","/Odometry","/aft_mapped_to_init","/mavros/vision_pose/pose"})if(topics.value(s)=="nav_msgs/msg/Odometry"||topics.value(s)=="geometry_msgs/msg/PoseStamped"){pose=s;break;}
    if(cloud.isEmpty()){result.error="ROS2录包缺少CDR世界点云";return result;}
    result.sources=QStringLiteral("点云 %1 · 轨迹 %2").arg(cloud,pose.isEmpty()?QStringLiteral("未录制匹配的里程计"):pose);
    q=query("SELECT topics.name,messages.timestamp,messages.data FROM messages JOIN topics ON topics.id=messages.topic_id WHERE topics.name=?1 OR topics.name=?2 ORDER BY messages.timestamp,messages.id");
    if(!q){result.error="ROS2录包messages表无效";return result;}
    std::unique_ptr<sqlite3_stmt,decltype(&sqlite3_finalize)> statement(q,sqlite3_finalize);
    sqlite3_bind_text(q,1,cloud.toUtf8().constData(),-1,SQLITE_TRANSIENT);sqlite3_bind_text(q,2,pose.toUtf8().constData(),-1,SQLITE_TRANSIENT);
    quint64 first=std::numeric_limits<quint64>::max(),last=0,total=0;QString world;
    // 第一遍计算总点数和源时间，第二遍有界抽稀，避免按点云吞吐无限分配内存。
    for(int pass=0;pass<2;++pass){
        sqlite3_reset(q);quint64 rawIndex=0;int rc;
        while((rc=sqlite3_step(q))==SQLITE_ROW){
            const bool isCloud=text(q,0)==cloud;const int n=sqlite3_column_bytes(q,2);
            if(n<=0||n>512*1024*1024){result.error="ROS2消息长度无效";return result;}
            QByteArray cdr=QByteArray::fromRawData(static_cast<const char*>(sqlite3_column_blob(q,2)),n);QString frame;
            auto bytes=ros2ToRos1(cdr,isCloud,topics.value(pose)=="nav_msgs/msg/Odometry",&frame);
            if(bytes.isEmpty()){result.error="ROS2 CDR编码不支持或消息损坏";return result;}
            const quint64 bagStamp=quint64(std::max<sqlite3_int64>(0,sqlite3_column_int64(q,1)))/1000;
            quint64 stamp=0;RosPointCloud2View view;PointCloudPlaybackPoseSample sample;
            if(isCloud){
                if(!parseRosPointCloud2View(bytes,0,bytes.size(),bagStamp,&view)||view.pointStep==0||view.isBigEndian||quint64(view.rowStep)*view.height!=view.dataLength||view.rowStep<quint64(view.width)*view.pointStep){result.error="ROS2 PointCloud2布局无效";return result;}
                if(world.isEmpty())world=frame;
                if(frame!=world||(frame!="camera_init"&&frame!="world"&&frame!="map")){result.error="世界点云坐标系不一致";return result;}
                stamp=view.stampUsec;
            }else{
                if(!parseRosPoseStampedSample(bytes,0,bytes.size(),bagStamp,&sample,&stamp,topics.value(pose)=="nav_msgs/msg/Odometry")){result.error="ROS2位姿无效";return result;}
            }
            if(pass==0){first=std::min(first,stamp);last=std::max(last,stamp);if(isCloud)total+=quint64(view.width)*view.height;continue;}
            const int time=int(std::min<quint64>((stamp-first)/1000,std::numeric_limits<int>::max()));
            if(!isCloud){if(frame!=world){result.error="点云与轨迹坐标系不一致";return result;}sample.timeMs=time;result.playbackPoseSamples.append(sample);continue;}
            int xi=rosPointFieldIndex(view.fields,{"x"}),yi=rosPointFieldIndex(view.fields,{"y"}),zi=rosPointFieldIndex(view.fields,{"z"}),ii=rosPointFieldIndex(view.fields,{"intensity","reflectivity"});
            if(xi<0||yi<0||zi<0){result.error="ROS2点云缺少XYZ";return result;}
            for(int idx:{xi,yi,zi})if(!rosPointFieldFitsPointStep(view.fields[idx],view.pointStep)){result.error="ROS2字段超出点步长";return result;}
            if(ii>=0&&!rosPointFieldFitsPointStep(view.fields[ii],view.pointStep))ii=-1;
            const int limit=maxPoints>0?std::max(1000,maxPoints)*4:0;
            for(quint32 row=0;row<view.height;++row)for(quint32 col=0;col<view.width;++col,++rawIndex){
                if(!keepSampledRawPoint(rawIndex,total,limit))continue;
                const char *p=bytes.constData()+view.dataOffset+quint64(row)*view.rowStep+quint64(col)*view.pointStep;
                float x,y,z,intensity=.75f;
                if(!readRosPointFieldScalar(p,view.fields[xi],&x)||!readRosPointFieldScalar(p,view.fields[yi],&y)||!readRosPointFieldScalar(p,view.fields[zi],&z))continue;
                QVector3D pos(x,y,z);if(!isUsablePoint(pos))continue;
                float value;if(ii>=0&&readRosPointFieldScalar(p,view.fields[ii],&value))intensity=normalizedIntensity(value);
                result.playbackPoints.append({pos,intensity});result.playbackTimesMs.append(time);
            }
        }
        if(rc!=SQLITE_DONE){result.error="ROS2 SQLite消息读取失败";return result;}
        if(progress)progress(pass?98:45);
    }
    if(first==std::numeric_limits<quint64>::max() || (result.playbackPoints.isEmpty() && (!recordingMode || result.playbackPoseSamples.isEmpty()))){result.error="ROS2录包中没有有效世界点云/轨迹";return result;}
    result.sourcePointCount=total;result.startStampUsec=first;result.playbackFrameDurationMs=50;
    result.playbackDurationMs=int(std::min<quint64>((last-first)/1000,std::numeric_limits<int>::max()));
    std::sort(result.playbackPoseSamples.begin(),result.playbackPoseSamples.end(),[](const auto&a,const auto&b){return a.timeMs<b.timeMs;});
    result.points=uniformlyDecimatedPointsWithTimes(result.playbackPoints,result.playbackTimesMs,maxPoints,nullptr);result.ok=true;return result;
}
#endif

TextLoadResult loadRosBagPointCloudStreaming(const QString &filePath,
                                             int maxPoints,
                                             const ProgressCallback &progressCallback,
                                             bool recordingMode = false)
{
    if(QFileInfo(filePath).suffix().toLower()=="db3"){
#ifdef GCS_HAVE_SQLITE3
        return loadRos2Bag(filePath,maxPoints,progressCallback,recordingMode);
#else
        TextLoadResult unsupported;unsupported.error="当前构建未启用ROS2 SQLite支持";return unsupported;
#endif
    }
    TextLoadResult result;
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        result.error = file.errorString();
        qCritical().noquote() << QStringLiteral("ROS bag 打开失败：path=%1，error=%2").arg(filePath, result.error);
        return result;
    }

    const qint64 fileSize = file.size();
    const auto mapped=file.map(0,fileSize);
    if(!mapped){result.error=QStringLiteral("无法映射录包文件");return result;}
    const QByteArray fileData=QByteArray::fromRawData(reinterpret_cast<const char*>(mapped),fileSize);
    constexpr qsizetype BagMagicLength = 13;
    if (fileSize <= BagMagicLength || !fileData.startsWith("#ROSBAG V2.0\n")) {
        result.error = QStringLiteral("不是有效的 ROS1 bag v2.0 文件。");
        qCritical().noquote() << QStringLiteral("ROS bag 文件头无效：path=%1，size=%2").arg(filePath).arg(fileSize);
        return result;
    }

    qInfo().noquote() << QStringLiteral("ROS bag 开始扫描：path=%1，size=%2，maxPoints=%3")
                          .arg(filePath)
                          .arg(fileSize)
                          .arg(maxPoints == 0 ? QStringLiteral("全部") : QString::number(maxPoints));
    if (progressCallback)
        progressCallback(1);

    QVector<RosBagChunk> chunks;
    chunks.reserve(256);
    QHash<quint32, RosBagConnection> connections;
    QString parseError;
    qsizetype cursor = BagMagicLength;
    int lastScanProgress = -1;
    while (cursor < fileData.size()) {
        if (cursor > fileData.size() - 4)
            break;

        RosBagRecord record;
        if (!readRosBagRecord(fileData, &cursor, &record, &parseError)) {
            result.error = QStringLiteral("ROS bag 顶层记录解析失败：%1").arg(parseError);
            qCritical().noquote() << QStringLiteral("ROS bag 顶层记录解析失败：path=%1，cursor=%2，error=%3")
                                        .arg(filePath)
                                        .arg(cursor)
                                        .arg(parseError);
            return result;
        }

        if (record.op == 0x05) { // CHUNK
            chunks.append({ record.dataOffset,
                            record.dataLength,
                            QString::fromUtf8(record.fields.value("compression")),
                            rosBagFieldU32(record.fields,"size") });
        } else if (record.op == 0x07) { // CONNECTION
            const quint32 connectionId = rosBagFieldU32(record.fields, "conn");
            QHash<QByteArray, QByteArray> connectionFields;
            if (!readRosBagHeaderFields(fileData,
                                        record.dataOffset,
                                        record.dataLength,
                                        &connectionFields,
                                        &parseError)) {
                result.error = QStringLiteral("ROS bag connection 记录解析失败：%1").arg(parseError);
                qCritical().noquote() << QStringLiteral("ROS bag connection 解析失败：path=%1，conn=%2，error=%3")
                                            .arg(filePath)
                                            .arg(connectionId)
                                            .arg(parseError);
                return result;
            }

            connections.insert(connectionId,
                               { QString::fromUtf8(record.fields.value("topic")),
                                 QString::fromUtf8(connectionFields.value("type")) });
        }

        if (progressCallback && fileSize > 0) {
            const int progress = int(std::clamp<qint64>(cursor * 8 / fileSize, 1, 8));
            if (progress != lastScanProgress) {
                lastScanProgress = progress;
                progressCallback(progress);
            }
        }
    }

    if (chunks.isEmpty()) {
        result.error = QStringLiteral("ROS bag 内没有 chunk 数据。");
        qCritical().noquote() << QStringLiteral("ROS bag 没有 chunk：path=%1").arg(filePath);
        return result;
    }

    QString chosenCloud,chosenPose;
    if(recordingMode){
        const QStringList clouds={"/factory_fastlio/cloud_world","/cloud_registered","/laser_cloud_surround","/livox/cloud_world"};
        const QStringList poses={"/factory_fastlio/odometry","/Odometry","/aft_mapped_to_init","/mavros/vision_pose/pose"};
        for(const auto &topic:clouds){for(const auto &c:connections)if(c.topic==topic&&c.type=="sensor_msgs/PointCloud2")chosenCloud=topic;if(!chosenCloud.isEmpty())break;}
        for(const auto &topic:poses){for(const auto &c:connections)if(c.topic==topic&&(c.type=="nav_msgs/Odometry"||c.type=="geometry_msgs/PoseStamped"))chosenPose=topic;if(!chosenPose.isEmpty())break;}
        if(chosenCloud.isEmpty()){result.error="录包缺少已配准的世界点云（FAST-LIO cloud_world/cloud_registered）";return result;}
        result.sources=QStringLiteral("点云 %1 · 轨迹 %2").arg(chosenCloud,chosenPose.isEmpty()?QStringLiteral("未录制匹配的里程计"):chosenPose);
    }
    int pointCloudConnectionCount = 0;
    QStringList pointCloudTopics;
    int poseConnectionCount = 0;
    QStringList poseTopics;
    for (auto it = connections.cbegin(); it != connections.cend(); ++it) {
        if (it.value().type == QLatin1String("sensor_msgs/PointCloud2")) {
            ++pointCloudConnectionCount;
            pointCloudTopics.append(it.value().topic);
        } else if (it.value().type == QLatin1String("geometry_msgs/PoseStamped")) {
            ++poseConnectionCount;
            poseTopics.append(it.value().topic);
        }
    }

    if (pointCloudConnectionCount == 0) {
        result.error = QStringLiteral("ROS bag 内没有 sensor_msgs/PointCloud2 点云话题。");
        qCritical().noquote() << QStringLiteral("ROS bag 没有 PointCloud2：path=%1，connections=%2")
                                    .arg(filePath)
                                    .arg(connections.size());
        return result;
    }

    qInfo().noquote() << QStringLiteral("ROS bag 扫描完成：path=%1，chunks=%2，connections=%3，PointCloud2 topics=%4，PoseStamped topics=%5")
                          .arg(filePath)
                          .arg(chunks.size())
                          .arg(connections.size())
                          .arg(pointCloudTopics.join(QStringLiteral(", ")))
                          .arg(poseConnectionCount > 0 ? poseTopics.join(QStringLiteral(", ")) : QStringLiteral("无"));

    struct RosMessageRange
    {
        int chunkIndex = 0;
        qsizetype messageOffset = 0;
        quint32 messageLength = 0;
        quint32 connectionId = 0;
        quint64 bagTimeUsec = 0;
        quint64 firstRawPointIndex = 0;
        quint32 rawPointCount = 0;
        int timeMs = 0;
    };

    QVector<RosMessageRange> pointCloudMessages;
    pointCloudMessages.reserve(20000);
    quint64 totalRawPointCount = 0;
    quint64 firstStampUsec = 0;
    int maxTimeMs = 0;
    auto relativeTimeMs = [&](quint64 stampUsec) {
        if (firstStampUsec == 0 && stampUsec > 0)
            firstStampUsec = stampUsec;

        if (firstStampUsec == 0 || stampUsec < firstStampUsec)
            return 0;

        return int(std::min<quint64>((stampUsec - firstStampUsec) / 1000ULL,
                                     quint64(std::numeric_limits<int>::max())));
    };
    int lastMessageScanProgress = 8;
    for (int chunkIndex = 0; chunkIndex < chunks.size(); ++chunkIndex) {
        const RosBagChunk &chunk = chunks.at(chunkIndex);
        if (chunk.dataOffset < 0 || chunk.dataOffset > fileData.size() - qsizetype(chunk.dataLength)) {
            result.error = QStringLiteral("ROS bag chunk 数据越界。");
            qCritical().noquote() << QStringLiteral("ROS bag chunk 越界：path=%1，chunk=%2").arg(filePath).arg(chunkIndex);
            return result;
        }

        QByteArray chunkData;
        if(!decodeBagChunk(fileData,chunk,chunkData,result.error))return result;
        qsizetype chunkCursor = 0;
        while (chunkCursor < chunkData.size()) {
            if (chunkCursor > chunkData.size() - 4)
                break;

            RosBagRecord messageRecord;
            if (!readRosBagRecord(chunkData, &chunkCursor, &messageRecord, &parseError)) {
                result.error = QStringLiteral("ROS bag chunk 内记录解析失败：%1").arg(parseError);
                qCritical().noquote() << QStringLiteral("ROS bag chunk 内记录解析失败：path=%1，chunk=%2，cursor=%3，error=%4")
                                            .arg(filePath)
                                            .arg(chunkIndex)
                                            .arg(chunkCursor)
                                            .arg(parseError);
                return result;
            }

            if (messageRecord.op != 0x02) // MSG_DATA
                continue;

            const quint32 connectionId = rosBagFieldU32(messageRecord.fields, "conn");
            const RosBagConnection connection = connections.value(connectionId);
            const quint64 bagTimeUsec = rosBagTimeUsec(messageRecord.fields.value("time"));

            if ((connection.type == QLatin1String("geometry_msgs/PoseStamped") || (recordingMode && connection.type=="nav_msgs/Odometry"))
                && (!recordingMode || connection.topic==chosenPose)) {
                PointCloudPlaybackPoseSample poseSample;
                quint64 poseStampUsec = 0;
                if (!parseRosPoseStampedSample(chunkData,
                                               messageRecord.dataOffset,
                                               messageRecord.dataLength,
                                               bagTimeUsec,
                                               &poseSample,
                                               &poseStampUsec,connection.type=="nav_msgs/Odometry")) {
                    qWarning().noquote() << QStringLiteral("跳过无法解析的 PoseStamped 消息：path=%1，chunk=%2，conn=%3")
                                                .arg(filePath)
                                                .arg(chunkIndex)
                                                .arg(connectionId);
                    continue;
                }

                poseSample.timeMs = relativeTimeMs(poseStampUsec);
                maxTimeMs = std::max(maxTimeMs, poseSample.timeMs);
                result.playbackPoseSamples.append(poseSample);
                continue;
            }

            if (connection.type != QLatin1String("sensor_msgs/PointCloud2") || (recordingMode && connection.topic!=chosenCloud))
                continue;

            RosPointCloud2View view;
            if (!parseRosPointCloud2View(chunkData,
                                         messageRecord.dataOffset,
                                         messageRecord.dataLength,
                                         bagTimeUsec,
                                         &view)) {
                qWarning().noquote() << QStringLiteral("跳过无法解析的 PointCloud2 消息：path=%1，chunk=%2，conn=%3")
                                            .arg(filePath)
                                            .arg(chunkIndex)
                                            .arg(connectionId);
                continue;
            }

            if (view.isBigEndian || view.pointStep == 0 || view.dataLength == 0)
                continue;

            const int xIndex = rosPointFieldIndex(view.fields, { "x" });
            const int yIndex = rosPointFieldIndex(view.fields, { "y" });
            const int zIndex = rosPointFieldIndex(view.fields, { "z" });
            if (xIndex < 0 || yIndex < 0 || zIndex < 0
                || !rosPointFieldFitsPointStep(view.fields.at(xIndex), view.pointStep)
                || !rosPointFieldFitsPointStep(view.fields.at(yIndex), view.pointStep)
                || !rosPointFieldFitsPointStep(view.fields.at(zIndex), view.pointStep)) {
                qWarning().noquote() << QStringLiteral("跳过字段布局不完整的 PointCloud2：topic=%1，pointStep=%2")
                                            .arg(connection.topic)
                                            .arg(view.pointStep);
                continue;
            }

            const int timeMs = relativeTimeMs(view.stampUsec);
            maxTimeMs = std::max(maxTimeMs, timeMs);

            const quint64 count = view.dataLength / view.pointStep;
            pointCloudMessages.append({ chunkIndex,
                                        messageRecord.dataOffset,
                                        messageRecord.dataLength,
                                        connectionId,
                                        bagTimeUsec,
                                        totalRawPointCount,
                                        quint32(std::min<quint64>(count, quint64(std::numeric_limits<quint32>::max()))),
                                        timeMs });
            totalRawPointCount += count;
        }

        if (progressCallback) {
            const int progress = 8 + int(double(chunkIndex + 1) * 24.0
                                         / double(std::max<qsizetype>(1, chunks.size())));
            if (progress != lastMessageScanProgress) {
                lastMessageScanProgress = progress;
                progressCallback(std::clamp(progress, 8, 32));
            }
        }
    }

    if (pointCloudMessages.isEmpty() || totalRawPointCount == 0) {
        result.error = QStringLiteral("ROS bag 中没有可显示的 PointCloud2 XYZ 点云。");
        qCritical().noquote() << QStringLiteral("ROS bag 没有可显示 PointCloud2：path=%1").arg(filePath);
        return result;
    }

    const quint64 rawSampleLimit = maxPoints > 0 && totalRawPointCount > quint64(maxPoints)
                                       ? std::min(totalRawPointCount, quint64(maxPoints) * 3ULL)
                                       : totalRawPointCount;
    const int rawSamplePointLimit = maxPoints <= 0
                                        ? 0
                                        : int(std::min<quint64>(rawSampleLimit,
                                                                quint64(std::numeric_limits<int>::max())));
    const quint64 reserveEstimate = maxPoints > 0 && rawSampleLimit > quint64(maxPoints)
                                        ? std::min<quint64>(rawSampleLimit, quint64(maxPoints) * 2ULL)
                                        : rawSampleLimit;
    result.points.reserve(qsizetype(std::min<quint64>(reserveEstimate,
                                                      quint64(std::numeric_limits<int>::max()))));
    QVector<int> candidateTimesMs;
    candidateTimesMs.reserve(result.points.capacity());

    std::sort(result.playbackPoseSamples.begin(),
              result.playbackPoseSamples.end(),
              [](const PointCloudPlaybackPoseSample &left, const PointCloudPlaybackPoseSample &right) {
                  return left.timeMs < right.timeMs;
              });

    qInfo().noquote() << QStringLiteral("ROS bag PointCloud2 索引完成：path=%1，messages=%2，rawPoints=%3，poses=%4，durationMs=%5")
                          .arg(filePath)
                          .arg(pointCloudMessages.size())
                          .arg(totalRawPointCount)
                          .arg(result.playbackPoseSamples.size())
                          .arg(maxTimeMs);

    int lastDecodeProgress = 32;
    int decodedChunk=-1;QByteArray decodedData;
    for (int messageIndex = 0; messageIndex < pointCloudMessages.size(); ++messageIndex) {
        const RosMessageRange &message = pointCloudMessages.at(messageIndex);
        if(decodedChunk!=message.chunkIndex){
            if(!decodeBagChunk(fileData,chunks.at(message.chunkIndex),decodedData,result.error))return result;
            decodedChunk=message.chunkIndex;
        }
        const QByteArray &chunkData=decodedData;

        RosPointCloud2View view;
        if (!parseRosPointCloud2View(chunkData,
                                     message.messageOffset,
                                     message.messageLength,
                                     message.bagTimeUsec,
                                     &view)) {
            continue;
        }

        if (view.isBigEndian || view.pointStep == 0 || view.dataLength == 0)
            continue;

        const int xIndex = rosPointFieldIndex(view.fields, { "x" });
        const int yIndex = rosPointFieldIndex(view.fields, { "y" });
        const int zIndex = rosPointFieldIndex(view.fields, { "z" });
        if (xIndex < 0 || yIndex < 0 || zIndex < 0)
            continue;

        const RosPointField xField = view.fields.at(xIndex);
        const RosPointField yField = view.fields.at(yIndex);
        const RosPointField zField = view.fields.at(zIndex);
        if (!rosPointFieldFitsPointStep(xField, view.pointStep)
            || !rosPointFieldFitsPointStep(yField, view.pointStep)
            || !rosPointFieldFitsPointStep(zField, view.pointStep)) {
            continue;
        }

        const int intensityIndex = rosPointFieldIndex(
            view.fields,
            { "intensity", "intensities", "reflectivity", "reflectance", "i" });
        const bool hasIntensity = intensityIndex >= 0
                                  && rosPointFieldFitsPointStep(view.fields.at(intensityIndex), view.pointStep);
        const RosPointField intensityField = hasIntensity ? view.fields.at(intensityIndex) : RosPointField {};

        const char *pointBytes = chunkData.constData() + view.dataOffset;
        const quint64 count = view.dataLength / view.pointStep;
        for (quint64 i = 0; i < count; ++i) {
            const quint64 rawIndex = message.firstRawPointIndex + i;
            if (!keepSampledRawPoint(rawIndex, totalRawPointCount, rawSamplePointLimit))
                continue;

            const char *pointData = pointBytes + i * view.pointStep;
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
            if (!readRosPointFieldScalar(pointData, xField, &x)
                || !readRosPointFieldScalar(pointData, yField, &y)
                || !readRosPointFieldScalar(pointData, zField, &z)) {
                continue;
            }

            const QVector3D position(x, y, z);
            if (!isUsablePoint(position))
                continue;

            float intensity = 0.75f;
            if (hasIntensity) {
                float parsedIntensity = 0.0f;
                if (readRosPointFieldScalar(pointData, intensityField, &parsedIntensity))
                    intensity = normalizedIntensity(parsedIntensity);
            }

            result.points.append({ position, intensity });
            candidateTimesMs.append(message.timeMs);
        }

        if (progressCallback && messageIndex % 128 == 0) {
            const int progress = 32 + int(double(messageIndex) * 66.0
                                          / double(std::max<qsizetype>(1, pointCloudMessages.size())));
            if (progress != lastDecodeProgress) {
                lastDecodeProgress = progress;
                progressCallback(std::clamp(progress, 32, 98));
            }
        }
    }

    result.ok = true;
    result.sourcePointCount = totalRawPointCount;
    result.playbackPoints = result.points;
    result.playbackTimesMs = candidateTimesMs;
    result.points = uniformlyDecimatedPointsWithTimes(result.playbackPoints,
                                                      result.playbackTimesMs,
                                                      maxPoints,
                                                      nullptr);
    result.startStampUsec=firstStampUsec;
    result.playbackFrameDurationMs = 50;
    result.playbackDurationMs = std::max(maxTimeMs, result.playbackFrameDurationMs);
    qInfo().noquote() << QStringLiteral("ROS bag 解码完成：path=%1，rawPoints=%2，usablePlayback=%3，visible=%4，poses=%5，durationMs=%6")
                          .arg(filePath)
                          .arg(totalRawPointCount)
                          .arg(result.playbackPoints.size())
                          .arg(result.points.size())
                          .arg(result.playbackPoseSamples.size())
                          .arg(result.playbackDurationMs);
    if (progressCallback)
        progressCallback(98);
    return result;
}

float normalizedIntensity(float value)
{
    if (!std::isfinite(value))
        return 0.75f;
    if (value > 1.0f)
        return std::clamp(value / 255.0f, 0.05f, 1.0f);
    return std::clamp(value, 0.05f, 1.0f);
}

int indexOfField(const QVector<FieldLayout> &fields, const QByteArrayList &names)
{
    for (int i = 0; i < fields.size(); ++i) {
        const QByteArray fieldName = fields.at(i).name.toLower();
        if (names.contains(fieldName))
            return i;
    }
    return -1;
}

int indexOfProperty(const QVector<PlyProperty> &properties, const QByteArrayList &names)
{
    for (int i = 0; i < properties.size(); ++i) {
        const QByteArray propertyName = properties.at(i).name.toLower();
        if (names.contains(propertyName))
            return i;
    }
    return -1;
}

template<typename T>
T readLittleEndian(const char *data)
{
    return qFromLittleEndian<T>(reinterpret_cast<const uchar *>(data));
}

bool readFloat32(const char *data, float *value)
{
    quint32 raw = readLittleEndian<quint32>(data);
    std::memcpy(value, &raw, sizeof(float));
    return std::isfinite(*value);
}

bool readFloat64(const char *data, double *value)
{
    quint64 raw = readLittleEndian<quint64>(data);
    std::memcpy(value, &raw, sizeof(double));
    return std::isfinite(*value);
}

bool readPcdScalar(const char *data, int size, const QByteArray &type, double *value)
{
    if (!data || !value || type.isEmpty())
        return false;

    const char typeCode = char(std::toupper(uchar(type.at(0))));
    if (typeCode == 'F') {
        if (size == 4) {
            float result = 0.0f;
            if (!readFloat32(data, &result))
                return false;
            *value = result;
            return true;
        }
        if (size == 8) {
            double result = 0.0;
            if (!readFloat64(data, &result))
                return false;
            *value = result;
            return true;
        }
        return false;
    }

    if (typeCode == 'I') {
        if (size == 1) {
            *value = *reinterpret_cast<const qint8 *>(data);
            return true;
        }
        if (size == 2) {
            *value = readLittleEndian<qint16>(data);
            return true;
        }
        if (size == 4) {
            *value = readLittleEndian<qint32>(data);
            return true;
        }
        if (size == 8) {
            *value = double(readLittleEndian<qint64>(data));
            return true;
        }
        return false;
    }

    if (typeCode == 'U') {
        if (size == 1) {
            *value = *reinterpret_cast<const quint8 *>(data);
            return true;
        }
        if (size == 2) {
            *value = readLittleEndian<quint16>(data);
            return true;
        }
        if (size == 4) {
            *value = readLittleEndian<quint32>(data);
            return true;
        }
        if (size == 8) {
            *value = double(readLittleEndian<quint64>(data));
            return true;
        }
    }

    return false;
}

int plyTypeSize(const QByteArray &typeName)
{
    const QByteArray type = typeName.toLower();
    if (type == "char" || type == "uchar" || type == "int8" || type == "uint8")
        return 1;
    if (type == "short" || type == "ushort" || type == "int16" || type == "uint16")
        return 2;
    if (type == "int" || type == "uint" || type == "int32" || type == "uint32"
        || type == "float" || type == "float32")
        return 4;
    if (type == "double" || type == "float64")
        return 8;
    return 0;
}

bool readPlyScalar(const char *data, const QByteArray &typeName, double *value)
{
    if (!data || !value)
        return false;

    const QByteArray type = typeName.toLower();
    if (type == "char" || type == "int8") {
        *value = *reinterpret_cast<const qint8 *>(data);
        return true;
    }
    if (type == "uchar" || type == "uint8") {
        *value = *reinterpret_cast<const quint8 *>(data);
        return true;
    }
    if (type == "short" || type == "int16") {
        *value = readLittleEndian<qint16>(data);
        return true;
    }
    if (type == "ushort" || type == "uint16") {
        *value = readLittleEndian<quint16>(data);
        return true;
    }
    if (type == "int" || type == "int32") {
        *value = readLittleEndian<qint32>(data);
        return true;
    }
    if (type == "uint" || type == "uint32") {
        *value = readLittleEndian<quint32>(data);
        return true;
    }
    if (type == "float" || type == "float32") {
        float result = 0.0f;
        if (!readFloat32(data, &result))
            return false;
        *value = result;
        return true;
    }
    if (type == "double" || type == "float64") {
        double result = 0.0;
        if (!readFloat64(data, &result))
            return false;
        *value = result;
        return true;
    }
    return false;
}

float textPointIntensity(const QStringList &tokens)
{
    if (tokens.size() >= 6) {
        bool okR = false;
        bool okG = false;
        bool okB = false;
        const float r = tokens.at(3).toFloat(&okR);
        const float g = tokens.at(4).toFloat(&okG);
        const float b = tokens.at(5).toFloat(&okB);
        if (okR && okG && okB)
            return normalizedIntensity((r + g + b) / 3.0f);
    }

    if (tokens.size() >= 4) {
        bool ok = false;
        const float intensity = tokens.at(3).toFloat(&ok);
        if (ok)
            return normalizedIntensity(intensity);
    }

    return 0.75f;
}

} // namespace

PointCloudFileLoader::PointCloudFileLoader(QObject *parent)
    : QObject(parent)
    , m_alive(std::make_shared<std::atomic_bool>(true))
{
    setStatus(QStringLiteral("未加载静态点云模型。"));
}

PointCloudFileLoader::~PointCloudFileLoader()
{
    qInfo() << "销毁静态点云加载器，取消未完成加载任务";
    m_alive->store(false, std::memory_order_release);
    ++m_loadToken;
}

void PointCloudFileLoader::setGeometry(PointCloudGeometry *geometry)
{
    if (m_geometry == geometry)
        return;

    m_geometry = geometry;
    qInfo().noquote() << QStringLiteral("静态点云加载器绑定主几何：0x%1").arg(quintptr(m_geometry), 0, 16);
    emit geometryChanged();
}

void PointCloudFileLoader::setPreviewGeometry(PointCloudGeometry *geometry)
{
    if (m_previewGeometry == geometry)
        return;

    m_previewGeometry = geometry;
    qInfo().noquote() << QStringLiteral("静态点云加载器绑定预览几何：0x%1").arg(quintptr(m_previewGeometry), 0, 16);
    if (m_previewGeometry && !m_interactivePreviewPoints.isEmpty()) {
        const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;
        m_previewGeometry->setPoints(m_interactivePreviewPoints, 0, timestampUsec);
    }
    emit previewGeometryChanged();
}

void PointCloudFileLoader::setMaxPoints(int maxPoints)
{
    // maxPoints == 0 表示“加载全部有效点”。大于 0 时限制一个合理上限，
    // 避免误输入极端数值导致一次性分配过量内存。
    const int safeMaxPoints = maxPoints <= 0 ? 0 : std::clamp(maxPoints, 1000, 20000000);
    if (m_maxPoints == safeMaxPoints)
        return;

    m_maxPoints = safeMaxPoints;
    qInfo().noquote() << QStringLiteral("静态点云显示上限变更：%1").arg(m_maxPoints == 0 ? QStringLiteral("全部") : QString::number(m_maxPoints));
    emit maxPointsChanged();
}

bool PointCloudFileLoader::loadFromUrl(const QUrl &url)
{
    if (m_loading) {
        qWarning() << "拒绝新的静态点云加载请求：已有加载任务正在运行";
        setStatus(QStringLiteral("已有点云模型正在加载，请稍等。"));
        return false;
    }

    if (!m_geometry) {
        qCritical() << "静态点云加载失败：主 PointCloudGeometry 未绑定";
        setStatus(QStringLiteral("加载失败：点云渲染对象未绑定。"));
        return false;
    }

    QString pathError;
    const QString filePath = pointCloudFilePathForUrl(url, &pathError);
    if (filePath.isEmpty()) {
        qWarning() << "静态点云加载失败：文件路径为空";
        setStatus(pathError.isEmpty()
                      ? QStringLiteral("加载失败：文件路径为空。")
                      : QStringLiteral("加载失败：%1").arg(pathError));
        return false;
    }

    const QFileInfo fileInfo(filePath);
    const QString suffix = fileInfo.suffix().toLower();
    qInfo().noquote() << QStringLiteral("开始加载静态点云：path=%1，suffix=%2，size=%3 bytes，maxPoints=%4")
                          .arg(filePath,
                               suffix,
                               QString::number(fileInfo.exists() ? fileInfo.size() : -1),
                               m_maxPoints == 0 ? QStringLiteral("全部") : QString::number(m_maxPoints));
    setProgress(0);
    setLoading(true);
    setStatus(QStringLiteral("正在后台加载静态点云模型：0%"));

    if (suffix == QLatin1String("lvx2")) {
        return loadLvx2Async(filePath);
    }

    if (suffix == QLatin1String("bag") || suffix == QLatin1String("db3")) {
        return loadRosBagAsync(filePath);
    }

    if (suffix == QLatin1String("xyz") || suffix == QLatin1String("txt")
        || suffix == QLatin1String("csv")) {
        return loadTextXyzAsync(filePath);
    }

    // PCD/PLY 一般比 Livox Viewer CSV 小很多，首版先保持同步解析；
    // 若后续要加载超大 PCD/PLY，也可以沿用 CSV 的后台线程模式迁移。
    QVector<PointCloudPoint> parsedPoints;
    QString error;
    bool ok = false;

    if (suffix == QLatin1String("ply")) {
        ok = loadPly(filePath, &parsedPoints, &error);
    } else if (suffix == QLatin1String("pcd")) {
        ok = loadPcd(filePath, &parsedPoints, &error);
    } else {
        error = QStringLiteral("暂不支持 .%1，请使用 LVX2、ROS bag、PCD、PLY、XYZ、TXT 或 CSV。").arg(suffix);
    }

    if (!ok) {
        qCritical().noquote() << QStringLiteral("静态点云加载失败：path=%1，error=%2").arg(filePath, error);
        setLoading(false);
        setProgress(0);
        setStatus(QStringLiteral("加载失败：%1").arg(error));
        emit loadFinished(false);
        return false;
    }

    if (parsedPoints.isEmpty()) {
        qWarning().noquote() << QStringLiteral("静态点云加载失败：文件内没有可用点，path=%1").arg(filePath);
        setLoading(false);
        setProgress(0);
        setStatus(QStringLiteral("加载失败：文件内没有可用的 XYZ 点。"));
        emit loadFinished(false);
        return false;
    }

    const QVector<PointCloudPoint> visiblePoints = decimated(parsedPoints);
    const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;
    setPlaybackData(visiblePoints, {}, {}, {}, 0, 0);
    m_geometry->setPoints(visiblePoints, 0, timestampUsec);
    qInfo().noquote() << QStringLiteral("静态点云同步加载完成：path=%1，source=%2，visible=%3")
                          .arg(filePath)
                          .arg(parsedPoints.size())
                          .arg(visiblePoints.size());
    setLoadedFile(filePath);
    setProgress(100);
    setLoading(false);
    setStatus(QStringLiteral("已加载静态模型：%1，显示 %2 个可用点 / 文件点记录 %3 条。")
                  .arg(QFileInfo(filePath).fileName())
                  .arg(visiblePoints.size())
                  .arg(parsedPoints.size()));
    emit loadFinished(true);
    return true;
}

bool PointCloudFileLoader::loadTextXyzAsync(const QString &filePath)
{
    const quint64 token = ++m_loadToken;
    const int maxPoints = m_maxPoints;
    qInfo().noquote() << QStringLiteral("启动文本/CSV 点云后台加载：token=%1，path=%2，maxPoints=%3")
                          .arg(token)
                          .arg(filePath)
                          .arg(maxPoints == 0 ? QStringLiteral("全部") : QString::number(maxPoints));
    QPointer<PointCloudFileLoader> self(this);
    const std::shared_ptr<std::atomic_bool> alive = m_alive;

    auto future = QtConcurrent::run([self, alive, token, filePath, maxPoints]() {
        TextLoadResult localResult = loadTextPointCloudStreaming(
            filePath,
            maxPoints,
            [self, alive, token](int progress) {
                if (!alive->load(std::memory_order_acquire) || !self)
                    return;

                PointCloudFileLoader *receiver = self.data();
                QMetaObject::invokeMethod(receiver, [receiver, alive, token, progress]() {
                    if (!alive->load(std::memory_order_acquire) || receiver->m_loadToken != token)
                        return;
                    receiver->setProgress(progress);
                }, Qt::QueuedConnection);
            });

        if (!alive->load(std::memory_order_acquire) || !self)
            return;

        auto result = std::make_shared<TextLoadResult>(std::move(localResult));
        PointCloudFileLoader *receiver = self.data();
        QMetaObject::invokeMethod(receiver, [receiver, alive, token, filePath, result]() {
            if (!alive->load(std::memory_order_acquire))
                return;

            receiver->finishAsyncLoad(token,
                                      filePath,
                                      result->ok,
                                      result->error,
                                      result->points,
                                      result->sourcePointCount,
                                      result->playbackPoints,
                                      result->playbackTimesMs,
                                      result->playbackPoseSamples,
                                      result->playbackDurationMs,
                                      result->playbackFrameDurationMs);
        }, Qt::QueuedConnection);
    });
    Q_UNUSED(future);
    return true;
}

bool PointCloudFileLoader::loadLvx2Async(const QString &filePath)
{
    const quint64 token = ++m_loadToken;
    const int maxPoints = m_maxPoints;
    qInfo().noquote() << QStringLiteral("启动 LVX2 点云后台加载：token=%1，path=%2，maxPoints=%3")
                          .arg(token)
                          .arg(filePath)
                          .arg(maxPoints == 0 ? QStringLiteral("全部") : QString::number(maxPoints));
    QPointer<PointCloudFileLoader> self(this);
    const std::shared_ptr<std::atomic_bool> alive = m_alive;

    auto future = QtConcurrent::run([self, alive, token, filePath, maxPoints]() {
        TextLoadResult localResult = loadLvx2PointCloudStreaming(
            filePath,
            maxPoints,
            [self, alive, token](int progress) {
                if (!alive->load(std::memory_order_acquire) || !self)
                    return;

                PointCloudFileLoader *receiver = self.data();
                QMetaObject::invokeMethod(receiver, [receiver, alive, token, progress]() {
                    if (!alive->load(std::memory_order_acquire) || receiver->m_loadToken != token)
                        return;
                    receiver->setProgress(progress);
                }, Qt::QueuedConnection);
            });

        if (!alive->load(std::memory_order_acquire) || !self)
            return;

        auto result = std::make_shared<TextLoadResult>(std::move(localResult));
        PointCloudFileLoader *receiver = self.data();
        QMetaObject::invokeMethod(receiver, [receiver, alive, token, filePath, result]() {
            if (!alive->load(std::memory_order_acquire))
                return;

            receiver->finishAsyncLoad(token,
                                      filePath,
                                      result->ok,
                                      result->error,
                                      result->points,
                                      result->sourcePointCount,
                                      result->playbackPoints,
                                      result->playbackTimesMs,
                                      result->playbackPoseSamples,
                                      result->playbackDurationMs,
                                      result->playbackFrameDurationMs);
        }, Qt::QueuedConnection);
    });
    Q_UNUSED(future);
    return true;
}

bool PointCloudFileLoader::loadRosBagAsync(const QString &filePath)
{
    const quint64 token = ++m_loadToken;
    const int maxPoints = m_maxPoints;
    qInfo().noquote() << QStringLiteral("启动 ROS bag 点云后台加载：token=%1，path=%2，maxPoints=%3")
                          .arg(token)
                          .arg(filePath)
                          .arg(maxPoints == 0 ? QStringLiteral("全部") : QString::number(maxPoints));
    QPointer<PointCloudFileLoader> self(this);
    const std::shared_ptr<std::atomic_bool> alive = m_alive;

    auto future = QtConcurrent::run([self, alive, token, filePath, maxPoints]() {
        TextLoadResult localResult = loadRosBagPointCloudStreaming(
            filePath,
            maxPoints,
            [self, alive, token](int progress) {
                if (!alive->load(std::memory_order_acquire) || !self)
                    return;

                PointCloudFileLoader *receiver = self.data();
                QMetaObject::invokeMethod(receiver, [receiver, alive, token, progress]() {
                    if (!alive->load(std::memory_order_acquire) || receiver->m_loadToken != token)
                        return;
                    receiver->setProgress(progress);
                }, Qt::QueuedConnection);
            });

        if (!alive->load(std::memory_order_acquire) || !self)
            return;

        auto result = std::make_shared<TextLoadResult>(std::move(localResult));
        PointCloudFileLoader *receiver = self.data();
        QMetaObject::invokeMethod(receiver, [receiver, alive, token, filePath, result]() {
            if (!alive->load(std::memory_order_acquire))
                return;

            receiver->finishAsyncLoad(token,
                                      filePath,
                                      result->ok,
                                      result->error,
                                      result->points,
                                      result->sourcePointCount,
                                      result->playbackPoints,
                                      result->playbackTimesMs,
                                      result->playbackPoseSamples,
                                      result->playbackDurationMs,
                                      result->playbackFrameDurationMs);
        }, Qt::QueuedConnection);
    });
    Q_UNUSED(future);
    return true;
}

void PointCloudFileLoader::finishAsyncLoad(quint64 token,
                                           const QString &filePath,
                                           bool ok,
                                           const QString &error,
                                           const QVector<PointCloudPoint> &points,
                                           quint64 sourcePointCount,
                                           const QVector<PointCloudPoint> &playbackPoints,
                                           const QVector<int> &playbackTimesMs,
                                           const QVector<PointCloudPlaybackPoseSample> &playbackPoseSamples,
                                           int playbackDurationMs,
                                           int playbackFrameDurationMs)
{
    if (token != m_loadToken) {
        qInfo().noquote() << QStringLiteral("忽略过期点云加载结果：token=%1，currentToken=%2，path=%3")
                              .arg(token)
                              .arg(m_loadToken)
                              .arg(filePath);
        return;
    }

    if (!ok) {
        qCritical().noquote() << QStringLiteral("静态点云后台加载失败：token=%1，path=%2，error=%3")
                                    .arg(token)
                                    .arg(filePath)
                                    .arg(error);
        setLoading(false);
        setProgress(0);
        setStatus(QStringLiteral("加载失败：%1").arg(error));
        emit loadFinished(false);
        return;
    }

    if (points.isEmpty()) {
        qWarning().noquote() << QStringLiteral("静态点云后台加载完成但无可用点：token=%1，path=%2，source=%3")
                                    .arg(token)
                                    .arg(filePath)
                                    .arg(sourcePointCount);
        setLoading(false);
        setProgress(0);
        setStatus(QStringLiteral("加载失败：文件内没有可用的 XYZ 点。"));
        emit loadFinished(false);
        return;
    }

    const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;
    setPlaybackData(points,
                    playbackPoints,
                    playbackTimesMs,
                    playbackPoseSamples,
                    playbackDurationMs,
                    playbackFrameDurationMs);
    m_geometry->setPoints(points, 0, timestampUsec);
    qInfo().noquote() << QStringLiteral("静态点云后台加载完成：token=%1，path=%2，source=%3，visible=%4，playback=%5，poses=%6，durationMs=%7")
                          .arg(token)
                          .arg(filePath)
                          .arg(sourcePointCount)
                          .arg(points.size())
                          .arg(playbackPoints.size())
                          .arg(playbackPoseSamples.size())
                          .arg(playbackDurationMs);
    setLoadedFile(filePath);
    setProgress(100);
    setLoading(false);
    const QString poseSuffix = playbackPoseSamples.isEmpty()
                                   ? QString()
                                   : QStringLiteral("，位姿 %1 条").arg(playbackPoseSamples.size());
    setStatus(QStringLiteral("已加载静态模型：%1，显示 %2 个可用点 / 文件点记录 %3 条%4。")
                  .arg(QFileInfo(filePath).fileName())
                  .arg(points.size())
                  .arg(sourcePointCount)
                  .arg(poseSuffix));
    emit loadFinished(true);
}

void PointCloudFileLoader::clearPlaybackData()
{
    const bool hadPlaybackState = m_playbackAvailable || m_showingPlaybackFrame
                                  || m_playbackDurationMs > 0 || m_playbackPositionMs > 0;
    if (hadPlaybackState || !m_accumulatedPoints.isEmpty() || !m_playbackPoints.isEmpty()) {
        qInfo().noquote() << QStringLiteral("清理静态点云回放缓存：累计点=%1，预览点=%2，回放点=%3，durationMs=%4")
                              .arg(m_accumulatedPoints.size())
                              .arg(m_interactivePreviewPoints.size())
                              .arg(m_playbackPoints.size())
                              .arg(m_playbackDurationMs);
    }

    m_accumulatedPoints.clear();
    m_interactivePreviewPoints.clear();
    m_playbackPoints.clear();
    m_playbackTimesMs.clear();
    m_playbackPoseSamples.clear();
    m_hasPlaybackPose = false;
    m_playbackPoseText.clear();
    m_playbackPosePositionMeters = QVector3D();
    m_playbackPoseYawDegrees = 0.0f;
    m_interactivePreviewEnabled = false;
    m_playbackAvailable = false;
    m_showingPlaybackFrame = false;
    m_playbackDurationMs = 0;
    m_playbackFrameDurationMs = 50;
    m_playbackPositionMs = 0;
    if (m_previewGeometry)
        m_previewGeometry->setPoints({}, 0, quint64(QDateTime::currentMSecsSinceEpoch()) * 1000);

    if (hadPlaybackState) {
        emit playbackStateChanged();
        emit playbackPositionChanged();
    }
    emit playbackPoseChanged();
    emit interactivePreviewChanged();
}

void PointCloudFileLoader::setPlaybackData(const QVector<PointCloudPoint> &accumulatedPoints,
                                           const QVector<PointCloudPoint> &playbackPoints,
                                           const QVector<int> &playbackTimesMs,
                                           const QVector<PointCloudPlaybackPoseSample> &playbackPoseSamples,
                                           int playbackDurationMs,
                                           int playbackFrameDurationMs)
{
    m_accumulatedPoints = accumulatedPoints;
    m_interactivePreviewPoints = uniformlyDecimatedPointsWithTimes(m_accumulatedPoints,
                                                                   {},
                                                                   InteractivePreviewPointLimit,
                                                                   nullptr);
    if (m_previewGeometry) {
        const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;
        m_previewGeometry->setPoints(m_interactivePreviewPoints, 0, timestampUsec);
    }
    m_playbackPoints = playbackPoints;
    m_playbackTimesMs = playbackTimesMs;
    m_playbackPoseSamples = playbackPoseSamples;
    m_playbackDurationMs = std::max(0, playbackDurationMs);
    m_playbackFrameDurationMs = playbackFrameDurationMs > 0 ? playbackFrameDurationMs : 50;
    m_playbackPositionMs = 0;
    m_showingPlaybackFrame = false;
    m_interactivePreviewEnabled = false;

    // 只有 LVX2、ROS bag 这类带时间戳的录制文件会填充 playbackTimesMs。
    // CSV/PCD/PLY 是静态地图文件，仍然只显示累计点云，不显示回放控件。
    m_playbackAvailable = !m_playbackPoints.isEmpty()
                          && m_playbackPoints.size() == m_playbackTimesMs.size()
                          && m_playbackDurationMs > 0;

    updatePlaybackPose();

    qInfo().noquote() << QStringLiteral("设置静态点云回放数据：累计点=%1，交互预览点=%2，回放点=%3，时间戳=%4，位姿=%5，durationMs=%6，frameDurationMs=%7，playbackAvailable=%8")
                          .arg(m_accumulatedPoints.size())
                          .arg(m_interactivePreviewPoints.size())
                          .arg(m_playbackPoints.size())
                          .arg(m_playbackTimesMs.size())
                          .arg(m_playbackPoseSamples.size())
                          .arg(m_playbackDurationMs)
                          .arg(m_playbackFrameDurationMs)
                          .arg(m_playbackAvailable ? QStringLiteral("true") : QStringLiteral("false"));

    emit playbackStateChanged();
    emit playbackPositionChanged();
    emit interactivePreviewChanged();
}

void PointCloudFileLoader::showAccumulatedPointCloud()
{
    if (!m_geometry || m_accumulatedPoints.isEmpty()) {
        qWarning().noquote() << QStringLiteral("恢复累计点云失败：geometry=%1，累计点=%2")
                                    .arg(m_geometry ? QStringLiteral("ready") : QStringLiteral("null"))
                                    .arg(m_accumulatedPoints.size());
        return;
    }

    const bool wasShowingPlaybackFrame = m_showingPlaybackFrame;
    const bool wasInteractivePreview = m_interactivePreviewEnabled;
    m_showingPlaybackFrame = false;
    m_interactivePreviewEnabled = false;
    if (wasShowingPlaybackFrame) {
        const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;
        m_geometry->setPoints(m_accumulatedPoints, 0, timestampUsec);
    }

    qInfo().noquote() << QStringLiteral("恢复累计点云显示：累计点=%1，原回放帧=%2，原交互预览=%3")
                          .arg(m_accumulatedPoints.size())
                          .arg(wasShowingPlaybackFrame ? QStringLiteral("true") : QStringLiteral("false"))
                          .arg(wasInteractivePreview ? QStringLiteral("true") : QStringLiteral("false"));

    if (wasShowingPlaybackFrame)
        emit playbackStateChanged();
    if (wasInteractivePreview)
        emit interactivePreviewChanged();
}

void PointCloudFileLoader::setInteractivePreviewEnabled(bool enabled)
{
    if (!m_geometry || !m_previewGeometry || m_accumulatedPoints.isEmpty()
        || m_interactivePreviewPoints.isEmpty()) {
        qWarning().noquote() << QStringLiteral("切换交互预览失败：geometry=%1，previewGeometry=%2，累计点=%3，预览点=%4")
                                    .arg(m_geometry ? QStringLiteral("ready") : QStringLiteral("null"),
                                         m_previewGeometry ? QStringLiteral("ready") : QStringLiteral("null"))
                                    .arg(m_accumulatedPoints.size())
                                    .arg(m_interactivePreviewPoints.size());
        return;
    }

    // 回放模式本身只显示一个较短时间窗，点数已经远小于完整地图；
    // 此时不再切交互预览，避免拖动视角时打断用户正在看的录制时间片。
    if (m_showingPlaybackFrame) {
        qInfo() << "忽略交互预览切换：当前正在显示录制文件回放时间片";
        return;
    }

    const bool shouldEnable = enabled
                              && m_accumulatedPoints.size() > m_interactivePreviewPoints.size();
    if (m_interactivePreviewEnabled == shouldEnable)
        return;

    m_interactivePreviewEnabled = shouldEnable;
    qInfo().noquote() << QStringLiteral("交互预览切换：%1，累计点=%2，预览点=%3")
                          .arg(m_interactivePreviewEnabled ? QStringLiteral("开启") : QStringLiteral("关闭"))
                          .arg(m_accumulatedPoints.size())
                          .arg(m_interactivePreviewPoints.size());
    emit interactivePreviewChanged();
}

void PointCloudFileLoader::setPlaybackPositionMs(int positionMs)
{
    if (!m_playbackAvailable) {
        qWarning().noquote() << QStringLiteral("设置回放进度失败：当前文件没有可用时间轴，positionMs=%1").arg(positionMs);
        return;
    }

    const int safePosition = std::clamp(positionMs, 0, m_playbackDurationMs);
    const bool positionChanged = safePosition != m_playbackPositionMs;
    const bool stateChanged = m_showingPlaybackFrame == m_accumulatedPlayback;

    m_playbackPositionMs = safePosition;
    m_showingPlaybackFrame = !m_accumulatedPlayback;
    // 飞行记录回放仅推进轨迹和位置，累计点云几何保持不变。
    if (!m_accumulatedPlayback) updatePlaybackGeometry();
    updatePlaybackPose();

    if (positionChanged)
        emit playbackPositionChanged();
    if (stateChanged)
        emit playbackStateChanged();
}

void PointCloudFileLoader::updatePlaybackGeometry()
{
    if (!m_geometry || m_playbackPoints.isEmpty()
        || m_playbackPoints.size() != m_playbackTimesMs.size()) {
        qWarning().noquote() << QStringLiteral("更新录制文件回放帧失败：geometry=%1，回放点=%2，时间戳=%3")
                                    .arg(m_geometry ? QStringLiteral("ready") : QStringLiteral("null"))
                                    .arg(m_playbackPoints.size())
                                    .arg(m_playbackTimesMs.size());
        return;
    }

    // 用一个短时间窗显示当前扫描片段。时间窗太短会导致点太稀，
    // 太长又会变成累计地图；4 个 frame duration 对 Mid-360 室内录制比较适中。
    const int windowMs = std::clamp(m_playbackFrameDurationMs * 4, 80, 300);
    int beginTimeMs = std::max(0, m_playbackPositionMs - windowMs / 2);
    int endTimeMs = std::min(m_playbackDurationMs, beginTimeMs + windowMs);
    beginTimeMs = std::max(0, endTimeMs - windowMs);

    auto beginIt = std::lower_bound(m_playbackTimesMs.cbegin(), m_playbackTimesMs.cend(), beginTimeMs);
    auto endIt = std::upper_bound(m_playbackTimesMs.cbegin(), m_playbackTimesMs.cend(), endTimeMs);

    // 如果某个时间点附近刚好没有被抽样保留下来的点，取最近一小段，避免播放时画面闪空。
    if (beginIt == endIt) {
        beginIt = std::lower_bound(m_playbackTimesMs.cbegin(), m_playbackTimesMs.cend(), m_playbackPositionMs);
        const qsizetype centerIndex = std::clamp<qsizetype>(std::distance(m_playbackTimesMs.cbegin(), beginIt),
                                                            0,
                                                            std::max<qsizetype>(0, m_playbackTimesMs.size() - 1));
        const qsizetype halfCount = 800;
        const qsizetype beginIndex = std::max<qsizetype>(0, centerIndex - halfCount);
        const qsizetype endIndex = std::min<qsizetype>(m_playbackPoints.size(), centerIndex + halfCount);
        beginIt = m_playbackTimesMs.cbegin() + beginIndex;
        endIt = m_playbackTimesMs.cbegin() + endIndex;
        qInfo().noquote() << QStringLiteral("录制文件回放窗口附近无点，改用邻近点补帧：positionMs=%1，index=%2..%3")
                              .arg(m_playbackPositionMs)
                              .arg(beginIndex)
                              .arg(endIndex);
    }

    const qsizetype beginIndex = std::distance(m_playbackTimesMs.cbegin(), beginIt);
    const qsizetype endIndex = std::distance(m_playbackTimesMs.cbegin(), endIt);
    QVector<PointCloudPoint> visiblePoints;
    visiblePoints.reserve(int(std::max<qsizetype>(0, endIndex - beginIndex)));
    for (qsizetype i = beginIndex; i < endIndex; ++i)
        visiblePoints.append(m_playbackPoints.at(i));

    if (visiblePoints.isEmpty()) {
        qWarning().noquote() << QStringLiteral("录制文件回放帧为空：positionMs=%1，window=%2..%3")
                                    .arg(m_playbackPositionMs)
                                    .arg(beginTimeMs)
                                    .arg(endTimeMs);
        return;
    }

    const quint64 timestampUsec = quint64(QDateTime::currentMSecsSinceEpoch()) * 1000;
    m_geometry->setPoints(visiblePoints, 0, timestampUsec);

    static int playbackUpdateLogCounter = 0;
    ++playbackUpdateLogCounter;
    if (playbackUpdateLogCounter <= 5 || playbackUpdateLogCounter % 50 == 0) {
        qInfo().noquote() << QStringLiteral("录制文件回放帧更新：positionMs=%1，window=%2..%3，显示点=%4")
                              .arg(m_playbackPositionMs)
                              .arg(beginTimeMs)
                              .arg(endTimeMs)
                              .arg(visiblePoints.size());
    }
}

void PointCloudFileLoader::updatePlaybackPose()
{
    if (m_playbackPoseSamples.isEmpty()) {
        const bool hadPose = m_hasPlaybackPose;
        m_hasPlaybackPose = false;
        m_playbackPoseText.clear();
        m_playbackPosePositionMeters = QVector3D();
        m_playbackPoseYawDegrees = 0.0f;
        if (hadPose)
            emit playbackPoseChanged();
        return;
    }

    auto lowerIt = std::lower_bound(m_playbackPoseSamples.cbegin(),
                                    m_playbackPoseSamples.cend(),
                                    m_playbackPositionMs,
                                    [](const PointCloudPlaybackPoseSample &sample, int timeMs) {
                                        return sample.timeMs < timeMs;
                                    });

    qsizetype poseIndex = 0;
    if (lowerIt == m_playbackPoseSamples.cend()) {
        poseIndex = m_playbackPoseSamples.size() - 1;
    } else if (lowerIt == m_playbackPoseSamples.cbegin()) {
        poseIndex = 0;
    } else {
        const qsizetype nextIndex = std::distance(m_playbackPoseSamples.cbegin(), lowerIt);
        const qsizetype previousIndex = nextIndex - 1;
        const int previousDelta = std::abs(m_playbackPositionMs - m_playbackPoseSamples.at(previousIndex).timeMs);
        const int nextDelta = std::abs(m_playbackPoseSamples.at(nextIndex).timeMs - m_playbackPositionMs);
        poseIndex = m_accumulatedPlayback ? (lowerIt->timeMs == m_playbackPositionMs ? nextIndex : previousIndex)
                                         : (previousDelta <= nextDelta ? previousIndex : nextIndex);
    }

    const PointCloudPlaybackPoseSample &sample = m_playbackPoseSamples.at(poseIndex);
    m_hasPlaybackPose = true;
    m_playbackPosePositionMeters = sample.positionMeters;
    m_playbackPoseYawDegrees = sample.yawDegrees;
    m_playbackPoseText = QStringLiteral("BAG 位姿 %1/%2  X:%3  Y:%4  Z:%5")
                             .arg(poseIndex + 1)
                             .arg(m_playbackPoseSamples.size())
                             .arg(sample.positionMeters.x(), 0, 'f', 2)
                             .arg(sample.positionMeters.y(), 0, 'f', 2)
                             .arg(sample.positionMeters.z(), 0, 'f', 2);

    static int poseUpdateLogCounter = 0;
    ++poseUpdateLogCounter;
    if (poseUpdateLogCounter <= 5 || poseUpdateLogCounter % 50 == 0) {
        qInfo().noquote() << QStringLiteral("录制位姿更新：positionMs=%1，poseIndex=%2/%3，位置=[%4,%5,%6]，yaw=%7")
                              .arg(m_playbackPositionMs)
                              .arg(poseIndex + 1)
                              .arg(m_playbackPoseSamples.size())
                              .arg(sample.positionMeters.x(), 0, 'f', 3)
                              .arg(sample.positionMeters.y(), 0, 'f', 3)
                              .arg(sample.positionMeters.z(), 0, 'f', 3)
                              .arg(sample.yawDegrees, 0, 'f', 2);
    }
    emit playbackPoseChanged();
}

bool PointCloudFileLoader::loadPly(const QString &filePath,
                                   QVector<PointCloudPoint> *points,
                                   QString *error) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error)
            *error = file.errorString();
        qCritical().noquote() << QStringLiteral("PLY 打开失败：path=%1，error=%2").arg(filePath, file.errorString());
        return false;
    }

    const QByteArray data = file.readAll();
    if (!data.startsWith("ply")) {
        if (error)
            *error = QStringLiteral("不是有效的 PLY 文件。");
        qCritical().noquote() << QStringLiteral("PLY 文件头无效：path=%1，size=%2").arg(filePath).arg(data.size());
        return false;
    }

    // PLY 的关键是先读 header：顶点数量、存储格式、每个顶点属性顺序都在头部声明。
    // 这里支持点云常见的 ascii 和 binary_little_endian；face 等网格元素会被自然忽略。
    QByteArray format;
    QByteArray currentElement;
    QVector<PlyProperty> vertexProperties;
    int vertexCount = 0;
    qsizetype dataStart = -1;
    qsizetype cursor = 0;
    while (cursor < data.size()) {
        const qsizetype lineEnd = data.indexOf('\n', cursor);
        const qsizetype end = lineEnd >= 0 ? lineEnd : data.size();
        QByteArray line = data.mid(cursor, end - cursor).trimmed();
        cursor = lineEnd >= 0 ? lineEnd + 1 : data.size();

        if (line == "end_header") {
            dataStart = cursor;
            break;
        }

        const QList<QByteArray> tokens = line.simplified().split(' ');
        if (tokens.isEmpty())
            continue;

        if (tokens.first() == "format" && tokens.size() >= 2) {
            format = tokens.at(1).toLower();
        } else if (tokens.first() == "element" && tokens.size() >= 3) {
            currentElement = tokens.at(1).toLower();
            if (currentElement == "vertex")
                vertexCount = tokens.at(2).toInt();
        } else if (tokens.first() == "property" && currentElement == "vertex") {
            if (tokens.size() >= 3 && tokens.at(1) != "list") {
                vertexProperties.append({ tokens.at(2), tokens.at(1) });
            } else {
                if (error)
                    *error = QStringLiteral("PLY 顶点属性包含 list，当前加载器暂不支持这种顶点布局。");
                return false;
            }
        }
    }

    if (dataStart < 0 || vertexCount <= 0 || vertexProperties.isEmpty()) {
        if (error)
            *error = QStringLiteral("PLY 头缺少 vertex 或 end_header。");
        qCritical().noquote() << QStringLiteral("PLY 头解析失败：path=%1，vertexCount=%2，properties=%3")
                                    .arg(filePath)
                                    .arg(vertexCount)
                                    .arg(vertexProperties.size());
        return false;
    }

    qInfo().noquote() << QStringLiteral("PLY 头解析完成：path=%1，format=%2，vertexCount=%3，properties=%4")
                          .arg(filePath,
                               QString::fromLatin1(format),
                               QString::number(vertexCount),
                               QString::number(vertexProperties.size()));

    const int xIndex = indexOfProperty(vertexProperties, { "x" });
    const int yIndex = indexOfProperty(vertexProperties, { "y" });
    const int zIndex = indexOfProperty(vertexProperties, { "z" });
    if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
        if (error)
            *error = QStringLiteral("PLY 顶点属性缺少 x/y/z。");
        qCritical().noquote() << QStringLiteral("PLY 缺少 x/y/z 属性：path=%1").arg(filePath);
        return false;
    }

    const int intensityIndex = indexOfProperty(vertexProperties,
                                               { "intensity", "reflectance", "scalar_intensity" });
    const int redIndex = indexOfProperty(vertexProperties, { "red", "r" });
    const int greenIndex = indexOfProperty(vertexProperties, { "green", "g" });
    const int blueIndex = indexOfProperty(vertexProperties, { "blue", "b" });

    points->clear();
    points->reserve(std::min(vertexCount, m_maxPoints));

    if (format == "ascii") {
        // ASCII PLY 每一行就是一个 vertex，属性顺序完全按 header 中 property 的顺序读取。
        qsizetype bodyCursor = dataStart;
        int parsedVertices = 0;
        while (bodyCursor < data.size() && parsedVertices < vertexCount) {
            const qsizetype lineEnd = data.indexOf('\n', bodyCursor);
            const qsizetype end = lineEnd >= 0 ? lineEnd : data.size();
            const QByteArray line = data.mid(bodyCursor, end - bodyCursor);
            bodyCursor = lineEnd >= 0 ? lineEnd + 1 : data.size();

            const QStringList tokens = splitNumberTokens(line);
            if (tokens.size() < vertexProperties.size())
                continue;

            bool okX = false;
            bool okY = false;
            bool okZ = false;
            const float x = tokens.at(xIndex).toFloat(&okX);
            const float y = tokens.at(yIndex).toFloat(&okY);
            const float z = tokens.at(zIndex).toFloat(&okZ);
            if (!okX || !okY || !okZ)
                continue;

            float intensity = 0.75f;
            if (intensityIndex >= 0) {
                bool okIntensity = false;
                intensity = normalizedIntensity(tokens.at(intensityIndex).toFloat(&okIntensity));
                if (!okIntensity)
                    intensity = 0.75f;
            } else if (redIndex >= 0 && greenIndex >= 0 && blueIndex >= 0) {
                intensity = normalizedIntensity((tokens.at(redIndex).toFloat()
                                                 + tokens.at(greenIndex).toFloat()
                                                 + tokens.at(blueIndex).toFloat()) / 3.0f);
            }

            points->append({ QVector3D(x, y, z), intensity });
            ++parsedVertices;
        }
        qInfo().noquote() << QStringLiteral("PLY ASCII 读取完成：path=%1，points=%2").arg(filePath).arg(points->size());
        return true;
    }

    if (format != "binary_little_endian") {
        if (error)
            *error = QStringLiteral("当前只支持 ASCII 或 binary_little_endian PLY。");
        qCritical().noquote() << QStringLiteral("PLY 格式不支持：path=%1，format=%2")
                                    .arg(filePath, QString::fromLatin1(format));
        return false;
    }

    // 二进制 PLY 没有字段名分隔符，必须按 property 类型尺寸计算每个字段的字节偏移。
    QVector<qsizetype> offsets;
    offsets.reserve(vertexProperties.size());
    qsizetype vertexStep = 0;
    for (const PlyProperty &property : vertexProperties) {
        const int size = plyTypeSize(property.type);
        if (size <= 0) {
            if (error)
                *error = QStringLiteral("PLY 顶点属性类型 %1 不支持。")
                             .arg(QString::fromLatin1(property.type));
            return false;
        }
        offsets.append(vertexStep);
        vertexStep += size;
    }

    const qsizetype availableVertices = (data.size() - dataStart) / vertexStep;
    const int count = std::min<int>(vertexCount, int(availableVertices));
    for (int i = 0; i < count; ++i) {
        const char *base = data.constData() + dataStart + qsizetype(i) * vertexStep;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!readPlyScalar(base + offsets.at(xIndex), vertexProperties.at(xIndex).type, &x)
            || !readPlyScalar(base + offsets.at(yIndex), vertexProperties.at(yIndex).type, &y)
            || !readPlyScalar(base + offsets.at(zIndex), vertexProperties.at(zIndex).type, &z)) {
            continue;
        }

        float intensity = 0.75f;
        if (intensityIndex >= 0) {
            double rawIntensity = 0.0;
            if (readPlyScalar(base + offsets.at(intensityIndex),
                              vertexProperties.at(intensityIndex).type,
                              &rawIntensity)) {
                intensity = normalizedIntensity(float(rawIntensity));
            }
        } else if (redIndex >= 0 && greenIndex >= 0 && blueIndex >= 0) {
            double r = 0.0;
            double g = 0.0;
            double b = 0.0;
            if (readPlyScalar(base + offsets.at(redIndex), vertexProperties.at(redIndex).type, &r)
                && readPlyScalar(base + offsets.at(greenIndex), vertexProperties.at(greenIndex).type, &g)
                && readPlyScalar(base + offsets.at(blueIndex), vertexProperties.at(blueIndex).type, &b)) {
                intensity = normalizedIntensity(float((r + g + b) / 3.0));
            }
        }

        points->append({ QVector3D(float(x), float(y), float(z)), intensity });
    }

    qInfo().noquote() << QStringLiteral("PLY binary_little_endian 读取完成：path=%1，declared=%2，available=%3，points=%4")
                          .arg(filePath)
                          .arg(vertexCount)
                          .arg(availableVertices)
                          .arg(points->size());
    return true;
}

bool PointCloudFileLoader::loadPcd(const QString &filePath,
                                   QVector<PointCloudPoint> *points,
                                   QString *error) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly)) {
        if (error)
            *error = file.errorString();
        qCritical().noquote() << QStringLiteral("PCD 打开失败：path=%1，error=%2").arg(filePath, file.errorString());
        return false;
    }

    const QByteArray data = file.readAll();
    // PCD 头部用 FIELDS/SIZE/TYPE/COUNT 描述点结构，DATA 行之后才是真正的点数据。
    // 这里按官方 PCD 语义解析字段偏移，因此字段顺序不是 x y z intensity 也可以读取。
    QByteArrayList names;
    QList<int> sizes;
    QByteArrayList types;
    QList<int> counts;
    QByteArray dataMode;
    int declaredPoints = -1;
    int width = 0;
    int height = 1;
    qsizetype dataStart = -1;

    qsizetype cursor = 0;
    while (cursor < data.size()) {
        const qsizetype lineEnd = data.indexOf('\n', cursor);
        const qsizetype end = lineEnd >= 0 ? lineEnd : data.size();
        QByteArray line = data.mid(cursor, end - cursor).trimmed();
        cursor = lineEnd >= 0 ? lineEnd + 1 : data.size();
        if (line.isEmpty() || line.startsWith('#'))
            continue;

        const QByteArrayList tokens = line.simplified().split(' ');
        if (tokens.isEmpty())
            continue;

        const QByteArray key = tokens.first().toUpper();
        if (key == "FIELDS" || key == "FIELD") {
            names = tokens.mid(1);
        } else if (key == "SIZE") {
            sizes.clear();
            for (int i = 1; i < tokens.size(); ++i)
                sizes.append(tokens.at(i).toInt());
        } else if (key == "TYPE") {
            types = tokens.mid(1);
        } else if (key == "COUNT") {
            counts.clear();
            for (int i = 1; i < tokens.size(); ++i)
                counts.append(std::max(1, tokens.at(i).toInt()));
        } else if (key == "WIDTH" && tokens.size() >= 2) {
            width = tokens.at(1).toInt();
        } else if (key == "HEIGHT" && tokens.size() >= 2) {
            height = std::max(1, tokens.at(1).toInt());
        } else if (key == "POINTS" && tokens.size() >= 2) {
            declaredPoints = tokens.at(1).toInt();
        } else if (key == "DATA" && tokens.size() >= 2) {
            dataMode = tokens.at(1).toLower();
            dataStart = cursor;
            break;
        }
    }

    if (names.isEmpty() || sizes.size() != names.size() || types.size() != names.size()
        || dataStart < 0) {
        if (error)
            *error = QStringLiteral("PCD 头缺少 FIELDS/SIZE/TYPE/DATA。");
        qCritical().noquote() << QStringLiteral("PCD 头解析失败：path=%1，fields=%2，sizes=%3，types=%4，dataStart=%5")
                                    .arg(filePath)
                                    .arg(names.size())
                                    .arg(sizes.size())
                                    .arg(types.size())
                                    .arg(dataStart);
        return false;
    }

    qInfo().noquote() << QStringLiteral("PCD 头解析完成：path=%1，mode=%2，declared=%3，width=%4，height=%5，fields=%6")
                          .arg(filePath,
                               QString::fromLatin1(dataMode),
                               QString::number(declaredPoints),
                               QString::number(width),
                               QString::number(height),
                               QString::number(names.size()));

    if (counts.isEmpty()) {
        while (counts.size() < names.size())
            counts.append(1);
    }
    if (counts.size() != names.size()) {
        if (error)
            *error = QStringLiteral("PCD COUNT 数量与字段数量不一致。");
        return false;
    }

    QVector<FieldLayout> fields;
    fields.reserve(names.size());
    qsizetype byteOffset = 0;
    int tokenOffset = 0;
    for (int i = 0; i < names.size(); ++i) {
        fields.append({ names.at(i).toLower(),
                        types.at(i).toUpper(),
                        sizes.at(i),
                        counts.at(i),
                        byteOffset,
                        tokenOffset });
        byteOffset += qsizetype(sizes.at(i)) * counts.at(i);
        tokenOffset += counts.at(i);
    }

    const int xIndex = indexOfField(fields, { "x" });
    const int yIndex = indexOfField(fields, { "y" });
    const int zIndex = indexOfField(fields, { "z" });
    if (xIndex < 0 || yIndex < 0 || zIndex < 0) {
        if (error)
            *error = QStringLiteral("PCD 字段缺少 x/y/z。");
        qCritical().noquote() << QStringLiteral("PCD 缺少 x/y/z 字段：path=%1").arg(filePath);
        return false;
    }

    const int intensityIndex = indexOfField(fields, { "intensity", "reflectivity", "i" });
    if (declaredPoints <= 0 && width > 0)
        declaredPoints = width * height;

    points->clear();
    points->reserve(declaredPoints > 0 ? std::min(declaredPoints, m_maxPoints) : m_maxPoints);

    if (dataMode == "ascii") {
        // ASCII PCD 按字段 token 下标读取；COUNT > 1 的字段会占用多个 token。
        qsizetype bodyCursor = dataStart;
        int parsedLines = 0;
        while (bodyCursor < data.size() && (declaredPoints <= 0 || parsedLines < declaredPoints)) {
            const qsizetype lineEnd = data.indexOf('\n', bodyCursor);
            const qsizetype end = lineEnd >= 0 ? lineEnd : data.size();
            const QByteArray line = data.mid(bodyCursor, end - bodyCursor);
            bodyCursor = lineEnd >= 0 ? lineEnd + 1 : data.size();

            const QStringList tokens = splitNumberTokens(line);
            if (tokens.size() < tokenOffset)
                continue;

            bool okX = false;
            bool okY = false;
            bool okZ = false;
            const float x = tokens.at(fields.at(xIndex).tokenOffset).toFloat(&okX);
            const float y = tokens.at(fields.at(yIndex).tokenOffset).toFloat(&okY);
            const float z = tokens.at(fields.at(zIndex).tokenOffset).toFloat(&okZ);
            if (!okX || !okY || !okZ)
                continue;

            float intensity = 0.75f;
            if (intensityIndex >= 0) {
                bool okIntensity = false;
                intensity = normalizedIntensity(tokens.at(fields.at(intensityIndex).tokenOffset)
                                                    .toFloat(&okIntensity));
                if (!okIntensity)
                    intensity = 0.75f;
            }

            points->append({ QVector3D(x, y, z), intensity });
            ++parsedLines;
        }
        qInfo().noquote() << QStringLiteral("PCD ASCII 读取完成：path=%1，points=%2").arg(filePath).arg(points->size());
        return true;
    }

    if (dataMode == "binary_compressed") {
        if (error)
            *error = QStringLiteral("PCD binary_compressed 需要解压，当前 demo 先支持 ascii 和 binary。");
        qCritical().noquote() << QStringLiteral("PCD 格式不支持 binary_compressed：path=%1").arg(filePath);
        return false;
    }

    if (dataMode != "binary") {
        if (error)
            *error = QStringLiteral("未知的 PCD DATA 类型：%1。").arg(QString::fromLatin1(dataMode));
        qCritical().noquote() << QStringLiteral("PCD DATA 类型未知：path=%1，mode=%2")
                                    .arg(filePath, QString::fromLatin1(dataMode));
        return false;
    }

    if (byteOffset <= 0) {
        if (error)
            *error = QStringLiteral("PCD 点步长无效。");
        return false;
    }

    // binary PCD 使用 header 中的 SIZE/TYPE/COUNT 算出单点步长，再按小端读取标量。
    const int availablePoints = int((data.size() - dataStart) / byteOffset);
    const int count = declaredPoints > 0 ? std::min(declaredPoints, availablePoints) : availablePoints;
    for (int i = 0; i < count; ++i) {
        const char *base = data.constData() + dataStart + qsizetype(i) * byteOffset;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        if (!readPcdScalar(base + fields.at(xIndex).byteOffset,
                           fields.at(xIndex).size,
                           fields.at(xIndex).type,
                           &x)
            || !readPcdScalar(base + fields.at(yIndex).byteOffset,
                              fields.at(yIndex).size,
                              fields.at(yIndex).type,
                              &y)
            || !readPcdScalar(base + fields.at(zIndex).byteOffset,
                              fields.at(zIndex).size,
                              fields.at(zIndex).type,
                              &z)) {
            continue;
        }

        float intensity = 0.75f;
        if (intensityIndex >= 0) {
            double rawIntensity = 0.0;
            if (readPcdScalar(base + fields.at(intensityIndex).byteOffset,
                              fields.at(intensityIndex).size,
                              fields.at(intensityIndex).type,
                              &rawIntensity)) {
                intensity = normalizedIntensity(float(rawIntensity));
            }
        }

        points->append({ QVector3D(float(x), float(y), float(z)), intensity });
    }

    qInfo().noquote() << QStringLiteral("PCD binary 读取完成：path=%1，declared=%2，available=%3，points=%4")
                          .arg(filePath)
                          .arg(declaredPoints)
                          .arg(availablePoints)
                          .arg(points->size());
    return true;
}

bool PointCloudFileLoader::loadTextXyz(const QString &filePath,
                                       QVector<PointCloudPoint> *points,
                                       QString *error) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        if (error)
            *error = file.errorString();
        return false;
    }

    // 简单文本点云用于快速联调：支持空格、逗号、分号或 tab 分隔。
    // 第 4 列可作为 intensity；若存在第 4~6 列，也可按 RGB 粗略换算亮度。
    points->clear();
    while (!file.atEnd()) {
        const QByteArray line = file.readLine().trimmed();
        if (line.isEmpty() || line.startsWith('#') || line.startsWith("//"))
            continue;

        const QStringList tokens = splitNumberTokens(line);
        if (tokens.size() < 3)
            continue;

        bool okX = false;
        bool okY = false;
        bool okZ = false;
        const float x = tokens.at(0).toFloat(&okX);
        const float y = tokens.at(1).toFloat(&okY);
        const float z = tokens.at(2).toFloat(&okZ);
        if (!okX || !okY || !okZ)
            continue;

        points->append({ QVector3D(x, y, z), textPointIntensity(tokens) });
    }

    return true;
}

QVector<PointCloudPoint> PointCloudFileLoader::decimated(const QVector<PointCloudPoint> &points) const
{
    if (m_maxPoints <= 0 || points.size() <= m_maxPoints)
        return points;

    // 使用均匀抽样而不是随机抽样，保证每次加载同一文件得到稳定的可视结果，
    // 也能尽量保留模型从头到尾的空间覆盖。
    QVector<PointCloudPoint> result;
    result.reserve(m_maxPoints);
    const double step = double(points.size()) / double(m_maxPoints);
    for (int i = 0; i < m_maxPoints; ++i) {
        const qsizetype sourceIndex = std::min<qsizetype>(points.size() - 1,
                                                          qsizetype(std::floor(double(i) * step)));
        result.append(points.at(sourceIndex));
    }
    return result;
}

void PointCloudFileLoader::setStatus(const QString &status)
{
    if (m_status == status)
        return;

    m_status = status;
    emit statusChanged();
}

void PointCloudFileLoader::setLoadedFile(const QString &filePath)
{
    if (m_loadedFile == filePath)
        return;

    m_loadedFile = filePath;
    emit loadedFileChanged();
}

void PointCloudFileLoader::setLoading(bool loading)
{
    if (m_loading == loading)
        return;

    m_loading = loading;
    emit loadingChanged();
}

void PointCloudFileLoader::setProgress(int progress)
{
    const int safeProgress = std::clamp(progress, 0, 100);
    if (m_progress == safeProgress)
        return;

    m_progress = safeProgress;
    emit progressChanged();
    if (m_loading)
        setStatus(QStringLiteral("正在后台加载静态点云模型：%1%").arg(m_progress));
}

QVariantList PointCloudFileLoader::flightTrajectory() const
{
    QVariantList points;
    const int step=std::max(1,int(m_playbackPoseSamples.size()/20000));
    for(int i=0;i<m_playbackPoseSamples.size();i+=step)points.append(QVariant::fromValue(m_playbackPoseSamples[i].positionMeters));
    if(!m_playbackPoseSamples.isEmpty()&&(m_playbackPoseSamples.size()-1)%step)points.append(QVariant::fromValue(m_playbackPoseSamples.last().positionMeters));
    return points;
}

QVariantList PointCloudFileLoader::playbackTrajectory() const
{
    const auto end=std::upper_bound(m_playbackPoseSamples.cbegin(),m_playbackPoseSamples.cend(),m_playbackPositionMs,
        [](int time,const PointCloudPlaybackPoseSample &p){return time<p.timeMs;});
    const int count=int(std::distance(m_playbackPoseSamples.cbegin(),end));
    QVariantList points;
    const int step=std::max(1,(count+19999)/20000);
    for(int i=0;i<count;i+=step)points.append(QVariant::fromValue(m_playbackPoseSamples[i].positionMeters));
    if(count>0&&(count-1)%step)points.append(QVariant::fromValue(m_playbackPoseSamples[count-1].positionMeters));
    return points;
}

bool PointCloudFileLoader::loadRecordingFiles(const QVariantList &urls)
{
    if(!m_geometry||urls.isEmpty()||loading())return false;
    QStringList paths;
    for(const auto &v:urls){const QUrl url=v.toUrl();const QString path=url.isLocalFile()?url.toLocalFile():v.toString();
        if(!QFileInfo(path).isFile()||(QFileInfo(path).suffix().toLower()!="bag"&&QFileInfo(path).suffix().toLower()!="db3")){setStatus("请选择本地ROS bag或ROS2 db3录包");emit loadFinished(false);return false;}
        paths.append(path);
    }
    const quint64 token=++m_loadToken;const int limit=m_maxPoints>0?m_maxPoints:500000;
    setLoading(true);setProgress(0);setStatus("正在解析飞行记录…");m_recordingSources.clear();
    clearPlaybackData();m_geometry->setPoints({},0,0);setLoadedFile({});
    QPointer<PointCloudFileLoader> self(this);const auto alive=m_alive;
    auto future=QtConcurrent::run([self,alive,token,paths,limit]{
        auto combined=std::make_shared<TextLoadResult>();QVector<TextLoadResult> parts;
        for(int i=0;i<paths.size();++i){
            if(!alive->load()||!self)return;
            auto part=loadRosBagPointCloudStreaming(paths[i],std::max(1000,limit/int(paths.size())),[self,alive,token,i,count=paths.size()](int value){
                if(!alive->load()||!self)return;
                QMetaObject::invokeMethod(self,[self,alive,token,i,count,value]{if(alive->load()&&self&&self->m_loadToken==token)self->setProgress((i*100+value)/count);},Qt::QueuedConnection);
            },true);
            if(!part.ok){combined->error=QFileInfo(paths[i]).fileName()+": "+part.error;break;}
            parts.append(std::move(part));
        }
        if(combined->error.isEmpty()&&!parts.isEmpty()){
            std::sort(parts.begin(),parts.end(),[](const auto &a,const auto &b){return a.startStampUsec<b.startStampUsec;});
            const quint64 origin=parts.first().startStampUsec;combined->sources=parts.first().sources;
            for(const auto &part:parts){
                if(part.sources!=combined->sources){combined->error="分包中的点云/里程计来源不一致，不能直接拼接";break;}
                const int offset=int(std::min<quint64>((part.startStampUsec-origin)/1000ULL,quint64(std::numeric_limits<int>::max()-part.playbackDurationMs)));
                combined->sourcePointCount+=part.sourcePointCount;
                combined->playbackPoints+=part.playbackPoints;
                for(int t:part.playbackTimesMs)combined->playbackTimesMs.append(offset+t);
                for(auto pose:part.playbackPoseSamples){pose.timeMs+=offset;combined->playbackPoseSamples.append(pose);}
                combined->playbackDurationMs=std::max(combined->playbackDurationMs,offset+part.playbackDurationMs);
            }
            combined->points=uniformlyDecimatedPointsWithTimes(combined->playbackPoints,combined->playbackTimesMs,limit,nullptr);
            combined->playbackFrameDurationMs=100;
            std::sort(combined->playbackPoseSamples.begin(),combined->playbackPoseSamples.end(),[](const auto &a,const auto &b){return a.timeMs<b.timeMs;});
            combined->ok=combined->error.isEmpty()&&!combined->points.isEmpty();
        }
        if(!alive->load()||!self)return;
        QMetaObject::invokeMethod(self,[self,alive,token,paths,combined]{
            if(!alive->load()||!self||self->m_loadToken!=token)return;
            self->m_recordingSources=combined->sources;
            self->finishAsyncLoad(token,paths.first(),combined->ok,combined->error,combined->points,combined->sourcePointCount,
                combined->playbackPoints,combined->playbackTimesMs,combined->playbackPoseSamples,combined->playbackDurationMs,combined->playbackFrameDurationMs);
        },Qt::QueuedConnection);
    });Q_UNUSED(future);return true;
}
