#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <vector>

// 比较发布时刻与测量时刻的差，避免把订阅回调的排队时间当成设备时钟偏差。
// 两流的 DDS source_timestamp 使用同一发布机时钟；共同的墙钟偏移会抵消。
// LiDAR 必须传扫描末点时刻，不能把约100ms的整帧采集时间算入时钟偏差。
class SourceClockAlignment {
public:
    void lidar(int64_t sensor_end, int64_t published) { add(lidar_, sensor_end, published); }
    void imu(int64_t sensor, int64_t published) { add(imu_, sensor, published); }

    bool offset(double &seconds) const {
        if (lidar_.size() < 5 || imu_.size() < 20) return false;
        // 不将两个不同时段的流配对；滞后的回调继续排队，等待有重叠的发布时段。
        if (std::llabs(lidar_.back().published - imu_.back().published) > 200000000)
            return false;
        const int64_t first = std::max(lidar_.back().published, imu_.back().published) - 800000000;
        std::vector<int64_t> lidar, imu;
        for (const auto &sample : lidar_)
            if (sample.published >= first) lidar.push_back(sample.delay);
        for (const auto &sample : imu_)
            if (sample.published >= first) imu.push_back(sample.delay);
        if (lidar.size() < 5 || imu.size() < 20) return false;
        // 发布端仍积压/突发时不锁定；不是增加定位/转发的源帧龄门槛。
        if (spread(lidar) > 50000000 || spread(imu) > 50000000) return false;
        seconds = double(median(imu) - median(lidar)) * 1e-9;
        return std::isfinite(seconds);
    }

private:
    struct Sample { int64_t published, delay; };
    std::deque<Sample> lidar_, imu_;

    static int64_t median(std::vector<int64_t> values) {
        const auto middle = values.begin() + values.size() / 2;
        std::nth_element(values.begin(), middle, values.end());
        return *middle;
    }
    static int64_t spread(const std::vector<int64_t> &values) {
        const auto bounds = std::minmax_element(values.begin(), values.end());
        return *bounds.second - *bounds.first;
    }
    static void add(std::deque<Sample> &samples, int64_t sensor, int64_t published) {
        if (sensor <= 0 || published <= 0) return;
        if (!samples.empty() && published <= samples.back().published) samples.clear();
        samples.push_back({published, published - sensor});
        while (samples.size() > 200 || published - samples.front().published > 1000000000)
            samples.pop_front();
    }
};
