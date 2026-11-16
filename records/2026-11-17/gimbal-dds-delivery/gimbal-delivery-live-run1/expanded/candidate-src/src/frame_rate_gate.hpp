#pragma once
#include <chrono>
#include <cmath>
#include <stdexcept>

// 按单调时钟保持发布节拍：不把每次晚到的误差累积到下一周期。
// 停流后只接受当前帧，不补发历史帧，也不更改ROS图像源戳。
class FrameRateGate {
public:
    using Clock = std::chrono::steady_clock;
    explicit FrameRateGate(double hz) {
        if (!std::isfinite(hz) || hz < 0) throw std::invalid_argument("color_hz must be finite and nonnegative");
        period_ = hz > 0 ? std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(1.0 / hz)) : Clock::duration::zero();
        if (hz > 0 && period_ <= Clock::duration::zero()) throw std::invalid_argument("color_hz exceeds clock resolution");
    }
    bool ready(Clock::time_point now) {
        if (period_ == Clock::duration::zero()) return false;
        // 半周期抖动预算只改变准入时刻，next仍按固定周期累加，平均速率不变。
        if (started_ && now + period_ / 2 < next_) return false;
        if (!started_ || now - next_ >= period_) next_ = now + period_;
        else next_ += period_;
        started_ = true;
        return true;
    }
private:
    Clock::duration period_{};
    Clock::time_point next_{};
    bool started_ = false;
};
