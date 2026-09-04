#pragma once

#include <array>
#include <algorithm>
#include <cmath>

namespace groot {

class ArmBezierTrajectory {
public:
    void start(const std::array<float, 14>& start,
               const std::array<float, 14>& target,
               float duration,
               double now = 0.0) {
        start_ = start;
        target_ = target;
        duration_ = std::max(0.0f, duration);
        start_time_ = now;
        active_ = duration_ > 0.0f;
        if (!active_) current_ = target_;
    }

    std::array<float, 14> sample(double now) {
        if (!active_) return current_;
        const double elapsed = std::max(0.0, now - start_time_);
        const float u = static_cast<float>(std::clamp(elapsed / duration_, 0.0, 1.0));
        const float s = u * u * u * (u * (u * 6.0f - 15.0f) + 10.0f);
        for (size_t i = 0; i < current_.size(); ++i)
            current_[i] = start_[i] + (target_[i] - start_[i]) * s;
        if (u >= 1.0f) {
            current_ = target_;
            active_ = false;
        }
        return current_;
    }

    bool finished() const { return !active_; }
    const std::array<float, 14>& current() const { return current_; }

private:
    std::array<float, 14> start_{};
    std::array<float, 14> target_{};
    std::array<float, 14> current_{};
    double start_time_ = 0.0;
    float duration_ = 0.0f;
    bool active_ = false;
};

}  // namespace groot
