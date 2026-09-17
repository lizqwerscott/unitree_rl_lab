#pragma once

#include "groot/ArmBezierTrajectory.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>

namespace groot {

enum class ControlMode { Gamepad, Navigation, VLA };
enum class LocomotionMode { Auto, Stand };

struct VelocityCommand {
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
};

struct CommandSnapshot {
    VelocityCommand velocity;
    std::array<float, 14> arm_q{};
    uint64_t sequence = 0;
    double timestamp = 0.0;
    std::chrono::steady_clock::time_point received{};
    bool valid = false;
};

class GrootModeManager {
public:
    GrootModeManager() {
        safe_home_.fill(0.0f);
        policy_default_.fill(0.0f);
    }

    void set_safe_home(const std::array<float, 29>& q) {
        for (size_t i = 0; i < 14; ++i) safe_home_[i] = q[i + 15];
    }
    void set_policy_default(const std::array<float, 29>& q) { policy_default_ = q; }
    void set_arm_velocity_limits(const std::array<float, 14>& limits, float scale = 1.0f) {
        const float safe_scale = std::clamp(scale, 0.01f, 1.0f);
        for (size_t i = 0; i < arm_velocity_limits_.size(); ++i)
            arm_velocity_limits_[i] = std::max(0.0f, limits[i]) * safe_scale;
    }
    void set_command_limits(const VelocityCommand& lower, const VelocityCommand& upper) {
        command_lower_ = lower;
        command_upper_ = upper;
    }
    // VLA 手臂目标的偏差上限（度，见 config.yaml `FSM.Groot.vla.max_arm_deviation_deg`）。
    // 任何 > 0 的值会被限幅到 [1, 180]；<= 0 表示关闭该保护。
    void set_vla_max_deviation_deg(float degrees) {
        vla_max_deviation_deg_ = degrees > 0.0f ? std::clamp(degrees, 1.0f, 180.0f) : 0.0f;
    }

    void update_navigation(const VelocityCommand& command) {
        std::lock_guard<std::mutex> lock(mutex_);
        navigation_.velocity = command;
        navigation_.received = std::chrono::steady_clock::now();
        navigation_.valid = finite(command);
    }

    // VLA 手臂目标的偏差上限（可配置，默认 40 deg）。VLA 内每一条有效包都拿目标与**实测
    // 姿态**比较，任一关节超过该值就判定这条目标会让手臂大幅跳变，不再接管手臂，由调用方
    // 退出到 Gamepad。进入 VLA 时同样适用：手臂此时停在 safe_home，若策略目标离它超过阈值
    // 就不会接管（这是刻意的：宁可进不去，也不执行一条差异过大的目标）。
    static constexpr float kDefaultVlaMaxDeviationDeg = 40.0f;
    static constexpr float kDegToRad = 0.017453292519943295f;
    static constexpr float kRadToDeg = 57.29577951308232f;

    // 触发保护时回报给调用方，由 FSM 退出到 Gamepad 并打印日志。
    struct ArmGuardEvent {
        std::array<float, 14> delta{};  // 目标 - 实测 (rad)
        size_t exceeded = 0;            // 超过阈值的关节数量
    };

    float vla_max_deviation_deg() const { return vla_max_deviation_deg_; }

    // VLA 手臂目标直接采用下发数据，不做 Bezier 过渡插值：通过校验的 `arm_q` 立即成为
    // 目标位置。运动仍受发布侧逐周期关节速度上限约束。
    std::optional<ArmGuardEvent> update_vla(const CommandSnapshot& snapshot,
                                            const std::array<float, 14>& actual_arm) {
        if (!finite(snapshot.velocity)) return std::nullopt;
        std::lock_guard<std::mutex> lock(mutex_);
        const bool new_session_command = !vla_entry_has_baseline_
            || snapshot.sequence > vla_entry_sequence_;
        if (mode_ == ControlMode::VLA && vla_transition_pending_ && !new_session_command) return std::nullopt;
        vla_ = snapshot;
        vla_.valid = true;

        std::optional<ArmGuardEvent> event;
        if (mode_ == ControlMode::VLA && new_session_command) {
            const auto gap = arm_deviation(snapshot.arm_q, actual_arm);
            if (gap.exceeded > 0) event = gap;
            vla_entry_has_baseline_ = true;
            vla_entry_sequence_ = snapshot.sequence;
            vla_transition_pending_ = false;
        }
        last_vla_arm_q_ = snapshot.arm_q;
        has_last_vla_arm_ = true;
        return event;
    }

    bool request_mode(ControlMode mode, double now,
                      const std::array<float, 14>& actual_arm) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode == mode_) return true;
        // 刚退出 VLA 时手臂还在插值回安全姿态：这段时间不允许切回 VLA，
        // 否则会被取消的过渡与 VLA 目标抢同一个手臂。等回到位再进。
        if (mode == ControlMode::VLA && !trajectory_.finished()) return false;
        if (mode_ == ControlMode::VLA && mode != ControlMode::VLA) {
            // 退出 VLA（含偏差保护触发的回退）：按 Bezier 插值回到安全姿态。
            trajectory_.start(actual_arm, safe_home_, arm_velocity_limits_, now);
            vla_transition_pending_ = false;
        } else if (mode_ != ControlMode::VLA && mode == ControlMode::VLA) {
            // 进入 VLA 时取消回安全姿态的过渡（此处已确保过渡结束，属兜底）；
            // 首个新指令到达前仍保持安全姿态。
            trajectory_.cancel();
            vla_entry_has_baseline_ = vla_.valid;
            vla_entry_sequence_ = vla_.sequence;
            vla_.valid = false;
            vla_transition_pending_ = true;
        }
        mode_ = mode;
        return true;
    }

    void set_locomotion_mode(LocomotionMode mode) { std::lock_guard<std::mutex> lock(mutex_); locomotion_ = mode; }
    ControlMode mode() const { std::lock_guard<std::mutex> lock(mutex_); return mode_; }
    LocomotionMode locomotion_mode() const { std::lock_guard<std::mutex> lock(mutex_); return locomotion_; }
    void set_timeouts(double navigation_seconds, double vla_seconds) {
        navigation_timeout_ = navigation_seconds; vla_timeout_ = vla_seconds;
    }
    VelocityCommand velocity(const VelocityCommand& gamepad, double now) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (locomotion_ == LocomotionMode::Stand) return {};
        const auto manual = clamp(gamepad);
        if (mode_ != ControlMode::Gamepad && active(manual)) return manual;
        if (mode_ == ControlMode::Navigation)
            return fresh(navigation_, now, navigation_timeout_) ? clamp(navigation_.velocity) : VelocityCommand{};
        if (mode_ == ControlMode::VLA)
            return fresh(vla_, now, vla_timeout_) ? clamp(vla_.velocity) : VelocityCommand{};
        return manual;
    }

    std::array<float, 14> arm_target(double now) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ == ControlMode::VLA) {
            if (vla_transition_pending_) return safe_home_;
            if (has_last_vla_arm_) return last_vla_arm_q_;
            return safe_home_;
        }
        if (!trajectory_.finished()) return trajectory_.sample(now);
        return safe_home_;
    }

    const std::array<float, 29>& policy_default() const { return policy_default_; }

private:
    static constexpr float kGamepadOverrideDeadband = 0.05f;
    static bool finite(const VelocityCommand& c) { return std::isfinite(c.vx) && std::isfinite(c.vy) && std::isfinite(c.wz); }
    static bool active(const VelocityCommand& c) {
        return finite(c) && (std::fabs(c.vx) > kGamepadOverrideDeadband
            || std::fabs(c.vy) > kGamepadOverrideDeadband
            || std::fabs(c.wz) > kGamepadOverrideDeadband);
    }
    VelocityCommand clamp(const VelocityCommand& command) const {
        return {
            std::clamp(command.vx, command_lower_.vx, command_upper_.vx),
            std::clamp(command.vy, command_lower_.vy, command_upper_.vy),
            std::clamp(command.wz, command_lower_.wz, command_upper_.wz)
        };
    }
    static bool fresh(const CommandSnapshot& c, double now, double timeout) {
        if (!c.valid) return false;
        const auto age = now - std::chrono::duration<double>(c.received.time_since_epoch()).count();
        return age >= 0.0 && age <= timeout;
    }
    // 逐关节统计目标与实测姿态的偏差，返回偏差数组与超限关节数量。
    // 阈值 <= 0 表示保护关闭，此时恒返回 exceeded == 0。
    ArmGuardEvent arm_deviation(const std::array<float, 14>& command,
                                const std::array<float, 14>& actual_arm) const {
        ArmGuardEvent event;
        if (vla_max_deviation_deg_ <= 0.0f) return event;
        const float limit = vla_max_deviation_deg_ * kDegToRad;
        for (size_t i = 0; i < command.size(); ++i) {
            event.delta[i] = command[i] - actual_arm[i];
            if (std::fabs(event.delta[i]) > limit) ++event.exceeded;
        }
        return event;
    }
    ControlMode mode_ = ControlMode::Gamepad;
    LocomotionMode locomotion_ = LocomotionMode::Auto;
    std::array<float, 14> safe_home_{};
    std::array<float, 29> policy_default_{};
    CommandSnapshot navigation_{};
    CommandSnapshot vla_{};
    ArmBezierTrajectory trajectory_;
    bool vla_transition_pending_ = false;
    bool vla_entry_has_baseline_ = false;
    float vla_max_deviation_deg_ = kDefaultVlaMaxDeviationDeg;
    uint64_t vla_entry_sequence_ = 0;
    std::array<float, 14> arm_velocity_limits_{};
    std::array<float, 14> last_vla_arm_q_{};
    bool has_last_vla_arm_ = false;
    double navigation_timeout_ = 0.3;
    double vla_timeout_ = 0.3;
    VelocityCommand command_lower_{-1.0f, -1.0f, -1.0f};
    VelocityCommand command_upper_{1.0f, 1.0f, 1.0f};
    mutable std::mutex mutex_;
};

}  // namespace groot
