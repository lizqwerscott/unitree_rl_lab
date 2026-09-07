#pragma once

#include "groot/ArmBezierTrajectory.h"
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>

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

    void update_navigation(const VelocityCommand& command) {
        std::lock_guard<std::mutex> lock(mutex_);
        navigation_.velocity = command;
        navigation_.received = std::chrono::steady_clock::now();
        navigation_.valid = finite(command);
    }

    void update_vla(const CommandSnapshot& snapshot,
                    const std::array<float, 14>& actual_arm,
                    double now) {
        if (!finite(snapshot.velocity)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        const bool new_session_command = !vla_entry_has_baseline_
            || snapshot.sequence > vla_entry_sequence_;
        if (mode_ == ControlMode::VLA && vla_transition_pending_ && !new_session_command) return;
        const bool transition_required = mode_ == ControlMode::VLA
            && (vla_transition_pending_ || !fresh(vla_, now, vla_timeout_));
        vla_ = snapshot;
        vla_.valid = true;
        if (transition_required && new_session_command) {
            trajectory_.start(actual_arm, vla_.arm_q, arm_velocity_limits_, now);
            vla_transition_pending_ = false;
        }
        if (mode_ == ControlMode::VLA && new_session_command) {
            vla_entry_has_baseline_ = true;
            vla_entry_sequence_ = snapshot.sequence;
        }
        last_vla_arm_q_ = snapshot.arm_q;
        has_last_vla_arm_ = true;
    }

    bool request_mode(ControlMode mode, double now,
                      const std::array<float, 14>& actual_arm) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode == mode_) return true;
        if (mode_ == ControlMode::VLA && mode != ControlMode::VLA) {
            trajectory_.start(actual_arm, safe_home_, arm_velocity_limits_, now);
            vla_transition_pending_ = false;
        } else if (mode_ != ControlMode::VLA && mode == ControlMode::VLA) {
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
            if (!trajectory_.finished()) return trajectory_.sample(now);
            if (has_last_vla_arm_) return last_vla_arm_q_;
            return safe_home_;
        }
        if (mode_ != ControlMode::VLA && !trajectory_.finished()) return trajectory_.sample(now);
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
    ControlMode mode_ = ControlMode::Gamepad;
    LocomotionMode locomotion_ = LocomotionMode::Auto;
    std::array<float, 14> safe_home_{};
    std::array<float, 29> policy_default_{};
    CommandSnapshot navigation_{};
    CommandSnapshot vla_{};
    ArmBezierTrajectory trajectory_;
    bool vla_transition_pending_ = false;
    bool vla_entry_has_baseline_ = false;
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
