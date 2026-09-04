#pragma once

#include "groot/ArmBezierTrajectory.h"
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
    void set_transition_duration(float seconds) { transition_duration_ = std::max(0.0f, seconds); }
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

    void update_vla(const CommandSnapshot& snapshot) {
        if (!finite(snapshot.velocity)) return;
        std::lock_guard<std::mutex> lock(mutex_);
        vla_ = snapshot;
        vla_.valid = true;
    }

    bool request_mode(ControlMode mode, double now,
                      const std::array<float, 14>& actual_arm) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode == ControlMode::Navigation && !fresh(navigation_, now, navigation_timeout_)) return false;
        if (mode == ControlMode::VLA && !fresh(vla_, now, vla_timeout_)) return false;
        if (mode == mode_) return true;
        if (mode_ == ControlMode::VLA && mode != ControlMode::VLA)
            trajectory_.start(actual_arm, safe_home_, transition_duration_, now);
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
        if (mode_ == ControlMode::Navigation)
            return fresh(navigation_, now, navigation_timeout_) ? clamp(navigation_.velocity) : VelocityCommand{};
        if (mode_ == ControlMode::VLA)
            return fresh(vla_, now, vla_timeout_) ? clamp(vla_.velocity) : VelocityCommand{};
        return clamp(gamepad);
    }

    std::array<float, 14> arm_target(double now) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ == ControlMode::VLA && fresh(vla_, now, vla_timeout_)) return vla_.arm_q;
        if (mode_ != ControlMode::VLA && !trajectory_.finished()) return trajectory_.sample(now);
        return safe_home_;
    }

    const std::array<float, 29>& policy_default() const { return policy_default_; }

private:
    static bool finite(const VelocityCommand& c) { return std::isfinite(c.vx) && std::isfinite(c.vy) && std::isfinite(c.wz); }
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
    float transition_duration_ = 0.5f;
    double navigation_timeout_ = 0.3;
    double vla_timeout_ = 0.3;
    VelocityCommand command_lower_{-1.0f, -1.0f, -1.0f};
    VelocityCommand command_upper_{1.0f, 1.0f, 1.0f};
    mutable std::mutex mutex_;
};

}  // namespace groot
