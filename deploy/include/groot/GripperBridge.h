#pragma once

#include "groot/GripperNameMap.h"
#include "groot/GripperStateBroadcaster.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <unitree/dds_wrapper/common/Publisher.h>
#include <unitree/dds_wrapper/common/Subscription.h>
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

namespace groot {

// Drives the Dex1_1 gripper over DDS on behalf of the ZMQ action frames.
//
// Downstream (host -> gripper) the target arrives inside the existing 6002
// action frame (see RemoteCommandReceiver); upstream (gripper -> host) the
// measured state is republished on ZMQ PUB 6004 by the bundled
// GripperStateBroadcaster.
//
// One thread does both jobs at 100 Hz, mirroring LowStateBroadcaster's fixed
// cadence loop:
//   * pull the newest target from the injected source (usually
//     RemoteCommandReceiver::latest()) and latch it per side;
//   * ramp the commanded q towards the latched target (per-cycle rate limit,
//     the dex1 service applies no limiting of its own);
//   * republish the command unconditionally, even with no new target and with
//     6002 silent -- dex1_1_service brakes the motors when its command stream
//     times out, so the refresh rate is a safety requirement, not an
//     optimisation;
//   * cache the measured MotorStates_ for the state broadcaster.
//
// No command is published for a side until that side has received a target:
// publishing an unconfirmed target on entry would move the fingers on its own.
//
// kp/kd come from FSM.Groot.gripper and mode is the fixed FOC constant; the
// action frame can only set q.
class GripperBridge {
public:
    using TargetSource = std::function<dex1::GripperTargets()>;
    using Measurement = GripperStateBroadcaster::Measurement;
    using Snapshot = GripperStateBroadcaster::Snapshot;

    GripperBridge(int state_port, float kp, float kd, float q_min, float q_max, float max_rate_rad_per_s,
                  double command_timeout_s)
        : state_port_(state_port),
          kp_default_(kp > 0.0f ? kp : dex1::kDefaultKp),
          kd_default_(kd > 0.0f ? kd : dex1::kDefaultKd),
          q_min_(q_max > q_min ? q_min : dex1::kDefaultQMin),
          q_max_(q_max > q_min ? q_max : dex1::kDefaultQMax),
          max_rate_(max_rate_rad_per_s > 0.0f ? max_rate_rad_per_s : dex1::kDefaultMaxRateRadPerS),
          command_timeout_(command_timeout_s > 0.0 ? command_timeout_s : dex1::kDefaultCommandTimeoutS),
          state_broadcaster_(std::make_unique<GripperStateBroadcaster>(state_port)) {
        for (size_t side = 0; side < dex1::kNumGrippers; ++side) {
            command_[side] = std::make_unique<unitree::robot::RealTimePublisher<MotorCmds>>(command_topic(side));
            command_[side]->msg_.cmds().resize(1);
            measured_sub_[side] = std::make_unique<unitree::robot::SubscriptionBase<MotorStates>>(state_topic(side));
            measured_sub_[side]->msg_.states().resize(1);
        }
    }

    ~GripperBridge() { stop(); }

    void start(TargetSource source) {
        source_ = std::move(source);
        if (!source_ || running_.exchange(true)) return;
        reset_session();
        thread_ = std::thread([this] { run(); });
        state_broadcaster_->start([this]() -> Snapshot { return snapshot(); });
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (state_broadcaster_) state_broadcaster_->stop();
    }

    int state_port() const { return state_port_; }

    // Manual release, triggered by a gamepad/keyboard key. Drives both sides to
    // q_max through exactly the same latch + rate-limit path as a frame target,
    // so the fingers ramp open instead of snapping. The next frame that carries
    // a gripper block still wins -- the host keeps owning the gripper semantics.
    void open_all() { local_open_requests_.fetch_add(1, std::memory_order_relaxed); }

    // Latest measured gripper state, for the ZMQ state broadcaster.
    Snapshot snapshot() const {
        std::lock_guard<std::mutex> lock(measure_mutex_);
        return measurements_;
    }

    std::array<float, dex1::kNumGrippers> measured_q() const {
        std::lock_guard<std::mutex> lock(measure_mutex_);
        std::array<float, dex1::kNumGrippers> q{};
        for (size_t side = 0; side < dex1::kNumGrippers; ++side) q[side] = measurements_[side].q;
        return q;
    }

private:
    static constexpr float kDt = 0.01f;  // 100 Hz loop

    // Per-run state. Cleared on every start() so a re-entry behaves exactly like
    // the first one: no command is published until *this* session receives a
    // target, instead of resuming the target left over from the previous one.
    void reset_session() {
        latched_.fill(false);
        target_q_.fill(0.0f);
        command_q_.fill(0.0f);
        last_target_.fill(std::chrono::steady_clock::time_point{});
        stale_logged_.fill(false);
        clamp_logged_.fill(false);
        last_frame_timestamp_ = 0.0;
        has_frame_timestamp_ = false;
        peer_warned_ = false;
        local_open_requests_ = 0;
    }

    static std::string command_topic(size_t side) {
        return std::string(dex1::kCmdTopicPrefix) + dex1::kSides[side] + "/cmd";
    }
    static std::string state_topic(size_t side) {
        return std::string(dex1::kStateTopicPrefix) + dex1::kSides[side] + "/state";
    }

    float measured_q_of(size_t side) const {
        std::lock_guard<std::mutex> lock(measure_mutex_);
        return measurements_[side].q;
    }

    // First target for a side: latch it and start the ramp from wherever the
    // gripper actually is, so re-entry never sweeps through 0 first.
    void latch_side(size_t side) {
        latched_[side] = true;
        command_q_[side] = measured_q_of(side);
        std::fprintf(stderr, "[GripperBridge] %s gripper target received; publishing %s at 100 Hz\n",
                     dex1::kSides[side], command_topic(side).c_str());
    }

    void apply_local_open_request() {
        if (local_open_requests_.exchange(0, std::memory_order_relaxed) == 0) return;
        const auto now = std::chrono::steady_clock::now();
        for (size_t side = 0; side < dex1::kNumGrippers; ++side) {
            if (!latched_[side]) latch_side(side);
            target_q_[side] = q_max_;
            last_target_[side] = now;
            stale_logged_[side] = false;
            clamp_logged_[side] = false;
        }
        std::fprintf(stderr, "[GripperBridge] manual release: both grippers -> q_max (%.4f rad)\n",
                     static_cast<double>(q_max_));
    }

    // The source re-returns the last snapshot on every call, so a target only
    // counts as new when the carrying frame's timestamp moves forward (the
    // receiver already rejects non-increasing timestamps).
    void pull_target() {
        apply_local_open_request();
        if (!source_) return;
        const dex1::GripperTargets targets = source_();
        if (!targets.valid) return;
        if (has_frame_timestamp_ && targets.timestamp == last_frame_timestamp_) return;
        has_frame_timestamp_ = true;
        last_frame_timestamp_ = targets.timestamp;
        const auto now = std::chrono::steady_clock::now();
        for (size_t side = 0; side < dex1::kNumGrippers; ++side) {
            if (!targets.has_target[side]) continue;  // side absent => keep last target
            if (!latched_[side]) latch_side(side);
            // Same two-stage shaping as the arm in publish_targets(): clamp to
            // the absolute range first, then limit the per-cycle rate. The
            // clamp is what keeps a bad command off the mechanical end stop,
            // because dex1_1_service applies no range check of its own.
            const float requested = targets.q[side];
            const float clamped = dex1::clamp_q(requested, q_min_, q_max_);
            if (clamped != requested) {
                if (!clamp_logged_[side]) {
                    clamp_logged_[side] = true;
                    std::fprintf(stderr,
                                 "[GripperBridge] %s q=%.4f rad is outside the calibrated range [%.4f, %.4f]; clamped to %.4f\n",
                                 dex1::kSides[side], static_cast<double>(requested), static_cast<double>(q_min_),
                                 static_cast<double>(q_max_), static_cast<double>(clamped));
                }
            } else {
                clamp_logged_[side] = false;
            }
            target_q_[side] = clamped;
            last_target_[side] = now;
            stale_logged_[side] = false;
        }
    }

    void refresh_commands() {
        const auto now = std::chrono::steady_clock::now();
        const float max_delta = max_rate_ * kDt;
        for (size_t side = 0; side < dex1::kNumGrippers; ++side) {
            // Never publish before the first target for this side.
            if (!latched_[side]) continue;
            if (command_timeout_ > 0.0 && !stale_logged_[side]) {
                const double age = std::chrono::duration<double>(now - last_target_[side]).count();
                if (age > command_timeout_) {
                    stale_logged_[side] = true;
                    std::fprintf(stderr,
                                 "[GripperBridge] %s gripper target is stale (>%.3fs); holding the last target and still republishing (no brake)\n",
                                 dex1::kSides[side], command_timeout_);
                }
            }
            const float previous = command_q_[side];
            command_q_[side] = std::clamp(target_q_[side], previous - max_delta, previous + max_delta);
            if (!command_[side]->trylock()) continue;  // publisher thread holds it; catch it next cycle
            auto& motor = command_[side]->msg_.cmds()[0];
            motor.mode() = static_cast<uint8_t>(dex1::kDefaultMode);
            motor.kp() = kp_default_;
            motor.kd() = kd_default_;
            motor.q() = command_q_[side];
            motor.dq() = 0.0f;
            motor.tau() = 0.0f;
            command_[side]->unlockAndPublish();
        }
    }

    void refresh_measurements() {
        std::lock_guard<std::mutex> lock(measure_mutex_);
        for (size_t side = 0; side < dex1::kNumGrippers; ++side) {
            std::lock_guard<std::mutex> sub_lock(measured_sub_[side]->mutex_);
            const auto& states = measured_sub_[side]->msg_.states();
            if (states.empty()) continue;
            measurements_[side].q = states[0].q();
            measurements_[side].dq = states[0].dq();
            measurements_[side].tau_est = states[0].tau_est();
        }
    }

    void log_waiting_for_first_target() {
        std::fprintf(stderr, "[GripperBridge] waiting for the first gripper target (no dex1 command is published until one arrives)\n");
    }

    // isTimeout() reads a time point the DDS callback writes under the
    // subscription's own mutex, so take that lock to keep the read race-free.
    bool state_is_timed_out(size_t side) const {
        std::lock_guard<std::mutex> lock(measured_sub_[side]->mutex_);
        return measured_sub_[side]->isTimeout();
    }

    void warn_if_no_peer() {
        if (peer_warned_) return;
        bool any_timeout = false;
        for (size_t side = 0; side < dex1::kNumGrippers; ++side) {
            if (state_is_timed_out(side)) any_timeout = true;
        }
        if (!any_timeout) return;
        peer_warned_ = true;
        std::fprintf(stderr,
                     "[GripperBridge] no dex1_1_service detected on %s*/state; gripper commands will be published but nobody consumes them "
                     "(check the DDS domain id and network interface)\n",
                     dex1::kStateTopicPrefix);
    }

    void run() {
        const auto period = std::chrono::microseconds(10000);  // 100 Hz
        auto next = std::chrono::steady_clock::now();
        const auto started = next;
        std::fprintf(stderr,
                     "[GripperBridge] bridging dex1 gripper: %s{right,left}/cmd (DDS out, 100 Hz), state on tcp://*:%d (ZMQ PUB, 100 Hz), "
                     "kp=%.3f kd=%.3f q_range=[%.4f,%.4f] rad max_rate=%.3f rad/s command_timeout=%.3fs\n",
                     dex1::kCmdTopicPrefix, state_port_, static_cast<double>(kp_default_), static_cast<double>(kd_default_),
                     static_cast<double>(q_min_), static_cast<double>(q_max_), static_cast<double>(max_rate_), command_timeout_);
        log_waiting_for_first_target();
        std::chrono::steady_clock::time_point peer_check_at = started + std::chrono::seconds(2);
        while (running_) {
            pull_target();
            refresh_commands();
            refresh_measurements();
            if (!peer_warned_ && std::chrono::steady_clock::now() >= peer_check_at) warn_if_no_peer();
            next += period;
            std::this_thread::sleep_until(next);
        }
    }

    using MotorCmds = unitree_go::msg::dds_::MotorCmds_;
    using MotorStates = unitree_go::msg::dds_::MotorStates_;

    int state_port_;
    float kp_default_;
    float kd_default_;
    float q_min_;
    float q_max_;
    float max_rate_;
    double command_timeout_;

    std::array<std::unique_ptr<unitree::robot::RealTimePublisher<MotorCmds>>, dex1::kNumGrippers> command_;
    std::array<std::unique_ptr<unitree::robot::SubscriptionBase<MotorStates>>, dex1::kNumGrippers> measured_sub_;
    std::unique_ptr<GripperStateBroadcaster> state_broadcaster_;

    std::array<bool, dex1::kNumGrippers> latched_{};
    std::array<float, dex1::kNumGrippers> target_q_{};
    std::array<float, dex1::kNumGrippers> command_q_{};
    std::array<std::chrono::steady_clock::time_point, dex1::kNumGrippers> last_target_{};
    std::array<bool, dex1::kNumGrippers> stale_logged_{};
    std::array<bool, dex1::kNumGrippers> clamp_logged_{};
    double last_frame_timestamp_ = 0.0;
    bool has_frame_timestamp_ = false;
    bool peer_warned_ = false;

    mutable std::mutex measure_mutex_;
    Snapshot measurements_{};

    TargetSource source_;
    std::atomic<bool> running_{false};
    std::atomic<uint32_t> local_open_requests_{0};
    std::thread thread_;
};

}  // namespace groot
