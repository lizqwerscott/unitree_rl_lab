#pragma once

#include "groot/GrootModeManager.h"
#include "groot/JointNameMap.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#if __has_include(<zmq.hpp>)
#include <zmq.hpp>
#include "groot/ZmqCompat.h"
#define GROOT_HAS_ZMQ 1
#else
#define GROOT_HAS_ZMQ 0
#endif

namespace groot {

class RemoteCommandReceiver {
public:
    explicit RemoteCommandReceiver(int port = 6002) : port_(port) {}
    ~RemoteCommandReceiver() { stop(); }

    // Logs each distinct gripper-block problem once. A malformed gripper block
    // must never be fatal: it only downgrades to "no gripper update this frame"
    // while the arm half of the frame keeps working (see the header docs).
    static void gripper_warn(const char* reason) {
        static std::mutex mutex;
        static std::set<std::string> seen;
        std::lock_guard<std::mutex> lock(mutex);
        if (seen.insert(reason).second)
            std::fprintf(stderr, "[RemoteCommandReceiver] gripper block ignored: %s\n", reason);
    }

    static bool parse_packet(const std::string& payload, CommandSnapshot& out,
                             const CommandSnapshot* previous = nullptr,
                             const char** fail_reason = nullptr) {
        auto fail = [fail_reason](const char* why) {
            if (fail_reason) *fail_reason = why;
            return false;
        };
        try {
            // LeRobot "action" frame, e.g.:
            //   {"cmd":"action",
            //    "action":{ "kLeftShoulderPitch.q":-0.20, ..., "kRightWristYaw.q":-0.01,
            //              "remote.lx":0.0,"remote.ly":0.0,"remote.rx":0.0,"remote.ry":0.0 },
            //    "timestamp": 1788514855.52}
            const YAML::Node root = YAML::Load(payload);
            if (!root["action"] || !root["timestamp"]) return fail("missing action/timestamp");
            const double timestamp = root["timestamp"].as<double>();
            if (!std::isfinite(timestamp)) return fail("non-finite timestamp");
            // No "seq" in LeRobot frames; keep a monotonic gate on the sender timestamp.
            if (previous && timestamp <= previous->timestamp) return fail("stale timestamp");
            const auto action = root["action"];
            if (!action.IsMap()) return fail("action not a map");

            auto axis = [&action](const char* key) -> float {
                const YAML::Node value = action[std::string("remote.") + key];
                return value ? value.as<float>() : 0.0f;
            };
            // Axis mapping matches the gamepad convention: vx=ly, vy=-lx, wz=-rx (ry unused).
            out.velocity = {axis("ly"), -axis("lx"), -axis("rx")};
            if (!finite(out.velocity)) return fail("non-finite velocity");

            // Arm joints (motor index 15..28): match "<name>.q" keys by name,
            // case-insensitively (dataset may spell kLeftWristYaw as kLeftWristyaw).
            std::array<bool, 14> filled{};
            int arm_count = 0;
            for (const auto& entry : action) {
                if (!entry.first.IsScalar()) continue;
                const std::string key = entry.first.as<std::string>();
                if (key.size() < 3 || key.compare(key.size() - 2, 2, ".q") != 0) continue;
                const std::string base = key.substr(0, key.size() - 2);
                const int slot = g1::arm_slot_of_lerobot_name(base);
                if (slot < 0) continue;  // ignore non-arm joints if the frame includes them
                const float value = entry.second.as<float>();
                if (!std::isfinite(value) || std::abs(value) > 3.2f) return fail("joint value out of range");
                if (!filled[slot]) { filled[slot] = true; ++arm_count; }
                out.arm_q[slot] = value;
            }
            if (arm_count != 14) return fail("expected 14 arm joints");

            // Optional gripper block. The arm contract above is unchanged; the
            // gripper half is validated separately and every failure here only
            // drops the gripper update for this frame, never the arm targets.
            //
            // Only `q` is read: kp/kd/mode belong to FSM.Groot.gripper and are
            // ignored (with a warning) when a frame tries to set them.
            //
            // q is *not* range-checked here. The calibrated range lives in
            // FSM.Groot.gripper.q_min/q_max, which a static parser cannot see,
            // and the contract is "clamp and log", not "drop the frame" -- so
            // out-of-range values are passed through and shaped by the bridge.
            // Only a non-finite q is unusable and rejects the side.
            out.gripper = {};
            if (const YAML::Node block = action["gripper"]) {
                if (!block.IsMap()) {
                    gripper_warn("not a map");
                } else {
                    bool any = false;
                    for (size_t side = 0; side < dex1::kNumGrippers; ++side) {
                        try {
                            const YAML::Node node = block[dex1::kSides[side]];
                            if (!node || !node.IsMap()) continue;  // side absent => keep last target
                            if (node["kp"] || node["kd"] || node["mode"])
                                gripper_warn("kp/kd/mode come from FSM.Groot.gripper; frame values ignored");
                            const YAML::Node q_node = node["q"];
                            if (!q_node) { gripper_warn("side is missing q"); continue; }
                            const float q = q_node.as<float>();
                            if (!std::isfinite(q)) { gripper_warn("q is not finite"); continue; }
                            out.gripper.q[side] = q;
                            out.gripper.has_target[side] = true;
                            any = true;
                        } catch (...) {
                            out.gripper.has_target[side] = false;
                            gripper_warn("malformed side");
                        }
                    }
                    out.gripper.timestamp = timestamp;
                    out.gripper.received = std::chrono::steady_clock::now();
                    out.gripper.valid = any;
                }
            }

            out.timestamp = timestamp;
            out.sequence = previous ? previous->sequence + 1 : 0;
            out.valid = true;
            out.received = std::chrono::steady_clock::now();
            return true;
        } catch (...) { return fail("yaml parse error"); }
    }

    bool latest(CommandSnapshot& out) const { std::lock_guard<std::mutex> lock(mutex_); if (!latest_.valid) return false; out = latest_; return true; }
    bool submit(const std::string& payload, const char** fail_reason = nullptr) {
        CommandSnapshot parsed;
        const char* reason = nullptr;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!parse_packet(payload, parsed, latest_.valid ? &latest_ : nullptr, fail_reason ? fail_reason : &reason)) return false;
        latest_ = parsed;
        return true;
    }
    void start() {
#if GROOT_HAS_ZMQ
        if (running_.exchange(true)) return;
        thread_ = std::thread([this] {
            zmq::context_t context(1);
            zmq::socket_t socket(context, zmq::socket_type::pull);
            zmq_detail::set_sockopt_int(socket, ZMQ_RCVTIMEO, 20);
            try {
                socket.bind("tcp://*:" + std::to_string(port_));
            } catch (const zmq::error_t& e) {
                std::fprintf(stderr, "[RemoteCommandReceiver] bind tcp://*:%d failed: %s\n", port_, e.what());
                running_ = false;
                return;
            }
            std::fprintf(stderr, "[RemoteCommandReceiver] listening on tcp://*:%d (PULL; expects LeRobot action frames from a PUSH peer)\n", port_);
            bool first_logged = false;
            while (running_) {
                zmq::message_t message;
                if (!socket.recv(message, zmq::recv_flags::none)) continue;
                const char* reason = nullptr;
                bool ok = false;
                if (message.size() <= 16384)
                    ok = submit(std::string(static_cast<const char*>(message.data()), message.size()), &reason);
                else
                    reason = "oversize message";
                if (!first_logged) {
                    first_logged = true;
                    if (ok)
                        std::fprintf(stderr, "[RemoteCommandReceiver] first action command received on tcp://*:%d\n", port_);
                    else
                        std::fprintf(stderr, "[RemoteCommandReceiver] first message received on tcp://*:%d ignored (%s)\n", port_, reason ? reason : "parse failed");
                }
            }
        });
#else
        running_ = false;
        std::fprintf(stderr, "[RemoteCommandReceiver] zmq.hpp not found; port %d receiver disabled\n", port_);
#endif
    }
    void stop() { running_ = false; if (thread_.joinable()) thread_.join(); }
    int port() const { return port_; }

private:
    static bool finite(const VelocityCommand& c) { return std::isfinite(c.vx) && std::isfinite(c.vy) && std::isfinite(c.wz); }
    int port_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    CommandSnapshot latest_{};
};

}  // namespace groot
