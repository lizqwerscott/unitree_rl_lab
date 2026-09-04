#pragma once

#include "groot/GrootModeManager.h"
#include "groot/JointNameMap.h"
#include <array>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include <yaml-cpp/yaml.h>
#if __has_include(<zmq.hpp>)
#include <zmq.hpp>
#define GROOT_HAS_ZMQ 1
#else
#define GROOT_HAS_ZMQ 0
#endif

namespace groot {

class RemoteCommandReceiver {
public:
    explicit RemoteCommandReceiver(int port = 6002) : port_(port) {}
    ~RemoteCommandReceiver() { stop(); }

    static bool parse_packet(const std::string& payload, CommandSnapshot& out,
                             const CommandSnapshot* previous = nullptr) {
        try {
            // LeRobot "action" frame, e.g.:
            //   {"cmd":"action",
            //    "action":{ "kLeftShoulderPitch.q":-0.20, ..., "kRightWristYaw.q":-0.01,
            //              "remote.lx":0.0,"remote.ly":0.0,"remote.rx":0.0,"remote.ry":0.0 },
            //    "timestamp": 1788514855.52}
            const YAML::Node root = YAML::Load(payload);
            if (!root["action"] || !root["timestamp"]) return false;
            const double timestamp = root["timestamp"].as<double>();
            if (!std::isfinite(timestamp)) return false;
            // No "seq" in LeRobot frames; keep a monotonic gate on the sender timestamp.
            if (previous && timestamp <= previous->timestamp) return false;
            const auto action = root["action"];
            if (!action.IsMap()) return false;

            auto axis = [&action](const char* key) -> float {
                const YAML::Node value = action[std::string("remote.") + key];
                return value ? value.as<float>() : 0.0f;
            };
            // Axis mapping matches the gamepad convention: vx=ly, vy=-lx, wz=-rx (ry unused).
            out.velocity = {axis("ly"), -axis("lx"), -axis("rx")};
            if (!finite(out.velocity)) return false;

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
                if (!std::isfinite(value) || std::abs(value) > 3.2f) return false;
                if (!filled[slot]) { filled[slot] = true; ++arm_count; }
                out.arm_q[slot] = value;
            }
            if (arm_count != 14) return false;
            out.timestamp = timestamp;
            out.sequence = previous ? previous->sequence + 1 : 0;
            out.valid = true;
            out.received = std::chrono::steady_clock::now();
            return true;
        } catch (...) { return false; }
    }

    bool latest(CommandSnapshot& out) const { std::lock_guard<std::mutex> lock(mutex_); if (!latest_.valid) return false; out = latest_; return true; }
    void submit(const std::string& payload) { CommandSnapshot parsed; std::lock_guard<std::mutex> lock(mutex_); if (parse_packet(payload, parsed, latest_.valid ? &latest_ : nullptr)) latest_ = parsed; }
    void start() {
#if GROOT_HAS_ZMQ
        if (running_.exchange(true)) return;
        thread_ = std::thread([this] {
            zmq::context_t context(1);
            zmq::socket_t socket(context, zmq::socket_type::pull);
            socket.set(zmq::sockopt::rcvtimeo, 20);
            socket.bind("tcp://*:" + std::to_string(port_));
            while (running_) {
                zmq::message_t message;
                if (socket.recv(message, zmq::recv_flags::none) && message.size() <= 16384)
                    submit(std::string(static_cast<const char*>(message.data()), message.size()));
            }
        });
#else
        running_ = false;
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
