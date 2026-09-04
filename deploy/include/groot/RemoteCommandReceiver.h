#pragma once

#include "groot/GrootModeManager.h"
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
            const YAML::Node root = YAML::Load(payload);
            if (!root["seq"] || !root["timestamp"] || !root["remote"] || !root["arm_q"]) return false;
            out.sequence = root["seq"].as<uint64_t>();
            if (previous && out.sequence <= previous->sequence) return false;
            const double timestamp = root["timestamp"].as<double>();
            if (!std::isfinite(timestamp)) return false;
            const auto remote = root["remote"];
            out.velocity = {remote["ly"].as<float>(), -remote["lx"].as<float>(), -remote["rx"].as<float>()};
            const auto arm = root["arm_q"];
            if (!arm.IsSequence() || arm.size() != 14 || !finite(out.velocity)) return false;
            for (size_t i = 0; i < 14; ++i) {
                out.arm_q[i] = arm[i].as<float>();
                if (!std::isfinite(out.arm_q[i]) || std::abs(out.arm_q[i]) > 3.2f) return false;
            }
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
