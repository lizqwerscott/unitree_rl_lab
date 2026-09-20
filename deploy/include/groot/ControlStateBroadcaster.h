#pragma once

#include "groot/GrootModeManager.h"
#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <string>
#include <thread>
#include <utility>

#if __has_include(<zmq.hpp>)
#include <zmq.hpp>
#include "groot/ZmqCompat.h"
#define GROOT_HAS_ZMQ 1
#else
#define GROOT_HAS_ZMQ 0
#endif

namespace groot {

// Broadcasts the active Groot control mode over ZMQ PUB (default tcp://*:6000),
// so an upper-level host (VLA inference, navigation, teleop UI) can tell which
// input source currently owns the robot.
//
// Payload (exactly one field):
//   {"state":"gamepad"} | {"state":"nav"} | {"state":"vla"}
//
// The frame is republished at a fixed 50 Hz rather than only on change: a PUB
// socket keeps no backlog, so a subscriber that connects later would otherwise
// stay blind until the next mode switch. With the heartbeat it learns the
// current mode within one period (20 ms). No subscriber => frames are dropped.
class ControlStateBroadcaster {
public:
    using Snapshot = ControlMode;

    explicit ControlStateBroadcaster(int port = 6000) : port_(port) {}
    ~ControlStateBroadcaster() { stop(); }

    void start(std::function<Snapshot()> snapshot) {
        snapshot_ = std::move(snapshot);
#if GROOT_HAS_ZMQ
        if (!snapshot_ || running_.exchange(true)) return;
        thread_ = std::thread([this] { run(); });
#endif
    }

    void stop() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

    int port() const { return port_; }

    // Wire names are part of the reported contract; keep them stable.
    static const char* state_name(ControlMode mode) {
        switch (mode) {
        case ControlMode::VLA:
            return "vla";
        case ControlMode::Navigation:
            return "nav";
        case ControlMode::Gamepad:
            return "gamepad";
        }
        return "gamepad";
    }

    static std::string serialize(ControlMode mode) {
        std::string out = "{\"state\":\"";
        out += state_name(mode);
        out += "\"}";
        return out;
    }

private:
    void run() {
        zmq::context_t context(1);
        zmq::socket_t socket(context, zmq::socket_type::pub);
        zmq_detail::set_sockopt_int(socket, ZMQ_SNDHWM, 4);
        try {
            socket.bind("tcp://*:" + std::to_string(port_));
        } catch (const zmq::error_t& e) {
            std::fprintf(stderr, "[ControlStateBroadcaster] bind tcp://*:%d failed: %s\n", port_, e.what());
            running_ = false;
            return;
        }
        std::fprintf(stderr, "[ControlStateBroadcaster] broadcasting control state on tcp://*:%d (PUB, 50 Hz)\n", port_);
        const auto period = std::chrono::microseconds(20000);  // 50 Hz
        auto next = std::chrono::steady_clock::now();
        while (running_) {
            const std::string payload = serialize(snapshot_());
            try {
                socket.send(zmq::buffer(payload), zmq::send_flags::dontwait);
            } catch (const zmq::error_t&) {
                // No subscribers yet or tx buffer full; drop this frame.
            }
            next += period;
            std::this_thread::sleep_until(next);
        }
    }

    int port_;
    std::function<Snapshot()> snapshot_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

}  // namespace groot
