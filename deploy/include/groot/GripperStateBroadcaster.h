#pragma once

#include "groot/GripperNameMap.h"
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
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

// Measured Dex1_1 gripper state for one side, copied straight out of the DDS
// MotorStates_ message (no unit conversion).
struct GripperMeasurement {
    float q = 0.0f;
    float dq = 0.0f;
    float tau_est = 0.0f;
};

// Broadcasts the measured gripper state over ZMQ PUB (default tcp://*:6004):
//   {"topic":"rt/dex1/state",
//    "data":{"right":{"q":0.501,"dq":0.0,"tau_est":0.02},
//            "left": {"q":0.500,"dq":0.0,"tau_est":0.02}}}
//
// Republished at a fixed 100 Hz for the same reason as the control-state
// broadcaster: a PUB socket keeps no backlog, so a subscriber that connects
// later would otherwise stay blind until the next change. No subscriber =>
// frames are dropped, sends are dontwait and never block the caller.
class GripperStateBroadcaster {
public:
    using Measurement = GripperMeasurement;
    using Snapshot = std::array<GripperMeasurement, dex1::kNumGrippers>;

    explicit GripperStateBroadcaster(int port = 6004) : port_(port) {}
    ~GripperStateBroadcaster() { stop(); }

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

    static std::string serialize(const Snapshot& snapshot) {
        std::string out;
        out.reserve(256);
        out += "{\"topic\":\"rt/dex1/state\",\"data\":{";
        for (size_t side = 0; side < snapshot.size(); ++side) {
            if (side) out += ',';
            out += '"';
            out += dex1::kSides[side];
            out += "\":{\"q\":";
            append_number(out, snapshot[side].q);
            out += ",\"dq\":";
            append_number(out, snapshot[side].dq);
            out += ",\"tau_est\":";
            append_number(out, snapshot[side].tau_est);
            out += '}';
        }
        out += "}}";
        return out;
    }

private:
    static void append_number(std::string& out, float value) {
        if (!std::isfinite(value)) value = 0.0f;
        char buffer[48];
        std::snprintf(buffer, sizeof(buffer), "%.9g", static_cast<double>(value));
        out += buffer;
    }

    void run() {
        zmq::context_t context(1);
        zmq::socket_t socket(context, zmq::socket_type::pub);
        zmq_detail::set_sockopt_int(socket, ZMQ_SNDHWM, 4);
        try {
            socket.bind("tcp://*:" + std::to_string(port_));
        } catch (const zmq::error_t& e) {
            std::fprintf(stderr, "[GripperStateBroadcaster] bind tcp://*:%d failed: %s\n", port_, e.what());
            running_ = false;
            return;
        }
        std::fprintf(stderr, "[GripperStateBroadcaster] broadcasting dex1 gripper state on tcp://*:%d (PUB, 100 Hz)\n", port_);
        const auto period = std::chrono::microseconds(10000);  // 100 Hz
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
