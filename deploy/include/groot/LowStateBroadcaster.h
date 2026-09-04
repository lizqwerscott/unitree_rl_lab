#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <utility>

#include <unitree/idl/hg/LowState_.hpp>

#if __has_include(<zmq.hpp>)
#include <zmq.hpp>
#define GROOT_HAS_ZMQ 1
#else
#define GROOT_HAS_ZMQ 0
#endif

namespace groot {

// Broadcasts the robot LowState over ZMQ PUB (default tcp://*:6001).
//
// The payload mirrors the JSON schema consumed by the LeRobot rollout host
// (see lerobot unitree_sdk2_socket.py / run_g1_server.py):
//   {"topic":"rt/lowstate","data":{motor_state:[{q,dq,tau_est,temperature}x35],
//                                  imu_state:{...}, wireless_remote:<base64>,
//                                  mode_machine:<int>}}
class LowStateBroadcaster {
public:
    using Snapshot = unitree_hg::msg::dds_::LowState_;

    explicit LowStateBroadcaster(int port = 6001) : port_(port) {}
    ~LowStateBroadcaster() { stop(); }

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

    static std::string serialize(const Snapshot& msg) {
        std::string out;
        out.reserve(4096);
        out += "{\"topic\":\"rt/lowstate\",\"data\":{";
        out += "\"motor_state\":[";
        for (size_t i = 0; i < msg.motor_state().size(); ++i) {
            if (i) out += ',';
            const auto& m = msg.motor_state()[i];
            out += '{';
            out += "\"q\":";
            append_number(out, m.q());
            out += ",\"dq\":";
            append_number(out, m.dq());
            out += ",\"tau_est\":";
            append_number(out, m.tau_est());
            out += ",\"temperature\":";
            append_number(out, (m.temperature()[0] + m.temperature()[1]) / 2.0);
            out += '}';
        }
        out += "],\"imu_state\":{";
        const auto& imu = msg.imu_state();
        out += "\"quaternion\":[";
        append_array(out, imu.quaternion());
        out += "],\"gyroscope\":[";
        append_array(out, imu.gyroscope());
        out += "],\"accelerometer\":[";
        append_array(out, imu.accelerometer());
        out += "],\"rpy\":[";
        append_array(out, imu.rpy());
        out += "],\"temperature\":";
        append_number(out, imu.temperature());
        out += "},\"wireless_remote\":\"";
        base64_encode(out, msg.wireless_remote().data(), msg.wireless_remote().size());
        out += "\",\"mode_machine\":";
        out += std::to_string(static_cast<unsigned>(msg.mode_machine()));
        out += "}}";
        return out;
    }

private:
    static void append_number(std::string& out, double value) {
        if (!std::isfinite(value)) value = 0.0;
        char buffer[48];
        std::snprintf(buffer, sizeof(buffer), "%.9g", value);
        out += buffer;
    }

    template <typename T, size_t N>
    static void append_array(std::string& out, const std::array<T, N>& values) {
        for (size_t i = 0; i < N; ++i) {
            if (i) out += ',';
            append_number(out, static_cast<double>(values[i]));
        }
    }

    static void base64_encode(std::string& out, const unsigned char* data, size_t size) {
        static const char alphabet[] =
            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        for (size_t i = 0; i < size; i += 3) {
            const uint32_t a = data[i];
            const uint32_t b = i + 1 < size ? data[i + 1] : 0;
            const uint32_t c = i + 2 < size ? data[i + 2] : 0;
            const uint32_t triplet = (a << 16) | (b << 8) | c;
            out += alphabet[(triplet >> 18) & 0x3f];
            out += alphabet[(triplet >> 12) & 0x3f];
            out += i + 1 < size ? alphabet[(triplet >> 6) & 0x3f] : '=';
            out += i + 2 < size ? alphabet[triplet & 0x3f] : '=';
        }
    }

    void run() {
        zmq::context_t context(1);
        zmq::socket_t socket(context, zmq::socket_type::pub);
        socket.set(zmq::sockopt::sndhwm, 2);
        try {
            socket.bind("tcp://*:" + std::to_string(port_));
        } catch (const zmq::error_t& e) {
            std::fprintf(stderr, "[LowStateBroadcaster] bind tcp://*:%d failed: %s\n", port_, e.what());
            running_ = false;
            return;
        }
        const auto period = std::chrono::microseconds(2000);  // 500 Hz
        auto next = std::chrono::steady_clock::now();
        uint32_t last_tick = 0;
        bool sent_any = false;
        while (running_) {
            const Snapshot snapshot = snapshot_();
            if (!sent_any || snapshot.tick() != last_tick) {
                const std::string payload = serialize(snapshot);
                try {
                    socket.send(zmq::buffer(payload), zmq::send_flags::dontwait);
                    last_tick = snapshot.tick();
                    sent_any = true;
                } catch (const zmq::error_t&) {
                    // No subscribers yet or tx buffer full; drop this frame.
                }
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
