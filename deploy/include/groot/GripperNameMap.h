#pragma once

#include <array>
#include <chrono>
#include <cstddef>

// Dex1_1 gripper side table and command defaults.
//
// The gripper is a separate serial<->DDS service (dex1_1_service); this header
// only names the two sides and the DDS topic shape so the same constants are
// shared by the ZMQ parser, the DDS bridge and the state broadcaster.
//
// Motor id order is 0 = right, 1 = left (note: right comes first).

namespace groot {
namespace dex1 {

inline constexpr size_t kNumGrippers = 2;
inline constexpr const char* kSides[kNumGrippers] = {"right", "left"};
inline constexpr size_t kRight = 0;
inline constexpr size_t kLeft = 1;

inline constexpr const char* kCmdTopicPrefix = "rt/dex1/";
inline constexpr const char* kStateTopicPrefix = "rt/dex1/";

inline constexpr int kDefaultMode = 1;      // FOC
inline constexpr float kDefaultKp = 5.0f;
inline constexpr float kDefaultKd = 0.05f;
inline constexpr float kDefaultMaxRateRadPerS = 6.0f;
inline constexpr double kDefaultCommandTimeoutS = 0.5;

// Calibrated mechanical range, as written by `dex1_1_service --calibration`
// (lower 0.0, upper 322 deg). q = 0 is fully closed, the upper bound fully
// open. These are only *defaults*: the live values come from
// FSM.Groot.gripper.q_min / q_max, because the real readings must be measured
// on the robot. The service itself never clamps, so this range is the only
// thing standing between a bad command and the mechanical end stop.
inline constexpr float kDefaultQMin = 0.0f;
inline constexpr float kDefaultQMax = 5.6217f;  // 322 deg

inline float clamp_q(float q, float q_min, float q_max) {
    return q < q_min ? q_min : (q > q_max ? q_max : q);
}

// One parsed gripper block from a LeRobot action frame, shared between
// RemoteCommandReceiver (producer) and GripperBridge (consumer).
//
// Only the target angle travels on the wire. kp/kd/mode are hardware
// parameters owned by FSM.Groot.gripper (see GripperBridge); the frame cannot
// override them.
//
// Each side is optional: `has_target` says whether this frame updated that
// side; the bridge keeps the previous target for sides it does not mention.
struct GripperTargets {
    std::array<bool, kNumGrippers> has_target{};
    std::array<float, kNumGrippers> q{};
    double timestamp = 0.0;  // shared with the carrying action frame
    std::chrono::steady_clock::time_point received{};
    bool valid = false;      // frame carried at least one usable side
};

}  // namespace dex1
}  // namespace groot
