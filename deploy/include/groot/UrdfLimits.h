#pragma once

#include <array>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <algorithm>
#include <tinyxml2.h>

namespace groot {

struct JointLimit { float lower; float upper; float velocity; float effort; };

inline std::array<JointLimit, 29> load_urdf_limits(const std::filesystem::path& path) {
    tinyxml2::XMLDocument document;
    if (document.LoadFile(path.string().c_str()) != tinyxml2::XML_SUCCESS)
        throw std::runtime_error("Unable to parse URDF: " + path.string());
    constexpr std::array<const char*, 29> expected_names = {
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
        "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint"
    };
    std::array<JointLimit, 29> limits{};
    std::array<bool, 29> seen{};
    auto* robot = document.FirstChildElement("robot");
    if (!robot) throw std::runtime_error("URDF has no robot element: " + path.string());
    for (auto* joint = robot->FirstChildElement("joint"); joint; joint = joint->NextSiblingElement("joint")) {
        const char* type = joint->Attribute("type");
        const char* name = joint->Attribute("name");
        if (!type || !name || std::string(type) == "fixed") continue;
        const auto expected = std::find(expected_names.begin(), expected_names.end(), std::string(name));
        if (expected == expected_names.end())
            throw std::runtime_error("Unexpected controllable URDF joint: " + std::string(name));
        const size_t index = static_cast<size_t>(std::distance(expected_names.begin(), expected));
        if (seen[index]) throw std::runtime_error("Duplicate URDF joint: " + std::string(name));
        seen[index] = true;
        const auto* limit = joint->FirstChildElement("limit");
        if (!limit || limit->QueryFloatAttribute("lower", &limits[index].lower) != tinyxml2::XML_SUCCESS ||
            limit->QueryFloatAttribute("upper", &limits[index].upper) != tinyxml2::XML_SUCCESS ||
            limit->QueryFloatAttribute("velocity", &limits[index].velocity) != tinyxml2::XML_SUCCESS)
            throw std::runtime_error("Missing limit for URDF joint: " + std::string(name));
        limit->QueryFloatAttribute("effort", &limits[index].effort);
        if (!(limits[index].lower < limits[index].upper) || !(limits[index].velocity > 0.0f))
            throw std::runtime_error("Invalid limit for URDF joint: " + std::string(name));
    }
    for (size_t i = 0; i < seen.size(); ++i) {
        if (!seen[i]) throw std::runtime_error("Missing URDF joint: " + std::string(expected_names[i]));
    }
    return limits;
}

}  // namespace groot
