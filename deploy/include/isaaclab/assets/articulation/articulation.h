// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

#include "isaaclab/utils/circular_buffer.h"

namespace isaaclab
{

class MotionLoader;

struct ArticulationData
{
    Eigen::Vector3f GRAVITY_VEC_W = Eigen::Vector3f(0.0f, 0.0f, -1.0f);
    Eigen::Vector3f FORWARD_VEC_B = Eigen::Vector3f(1.0f, 0.0f, 0.0f);

    std::vector<float> joint_stiffness; // sdk order
    std::vector<float> joint_damping; // sdk order

    // Joint positions of all joints.
    Eigen::VectorXf joint_pos;

    // Default joint positions of all joints.
    Eigen::VectorXf default_joint_pos;

    // Joint velocities of all joints.
    Eigen::VectorXf joint_vel;

    // Estimated joint torques, same ordering and sign convention as joint_pos / joint_vel.
    // Recorded for diagnostics only; no observation term reads it.
    Eigen::VectorXf joint_tau;

    // Root angular velocity in base world frame.
    Eigen::Vector3f root_ang_vel_b;

    // Projection of the gravity direction on base frame.
    Eigen::Vector3f projected_gravity_b;

    Eigen::Quaternionf root_quat_w;

    std::vector<float> joint_ids_map;

    unitree::common::UnitreeJoystick* joystick = nullptr;

    Eigen::Vector3f nav_cmd;

    bool nav_flag = false;

    std::unique_ptr<CircularBuffer<std::vector<float>>> depth_image_buffer = nullptr;

    // Capture time of the most recent camera message, in seconds, from its ROS header stamp.
    // Paired with the policy tick's own clock this measures the real camera-to-policy latency,
    // which nothing else on the robot observes. 0 until the first frame arrives.
    double last_camera_stamp = 0.0;

    // When true, `depth_image` substitutes a flat 0.1 for the measured frame. The block that reads
    // it predates this declaration, which is why that translation unit did not compile.
    // NOTHING SETS THIS YET, so the substitution is currently unreachable and behaviour is
    // unchanged. The evident intent is a camera-failure fallback; wiring it to a staleness check on
    // `last_camera_stamp` would complete it, but that changes what the robot does when the camera
    // hiccups and must not be switched on without a deliberate hardware test.
    bool depth_image_clear = false;
};

class Articulation
{
public:
    Articulation(){}

    virtual void update(){};

    ArticulationData data;
};

};
