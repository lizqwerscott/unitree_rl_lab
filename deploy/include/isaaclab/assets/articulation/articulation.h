// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <eigen3/Eigen/Dense>
#include "unitree/dds_wrapper/common/unitree_joystick.hpp"

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

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

    // Root angular velocity in base world frame.
    Eigen::Vector3f root_ang_vel_b;

    // Projection of the gravity direction on base frame.
    Eigen::Vector3f projected_gravity_b;

    Eigen::Quaternionf root_quat_w;

    std::vector<float> joint_ids_map;

    unitree::common::UnitreeJoystick* joystick = nullptr;

    isaaclab::MotionLoader* motion_loader = nullptr;

    // Pinocchio
    pinocchio::Model model_biped_fixed;
    int model_nv;
    pinocchio::Data data_biped_fixed;
};

class Articulation
{
public:
    Articulation(std::string urdf_pathIn = ""){
        pinocchio::urdf::buildModel(urdf_pathIn, data.model_biped_fixed);

        data.data_biped_fixed = pinocchio::Data(data.model_biped_fixed);
        data.model_nv = data.model_biped_fixed.nv;
    }

    virtual void update(){};

    ArticulationData data;
};

};
