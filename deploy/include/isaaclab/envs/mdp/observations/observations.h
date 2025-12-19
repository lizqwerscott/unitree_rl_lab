// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <string>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "isaaclab/envs/manager_based_rl_env.h"

namespace isaaclab
{
namespace mdp
{

REGISTER_OBSERVATION(base_ang_vel)
{
    auto & asset = env->robot;
    auto & data = asset->data.root_ang_vel_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(projected_gravity)
{
    auto & asset = env->robot;
    auto & data = asset->data.projected_gravity_b;
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(joint_pos)
{
    auto & asset = env->robot;
    std::vector<float> data;

    std::vector<int> joint_ids;
    try {
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
    } catch(const std::exception& e) {
    }

    if(joint_ids.empty())
    {
        data.resize(asset->data.joint_pos.size());
        for(size_t i = 0; i < asset->data.joint_pos.size(); ++i)
        {
            data[i] = asset->data.joint_pos[i];
        }
    }
    else
    {
        data.resize(joint_ids.size());
        for(size_t i = 0; i < joint_ids.size(); ++i)
        {
            data[i] = asset->data.joint_pos[joint_ids[i]];
        }
    }

    return data;
}

REGISTER_OBSERVATION(joint_pos_rel)
{
    auto & asset = env->robot;
    std::vector<float> data;

    data.resize(asset->data.joint_pos.size());
    for(size_t i = 0; i < asset->data.joint_pos.size(); ++i) {
        data[i] = asset->data.joint_pos[i] - asset->data.default_joint_pos[i];
    }

    try {
        std::vector<int> joint_ids;
        joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();
        if(!joint_ids.empty()) {
            std::vector<float> tmp_data;
            tmp_data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i){
                tmp_data[i] = data[joint_ids[i]];
            }
            data = tmp_data;
        }
    } catch(const std::exception& e) {
    
    }

    return data;
}

REGISTER_OBSERVATION(joint_vel_rel)
{
    auto & asset = env->robot;
    auto data = asset->data.joint_vel;

    try {
        const std::vector<int> joint_ids = params["asset_cfg"]["joint_ids"].as<std::vector<int>>();

        if(!joint_ids.empty()) {
            data.resize(joint_ids.size());
            for(size_t i = 0; i < joint_ids.size(); ++i) {
                data[i] = asset->data.joint_vel[joint_ids[i]];
            }
        }
    } catch(const std::exception& e) {
    }
    return std::vector<float>(data.data(), data.data() + data.size());
}

REGISTER_OBSERVATION(last_action)
{
    auto data = env->action_manager->action();
    return std::vector<float>(data.data(), data.data() + data.size());
};

REGISTER_OBSERVATION(velocity_commands)
{
    std::vector<float> obs(3);
    auto & joystick = env->robot->data.joystick;

    const auto cfg = env->cfg["commands"]["base_velocity"]["ranges"];

    // 优化：使用线性映射，根据摇杆输入的正负分别使用最大和最小速度的绝对值
    float ly = joystick->ly();
    float min_vel = cfg["lin_vel_x"][0].as<float>();
    float max_vel = cfg["lin_vel_x"][1].as<float>();
    obs[0] = ly * (ly < 0 ? std::abs(min_vel) : std::abs(max_vel));
    min_vel = cfg["lin_vel_y"][0].as<float>();
    max_vel = cfg["lin_vel_y"][1].as<float>();
    float lx = -joystick->lx();
    obs[1] = lx * (lx < 0 ? std::abs(min_vel) : std::abs(max_vel));
    // obs[1] = std::clamp(-joystick->lx(), cfg["lin_vel_y"][0].as<float>(), cfg["lin_vel_y"][1].as<float>());
    obs[2] = std::clamp(-joystick->rx(), cfg["ang_vel_z"][0].as<float>(), cfg["ang_vel_z"][1].as<float>());

    return obs;
}

REGISTER_OBSERVATION(gait_phase)
{
    float period = params["period"].as<float>();
    float delta_phase = env->step_dt * (1.0f / period);

    env->global_phase += delta_phase;
    env->global_phase = std::fmod(env->global_phase, 1.0f);

    std::vector<float> obs(2);
    obs[0] = std::sin(env->global_phase * 2 * M_PI);
    obs[1] = std::cos(env->global_phase * 2 * M_PI);
    return obs;
}

// for amp
Eigen::Quaternionf quat_yaw_quat(const Eigen::Quaternionf& quat) {
    // Extract quaternion components
    double qw = quat.w();
    double qx = quat.x();
    double qy = quat.y();
    double qz = quat.z();

    // Compute yaw angle
    // yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    double yaw = std::atan2(2.0f * (qw * qz + qx * qy), 
                          1.0f - 2.0f * (qy * qy + qz * qz));

    // Create quaternion with only yaw rotation around Z-axis
    Eigen::Quaternionf quat_yaw;
    quat_yaw = Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
    
    return quat_yaw.normalized();
}


REGISTER_OBSERVATION(root_local_rot_tan_norm)
{
    auto & asset = env->robot;
    auto & root_quat = asset->data.root_quat_w;
    Eigen::Quaternionf yaw_quat = quat_yaw_quat(root_quat);

    Eigen::Quaternionf yaw_quat_conj = yaw_quat.conjugate();
    
    // Step 3: Multiply: yaw_quat_conj * root_quat
    Eigen::Quaternionf root_quat_local = yaw_quat_conj * root_quat;

    Eigen::Matrix3f rot_mat = root_quat_local.toRotationMatrix();
    std::vector<float> data(6);
    
    // 使用旋转矩阵的第一列作为切向量，最后一列作为法向量
    // 第一列 (column 0): tangent vector
    data[0] = rot_mat(0, 0);
    data[1] = rot_mat(1, 0);
    data[2] = rot_mat(2, 0);
    
    // // 最后一列 (column 2): normal vector  
    data[3] = rot_mat(0, 2);
    data[4] = rot_mat(1, 2);
    data[5] = rot_mat(2, 2);
            
    return data;
}

REGISTER_OBSERVATION(key_body_pos_b)
{
    auto & asset = env->robot;
    auto & quat = asset->data.root_quat_w;
    Eigen::Matrix3f rot_mat = quat.toRotationMatrix();
    std::vector<std::string> body_names;
    try {
        body_names = params["asset_cfg"]["body_names"].as<std::vector<std::string>>();
    } catch(const std::exception& e) {
    }
    std::vector<float> data(body_names.size() * 3);
    std::vector<pinocchio::FrameIndex> body_ids;
    for (int i = 0; i < body_names.size(); i++) {
        
        pinocchio::FrameIndex id = asset->data.model_biped_fixed.getFrameId(body_names[i]);
        pinocchio::updateFramePlacement(asset->data.model_biped_fixed, asset->data.data_biped_fixed, id);
        
        Eigen::Vector3d pos = asset->data.data_biped_fixed.oMf[id].translation();

        data[i * 3 + 0] = pos(0);
        data[i * 3 + 1] = pos(1);
        data[i * 3 + 2] = pos(2);
    }
            
    return data;
}
}
}
