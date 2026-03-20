// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"

namespace unitree
{

template <typename LowStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_)
    : lowstate(lowstate_)
    {
        data.joystick = &lowstate->joystick;
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        // base_angular_velocity
        for(int i(0); i<3; i++) {
            data.root_ang_vel_b[i] = lowstate->msg_.imu_state().gyroscope()[i];
        }
        // project_gravity_body
        data.root_quat_w = Eigen::Quaternionf(
            lowstate->msg_.imu_state().quaternion()[0],
            lowstate->msg_.imu_state().quaternion()[1],
            lowstate->msg_.imu_state().quaternion()[2],
            lowstate->msg_.imu_state().quaternion()[3]
        );
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        // joint positions and velocities
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].q();
            data.joint_vel[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].dq();
        }
    }

    LowStatePtr lowstate;
};

template <typename LowStatePtr, typename CameraDataPtr, typename TorsoImuPtr>
class CameraArticulation : public isaaclab::Articulation
{
public:
    CameraArticulation(LowStatePtr lowstate_, CameraDataPtr cameradata_ = nullptr, TorsoImuPtr torsoimu_ = nullptr)
    : lowstate(lowstate_), cameradata(cameradata_), torsoimu(torsoimu_)
    {
        data.joystick = &lowstate->joystick;
        data.depth_image_buffer = std::make_unique<CircularBuffer<std::vector<float>>>(30);
    }

    void update() override
    {
        std::lock_guard<std::mutex> lock(lowstate->mutex_);
        // base_angular_velocity
        for(int i(0); i<3; i++) {
            data.root_ang_vel_b[i] = torsoimu->msg_.gyroscope()[i];
        }
        // project_gravity_body
        data.root_quat_w = Eigen::Quaternionf(
            torsoimu->msg_.quaternion()[0],
            torsoimu->msg_.quaternion()[1],
            torsoimu->msg_.quaternion()[2],
            torsoimu->msg_.quaternion()[3]
        );
        data.projected_gravity_b = data.root_quat_w.conjugate() * data.GRAVITY_VEC_W;
        // joint positions and velocities
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].q();
            data.joint_vel[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].dq();
        }

        std::vector<float> image;
        for (int i = 0; i < cameradata->msg_.width() * cameradata->msg_.height(); ++i) {
            image.push_back(cameradata->msg_.data()[i]);
        }
        data.depth_image_buffer->append(image);
    }

    LowStatePtr lowstate;
    CameraDataPtr cameradata;
    TorsoImuPtr torsoimu;
};

}
