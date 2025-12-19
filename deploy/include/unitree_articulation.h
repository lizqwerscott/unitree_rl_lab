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
    BaseArticulation(LowStatePtr lowstate_, std::string urdf_pathIn = "")
    : isaaclab::Articulation(urdf_pathIn), lowstate(lowstate_)
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

        const int MOTOR_NUM = 29;

        const std::array<float, MOTOR_NUM> robot_control_index{
            0,  1,  2,  3,  4,  5,      // left leg
            6,  7,  8,  9,  10, 11,     // right leg
            12, -1, -1,                 // waist
            13, 14, 15, 16, 17, -1, -1, // left arms
            18, 19, 20, 21, 22, -1, -1  // right arms
        };
        
        Eigen::VectorXd q_fixed = Eigen::VectorXd::Zero(23);
        

        for (int i = 0; i < MOTOR_NUM; ++i) {
            if (robot_control_index[i] < 0) {
                continue;
            }
            q_fixed[robot_control_index[i]] = lowstate->msg_.motor_state()[i].q();
        }

        // update pinocchio
        pinocchio::forwardKinematics(data.model_biped_fixed, data.data_biped_fixed, q_fixed);
    }
    
    LowStatePtr lowstate;
};

}
