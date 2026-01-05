// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include "isaaclab/assets/articulation/articulation.h"

const std::string UNITREE_ROS_DIR = "PATH/UNITREE_ROS/";

namespace unitree
{

template <typename LowStatePtr>
class BaseArticulation : public isaaclab::Articulation
{
public:
    BaseArticulation(LowStatePtr lowstate_, bool g1_29p)
    : lowstate(lowstate_)
    {
        data.joystick = &lowstate->joystick;
        std::string urdf_pathIn;
        if (g1_29p) {
            urdf_pathIn = std::string(UNITREE_ROS_DIR + "robots/g1_description/g1_29dof_rev_1_0.urdf");
            ROBOT_NUM = 29;

            robot_control_index = {
                0,  1,  2,  3,  4,  5,      // left leg
                6,  7,  8,  9,  10, 11,     // right leg
                12, 13, 14,                 // waist
                15, 16, 17, 18, 19, 20, 21, // left arms
                22, 23, 24, 25, 26, 27, 28  // right arms
            };
        } else {
            urdf_pathIn = std::string(UNITREE_ROS_DIR + "robots/g1_description/g1_23dof_rev_1_0.urdf");
            ROBOT_NUM = 23;
            
            robot_control_index = {
                0,  1,  2,  3,  4,  5,      // left leg
                6,  7,  8,  9,  10, 11,     // right leg
                12, -1, -1,                 // waist
                13, 14, 15, 16, 17, -1, -1, // left arms
                18, 19, 20, 21, 22, -1, -1  // right arms
            };
        }
    
        pinocchio::urdf::buildModel(urdf_pathIn, data.model_biped_fixed);

        data.data_biped_fixed = pinocchio::Data(data.model_biped_fixed);
        data.model_nv = data.model_biped_fixed.nv;
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

        Eigen::VectorXd q_fixed = Eigen::VectorXd::Zero(ROBOT_NUM);
        
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
    bool g1_29p;
    int MOTOR_NUM = 29;
    int ROBOT_NUM;
    std::vector<float> robot_control_index;
};

}
