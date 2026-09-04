// Copyright (c) 2025, Unitree Robotics Co., Ltd.
// All rights reserved.

#pragma once

#include <cstddef>
#include <cstring>
#include <mutex>

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

template <typename LowStatePtr, typename CameraDataPtr, typename TorsoImuPtr, typename NavCmdPtr>
class CameraArticulation : public isaaclab::Articulation
{
public:
    CameraArticulation(LowStatePtr lowstate_, CameraDataPtr cameradata_ = nullptr, TorsoImuPtr torsoimu_ = nullptr, NavCmdPtr navcmd_ = nullptr)
    : lowstate(lowstate_), cameradata(cameradata_), torsoimu(torsoimu_), navcmd(navcmd_)
    {
        data.joystick = &lowstate->joystick;
        data.depth_image_buffer = std::make_unique<CircularBuffer<std::vector<float>>>(30);
    }

    void update() override
    {
        std::scoped_lock lock(lowstate->mutex_, cameradata->mutex_, torsoimu->mutex_, navcmd->mutex_);
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
        std::vector<float> joint_signs = {
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            -1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
            1,
        };

        if(data.joint_tau.size() != (int)data.joint_ids_map.size()) {
            data.joint_tau.setZero(data.joint_ids_map.size());
        }
        for(int i(0); i< data.joint_ids_map.size(); i++) {
            data.joint_pos[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].q() * joint_signs[i];
            data.joint_vel[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].dq() * joint_signs[i];
            // Same sign convention as position and velocity, so torque stays in policy space.
            data.joint_tau[i] = lowstate->msg_.motor_state()[data.joint_ids_map[i]].tau_est() * joint_signs[i];
        }

		const auto width = cameradata->msg_.width();
		const auto height = cameradata->msg_.height();
		const size_t pixel_count = static_cast<size_t>(width) * static_cast<size_t>(height);
		const size_t required_bytes = pixel_count * sizeof(float);
		const bool image_layout = width == 32 && height == 18;
		const bool flat_layout = width == 32 * 18 && height == 1;
		if ((!image_layout && !flat_layout) || pixel_count != 32 * 18
		    || cameradata->msg_.data().size() < required_bytes) {
			image_data_buffer_.assign(32 * 18, 0.0f);
		} else {
			image_data_buffer_.resize(pixel_count);
			std::memcpy(image_data_buffer_.data(), cameradata->msg_.data().data(), required_bytes);
		}
        data.depth_image_buffer->append(image_data_buffer_);
        // Kept for diagnostics: with the tick clock this is the real camera-to-policy latency.
        data.last_camera_stamp = cameradata->msg_.header().stamp().sec()
                               + 1e-9 * cameradata->msg_.header().stamp().nanosec();

        // nav cmd
        data.nav_cmd[0] = navcmd->msg_.linear().x();
        data.nav_cmd[1] = navcmd->msg_.linear().y();
        data.nav_cmd[2] = navcmd->msg_.angular().z();

    }

	std::vector<float> image_data_buffer_;

    LowStatePtr lowstate;
    CameraDataPtr cameradata;
    TorsoImuPtr torsoimu;
    NavCmdPtr navcmd;
};

}
