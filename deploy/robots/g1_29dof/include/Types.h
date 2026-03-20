#pragma once

#include "unitree/dds_wrapper/robots/go2/go2.h"
#include "unitree/dds_wrapper/robots/g1/g1.h"

#include "ros2_sub.h"

using LowCmd_t = unitree::robot::g1::publisher::LowCmd;
using LowState_t = unitree::robot::g1::subscription::LowState;
using CameraData_t = unitree::robot::g1::subscription::CameraData;
using TorsoImu_t = unitree::robot::g1::subscription::TorsoImu;
