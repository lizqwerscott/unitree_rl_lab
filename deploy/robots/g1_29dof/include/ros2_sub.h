#ifndef ROS2_SUB_H
#define ROS2_SUB_H

#include <eigen3/Eigen/Dense>
#include "unitree/dds_wrapper/common/Subscription.h"
#include "unitree/dds_wrapper/robots/g1/defines.h"

#include "unitree/idl/ros2/PointCloud2_.hpp"
#include "unitree/idl/hg/IMUState_.hpp"

namespace unitree
{
namespace robot
{
namespace g1
{
namespace subscription
{

class CameraData : public SubscriptionBase<sensor_msgs::msg::dds_::PointCloud2_>
{
public:
    using SharedPtr = std::shared_ptr<CameraData>;

    CameraData(std::string topic = "rt/camera/processed_depth_cloud") : SubscriptionBase<MsgType>(topic) {}

};

class TorsoImu : public SubscriptionBase<unitree_hg::msg::dds_::IMUState_>
{
public:
    using SharedPtr = std::shared_ptr<TorsoImu>;

    TorsoImu(std::string topic = "rt/secondary_imu") : SubscriptionBase<MsgType>(topic) {}
};

} // namespace subscription
} // namespace g1
} // namespace robot
} // namespace unitree

#endif
