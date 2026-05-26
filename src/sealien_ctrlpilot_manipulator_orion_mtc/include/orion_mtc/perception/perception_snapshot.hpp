/* 感知快照：编排/决策层统一读取，避免直接耦合多个 cache */

#ifndef ORION_MTC_PERCEPTION_PERCEPTION_SNAPSHOT_HPP
#define ORION_MTC_PERCEPTION_PERCEPTION_SNAPSHOT_HPP

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <orion_mtc_msgs/msg/target_set.hpp>
#include <optional>
#include <rclcpp/time.hpp>

namespace orion_mtc
{

struct PerceptionSnapshot
{
    std::optional<geometry_msgs::msg::PoseStamped> object_pose;
    std::optional<orion_mtc_msgs::msg::TargetSet> target_set;
    rclcpp::Time stamp;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_PERCEPTION_SNAPSHOT_HPP
