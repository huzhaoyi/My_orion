/* 从 PerceptionSnapshot 中选出当前抓取目标（多目标优先，否则回退单物体位姿） */

#ifndef ORION_MTC_DECISION_TARGET_SELECTOR_HPP
#define ORION_MTC_DECISION_TARGET_SELECTOR_HPP

#include "orion_mtc/perception/perception_snapshot.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <optional>

namespace orion_mtc
{

class TargetSelector
{
public:
    std::optional<geometry_msgs::msg::PoseStamped> selectPickTarget(const PerceptionSnapshot& snap) const;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_DECISION_TARGET_SELECTOR_HPP
