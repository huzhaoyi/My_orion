#include "orion_mtc/decision/target_selector.hpp"

namespace orion_mtc
{

std::optional<geometry_msgs::msg::PoseStamped> TargetSelector::selectPickTarget(
    const PerceptionSnapshot& snap) const
{
    if (snap.target_set.has_value() && !snap.target_set->targets.empty())
    {
        return snap.target_set->targets.front();
    }
    return snap.object_pose;
}

}  // namespace orion_mtc
