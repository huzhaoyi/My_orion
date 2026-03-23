#include "orion_mtc/perception/perception_snapshot_provider.hpp"
#include "orion_mtc/perception/pose_cache.hpp"
#include "orion_mtc/perception/target_cache.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orion_mtc
{

PerceptionSnapshotProvider::PerceptionSnapshotProvider(const std::shared_ptr<PoseCache>& object_pose_cache,
                                                       const std::shared_ptr<TargetCache>& target_cache,
                                                       rclcpp::Clock::SharedPtr clock)
  : object_pose_cache_(object_pose_cache)
  , target_cache_(target_cache)
  , clock_(std::move(clock))
{
}

PerceptionSnapshot PerceptionSnapshotProvider::snapshot() const
{
    PerceptionSnapshot out;
    out.stamp = clock_ ? clock_->now() : rclcpp::Time(0, 0, RCL_SYSTEM_TIME);
    if (object_pose_cache_)
    {
        out.object_pose = object_pose_cache_->latest();
    }
    if (target_cache_)
    {
        out.target_set = target_cache_->latest();
    }
    return out;
}

}  // namespace orion_mtc
