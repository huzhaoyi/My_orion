/* perception_snapshot_provider：从各 cache 组装 PerceptionSnapshot 快照 */

#include "orion_mtc/perception/perception_snapshot_provider.hpp"
#include "orion_mtc/perception/pose_cache.hpp"
#include "orion_mtc/perception/target_cache.hpp"
#include <rclcpp/rclcpp.hpp>

namespace orion_mtc
{

/*
 * 注入 object_pose 与 target 缓存及可选时钟；指针可为空，对应字段快照留空。
 */
PerceptionSnapshotProvider::PerceptionSnapshotProvider(const std::shared_ptr<PoseCache>& object_pose_cache,
                                                       const std::shared_ptr<TargetCache>& target_cache,
                                                       rclcpp::Clock::SharedPtr clock)
  : object_pose_cache_(object_pose_cache)
  , target_cache_(target_cache)
  , clock_(std::move(clock))
{
}

/*
 * 组装单一时间戳下的 PerceptionSnapshot：填入 optional pose 与 TargetSet。
 */
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
