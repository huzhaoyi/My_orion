/* 聚合 PoseCache / TargetCache 为单一 PerceptionSnapshot */

#ifndef ORION_MTC_PERCEPTION_PERCEPTION_SNAPSHOT_PROVIDER_HPP
#define ORION_MTC_PERCEPTION_PERCEPTION_SNAPSHOT_PROVIDER_HPP

#include "orion_mtc/perception/perception_snapshot.hpp"
#include <memory>
#include <rclcpp/clock.hpp>

namespace orion_mtc
{

class PoseCache;
class TargetCache;

class PerceptionSnapshotProvider
{
public:
    PerceptionSnapshotProvider(const std::shared_ptr<PoseCache>& object_pose_cache,
                               const std::shared_ptr<TargetCache>& target_cache,
                               rclcpp::Clock::SharedPtr clock);

    PerceptionSnapshot snapshot() const;

private:
    std::shared_ptr<PoseCache> object_pose_cache_;
    std::shared_ptr<TargetCache> target_cache_;
    rclcpp::Clock::SharedPtr clock_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_PERCEPTION_SNAPSHOT_PROVIDER_HPP
