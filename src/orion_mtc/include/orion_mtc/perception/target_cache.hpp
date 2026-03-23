/* 多目标缓存：/manipulator/target_set */

#ifndef ORION_MTC_PERCEPTION_TARGET_CACHE_HPP
#define ORION_MTC_PERCEPTION_TARGET_CACHE_HPP

#include <orion_mtc_msgs/msg/target_set.hpp>
#include <mutex>
#include <optional>

namespace orion_mtc
{

class TargetCache
{
public:
    void update(const orion_mtc_msgs::msg::TargetSet& msg);
    std::optional<orion_mtc_msgs::msg::TargetSet> latest() const;

private:
    mutable std::mutex mutex_;
    orion_mtc_msgs::msg::TargetSet last_;
    bool has_data_ = false;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_TARGET_CACHE_HPP
