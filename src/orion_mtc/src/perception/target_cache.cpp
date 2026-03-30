/* target_cache：/manipulator/target_set 最近一帧的互斥保护存储 */

#include "orion_mtc/perception/target_cache.hpp"

namespace orion_mtc
{

void TargetCache::update(const orion_mtc_msgs::msg::TargetSet& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_ = msg;
    has_data_ = true;
}

std::optional<orion_mtc_msgs::msg::TargetSet> TargetCache::latest() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_data_)
    {
        return std::nullopt;
    }
    return last_;
}

}  // namespace orion_mtc
