/* target_cache：/manipulator/target_set 最近一帧的互斥保护存储 */

#include "orion_mtc/perception/target_cache.hpp"

namespace orion_mtc
{

/*
 * 互斥下覆盖 last_ 并置 has_data_，供多线程订阅回调与决策线程读快照。
 */
void TargetCache::update(const orion_mtc_msgs::msg::TargetSet& msg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_ = msg;
    has_data_ = true;
}

/*
 * 拷贝最近 TargetSet；从未收到则 nullopt。
 */
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
