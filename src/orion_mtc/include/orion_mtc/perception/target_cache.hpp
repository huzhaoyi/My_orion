/* 多目标缓存：收 TargetSet、校验长度、解析为 TargetObject 列表、线程安全访问 */

#ifndef ORION_MTC_PERCEPTION_TARGET_CACHE_HPP
#define ORION_MTC_PERCEPTION_TARGET_CACHE_HPP

#include "orion_mtc/core/target_object.hpp"
#include <orion_mtc_msgs/msg/target_set.hpp>
#include <mutex>
#include <string>
#include <vector>

namespace orion_mtc
{

class TargetCache
{
public:
  /** expected_frame_id：与 header.frame_id 一致才接受，空串表示不校验 */
  explicit TargetCache(const std::string& expected_frame_id = "base_link");

  /** 用最新一帧 TargetSet 更新缓存；校验 num_targets 与 positions/directions 长度 */
  void update(const orion_mtc_msgs::msg::TargetSet& msg);

  /** 返回当前帧解析后的目标列表（副本），无数据时为空 */
  std::vector<TargetObject> latest() const;

  /** 是否有至少一个有效目标 */
  bool hasTargets() const;

private:
  std::string expected_frame_id_;
  mutable std::mutex mutex_;
  std::vector<TargetObject> targets_;
  bool has_targets_ = false;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_TARGET_CACHE_HPP
