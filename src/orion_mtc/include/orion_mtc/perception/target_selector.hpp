/* 目标选择：从多个 TargetObject 中选一个当前最优（过滤非法、打分） */

#ifndef ORION_MTC_PERCEPTION_TARGET_SELECTOR_HPP
#define ORION_MTC_PERCEPTION_TARGET_SELECTOR_HPP

#include "orion_mtc/core/target_object.hpp"
#include <optional>
#include <vector>

namespace orion_mtc
{

/** 选目标时参考的“名义抓取参考点”（如机械臂前方），用于距离打分，单位 m */
struct TargetSelectorParams
{
  double nominal_x = 0.0;
  double nominal_y = 0.0;
  double nominal_z = 0.3;
  /** 优先更接近竖直的轴（top grasp），即 axis_direction 与 world z 的点积权重 */
  double alignment_weight = 0.2;
  /** 距离权重：越近越好 */
  double distance_weight = 1.0;
  /** 最低置信度阈值，低于则过滤 */
  double min_confidence = 0.0;
};

class TargetSelector
{
public:
  explicit TargetSelector(const TargetSelectorParams& params = TargetSelectorParams());

  /** 从 targets 中选一个最优目标；无有效目标返回 std::nullopt */
  std::optional<TargetObject> select(const std::vector<TargetObject>& targets) const;

  void setParams(const TargetSelectorParams& params);

private:
  TargetSelectorParams params_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_TARGET_SELECTOR_HPP
