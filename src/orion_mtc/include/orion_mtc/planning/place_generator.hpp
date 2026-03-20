/* 放置候选生成：从目标位姿/区域采样多个 PlaceCandidate，供依次 plan/execute */

#ifndef ORION_MTC_PLANNING_PLACE_GENERATOR_HPP
#define ORION_MTC_PLANNING_PLACE_GENERATOR_HPP

#include "orion_mtc/core/held_object.hpp"
#include "orion_mtc/core/place_types.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>

namespace orion_mtc
{

struct PlaceGeneratorParams
{
  /** 位置扰动步长 [m]，在 xy 上生成 center, ±step */
  double position_step_xy = 0.01;
  /** 是否生成位置扰动（中心 + 四向） */
  bool enable_position_samples = true;
  /** 绕世界 z 的 yaw 采样 [度]，如 0, ±10, ±20 */
  std::vector<double> yaw_degrees = { 0.0, 10.0, -10.0, 20.0, -20.0 };
  /** 输出 frame_id */
  std::string frame_id = "base_link";
};

class PlaceGenerator
{
public:
  explicit PlaceGenerator(const PlaceGeneratorParams& params = PlaceGeneratorParams());

  /**
   * 从单目标位姿生成多候选：位置微调 + 绕 z 的 yaw 采样。
   * 每个候选含 object_pose（及由 held 反推的 tcp_pose）、score、source。
   */
  std::vector<PlaceCandidate> generate(const geometry_msgs::msg::PoseStamped& target_pose,
                                       const HeldObjectContext& held) const;

  void setParams(const PlaceGeneratorParams& params);

private:
  PlaceGeneratorParams params_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_PLACE_GENERATOR_HPP
