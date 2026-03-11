/* 抓取候选生成：根据 TargetObject 的 position + axis_direction 生成多个 object 位姿候选 */

#ifndef ORION_MTC_PERCEPTION_GRASP_GENERATOR_HPP
#define ORION_MTC_PERCEPTION_GRASP_GENERATOR_HPP

#include "orion_mtc/core/target_object.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>

namespace orion_mtc
{

/** 抓取候选：物体在 base_link 下的位姿，供 MTC pick 使用 */
using GraspCandidate = geometry_msgs::msg::PoseStamped;

struct GraspGeneratorParams
{
  /** 绕圆柱轴采样角度（度），多采样提高 IK 可解概率，如 0,30,60,90,120,150 */
  std::vector<double> yaw_degrees = { 0.0, 30.0, 60.0, 90.0, 120.0, 150.0 };
  /** 输出 PoseStamped 的 frame_id */
  std::string frame_id = "base_link";
};

class GraspGenerator
{
public:
  explicit GraspGenerator(const GraspGeneratorParams& params = GraspGeneratorParams());

  /** 根据 target 的 position + axis_direction 生成一组物体位姿候选（多组绕轴 yaw） */
  std::vector<GraspCandidate> generate(const TargetObject& target) const;

  void setParams(const GraspGeneratorParams& params);

private:
  GraspGeneratorParams params_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PERCEPTION_GRASP_GENERATOR_HPP
