/* Pick 任务构图层：仅负责构建 pick task，不负责执行、不改状态 */

#ifndef ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP
#define ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/planning/cable_segments.hpp"
#include "orion_mtc/planning/cable_side_grasp.hpp"
#include <moveit/task_constructor/task.h>
#include <string>
#include <vector>

#include <rclcpp/node.hpp>

namespace orion_mtc
{

class PickTaskBuilder
{
public:
  PickTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config);

  /* 缆绳侧向包夹 + 分段碰撞：加入多段缆绳、仅对 local 段放宽 ACM、闭爪后删除全部段并 retreat/lift */
  moveit::task_constructor::Task buildFromCableCandidate(
      const std::vector<CableSegment>& segments,
      const CableGraspCandidate& candidate,
      const std::string& plan_frame);

private:
  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP
