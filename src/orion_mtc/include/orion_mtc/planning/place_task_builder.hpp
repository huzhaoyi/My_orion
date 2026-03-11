/* Place 任务构图层：仅负责 tracked precise place，不负责执行 */

#ifndef ORION_MTC_PLANNING_PLACE_TASK_BUILDER_HPP
#define ORION_MTC_PLANNING_PLACE_TASK_BUILDER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/core/held_object.hpp"
#include <moveit/task_constructor/task.h>

#include <rclcpp/node.hpp>

namespace orion_mtc
{

class PlaceTaskBuilder
{
public:
  PlaceTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config);
  moveit::task_constructor::Task build(double place_x, double place_y, double place_z,
                                       double place_qx, double place_qy, double place_qz, double place_qw,
                                       const HeldObjectContext& held);

private:
  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_PLACE_TASK_BUILDER_HPP
