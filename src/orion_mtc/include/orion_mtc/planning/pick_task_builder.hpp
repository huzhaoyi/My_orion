/* Pick 任务构图层：仅负责构建 pick task，不负责执行、不改状态 */

#ifndef ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP
#define ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/task_constructor/task.h>
#include <string>

#include <rclcpp/node.hpp>

namespace orion_mtc
{

class PickTaskBuilder
{
public:
  PickTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config);
  moveit::task_constructor::Task build(double obj_x, double obj_y, double obj_z,
                                       const geometry_msgs::msg::Quaternion& object_orientation,
                                       const std::string& object_id);

private:
  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP
