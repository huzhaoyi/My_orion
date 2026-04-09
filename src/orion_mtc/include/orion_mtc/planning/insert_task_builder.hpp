/* TargetSensor 插孔 MTC 构图：孔轴由 target_pose 姿态定义（局部 -Z 为插入方向），与缆绳 pick 链分离 */

#ifndef ORION_MTC_PLANNING_INSERT_TASK_BUILDER_HPP
#define ORION_MTC_PLANNING_INSERT_TASK_BUILDER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/task.h>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>

namespace orion_mtc
{

struct InsertTaskBuildResult
{
  moveit::task_constructor::Task task;
  std::vector<std::string> stage_names;
};

class InsertTaskBuilder
{
public:
  InsertTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config);

  /*
   * target_pose：孔位姿（base_link），姿态的局部 -Z 为插入轴（与历史竖直孔「沿 -world Z 下降」一致）。
   */
  InsertTaskBuildResult buildTargetInsertTask(const geometry_msgs::msg::PoseStamped& target_pose) const;

private:
  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
};

}  // namespace orion_mtc

#endif
