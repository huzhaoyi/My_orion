/* PlaceRelease 任务构图层：仅负责 release 类放置，不负责执行 */

#ifndef ORION_MTC_PLANNING_PLACE_RELEASE_TASK_BUILDER_HPP
#define ORION_MTC_PLANNING_PLACE_RELEASE_TASK_BUILDER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/task.h>
#include <string>

#include <rclcpp/node.hpp>

namespace orion_mtc
{

class PlaceReleaseTaskBuilder
{
public:
  PlaceReleaseTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config);
  moveit::task_constructor::Task build(const geometry_msgs::msg::PoseStamped& target_tcp_pose,
                                       const std::string& attached_object_id);

private:
  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_PLACE_RELEASE_TASK_BUILDER_HPP
