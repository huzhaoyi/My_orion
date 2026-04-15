/* Pick 任务构图层：仅负责构建 pick task，不负责执行、不改状态 */

#ifndef ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP
#define ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/planning/cable_segments.hpp"
#include "orion_mtc/planning/cable_side_grasp.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
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

  /* 缆绳侧向包夹 + 分段碰撞：闭爪后删段、短退、回预抓取位并在预抓取位再次闭爪 */
  moveit::task_constructor::Task buildFromCableCandidate(
      const std::vector<CableSegment>& segments,
      const CableGraspCandidate& candidate,
      const std::string& plan_frame);

  /* TargetSensor 目标抓取：peg 场景体 + 与缆绳同构的 SerialContainer pick 链（不修改缆绳 buildFromCableCandidate）。
   * pregrasp_distance_m：沿 approach 方向 grasp→pregrasp 的距离 [m]，与缆绳 pregrasp_offset 含义一致。
   * approach_sign：接近方向符号（+1/-1），用于候选回退时翻转接近侧。
   * tool_roll_about_approach_deg：末端绕接近轴滚转 [deg]，用于姿态扰动候选。 */
  moveit::task_constructor::Task buildFromTargetSensorPose(
      const geometry_msgs::msg::PoseStamped& object_pose,
      const std::string& plan_frame,
      double pregrasp_distance_m,
      double approach_sign,
      double tool_roll_about_approach_deg);

private:
  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_PLANNING_PICK_TASK_BUILDER_HPP
