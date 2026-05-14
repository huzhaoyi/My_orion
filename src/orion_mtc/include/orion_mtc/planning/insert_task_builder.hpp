/* TargetSensor 插孔 MTC 构图：孔轴由 target_pose 姿态定义（局部 -Z 为插入方向），与缆绳 pick 链分离 */

#ifndef ORION_MTC_PLANNING_INSERT_TASK_BUILDER_HPP
#define ORION_MTC_PLANNING_INSERT_TASK_BUILDER_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>
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

  /*
   * 早锁存跳过部分轴向插入后，从 CurrentState 重规划与 buildTargetInsertTask 同几何的 lift clear
   * 三段笛卡尔子轨迹（peg_tip IK），避免沿用「整段插入完成」名义起点导致的先沿孔轴补深度再外撤。
   */
  bool planArmLiftRetreatSubTrajectories(const geometry_msgs::msg::PoseStamped& hole_pose_base,
                                          std::vector<moveit_task_constructor_msgs::msg::SubTrajectory>& out_arm_segments) const;

  /*
   * 早锁存缩短插入后：从 CurrentState 重规划「move to pre-insert (post release)」单段子轨迹，
   * 与 buildTargetInsertTask 中 pre_pose 几何一致，避免沿用整链名义起点的 PTP 先向孔侧甩动。
   */
  bool planPostReleaseMoveToPreInsertSubTrajectory(
      const geometry_msgs::msg::PoseStamped& hole_pose_base,
      moveit_task_constructor_msgs::msg::SubTrajectory& out_arm_sub) const;

private:
  rclcpp::Node::SharedPtr node_;
  MTCConfig config_;
};

}  // namespace orion_mtc

#endif
