/* SolutionExecutor：解析 MTC Solution、分段改 scene、调 TrajectoryExecutor 与 gripped 等待 */

#include "orion_mtc/execution/solution_executor.hpp"
#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/core/constants.hpp"
#include <moveit/robot_state/robot_state.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <algorithm>

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.execution");

namespace
{
/*
 * 子轨迹关节名若仅含 HAND_JOINTS 而不含 ARM_JOINTS，则视为纯夹爪段（用于 gripped 等待逻辑）。
 */
bool isHandOnlySegment(const moveit_task_constructor_msgs::msg::SubTrajectory& sub)
{
  const auto& names = sub.trajectory.joint_trajectory.joint_names;
  for (const auto& n : names)
  {
    if (std::find(ARM_JOINTS.begin(), ARM_JOINTS.end(), n) != ARM_JOINTS.end())
      return false;
  }
  return std::find_first_of(names.begin(), names.end(), HAND_JOINTS.begin(), HAND_JOINTS.end()) !=
         names.end();
}

/*
 * 轨迹末点左右夹爪关节位置绝对值均小于阈值时视为闭合姿态（与硬件开度标定一致）。
 */
bool isGripperClosedInSegment(const moveit_task_constructor_msgs::msg::SubTrajectory& sub)
{
  const auto& traj = sub.trajectory.joint_trajectory;
  if (traj.points.empty() || traj.joint_names.size() != traj.points.back().positions.size())
    return false;
  std::vector<size_t> hand_idx;
  for (const auto& hn : HAND_JOINTS)
  {
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
      if (traj.joint_names[i] == hn)
      {
        hand_idx.push_back(i);
        break;
      }
    }
  }
  if (hand_idx.size() != 2u)
    return false;
  const auto& p = traj.points.back().positions;
  double j0 = p[hand_idx[0]];
  double j1 = p[hand_idx[1]];
  return std::abs(j0) < 0.15f && std::abs(j1) < 0.15f;
}

/*
 * 与 SRDF hand「open」量级一致（约 ±0.4）：末端姿态明显张开时才做张开后等待，避免二次闭合段误触 unlock。
 */
bool isGripperOpenInSegment(const moveit_task_constructor_msgs::msg::SubTrajectory& sub)
{
  const auto& traj = sub.trajectory.joint_trajectory;
  if (traj.points.empty() || traj.joint_names.size() != traj.points.back().positions.size())
    return false;
  std::vector<size_t> hand_idx;
  for (const auto& hn : HAND_JOINTS)
  {
    for (size_t i = 0; i < traj.joint_names.size(); ++i)
    {
      if (traj.joint_names[i] == hn)
      {
        hand_idx.push_back(i);
        break;
      }
    }
  }
  if (hand_idx.size() != 2u)
    return false;
  const auto& p = traj.points.back().positions;
  double j0 = p[hand_idx[0]];
  double j1 = p[hand_idx[1]];
  return std::abs(j0) > 0.25f && std::abs(j1) > 0.25f;
}

/*
 * 该段 scene_diff 是否包含附着碰撞体变更（attach/detach 语义）。
 */
bool sceneDiffHasAttach(const moveit_task_constructor_msgs::msg::SubTrajectory& sub)
{
  return sub.scene_diff.is_diff &&
         !sub.scene_diff.robot_state.attached_collision_objects.empty();
}

/*
 * 用轨迹末点关节位置前向运动学到手系 hand_frame，输出 tcp_pose_out（抓取成功时写入持物上下文）。
 */
bool computeTcpPoseFromTrajectoryEnd(const moveit::core::RobotModelConstPtr& robot_model,
                                    const trajectory_msgs::msg::JointTrajectory& traj,
                                    const std::string& hand_frame,
                                    geometry_msgs::msg::Pose& tcp_pose_out)
{
  if (traj.points.empty() || traj.joint_names.empty())
    return false;
  const auto& last_pt = traj.points.back();
  if (last_pt.positions.size() != traj.joint_names.size())
    return false;
  moveit::core::RobotState state(robot_model);
  state.setToDefaultValues();
  for (size_t i = 0; i < traj.joint_names.size(); ++i)
  {
    const std::string& name = traj.joint_names[i];
    if (state.getRobotModel()->hasJointModel(name))
    {
      state.setVariablePosition(name, last_pt.positions[i]);
    }
  }
  state.update();
  if (!state.knowsFrameTransform(hand_frame))
    return false;
  const Eigen::Isometry3d& T = state.getGlobalLinkTransform(hand_frame);
  tcp_pose_out.position.x = T.translation().x();
  tcp_pose_out.position.y = T.translation().y();
  tcp_pose_out.position.z = T.translation().z();
  Eigen::Quaterniond q(T.rotation());
  tcp_pose_out.orientation.x = q.x();
  tcp_pose_out.orientation.y = q.y();
  tcp_pose_out.orientation.z = q.z();
  tcp_pose_out.orientation.w = q.w();
  return true;
}
}  // namespace

/* 持有 scene 与 trajectory 执行器指针，不拥有所有权（由 OrionMTCNode 管理生命周期）。 */
SolutionExecutor::SolutionExecutor(PlanningSceneManager* scene_manager,
                                   TrajectoryExecutor* trajectory_executor)
  : scene_manager_(scene_manager), trajectory_executor_(trajectory_executor)
{
}

/*
 * 通用多段执行：按序 executeSubTrajectory；should_abort 为真时中断并上报 FAILED/E_STOP；
 * 手爪段可根据 wait_for_gripped 在闭合/张开后等待传感器。stage_report 可选。
 */
bool SolutionExecutor::executeSolution(
    const moveit_task_constructor_msgs::msg::Solution& solution_msg,
    WaitForGrippedFn wait_for_gripped,
    StageReportFn stage_report,
    const std::string& job_id,
    const std::string& task_type,
    const std::vector<std::string>& stage_names,
    ShouldAbortFn should_abort)
{
  if (solution_msg.sub_trajectory.empty())
  {
    RCLCPP_WARN(LOGGER, "executeSolution: no sub_trajectory in solution");
    return false;
  }
  RCLCPP_INFO(LOGGER, "executeSolution: executing %zu trajectory segments",
              solution_msg.sub_trajectory.size());

  auto stage_name_at = [&stage_names](size_t i) -> std::string {
    if (i < stage_names.size())
    {
      return stage_names[i];
    }
    return "segment_" + std::to_string(i);
  };

  bool have_waited_gripped = false;
  for (size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i)
  {
    if (should_abort && should_abort())
    {
      RCLCPP_WARN(LOGGER, "executeSolution: aborted before segment %zu (E_STOP)", i);
      if (stage_report && i < solution_msg.sub_trajectory.size())
      {
        const std::string name = stage_name_at(i);
        stage_report(job_id, task_type, i, name, "FAILED", "E_STOP");
      }
      return false;
    }
    const std::string name = stage_name_at(i);
    if (stage_report)
    {
      stage_report(job_id, task_type, i, name, "ENTER", "");
      stage_report(job_id, task_type, i, name, "RUNNING", "");
    }
    RCLCPP_INFO(LOGGER, "Executing segment %zu / %zu", i + 1, solution_msg.sub_trajectory.size());
    if (!trajectory_executor_->executeSubTrajectory(solution_msg.sub_trajectory[i], scene_manager_))
    {
      if (stage_report)
      {
        stage_report(job_id, task_type, i, name, "FAILED", "segment execution failed");
      }
      RCLCPP_ERROR(LOGGER, "executeSolution: segment %zu failed", i);
      return false;
    }
    if (stage_report)
    {
      stage_report(job_id, task_type, i, name, "DONE", "");
    }
    const auto& sub = solution_msg.sub_trajectory[i];
    if (isHandOnlySegment(sub) && wait_for_gripped)
    {
      if (isGripperClosedInSegment(sub) && !have_waited_gripped)
      {
        if (should_abort && should_abort())
        {
          RCLCPP_WARN(LOGGER, "executeSolution: aborted before wait gripped");
          return false;
        }
        if (!wait_for_gripped(true, 5.0))
          RCLCPP_WARN(LOGGER, "executeSolution: wait gripped timeout, continue anyway");
        have_waited_gripped = true;
      }
      else if (have_waited_gripped && isGripperOpenInSegment(sub))
      {
        if (should_abort && should_abort())
        {
          RCLCPP_WARN(LOGGER, "executeSolution: aborted before wait unlock");
          return false;
        }
        if (!wait_for_gripped(false, 5.0))
          RCLCPP_WARN(LOGGER, "executeSolution: wait unlock timeout, continue anyway");
      }
    }
  }
  return true;
}

/*
 * 抓取专用：在 executeSolution 基础上跟踪 attach 段、cable_world_object_ids 清理 world 缆段、
 * 闭合后若 wait gripped 失败可置 failed_no_grip_out；成功则填充 held_context_out（TCP、物体位姿等）。
 */
bool SolutionExecutor::executePickSolution(
    const moveit_task_constructor_msgs::msg::Solution& solution_msg,
    const geometry_msgs::msg::Pose& object_pose_at_grasp,
    const std::string& object_id,
    const moveit::core::RobotModelConstPtr& robot_model,
    HeldObjectContext& held_context_out,
    WaitForGrippedFn wait_for_gripped,
  StageReportFn stage_report,
  const std::string& job_id,
  const std::string& task_type,
  const std::vector<std::string>& stage_names,
  const std::vector<std::string>& cable_world_object_ids,
  ShouldAbortFn should_abort,
  bool* failed_no_grip_out)
{
  if (failed_no_grip_out)
  {
    *failed_no_grip_out = false;
  }
  if (solution_msg.sub_trajectory.empty())
  {
    RCLCPP_WARN(LOGGER, "executePickSolution: no sub_trajectory");
    return false;
  }
  const std::string hand_frame = "gripper_tcp";
  trajectory_msgs::msg::JointTrajectory last_trajectory;
  bool have_waited_gripped = false;

  auto stage_name_at = [&stage_names](size_t i) -> std::string {
    if (i < stage_names.size())
    {
      return stage_names[i];
    }
    return "segment_" + std::to_string(i);
  };

  for (size_t i = 0; i < solution_msg.sub_trajectory.size(); ++i)
  {
    if (should_abort && should_abort())
    {
      RCLCPP_WARN(LOGGER, "executePickSolution: aborted before segment %zu (E_STOP)", i);
      if (stage_report)
      {
        const std::string name = stage_name_at(i);
        stage_report(job_id, task_type, i, name, "FAILED", "E_STOP");
      }
      return false;
    }
    const std::string name = stage_name_at(i);
    if (stage_report)
    {
      stage_report(job_id, task_type, i, name, "ENTER", "");
      stage_report(job_id, task_type, i, name, "RUNNING", "");
    }
    const auto& sub = solution_msg.sub_trajectory[i];
    RCLCPP_INFO(LOGGER, "Executing pick segment %zu / %zu", i + 1, solution_msg.sub_trajectory.size());
    if (!trajectory_executor_->executeSubTrajectory(sub, scene_manager_))
    {
      if (stage_report)
      {
        stage_report(job_id, task_type, i, name, "FAILED", "segment execution failed");
      }
      RCLCPP_ERROR(LOGGER, "executePickSolution: segment %zu failed", i);
      return false;
    }
    if (stage_report)
    {
      stage_report(job_id, task_type, i, name, "DONE", "");
    }
    RCLCPP_INFO(LOGGER, "executePickSolution: segment %zu DONE, entering post-process name=%s", i,
                name.c_str());
    if (!sub.trajectory.joint_trajectory.points.empty())
    {
      last_trajectory = sub.trajectory.joint_trajectory;
    }
    const bool has_attach = sceneDiffHasAttach(sub);
    const bool is_remove_cable_segments = (name == "remove_cable_segments");
    const bool is_remove_targetsensor_peg = (name == "remove targetsensor peg mesh");
    if ((has_attach || is_remove_cable_segments || is_remove_targetsensor_peg) &&
        !last_trajectory.points.empty())
    {
      if (has_attach && scene_manager_)
      {
        scene_manager_->removeWorldObject("object");
      }
      if (is_remove_cable_segments && scene_manager_ && !cable_world_object_ids.empty())
      {
        for (const std::string& seg_id : cable_world_object_ids)
        {
          scene_manager_->removeWorldObject(seg_id);
        }
      }
      if (is_remove_targetsensor_peg && scene_manager_)
      {
        scene_manager_->removeWorldObject(TARGET_SENSOR_PEG_COLLISION_ID);
      }
      geometry_msgs::msg::Pose tcp_pose;
      if (computeTcpPoseFromTrajectoryEnd(robot_model, last_trajectory, hand_frame, tcp_pose))
      {
        held_context_out.valid = true;
        held_context_out.object_id = object_id.empty() ? "object" : object_id;
        held_context_out.scene_attach_id = has_attach ? "object" : "";
        held_context_out.attach_link = hand_frame;
        held_context_out.object_pose_at_grasp = object_pose_at_grasp;
        held_context_out.tcp_pose_at_grasp = tcp_pose;
        Eigen::Isometry3d T_base_tcp = Eigen::Isometry3d::Identity();
        T_base_tcp.translate(Eigen::Vector3d(tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z));
        T_base_tcp.rotate(Eigen::Quaterniond(tcp_pose.orientation.w, tcp_pose.orientation.x,
                                             tcp_pose.orientation.y, tcp_pose.orientation.z));
        Eigen::Isometry3d T_base_obj = Eigen::Isometry3d::Identity();
        T_base_obj.translate(Eigen::Vector3d(object_pose_at_grasp.position.x, object_pose_at_grasp.position.y,
                                             object_pose_at_grasp.position.z));
        T_base_obj.rotate(Eigen::Quaterniond(object_pose_at_grasp.orientation.w,
                                             object_pose_at_grasp.orientation.x,
                                             object_pose_at_grasp.orientation.y,
                                             object_pose_at_grasp.orientation.z));
        held_context_out.tcp_to_object = T_base_tcp.inverse() * T_base_obj;
        RCLCPP_INFO(LOGGER, "executePickSolution: saved held context object_id=%s (attach=%d)",
                    held_context_out.object_id.c_str(), static_cast<int>(has_attach));
      }
      else
      {
        RCLCPP_WARN(LOGGER, "executePickSolution: FK for TCP at grasp failed, held context incomplete");
      }
    }
    if (isHandOnlySegment(sub) && wait_for_gripped)
    {
      if (isGripperClosedInSegment(sub) && !have_waited_gripped)
      {
        if (should_abort && should_abort())
        {
          RCLCPP_WARN(LOGGER, "executePickSolution: aborted before wait gripped");
          return false;
        }
        if (!wait_for_gripped(true, 5.0))
        {
          RCLCPP_WARN(LOGGER, "executePickSolution: wait gripped timeout, pick failed (no grip detected)");
          if (failed_no_grip_out)
          {
            *failed_no_grip_out = true;
          }
          return false;
        }
        have_waited_gripped = true;
      }
      else if (have_waited_gripped && isGripperOpenInSegment(sub))
      {
        if (should_abort && should_abort())
        {
          RCLCPP_WARN(LOGGER, "executePickSolution: aborted before wait unlock");
          return false;
        }
        if (!wait_for_gripped(false, 5.0))
        {
          RCLCPP_WARN(LOGGER, "executePickSolution: wait unlock timeout");
        }
      }
    }
    RCLCPP_INFO(LOGGER, "executePickSolution: segment %zu post-process finished name=%s", i,
                name.c_str());
  }
  RCLCPP_INFO(LOGGER, "executePickSolution: all segments finished, return true");
  return true;
}

}  // namespace orion_mtc
