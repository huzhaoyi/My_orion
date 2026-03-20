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
  return std::abs(j0) < 0.15 && std::abs(j1) < 0.15;
}

bool sceneDiffHasAttach(const moveit_task_constructor_msgs::msg::SubTrajectory& sub)
{
  return sub.scene_diff.is_diff &&
         !sub.scene_diff.robot_state.attached_collision_objects.empty();
}

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

SolutionExecutor::SolutionExecutor(PlanningSceneManager* scene_manager,
                                   TrajectoryExecutor* trajectory_executor)
  : scene_manager_(scene_manager), trajectory_executor_(trajectory_executor)
{
}

bool SolutionExecutor::executeSolution(
    const moveit_task_constructor_msgs::msg::Solution& solution_msg,
    WaitForGrippedFn wait_for_gripped,
    StageReportFn stage_report,
    const std::string& job_id,
    const std::string& task_type,
    const std::vector<std::string>& stage_names)
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
        if (!wait_for_gripped(true, 5.0))
          RCLCPP_WARN(LOGGER, "executeSolution: wait gripped timeout, continue anyway");
        have_waited_gripped = true;
      }
      else if (have_waited_gripped)
      {
        if (!wait_for_gripped(false, 5.0))
          RCLCPP_WARN(LOGGER, "executeSolution: wait unlock timeout, continue anyway");
      }
    }
  }
  return true;
}

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
    const std::vector<std::string>& cable_world_object_ids)
{
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
    if ((has_attach || is_remove_cable_segments) && !last_trajectory.points.empty())
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
        if (!wait_for_gripped(true, 5.0))
        {
          RCLCPP_WARN(LOGGER, "executePickSolution: wait gripped timeout, pick failed (no grip detected)");
          return false;
        }
        have_waited_gripped = true;
      }
      else if (have_waited_gripped)
      {
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
