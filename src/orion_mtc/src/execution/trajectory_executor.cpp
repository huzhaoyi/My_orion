#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/core/constants.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <string>
#include <thread>

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.execution");

TrajectoryExecutor::TrajectoryExecutor(rclcpp::Node* node) : node_(node)
{
}

bool TrajectoryExecutor::sendJointTrajectory(const std::string& controller_name,
                                              const trajectory_msgs::msg::JointTrajectory& jt)
{
  if (jt.points.empty())
  {
    RCLCPP_DEBUG(LOGGER, "sendJointTrajectory: empty trajectory for %s", controller_name.c_str());
    return true;
  }
  constexpr double RAD_TO_DEG = 180.0 / 3.141592653589793;
  RCLCPP_INFO(LOGGER, "sendJointTrajectory: %s joints=%zu points=%zu (角度°)",
              controller_name.c_str(), jt.joint_names.size(), jt.points.size());
  for (size_t i = 0; i < jt.joint_names.size(); ++i)
  {
    RCLCPP_INFO(LOGGER, "  [%zu] %s", i, jt.joint_names[i].c_str());
  }
  for (size_t p = 0; p < jt.points.size(); ++p)
  {
    const auto& pt = jt.points[p];
    std::string buf = "  point[" + std::to_string(p) + "]";
    if (pt.positions.size() >= jt.joint_names.size())
    {
      for (size_t j = 0; j < jt.joint_names.size(); ++j)
      {
        buf += " " + std::to_string(static_cast<float>(pt.positions[j] * RAD_TO_DEG));
      }
    }
    RCLCPP_INFO(LOGGER, "%s", buf.c_str());
  }

  rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr client;
  bool need_wait_server = false;
  {
    std::lock_guard<std::mutex> lk(follow_jt_mutex_);
    auto it = follow_jt_clients_.find(controller_name);
    if (it == follow_jt_clients_.end())
    {
      std::string action_name = "/" + controller_name + "/follow_joint_trajectory";
      auto c = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
          node_, action_name);
      follow_jt_clients_.emplace(controller_name, c);
      client = c;
      need_wait_server = true;
    }
    else
    {
      client = it->second;
    }
  }
  if (need_wait_server && !client->wait_for_action_server(std::chrono::seconds(5)))
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: action /%s/follow_joint_trajectory not available",
                 controller_name.c_str());
    return false;
  }

  auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
  goal_msg.trajectory = jt;
  goal_msg.trajectory.header.stamp.sec = 0;
  goal_msg.trajectory.header.stamp.nanosec = 0;
  auto goal_handle_future = client->async_send_goal(goal_msg);
  if (goal_handle_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: send_goal timeout for %s", controller_name.c_str());
    return false;
  }
  auto goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: goal rejected for %s", controller_name.c_str());
    return false;
  }
  auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(60)) != std::future_status::ready)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: get_result timeout for %s", controller_name.c_str());
    client->async_cancel_goal(goal_handle);
    return false;
  }
  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(LOGGER, "sendJointTrajectory: %s failed result code %d",
                 controller_name.c_str(), static_cast<int>(result.code));
    return false;
  }
  return true;
}

bool TrajectoryExecutor::executeSubTrajectory(
    const moveit_task_constructor_msgs::msg::SubTrajectory& sub,
    PlanningSceneManager* scene_manager)
{
  if (sub.scene_diff.is_diff && scene_manager)
  {
    if (!scene_manager->applySceneDiff(sub.scene_diff))
    {
      RCLCPP_WARN(LOGGER, "executeSubTrajectory: apply_planning_scene failed or timed out");
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  const auto& traj = sub.trajectory.joint_trajectory;
  if (traj.points.empty())
  {
    return true;
  }

  std::vector<std::string> ctrl_names(sub.execution_info.controller_names.begin(),
                                      sub.execution_info.controller_names.end());
  if (ctrl_names.empty())
  {
    auto is_in = [](const std::string& j, const std::vector<std::string>& set) {
      return std::find(set.begin(), set.end(), j) != set.end();
    };
    bool any_arm = false;
    bool any_hand = false;
    bool any_other = false;
    for (const auto& j : traj.joint_names)
    {
      if (is_in(j, ARM_JOINTS))
        any_arm = true;
      else if (is_in(j, HAND_JOINTS))
        any_hand = true;
      else
        any_other = true;
    }
    if (traj.joint_names.empty())
    {
      RCLCPP_WARN(LOGGER, "executeSubTrajectory: trajectory has points but no joint_names");
      return true;
    }
    if (any_other)
      ctrl_names = { "arm_controller" };
    else if (any_arm && any_hand)
      ctrl_names = { "arm_controller", "hand_controller" };
    else if (any_arm)
      ctrl_names = { "arm_controller" };
    else if (any_hand)
      ctrl_names = { "hand_controller" };
    else
      ctrl_names = { "arm_controller" };
    RCLCPP_DEBUG(LOGGER, "executeSubTrajectory: controller_names empty, inferred %zu controller(s)",
                 ctrl_names.size());
  }

  std::vector<std::string> arm_joints(ARM_JOINTS.begin(), ARM_JOINTS.end());
  std::vector<std::string> hand_joints(HAND_JOINTS.begin(), HAND_JOINTS.end());

  auto jointIndex = [&traj](const std::vector<std::string>& names) {
    std::vector<size_t> idx;
    for (const auto& n : names)
    {
      for (size_t i = 0; i < traj.joint_names.size(); ++i)
      {
        if (traj.joint_names[i] == n)
        {
          idx.push_back(i);
          break;
        }
      }
    }
    return idx;
  };

  auto safe_extract = [](std::vector<double>& dst, const std::vector<double>& src,
                         const std::vector<size_t>& idx) -> bool {
    if (src.empty() || idx.empty())
      return false;
    size_t max_i = *std::max_element(idx.begin(), idx.end());
    if (max_i >= src.size())
      return false;
    for (size_t i : idx)
      dst.push_back(src[i]);
    return true;
  };

  auto extractSubTrajectory = [&traj, &safe_extract](const std::vector<size_t>& indices)
      -> trajectory_msgs::msg::JointTrajectory {
    trajectory_msgs::msg::JointTrajectory out;
    out.header = traj.header;
    if (indices.empty())
      return out;
    for (size_t i : indices)
    {
      if (i < traj.joint_names.size())
        out.joint_names.push_back(traj.joint_names[i]);
    }
    if (out.joint_names.size() != indices.size())
      return out;
    bool warned_vel = false;
    bool warned_acc = false;
    for (const auto& pt : traj.points)
    {
      trajectory_msgs::msg::JointTrajectoryPoint p;
      if (!safe_extract(p.positions, pt.positions, indices))
      {
        RCLCPP_ERROR(LOGGER, "executeSubTrajectory: point positions index out of range");
        return trajectory_msgs::msg::JointTrajectory();
      }
      if (!pt.velocities.empty() && !safe_extract(p.velocities, pt.velocities, indices) && !warned_vel)
      {
        RCLCPP_WARN(LOGGER, "executeSubTrajectory: velocities dimension mismatch, leaving empty");
        warned_vel = true;
      }
      if (!pt.accelerations.empty() &&
          !safe_extract(p.accelerations, pt.accelerations, indices) && !warned_acc)
      {
        RCLCPP_WARN(LOGGER, "executeSubTrajectory: accelerations dimension mismatch, leaving empty");
        warned_acc = true;
      }
      p.time_from_start = pt.time_from_start;
      out.points.push_back(p);
    }
    return out;
  };

  std::vector<std::pair<std::string, trajectory_msgs::msg::JointTrajectory>> to_send;
  for (const auto& ctrl : ctrl_names)
  {
    trajectory_msgs::msg::JointTrajectory jt_to_send;
    if (ctrl == "arm_controller")
    {
      auto idx = jointIndex(arm_joints);
      if (idx.empty())
        continue;
      jt_to_send = extractSubTrajectory(idx);
      if (jt_to_send.points.empty() && !idx.empty())
      {
        RCLCPP_ERROR(LOGGER, "executeSubTrajectory: arm segment extraction failed");
        return false;
      }
    }
    else if (ctrl == "hand_controller")
    {
      auto idx = jointIndex(hand_joints);
      if (idx.empty())
        continue;
      jt_to_send = extractSubTrajectory(idx);
      if (jt_to_send.points.empty() && !idx.empty())
      {
        RCLCPP_ERROR(LOGGER, "executeSubTrajectory: hand segment extraction failed");
        return false;
      }
    }
    else
    {
      jt_to_send = traj;
    }
    if (!jt_to_send.points.empty())
      to_send.emplace_back(ctrl, jt_to_send);
  }

  if (!traj.points.empty() && to_send.empty())
  {
    RCLCPP_WARN(LOGGER,
                "executeSubTrajectory: trajectory non-empty but no controller selected; joint_names=%zu",
                traj.joint_names.size());
  }

  if (to_send.size() == 1u)
  {
    if (!sendJointTrajectory(to_send[0].first, to_send[0].second))
      return false;
  }
  else if (to_send.size() > 1u)
  {
    std::vector<std::shared_future<bool>> results;
    for (const auto& p : to_send)
    {
      results.push_back(
          std::async(std::launch::async, [this, p]() {
            return sendJointTrajectory(p.first, p.second);
          }).share());
    }
    for (auto& r : results)
    {
      if (!r.get())
        return false;
    }
  }
  return true;
}

}  // namespace orion_mtc
