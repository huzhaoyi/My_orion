/* 执行层：controller client 管理、轨迹按 arm/hand 拆分、发送 FollowJointTrajectory、等待结果 */

#ifndef ORION_MTC_EXECUTION_TRAJECTORY_EXECUTOR_HPP
#define ORION_MTC_EXECUTION_TRAJECTORY_EXECUTOR_HPP

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <moveit_task_constructor_msgs/msg/sub_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace rclcpp
{
class Node;
}

namespace orion_mtc
{
class PlanningSceneManager;
}

namespace orion_mtc
{

class TrajectoryExecutor
{
public:
  explicit TrajectoryExecutor(rclcpp::Node* node);
  ~TrajectoryExecutor() = default;

  bool sendJointTrajectory(const std::string& controller_name,
                           const trajectory_msgs::msg::JointTrajectory& jt);

  /* 先应用 scene_diff（经 scene_manager），再按控制器拆分并发送轨迹 */
  bool executeSubTrajectory(const moveit_task_constructor_msgs::msg::SubTrajectory& sub,
                            PlanningSceneManager* scene_manager);

  /** 取消当前正在执行的 FollowJointTrajectory 目标（急停） */
  void cancelOngoingGoals();

private:
  using FollowJointAction = control_msgs::action::FollowJointTrajectory;
  using FollowJointClient = rclcpp_action::Client<FollowJointAction>;
  using GoalHandle = rclcpp_action::ClientGoalHandle<FollowJointAction>::SharedPtr;

  void registerActiveGoal(const FollowJointClient::SharedPtr& client, const GoalHandle& goal_handle);
  void unregisterActiveGoal(const FollowJointClient::SharedPtr& client, const GoalHandle& goal_handle);

  rclcpp::Node* node_;
  std::unordered_map<std::string, FollowJointClient::SharedPtr> follow_jt_clients_;
  std::mutex follow_jt_mutex_;
  std::mutex active_goals_mutex_;
  std::vector<std::pair<FollowJointClient::SharedPtr, GoalHandle>> active_goals_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_EXECUTION_TRAJECTORY_EXECUTOR_HPP
