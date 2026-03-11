/* MTC pick-and-place 节点：接口层薄壳，仅负责 ROS 接口与回调，业务委托 TaskManager */

#ifndef ORION_MTC_APP_ORION_MTC_NODE_HPP
#define ORION_MTC_APP_ORION_MTC_NODE_HPP

#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/config/runtime_policy.hpp"
#include "orion_mtc/core/constants.hpp"
#include <orion_mtc_msgs/action/pick.hpp>
#include <orion_mtc_msgs/action/place.hpp>
#include <orion_mtc_msgs/action/place_release.hpp>
#include <orion_mtc_msgs/srv/get_robot_state.hpp>
#include <orion_mtc_msgs/srv/get_queue_state.hpp>
#include <orion_mtc_msgs/srv/get_recent_jobs.hpp>
#include <orion_mtc_msgs/srv/cancel_job.hpp>
#include <orion_mtc_msgs/srv/reset_held_object.hpp>
#include <orion_mtc_msgs/srv/submit_job.hpp>
#include <orion_mtc_msgs/srv/sync_held_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <atomic>
#include <memory>
#include <string>

namespace orion_mtc
{
class PoseCache;
class PlanningSceneManager;
class TrajectoryExecutor;
class SolutionExecutor;
class TaskManager;
}

namespace orion_mtc
{

class OrionMTCNode
{
public:
  explicit OrionMTCNode(const rclcpp::NodeOptions& options);
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();
  void setupPlanningScene();

private:
  void initModules();
  void initInterfaces();

  void onObjectPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPlacePoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onPickTriggerReceived(const std_msgs::msg::Empty::SharedPtr msg);
  void onPlaceTriggerReceived(const std_msgs::msg::Empty::SharedPtr msg);
  void onLeftArmGrippedReceived(const std_msgs::msg::Float32::SharedPtr msg);

  rclcpp_action::GoalResponse handlePickGoalRequest(const rclcpp_action::GoalUUID& uuid,
                                                    std::shared_ptr<const orion_mtc_msgs::action::Pick::Goal> goal);
  rclcpp_action::CancelResponse handlePickGoalCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& goal_handle);
  void handlePickGoalAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& goal_handle);

  rclcpp_action::GoalResponse handlePlaceGoalRequest(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const orion_mtc_msgs::action::Place::Goal> goal);
  rclcpp_action::CancelResponse handlePlaceGoalCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Place>>& goal_handle);
  void handlePlaceGoalAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Place>>& goal_handle);

  rclcpp_action::GoalResponse handlePlaceReleaseGoalRequest(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const orion_mtc_msgs::action::PlaceRelease::Goal> goal);
  rclcpp_action::CancelResponse handlePlaceReleaseGoalCancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::PlaceRelease>>& goal_handle);
  void handlePlaceReleaseGoalAccepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::PlaceRelease>>& goal_handle);

  void handleGetRobotState(const std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Request> req,
                            std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Response> res);
  void handleGetQueueState(const std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Request> req,
                           std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Response> res);
  void handleGetRecentJobs(const std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Request> req,
                           std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Response> res);
  void handleResetHeldObject(const std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Request> req,
                             std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Response> res);
  void handleSubmitJob(const std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Request> req,
                       std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Response> res);
  void handleCancelJob(const std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Request> req,
                      std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Response> res);
  void handleSyncHeldObject(const std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Request> req,
                            std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Response> res);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr action_client_node_;
  MTCConfig config_;
  RuntimePolicy runtime_policy_;
  std::shared_ptr<PoseCache> object_pose_cache_;
  std::shared_ptr<PoseCache> place_pose_cache_;
  std::shared_ptr<PlanningSceneManager> scene_manager_;
  std::shared_ptr<TrajectoryExecutor> trajectory_executor_;
  std::shared_ptr<SolutionExecutor> solution_executor_;
  std::shared_ptr<TaskManager> task_manager_;

  std::atomic<double> left_arm_gripped_{ 0.0 };
  std::atomic<bool> do_task_running_{ false };

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_object_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_place_pose_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_pick_trigger_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_place_trigger_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_arm_gripped_;

  rclcpp_action::Server<orion_mtc_msgs::action::Pick>::SharedPtr pick_action_server_;
  rclcpp_action::Server<orion_mtc_msgs::action::Place>::SharedPtr place_action_server_;
  rclcpp_action::Server<orion_mtc_msgs::action::PlaceRelease>::SharedPtr place_release_action_server_;
  rclcpp::Service<orion_mtc_msgs::srv::GetRobotState>::SharedPtr get_robot_state_srv_;
  rclcpp::Service<orion_mtc_msgs::srv::GetQueueState>::SharedPtr get_queue_state_srv_;
  rclcpp::Service<orion_mtc_msgs::srv::GetRecentJobs>::SharedPtr get_recent_jobs_srv_;
  rclcpp::Service<orion_mtc_msgs::srv::CancelJob>::SharedPtr cancel_job_srv_;
  rclcpp::Service<orion_mtc_msgs::srv::ResetHeldObject>::SharedPtr reset_held_object_srv_;
  rclcpp::Service<orion_mtc_msgs::srv::SubmitJob>::SharedPtr submit_job_srv_;
  rclcpp::Service<orion_mtc_msgs::srv::SyncHeldObject>::SharedPtr sync_held_object_srv_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_APP_ORION_MTC_NODE_HPP
