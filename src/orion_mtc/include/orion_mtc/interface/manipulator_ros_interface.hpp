/* ROS 接口层：订阅/服务/Action 与 app 层装配分离，话题名保持 manipulator 命名空间下不变 */

#ifndef ORION_MTC_INTERFACE_MANIPULATOR_ROS_INTERFACE_HPP
#define ORION_MTC_INTERFACE_MANIPULATOR_ROS_INTERFACE_HPP

#include "orion_mtc/core/held_object.hpp"
#include "orion_mtc/decision/feasibility_checker.hpp"
#include "orion_mtc/decision/target_selector.hpp"
#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/orchestration/task_queue.hpp"
#include "orion_mtc/perception/perception_snapshot_provider.hpp"
#include "orion_mtc/perception/pose_cache.hpp"
#include "orion_mtc/perception/target_cache.hpp"
#include "orion_mtc/perception/vector3_cache.hpp"
#include <orion_mtc_msgs/action/pick.hpp>
#include <orion_mtc_msgs/srv/get_robot_state.hpp>
#include <orion_mtc_msgs/srv/get_queue_state.hpp>
#include <orion_mtc_msgs/srv/get_recent_jobs.hpp>
#include <orion_mtc_msgs/srv/cancel_job.hpp>
#include <orion_mtc_msgs/srv/reset_held_object.hpp>
#include <orion_mtc_msgs/srv/submit_job.hpp>
#include <orion_mtc_msgs/srv/sync_held_object.hpp>
#include <orion_mtc_msgs/srv/check_pick.hpp>
#include <orion_mtc_msgs/msg/runtime_status.hpp>
#include <orion_mtc_msgs/msg/job_event.hpp>
#include <orion_mtc_msgs/msg/task_stage.hpp>
#include <orion_mtc_msgs/msg/held_object_state.hpp>
#include <orion_mtc_msgs/msg/recovery_event.hpp>
#include <orion_mtc_msgs/msg/target_set.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <atomic>
#include <memory>
#include <string>

namespace orion_mtc
{

struct ManipulatorInterfaceContext
{
    rclcpp::Logger logger;
    rclcpp::Node::SharedPtr action_client_node;
    std::shared_ptr<TaskManager> task_manager;
    std::shared_ptr<FeasibilityChecker> feasibility_checker;
    std::shared_ptr<PoseCache> object_pose_cache;
    std::shared_ptr<TargetCache> target_cache;
    std::shared_ptr<Vector3Cache> object_axis_cache;
    std::shared_ptr<PerceptionSnapshotProvider> perception_provider;
    std::shared_ptr<TargetSelector> target_selector;
    std::atomic<double>* left_arm_gripped;
};

class ManipulatorRosInterface
{
public:
    explicit ManipulatorRosInterface(ManipulatorInterfaceContext ctx);

    void registerSubscriptionsAndServices();
    void registerStatusPublishersAndCallbacks();

private:
    void publishRuntimeStatus();
    void onPickTriggerReceived(const std_msgs::msg::Empty::SharedPtr msg);

    bool isGripperLocked() const;

    rclcpp_action::GoalResponse handlePickGoalRequest(const rclcpp_action::GoalUUID& uuid,
                                                      std::shared_ptr<const orion_mtc_msgs::action::Pick::Goal> goal);
    rclcpp_action::CancelResponse handlePickGoalCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& goal_handle);
    void handlePickGoalAccepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& goal_handle);

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
    void handleCheckPick(const std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Request> req,
                         std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Response> res);
    void handleOpenGripper(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                           std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleCloseGripper(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleEmergencyStopService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                    std::shared_ptr<std_srvs::srv::Trigger::Response> res);
    void handleGoToReadyService(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> res);

    ManipulatorInterfaceContext ctx_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_object_pose_;
    rclcpp::Subscription<orion_mtc_msgs::msg::TargetSet>::SharedPtr sub_target_set_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr sub_object_axis_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr sub_pick_trigger_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_arm_gripped_;

    rclcpp_action::Server<orion_mtc_msgs::action::Pick>::SharedPtr pick_action_server_;
    rclcpp::Service<orion_mtc_msgs::srv::GetRobotState>::SharedPtr get_robot_state_srv_;
    rclcpp::Service<orion_mtc_msgs::srv::GetQueueState>::SharedPtr get_queue_state_srv_;
    rclcpp::Service<orion_mtc_msgs::srv::GetRecentJobs>::SharedPtr get_recent_jobs_srv_;
    rclcpp::Service<orion_mtc_msgs::srv::CancelJob>::SharedPtr cancel_job_srv_;
    rclcpp::Service<orion_mtc_msgs::srv::ResetHeldObject>::SharedPtr reset_held_object_srv_;
    rclcpp::Service<orion_mtc_msgs::srv::SubmitJob>::SharedPtr submit_job_srv_;
    rclcpp::Service<orion_mtc_msgs::srv::SyncHeldObject>::SharedPtr sync_held_object_srv_;
    rclcpp::Service<orion_mtc_msgs::srv::CheckPick>::SharedPtr check_pick_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_gripper_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_gripper_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr go_to_ready_srv_;

    rclcpp::Publisher<orion_mtc_msgs::msg::RuntimeStatus>::SharedPtr pub_runtime_status_;
    rclcpp::Publisher<orion_mtc_msgs::msg::JobEvent>::SharedPtr pub_job_event_;
    rclcpp::Publisher<orion_mtc_msgs::msg::TaskStage>::SharedPtr pub_task_stage_;
    rclcpp::Publisher<orion_mtc_msgs::msg::HeldObjectState>::SharedPtr pub_held_object_state_;
    rclcpp::Publisher<orion_mtc_msgs::msg::RecoveryEvent>::SharedPtr pub_recovery_event_;
    rclcpp::TimerBase::SharedPtr runtime_status_timer_;
};

}  // namespace orion_mtc

#endif  // ORION_MTC_INTERFACE_MANIPULATOR_ROS_INTERFACE_HPP
