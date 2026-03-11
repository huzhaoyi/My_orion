/* MTC pick-and-place 节点：接口层薄壳，业务委托 TaskManager */

#include "orion_mtc/app/orion_mtc_node.hpp"
#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/config/runtime_policy_loader.hpp"
#include "orion_mtc/core/constants.hpp"
#include "orion_mtc/core/manipulation_job.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include "orion_mtc/core/task_state.hpp"
#include "orion_mtc/perception/pose_cache.hpp"
#include "orion_mtc/perception/target_cache.hpp"
#include "orion_mtc/perception/target_selector.hpp"
#include "orion_mtc/perception/grasp_generator.hpp"
#include "orion_mtc/planning/place_generator.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/execution/solution_executor.hpp"
#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/orchestration/task_queue.hpp"
#include "orion_mtc/core/job_result_code.hpp"
#include <orion_mtc_msgs/msg/job_execution_record.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc");

namespace orion_mtc
{

OrionMTCNode::OrionMTCNode(const rclcpp::NodeOptions& options)
  : node_(std::make_shared<rclcpp::Node>("orion_mtc_node", options))
  , action_client_node_(std::make_shared<rclcpp::Node>("orion_mtc_action_client", options))
{
  declareParameters(node_.get());
  declareRuntimePolicyParameters(node_);
  loadFromNode(node_.get(), config_);
  loadRuntimePolicyFromNode(node_, runtime_policy_);
  initModules();
  initInterfaces();
}

void OrionMTCNode::initModules()
{
  object_pose_cache_ = std::make_shared<PoseCache>("base_link");
  place_pose_cache_ = std::make_shared<PoseCache>("base_link");
  target_cache_ = std::make_shared<TargetCache>("base_link");
  target_selector_ = std::make_shared<TargetSelector>(TargetSelectorParams());
  grasp_generator_ = std::make_shared<GraspGenerator>(GraspGeneratorParams());
  place_generator_ = std::make_shared<PlaceGenerator>(PlaceGeneratorParams());
  scene_manager_ = std::make_shared<PlanningSceneManager>(action_client_node_.get());
  trajectory_executor_ = std::make_shared<TrajectoryExecutor>(action_client_node_.get());
  solution_executor_ =
      std::make_shared<SolutionExecutor>(scene_manager_.get(), trajectory_executor_.get());

  WaitForGrippedFn wait_fn = [this](bool expect_gripped, double timeout_sec) {
    const double threshold = 0.5;
    const int total_ticks = static_cast<int>(timeout_sec * 20.0);
    for (int i = 0; i < total_ticks; ++i)
    {
      double v = left_arm_gripped_.load();
      if (expect_gripped && v >= threshold)
      {
        RCLCPP_INFO(LOGGER, "waitForGripped: gripped (%.3f >= %.3f)", v, threshold);
        return true;
      }
      if (!expect_gripped && v < threshold)
      {
        RCLCPP_INFO(LOGGER, "waitForGripped: unlocked (%.3f < %.3f)", v, threshold);
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    RCLCPP_WARN(LOGGER, "waitForGripped: timeout (expect_gripped=%d, last=%.3f)", expect_gripped,
                left_arm_gripped_.load());
    return false;
  };

  task_manager_ = std::make_shared<TaskManager>(
      node_, config_, scene_manager_.get(), trajectory_executor_.get(),
      solution_executor_.get(), std::move(wait_fn));
  task_manager_->setPolicy(runtime_policy_);
  task_manager_->setTargetSelection(target_cache_.get(), target_selector_.get(), grasp_generator_.get());
  task_manager_->setPlaceGenerator(place_generator_.get());
}

void OrionMTCNode::initInterfaces()
{
  const std::string ns(MANIPULATOR_NS);
  sub_object_pose_ = action_client_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/object_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        object_pose_cache_->update(*msg);
      });
  sub_place_pose_ = action_client_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/place_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        place_pose_cache_->update(*msg);
      });
  sub_target_set_ = action_client_node_->create_subscription<orion_mtc_msgs::msg::TargetSet>(
      ns + "/target_set", 10, [this](const orion_mtc_msgs::msg::TargetSet::SharedPtr msg) {
        target_cache_->update(*msg);
      });
  sub_pick_trigger_ = action_client_node_->create_subscription<std_msgs::msg::Empty>(
      ns + "/pick_trigger", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
        onPickTriggerReceived(msg);
      });
  sub_place_trigger_ = action_client_node_->create_subscription<std_msgs::msg::Empty>(
      ns + "/place_trigger", 10, [this](const std_msgs::msg::Empty::SharedPtr msg) {
        onPlaceTriggerReceived(msg);
      });
  sub_left_arm_gripped_ = action_client_node_->create_subscription<std_msgs::msg::Float32>(
      ns + "/left_arm_gripped", 10, [this](const std_msgs::msg::Float32::SharedPtr msg) {
        left_arm_gripped_.store(static_cast<double>(msg->data));
      });

  pick_action_server_ = rclcpp_action::create_server<orion_mtc_msgs::action::Pick>(
      action_client_node_, ns + "/pick",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const orion_mtc_msgs::action::Pick::Goal> goal) {
        return handlePickGoalRequest(uuid, goal);
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& h) {
        return handlePickGoalCancel(h);
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& h) {
        handlePickGoalAccepted(h);
      });
  place_action_server_ = rclcpp_action::create_server<orion_mtc_msgs::action::Place>(
      action_client_node_, ns + "/place",
      [this](const rclcpp_action::GoalUUID& uuid,
             std::shared_ptr<const orion_mtc_msgs::action::Place::Goal> goal) {
        return handlePlaceGoalRequest(uuid, goal);
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Place>>& h) {
        return handlePlaceGoalCancel(h);
      },
      [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Place>>& h) {
        handlePlaceGoalAccepted(h);
      });
  place_release_action_server_ =
      rclcpp_action::create_server<orion_mtc_msgs::action::PlaceRelease>(
          action_client_node_, ns + "/place_release",
          [this](const rclcpp_action::GoalUUID& uuid,
                 std::shared_ptr<const orion_mtc_msgs::action::PlaceRelease::Goal> goal) {
            return handlePlaceReleaseGoalRequest(uuid, goal);
          },
          [this](const std::shared_ptr<
                 rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::PlaceRelease>>& h) {
            return handlePlaceReleaseGoalCancel(h);
          },
          [this](const std::shared_ptr<
                 rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::PlaceRelease>>& h) {
            handlePlaceReleaseGoalAccepted(h);
          });
  get_robot_state_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::GetRobotState>(
      ns + "/get_robot_state",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Response> res) {
        handleGetRobotState(req, res);
      });
  get_queue_state_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::GetQueueState>(
      ns + "/get_queue_state",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Response> res) {
        handleGetQueueState(req, res);
      });
  get_recent_jobs_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::GetRecentJobs>(
      ns + "/get_recent_jobs",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Response> res) {
        handleGetRecentJobs(req, res);
      });
  submit_job_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::SubmitJob>(
      ns + "/submit_job",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Response> res) {
        handleSubmitJob(req, res);
      });
  cancel_job_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::CancelJob>(
      ns + "/cancel_job",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Response> res) {
        handleCancelJob(req, res);
      });
  reset_held_object_srv_ =
      action_client_node_->create_service<orion_mtc_msgs::srv::ResetHeldObject>(
          ns + "/reset_held_object",
          [this](const std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Request> req,
                 std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Response> res) {
            handleResetHeldObject(req, res);
          });
  sync_held_object_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::SyncHeldObject>(
      ns + "/sync_held_object",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Response> res) {
        handleSyncHeldObject(req, res);
      });
}

void OrionMTCNode::onObjectPoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  object_pose_cache_->update(*msg);
}

void OrionMTCNode::onPlacePoseReceived(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  place_pose_cache_->update(*msg);
}

void OrionMTCNode::onPickTriggerReceived(const std_msgs::msg::Empty::SharedPtr)
{
  /* 优先多目标：有 target_set 则入队 PICK（无 object_pose），worker 内选目标+候选抓取；否则回退单 pose */
  std::thread([this]() {
    ManipulationJob job;
    job.type = JobType::PICK;
    job.object_id = "";
    job.source = "topic_pick_trigger";
    if (target_cache_->hasTargets())
    {
      job.object_pose = std::nullopt;
      std::string reject_reason;
      std::string job_id = task_manager_->submitJob(job, &reject_reason);
      if (job_id.empty())
      {
        RCLCPP_INFO(LOGGER, "topic_pick_trigger: rejected (%s)", reject_reason.c_str());
      }
      else
      {
        RCLCPP_INFO(LOGGER, "topic_pick_trigger: accepted job_id=%s (multi-target)", job_id.c_str());
      }
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    if (!object_pose_cache_->waitForPose(std::chrono::milliseconds(3000), pose))
    {
      RCLCPP_WARN(LOGGER, "topic_pick_trigger: no target_set and no object pose after wait, not enqueued");
      return;
    }
    job.object_pose = pose;
    std::string reject_reason;
    std::string job_id = task_manager_->submitJob(job, &reject_reason);
    if (job_id.empty())
    {
      RCLCPP_INFO(LOGGER, "topic_pick_trigger: rejected (%s)", reject_reason.c_str());
    }
    else
    {
      RCLCPP_INFO(LOGGER, "topic_pick_trigger: accepted job_id=%s", job_id.c_str());
    }
  }).detach();
}

void OrionMTCNode::onPlaceTriggerReceived(const std_msgs::msg::Empty::SharedPtr)
{
  /* topic 只负责受理：短等待 place_pose，无则回退参数位姿 → 组 job → submitJob */
  std::thread([this]() {
    geometry_msgs::msg::PoseStamped pose;
    if (!place_pose_cache_->waitForPose(std::chrono::milliseconds(3000), pose))
    {
      pose.header.frame_id = "base_link";
      pose.pose.position.x = config_.default_place_x;
      pose.pose.position.y = config_.default_place_y;
      pose.pose.position.z = config_.default_place_z;
      pose.pose.orientation.x = config_.default_place_qx;
      pose.pose.orientation.y = config_.default_place_qy;
      pose.pose.orientation.z = config_.default_place_qz;
      pose.pose.orientation.w = config_.default_place_qw;
    }
    ManipulationJob job;
    job.type = JobType::PLACE;
    job.target_pose = pose;
    job.source = "topic_place_trigger";
    std::string reject_reason;
    std::string job_id = task_manager_->submitJob(job, &reject_reason);
    if (job_id.empty())
    {
      RCLCPP_INFO(LOGGER, "topic_place_trigger: rejected (%s)", reject_reason.c_str());
    }
    else
    {
      RCLCPP_INFO(LOGGER, "topic_place_trigger: accepted job_id=%s", job_id.c_str());
    }
  }).detach();
}

void OrionMTCNode::onLeftArmGrippedReceived(const std_msgs::msg::Float32::SharedPtr msg)
{
  left_arm_gripped_.store(static_cast<double>(msg->data));
}

rclcpp_action::GoalResponse OrionMTCNode::handlePickGoalRequest(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const orion_mtc_msgs::action::Pick::Goal>)
{
  RobotTaskMode mode = task_manager_->getMode();
  if (isHolding(mode))
  {
    RCLCPP_INFO(LOGGER, "Pick goal rejected: already holding (place or reset_held_object first)");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!canAcceptPick(mode))
  {
    RCLCPP_INFO(LOGGER, "Pick goal rejected: busy");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OrionMTCNode::handlePickGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>&)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void OrionMTCNode::handlePickGoalAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Pick>>& goal_handle)
{
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    bool ok = task_manager_->handlePick(goal->object_pose,
                                        goal->object_id.empty() ? "object" : goal->object_id);
    auto result = std::make_shared<orion_mtc_msgs::action::Pick::Result>();
    result->success = ok;
    result->task_id = task_manager_->getTaskId();
    result->held_object_id = ok ? task_manager_->getHeldObject().object_id : "";
    result->message = ok ? "pick success" : task_manager_->getLastError();
    if (ok)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }).detach();
}

rclcpp_action::GoalResponse OrionMTCNode::handlePlaceGoalRequest(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const orion_mtc_msgs::action::Place::Goal>)
{
  if (task_manager_->getMode() != RobotTaskMode::HOLDING_TRACKED || !task_manager_->getHeldObject().valid)
  {
    RCLCPP_INFO(LOGGER, "Place goal rejected: precise place requires HOLDING_TRACKED with held_object");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OrionMTCNode::handlePlaceGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Place>>&)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void OrionMTCNode::handlePlaceGoalAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::Place>>& goal_handle)
{
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    bool ok = task_manager_->handlePlace(goal->target_pose);
    auto result = std::make_shared<orion_mtc_msgs::action::Place::Result>();
    result->success = ok;
    result->task_id = task_manager_->getTaskId();
    result->message = ok ? "place success" : task_manager_->getLastError();
    if (ok)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }).detach();
}

rclcpp_action::GoalResponse OrionMTCNode::handlePlaceReleaseGoalRequest(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const orion_mtc_msgs::action::PlaceRelease::Goal>)
{
  RobotTaskMode mode = task_manager_->getMode();
  if (mode != RobotTaskMode::HOLDING_TRACKED && mode != RobotTaskMode::HOLDING_UNTRACKED)
  {
    RCLCPP_INFO(LOGGER, "PlaceRelease goal rejected: not holding (mode=%d)", static_cast<int>(mode));
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse OrionMTCNode::handlePlaceReleaseGoalCancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::PlaceRelease>>&)
{
  return rclcpp_action::CancelResponse::REJECT;
}

void OrionMTCNode::handlePlaceReleaseGoalAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<orion_mtc_msgs::action::PlaceRelease>>& goal_handle)
{
  std::thread([this, goal_handle]() {
    const auto goal = goal_handle->get_goal();
    bool ok = task_manager_->handlePlaceRelease(goal->target_tcp_pose);
    auto result = std::make_shared<orion_mtc_msgs::action::PlaceRelease::Result>();
    result->success = ok;
    result->task_id = task_manager_->getTaskId();
    result->message = ok ? "place_release success" : task_manager_->getLastError();
    if (ok)
      goal_handle->succeed(result);
    else
      goal_handle->abort(result);
  }).detach();
}

void OrionMTCNode::handleGetRobotState(
    const std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Request>,
    std::shared_ptr<orion_mtc_msgs::srv::GetRobotState::Response> res)
{
  res->mode = toStateString(task_manager_->getMode());
  res->task_id = task_manager_->getTaskId();
  res->held_object_id = task_manager_->getHeldObject().valid ? task_manager_->getHeldObject().object_id : "";
  res->has_held_object = task_manager_->getHeldObject().valid;
  res->last_error = task_manager_->getLastError();
}

void OrionMTCNode::handleGetQueueState(
    const std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Request>,
    std::shared_ptr<orion_mtc_msgs::srv::GetQueueState::Response> res)
{
  std::shared_ptr<TaskQueue> q = task_manager_->getQueue();
  res->queue_size = q ? static_cast<uint32_t>(q->size()) : 0u;
  res->current_job_id = task_manager_->getCurrentJobId();
  res->current_job_type = task_manager_->getCurrentJobType();
  res->next_job_type = "NONE";
  res->next_job_id = "";
  res->next_job_priority = -1;
  if (q)
  {
    ManipulationJob next;
    if (q->peekFront(next))
    {
      res->next_job_type = jobTypeToCString(next.type);
      res->next_job_id = next.job_id;
      res->next_job_priority = static_cast<int32_t>(next.priority);
    }
  }
  WorkerStatus ws = task_manager_->getWorkerStatus();
  res->worker_status = static_cast<uint8_t>(ws);
  res->task_mode = toStateString(task_manager_->getMode());
  res->last_error = task_manager_->getLastError();
  res->worker_running = task_manager_->isWorkerRunning();
  res->queue_empty = !q || q->empty();
}

void OrionMTCNode::handleGetRecentJobs(
    const std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::GetRecentJobs::Response> res)
{
  const std::uint32_t max_count = req->max_count > 0u ? req->max_count : 50u;
  std::vector<TaskManager::JobExecutionRecordEntry> entries = task_manager_->getRecentRecords(max_count);
  res->records.clear();
  res->records.reserve(entries.size());
  for (const auto& e : entries)
  {
    orion_mtc_msgs::msg::JobExecutionRecord msg;
    msg.job_id = e.job_id;
    msg.job_type = e.job_type;
    msg.source = e.source;
    msg.result_code = static_cast<uint8_t>(e.result_code);
    msg.message = e.message;
    msg.created_at_ns = e.created_at_ns;
    msg.started_at_ns = e.started_at_ns;
    msg.finished_at_ns = e.finished_at_ns;
    res->records.push_back(msg);
  }
}

void OrionMTCNode::handleSubmitJob(
    const std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::SubmitJob::Response> res)
{
  if (req->job_type > 4)
  {
    res->success = false;
    res->message = "invalid job_type (0=PICK,1=PLACE,2=PLACE_RELEASE,3=RESET_HELD_OBJECT,4=SYNC_HELD_OBJECT)";
    return;
  }
  ManipulationJob job;
  job.job_id = req->job_id;
  job.type = static_cast<JobType>(req->job_type);
  job.object_id = req->object_id;
  job.tracked = req->tracked;
  job.priority = req->priority;
  job.target_pose = req->target_pose;
  job.object_pose = req->object_pose;
  job.tcp_pose = req->tcp_pose;
  job.source = "submit_job_service";
  std::string reject_reason;
  std::string assigned_id = task_manager_->submitJob(job, &reject_reason);
  if (assigned_id.empty())
  {
    res->success = false;
    res->message = reject_reason.empty() ? "policy rejected" : reject_reason;
    return;
  }
  res->success = true;
  res->message = "queued";
  res->job_id = assigned_id;
}

void OrionMTCNode::handleCancelJob(
    const std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::CancelJob::Response> res)
{
  std::string message;
  bool ok = task_manager_->cancelJob(req->job_id, &message);
  res->success = ok;
  res->message = message;
}

void OrionMTCNode::handleResetHeldObject(
    const std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Request>,
    std::shared_ptr<orion_mtc_msgs::srv::ResetHeldObject::Response> res)
{
  res->success = task_manager_->handleResetHeldObject(res->message);
}

void OrionMTCNode::handleSyncHeldObject(
    const std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::SyncHeldObject::Response> res)
{
  res->success = task_manager_->handleSyncHeldObject(
      req->set_holding, req->tracked, req->object_id, req->object_pose, req->tcp_pose, res->message);
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr OrionMTCNode::getNodeBaseInterface()
{
  return action_client_node_->get_node_base_interface();
}

void OrionMTCNode::setupPlanningScene()
{
  RCLCPP_INFO(LOGGER, "setupPlanningScene: object added in-task (add object stage), no /collision_object publish");
  if (task_manager_->getPolicy().auto_start_worker)
  {
    task_manager_->startWorker();
  }
}

}  // namespace orion_mtc
