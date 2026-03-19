/* MTC pick-and-place 节点：接口层薄壳，业务委托 TaskManager */

#include "orion_mtc/app/orion_mtc_node.hpp"
#include "orion_mtc/config/mtc_config.hpp"
#include "orion_mtc/config/runtime_policy_loader.hpp"
#include "orion_mtc/core/constants.hpp"
#include "orion_mtc/core/held_object.hpp"
#include "orion_mtc/core/manipulation_job.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include "orion_mtc/core/task_state.hpp"
#include "orion_mtc/perception/pose_cache.hpp"
#include "orion_mtc/perception/vector3_cache.hpp"
#include "orion_mtc/planning/place_generator.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/execution/solution_executor.hpp"
#include "orion_mtc/feasibility/feasibility_checker.hpp"
#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/orchestration/task_queue.hpp"
#include "orion_mtc/core/job_result_code.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include <orion_mtc_msgs/msg/job_execution_record.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
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
  /* 接受任意 frame_id，handlePick 内将世界系变换到 base_link */
  object_pose_cache_ = std::make_shared<PoseCache>("");
  object_axis_cache_ = std::make_shared<Vector3Cache>("");
  place_pose_cache_ = std::make_shared<PoseCache>("base_link");
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
  task_manager_->setPlaceGenerator(place_generator_.get());
  /* locked = 夹爪有物，此时禁止 pick 避免 object 在夹爪上导致规划失败 */
  task_manager_->setGripperLockedCallback([this]() { return isGripperLocked(); });
  /* 执行 PICK 时取当前缓存的 latest：object_pose 由订阅持续接收并更新，规划时直接用最新一帧 */
  task_manager_->setGetLatestObjectPoseCallback([this]() { return object_pose_cache_->latest(); });
  task_manager_->setGetLatestObjectAxisCallback([this]() { return object_axis_cache_->latest(); });
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(action_client_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, action_client_node_, false);
  task_manager_->setTransformToBaseLinkCallback(
      [this](geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::Vector3Stamped* axis) {
        try
        {
          rclcpp::Time t(pose.header.stamp.sec, pose.header.stamp.nanosec);
          geometry_msgs::msg::TransformStamped T = tf_buffer_->lookupTransform(
              "base_link", pose.header.frame_id, t);
          geometry_msgs::msg::PoseStamped out;
          tf2::doTransform(pose, out, T);
          pose = out;
          pose.header.frame_id = "base_link";
          if (axis != nullptr)
          {
            geometry_msgs::msg::Vector3Stamped axis_out;
            tf2::doTransform(*axis, axis_out, T);
            axis->vector = axis_out.vector;
            axis->header.frame_id = "base_link";
          }
          return true;
        }
        catch (const std::exception& e)
        {
          RCLCPP_WARN(LOGGER, "transform to base_link failed: %s", e.what());
          return false;
        }
      });
  feasibility_checker_ = std::make_shared<FeasibilityChecker>(node_);
  feasibility_checker_->setMTCConfig(&config_);
}

namespace
{
void nsToTime(int64_t ns, builtin_interfaces::msg::Time& t)
{
  if (ns <= 0)
  {
    t.sec = 0;
    t.nanosec = 0u;
    return;
  }
  const int64_t sec = ns / 1000000000;
  const int64_t nsec = ns % 1000000000;
  t.sec = static_cast<int32_t>(sec);
  t.nanosec = static_cast<uint32_t>(nsec);
}
}  // namespace

void OrionMTCNode::initInterfaces()
{
  const std::string ns(MANIPULATOR_NS);
  sub_object_pose_ = action_client_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/object_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        object_pose_cache_->update(*msg);
      });
  sub_object_axis_ = action_client_node_->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      ns + "/object_axis", 10, [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) {
        object_axis_cache_->update(*msg);
      });
  sub_place_pose_ = action_client_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      ns + "/place_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        place_pose_cache_->update(*msg);
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
  /* 以下服务均返回 success + message（及业务字段），供前端/客户端弹框与日志 */
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
  open_gripper_srv_ = action_client_node_->create_service<std_srvs::srv::Trigger>(
      ns + "/open_gripper",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        handleOpenGripper(req, res);
      });
  close_gripper_srv_ = action_client_node_->create_service<std_srvs::srv::Trigger>(
      ns + "/close_gripper",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        handleCloseGripper(req, res);
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
  check_pick_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::CheckPick>(
      ns + "/check_pick",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Response> res) {
        handleCheckPick(req, res);
      });
  check_place_srv_ = action_client_node_->create_service<orion_mtc_msgs::srv::CheckPlace>(
      ns + "/check_place",
      [this](const std::shared_ptr<orion_mtc_msgs::srv::CheckPlace::Request> req,
             std::shared_ptr<orion_mtc_msgs::srv::CheckPlace::Response> res) {
        handleCheckPlace(req, res);
      });

  setupStatusPublishersAndCallbacks();
}

void OrionMTCNode::setupStatusPublishersAndCallbacks()
{
  const std::string ns(MANIPULATOR_NS);
  pub_runtime_status_ =
      action_client_node_->create_publisher<orion_mtc_msgs::msg::RuntimeStatus>(ns + "/runtime_status", 10);
  pub_job_event_ =
      action_client_node_->create_publisher<orion_mtc_msgs::msg::JobEvent>(ns + "/job_event", 10);
  pub_task_stage_ =
      action_client_node_->create_publisher<orion_mtc_msgs::msg::TaskStage>(ns + "/task_stage", 10);
  pub_held_object_state_ =
      action_client_node_->create_publisher<orion_mtc_msgs::msg::HeldObjectState>(ns + "/held_object_state", 10);
  pub_recovery_event_ =
      action_client_node_->create_publisher<orion_mtc_msgs::msg::RecoveryEvent>(ns + "/recovery_event", 10);

  runtime_status_timer_ = action_client_node_->create_wall_timer(
      std::chrono::milliseconds(500), [this]() { publishRuntimeStatus(); });

  task_manager_->setJobEventCallback([this](const std::string& job_id, const std::string& job_type,
                                            const std::string& source, uint32_t priority,
                                            const std::string& event_type, bool success,
                                            const std::string& reason, int64_t created_at_ns,
                                            int64_t started_at_ns, int64_t finished_at_ns) {
    orion_mtc_msgs::msg::JobEvent msg;
    msg.header.stamp = action_client_node_->now();
    msg.job_id = job_id;
    msg.job_type = job_type;
    msg.source = source;
    msg.priority = priority;
    msg.event_type = event_type;
    msg.success = success;
    msg.reason = reason;
    nsToTime(created_at_ns, msg.created_at);
    nsToTime(started_at_ns, msg.started_at);
    nsToTime(finished_at_ns, msg.finished_at);
    pub_job_event_->publish(msg);
  });

  task_manager_->setHeldObjectStateCallback([this](const HeldObjectContext& ctx) {
    orion_mtc_msgs::msg::HeldObjectState msg;
    msg.header.stamp = action_client_node_->now();
    msg.valid = ctx.valid;
    msg.tracked = isTracked(ctx);
    msg.object_id = ctx.object_id;
    msg.scene_attach_id = ctx.scene_attach_id;
    msg.attach_link = ctx.attach_link;
    msg.object_pose_at_grasp = ctx.object_pose_at_grasp;
    msg.tcp_pose_at_grasp = ctx.tcp_pose_at_grasp;
    msg.weight = static_cast<float>(ctx.weight);
    pub_held_object_state_->publish(msg);
  });

  task_manager_->setRecoveryEventCallback([this](const std::string& recovery_type,
                                                 const std::string& trigger_reason,
                                                 bool success, const std::string& detail) {
    orion_mtc_msgs::msg::RecoveryEvent msg;
    msg.header.stamp = action_client_node_->now();
    msg.recovery_type = recovery_type;
    msg.trigger_reason = trigger_reason;
    msg.success = success;
    msg.detail = detail;
    pub_recovery_event_->publish(msg);
  });

  task_manager_->setStageReportCallback([this](const std::string& job_id, const std::string& task_type,
                                               std::size_t stage_index, const std::string& stage_name,
                                               const std::string& stage_state, const std::string& detail) {
    (void)stage_index;
    orion_mtc_msgs::msg::TaskStage msg;
    msg.header.stamp = action_client_node_->now();
    msg.job_id = job_id;
    msg.task_type = task_type;
    msg.stage_name = stage_name;
    msg.stage_state = stage_state;
    msg.detail = detail;
    pub_task_stage_->publish(msg);
  });
}

void OrionMTCNode::publishRuntimeStatus()
{
  orion_mtc_msgs::msg::RuntimeStatus msg;
  msg.header.stamp = action_client_node_->now();
  msg.header.frame_id = "";
  msg.worker_status = toCString(task_manager_->getWorkerStatus());
  msg.task_mode = toStateString(task_manager_->getMode());
  msg.current_job_id = task_manager_->getCurrentJobId();
  msg.current_job_type = task_manager_->getCurrentJobType();
  msg.next_job_type = task_manager_->getNextJobType();
  msg.worker_running = task_manager_->isWorkerRunning();
  std::shared_ptr<TaskQueue> q = task_manager_->getQueue();
  msg.queue_empty = !q || q->empty();
  msg.queue_size = q ? static_cast<uint32_t>(q->size()) : 0u;
  HeldObjectContext held = task_manager_->getHeldObject();
  msg.has_held_object = held.valid;
  msg.held_object_id = held.valid ? held.object_id : "";
  msg.held_scene_attach_id = held.valid ? held.scene_attach_id : "";
  msg.last_error = task_manager_->getLastError();
  pub_runtime_status_->publish(msg);
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
  /* Pick 仅使用话题 /object_pose：有则入队，无则等待后入队；夹爪 locked（有物）时拒绝 */
  std::thread([this]() {
    if (isGripperLocked())
    {
      RCLCPP_WARN(LOGGER, "topic_pick_trigger: gripper locked (has object), not enqueued (place or open_gripper first)");
      return;
    }
    ManipulationJob job;
    job.type = JobType::PICK;
    job.object_id = "";
    job.source = "topic_pick_trigger";
    std::optional<geometry_msgs::msg::PoseStamped> topic_pose = object_pose_cache_->latest();
    if (!topic_pose.has_value())
    {
      geometry_msgs::msg::PoseStamped pose;
      if (!object_pose_cache_->waitForPose(std::chrono::milliseconds(3000), pose))
      {
        RCLCPP_WARN(LOGGER, "topic_pick_trigger: no object_pose after wait, not enqueued");
        return;
      }
      topic_pose = pose;
    }
    job.object_pose = *topic_pose;
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

bool OrionMTCNode::isGripperLocked() const
{
  const double threshold = 0.5;
  return left_arm_gripped_.load() >= threshold;
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
  if (isGripperLocked())
  {
    RCLCPP_INFO(LOGGER, "Pick goal rejected: gripper locked (has object), place or open_gripper first");
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
  if (req->job_type > 6)
  {
    res->success = false;
    res->message = "invalid job_type (0=PICK,1=PLACE,2=PLACE_RELEASE,3=RESET,4=SYNC,5=OPEN_GRIPPER,6=CLOSE_GRIPPER)";
    return;
  }
  if (req->job_type == static_cast<uint8_t>(JobType::PICK) && isGripperLocked())
  {
    res->success = false;
    res->message = "gripper locked (has object), place or open_gripper first";
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

void OrionMTCNode::handleCheckPick(
    const std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::CheckPick::Response> res)
{
  if (feasibility_checker_)
  {
    feasibility_checker_->checkPick(req, res);
  }
  else
  {
    res->approved = false;
    res->severity = 2;
    res->summary = "审批模块未就绪";
  }
}

void OrionMTCNode::handleCheckPlace(
    const std::shared_ptr<orion_mtc_msgs::srv::CheckPlace::Request> req,
    std::shared_ptr<orion_mtc_msgs::srv::CheckPlace::Response> res)
{
  if (feasibility_checker_)
  {
    feasibility_checker_->checkPlace(req, res);
  }
  else
  {
    res->approved = false;
    res->severity = 2;
    res->summary = "审批模块未就绪";
  }
}

void OrionMTCNode::handleOpenGripper(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  ManipulationJob job;
  job.type = JobType::OPEN_GRIPPER;
  job.source = "open_gripper_srv";
  std::string reject_reason;
  std::string job_id = task_manager_->submitJob(job, &reject_reason);
  res->success = !job_id.empty();
  res->message = res->success ? job_id : ("rejected: " + reject_reason);
}

void OrionMTCNode::handleCloseGripper(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  ManipulationJob job;
  job.type = JobType::CLOSE_GRIPPER;
  job.source = "close_gripper_srv";
  std::string reject_reason;
  std::string job_id = task_manager_->submitJob(job, &reject_reason);
  res->success = !job_id.empty();
  res->message = res->success ? job_id : ("rejected: " + reject_reason);
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
