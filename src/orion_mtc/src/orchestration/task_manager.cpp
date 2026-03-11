#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/orchestration/task_queue.hpp"
#include "orion_mtc/orchestration/recovery_actions.hpp"
#include "orion_mtc/planning/pick_task_builder.hpp"
#include "orion_mtc/planning/place_task_builder.hpp"
#include "orion_mtc/planning/place_release_task_builder.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/execution/solution_executor.hpp"
#include "orion_mtc/core/job_result_code.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.orchestration");

/* 去重阈值：位置 1cm，姿态约 5°（quat dot ≥ 0.99），时间窗口 1s */
static constexpr double DEDUP_POS_TOL_M = 0.01;
static constexpr double DEDUP_QUAT_DOT_MIN = 0.99;
static constexpr int64_t DEDUP_TIME_WINDOW_NS = 1000000000;

bool TaskManager::getJobPoseForDedup(const ManipulationJob& job, geometry_msgs::msg::Pose* out)
{
  if (!out)
  {
    return false;
  }
  switch (job.type)
  {
    case JobType::PICK:
      if (job.object_pose.has_value())
      {
        *out = job.object_pose->pose;
        return true;
      }
      return false;
    case JobType::PLACE:
    case JobType::PLACE_RELEASE:
      if (job.target_pose.has_value())
      {
        *out = job.target_pose->pose;
        return true;
      }
      return false;
    case JobType::SYNC_HELD_OBJECT:
      if (job.object_pose.has_value())
      {
        *out = job.object_pose->pose;
        return true;
      }
      return false;
    case JobType::RESET_HELD_OBJECT:
      return false;
    default:
      return false;
  }
}

bool TaskManager::posesNear(const geometry_msgs::msg::Pose& a, const geometry_msgs::msg::Pose& b,
                            double pos_tol_m, double quat_dot_min)
{
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  double dz = a.position.z - b.position.z;
  double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (dist > pos_tol_m)
  {
    return false;
  }
  double dot = a.orientation.w * b.orientation.w + a.orientation.x * b.orientation.x +
               a.orientation.y * b.orientation.y + a.orientation.z * b.orientation.z;
  if (dot < 0.0)
  {
    dot = -dot;
  }
  return dot >= quat_dot_min;
}

bool TaskManager::isDuplicateJob(const ManipulationJob& job, int64_t now_ns, std::string* out_reason) const
{
  if (out_reason)
  {
    out_reason->clear();
  }
  const char* type_str = jobTypeToCString(job.type);
  geometry_msgs::msg::Pose job_pose;
  bool job_has_pose = getJobPoseForDedup(job, &job_pose);

  /* 当前正在执行同类型同目标：拒绝 */
  if (worker_status_ == WorkerStatus::RUNNING_JOB && current_job_type_ == type_str)
  {
    if (job_has_pose && current_job_has_pose_ &&
        posesNear(current_job_target_pose_, job_pose, DEDUP_POS_TOL_M, DEDUP_QUAT_DOT_MIN))
    {
      if (out_reason)
      {
        *out_reason = "duplicate of running job (same type and target)";
      }
      return true;
    }
    if (!job_has_pose && !current_job_has_pose_)
    {
      if (out_reason)
      {
        *out_reason = "duplicate of running job (same type, no target)";
      }
      return true;
    }
  }

  /* 短时间窗口内同类型同目标：拒绝 */
  if ((now_ns - last_accepted_time_ns_) < DEDUP_TIME_WINDOW_NS && last_accepted_type_ == job.type)
  {
    if (job_has_pose && last_accepted_has_pose_ &&
        posesNear(last_accepted_pose_, job_pose, DEDUP_POS_TOL_M, DEDUP_QUAT_DOT_MIN))
    {
      if (out_reason)
      {
        *out_reason = "duplicate within time window (same type and target)";
      }
      return true;
    }
    if (!job_has_pose && !last_accepted_has_pose_)
    {
      if (out_reason)
      {
        *out_reason = "duplicate within time window (same type, no target)";
      }
      return true;
    }
  }

  return false;
}

TaskManager::TaskManager(const rclcpp::Node::SharedPtr& node,
                         const MTCConfig& config,
                         PlanningSceneManager* scene_manager,
                         TrajectoryExecutor* trajectory_executor,
                         SolutionExecutor* solution_executor,
                         WaitForGrippedFn wait_for_gripped_fn)
  : node_(node)
  , config_(config)
  , scene_manager_(scene_manager)
  , trajectory_executor_(trajectory_executor)
  , solution_executor_(solution_executor)
  , wait_for_gripped_fn_(std::move(wait_for_gripped_fn))
  , pick_builder_(std::make_unique<PickTaskBuilder>(node, config))
  , place_builder_(std::make_unique<PlaceTaskBuilder>(node, config))
  , place_release_builder_(std::make_unique<PlaceReleaseTaskBuilder>(node, config))
  , queue_(std::make_shared<TaskQueue>())
  , recovery_actions_(std::make_unique<RecoveryActions>(scene_manager, this))
{
}

TaskManager::~TaskManager()
{
  stopWorker();
}

void TaskManager::setState(RobotTaskMode mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  task_mode_ = mode;
}

void TaskManager::setStateError(const std::string& err)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  task_mode_ = RobotTaskMode::ERROR;
  last_error_ = err;
  RCLCPP_ERROR(LOGGER, "state ERROR: %s", err.c_str());
}

std::string TaskManager::genTaskId(const char* prefix)
{
  auto now = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  std::ostringstream oss;
  oss << prefix << "_" << ns;
  return oss.str();
}

bool TaskManager::handlePick(const geometry_msgs::msg::PoseStamped& object_pose,
                             const std::string& object_id)
{
  RobotTaskMode mode;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mode = task_mode_;
  }
  if (isHolding(mode))
  {
    RCLCPP_ERROR(LOGGER, "handlePick: already holding, reject (place or reset_held_object first)");
    return false;
  }
  if (!canAcceptPick(mode))
  {
    RCLCPP_ERROR(LOGGER, "handlePick: busy (mode not IDLE/ERROR), reject");
    return false;
  }

  setState(RobotTaskMode::PICKING);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_task_id_ = genTaskId("pick");
  }

  double obj_x = object_pose.pose.position.x;
  double obj_y = object_pose.pose.position.y;
  double obj_z = object_pose.pose.position.z;
  auto object_orientation = object_pose.pose.orientation;
  if (object_pose.header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "handlePick: frame_id '%s', expected base_link", object_pose.header.frame_id.c_str());
  }

  mtc::Task task = pick_builder_->build(obj_x, obj_y, obj_z, object_orientation, object_id);
  try
  {
    task.init();
    task.enableIntrospection(true);
    task.introspection().publishTaskDescription();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    setStateError(std::string("pick init: ") + e.what());
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Pick planning failed (code " << plan_result.val << ")");
    task.explainFailure(std::cout);
    setStateError("pick plan failed");
    return false;
  }
  if (task.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "Pick plan returned no solutions");
    setStateError("pick no solutions");
    return false;
  }

  geometry_msgs::msg::Pose object_pose_at_grasp;
  object_pose_at_grasp.position.x = obj_x;
  object_pose_at_grasp.position.y = obj_y;
  object_pose_at_grasp.position.z = obj_z;
  object_pose_at_grasp.orientation = object_orientation;

  moveit_task_constructor_msgs::msg::Solution solution_msg;
  task.solutions().front()->toMsg(solution_msg, &task.introspection());
  HeldObjectContext new_held;
  if (!solution_executor_->executePickSolution(
          solution_msg, object_pose_at_grasp,
          object_id.empty() ? "object" : object_id,
          task.getRobotModel(), new_held, wait_for_gripped_fn_))
  {
    RCLCPP_ERROR(LOGGER, "Pick execution failed");
    setStateError("pick execution failed");
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_ = new_held;
  }
  setState(RobotTaskMode::HOLDING_TRACKED);
  RCLCPP_INFO(LOGGER, "Pick finished successfully, state=HOLDING_TRACKED");
  return true;
}

bool TaskManager::handlePlace(const geometry_msgs::msg::PoseStamped& target_pose)
{
  HeldObjectContext held;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (task_mode_ != RobotTaskMode::HOLDING_TRACKED || !held_object_.valid)
    {
      RCLCPP_ERROR(LOGGER,
                   "handlePlace: precise place requires HOLDING_TRACKED and held_object (mode=%d, held_valid=%d), reject",
                   static_cast<int>(task_mode_), held_object_.valid ? 1 : 0);
      return false;
    }
    held = held_object_;
  }

  setState(RobotTaskMode::PLACING);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_task_id_ = genTaskId("place");
  }

  double px = target_pose.pose.position.x;
  double py = target_pose.pose.position.y;
  double pz = target_pose.pose.position.z;
  double qx = target_pose.pose.orientation.x;
  double qy = target_pose.pose.orientation.y;
  double qz = target_pose.pose.orientation.z;
  double qw = target_pose.pose.orientation.w;
  if (target_pose.header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "handlePlace: frame_id '%s', expected base_link", target_pose.header.frame_id.c_str());
  }

  mtc::Task task = place_builder_->build(px, py, pz, qx, qy, qz, qw, held);
  if (task.stages()->numChildren() == 0)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = "buildPlaceTask: no held context";
    setState(RobotTaskMode::HOLDING_TRACKED);
    return false;
  }

  try
  {
    task.init();
    task.enableIntrospection(true);
    task.introspection().publishTaskDescription();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = std::string("place init: ") + e.what();
    }
    setState(RobotTaskMode::HOLDING_TRACKED);
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Place planning failed (code " << plan_result.val << ")");
    task.explainFailure(std::cout);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "place plan failed";
    }
    setState(RobotTaskMode::HOLDING_TRACKED);
    return false;
  }
  if (task.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "Place plan returned no solutions");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "place no solutions";
    }
    setState(RobotTaskMode::HOLDING_TRACKED);
    return false;
  }

  moveit_task_constructor_msgs::msg::Solution place_solution_msg;
  task.solutions().front()->toMsg(place_solution_msg, &task.introspection());
  task.introspection().publishSolution(*task.solutions().front());
  if (!solution_executor_->executeSolution(place_solution_msg, wait_for_gripped_fn_))
  {
    RCLCPP_ERROR(LOGGER, "Place execution failed");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "place execution failed";
    }
    setState(RobotTaskMode::HOLDING_TRACKED);
    return false;
  }

  if (scene_manager_)
  {
    scene_manager_->applyObjectPoseToPlanningScene(px, py, pz, qx, qy, qz, qw);
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_.valid = false;
  }
  setState(RobotTaskMode::IDLE);
  RCLCPP_INFO(LOGGER, "Place finished successfully, state=IDLE");
  return true;
}

bool TaskManager::handlePlaceRelease(const geometry_msgs::msg::PoseStamped& target_tcp_pose)
{
  RobotTaskMode prev_mode;
  std::string attached_id;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (task_mode_ != RobotTaskMode::HOLDING_TRACKED && task_mode_ != RobotTaskMode::HOLDING_UNTRACKED)
    {
      RCLCPP_ERROR(LOGGER, "handlePlaceRelease: require HOLDING_TRACKED or HOLDING_UNTRACKED (mode=%d), reject",
                   static_cast<int>(task_mode_));
      return false;
    }
    prev_mode = task_mode_;
    attached_id = (prev_mode == RobotTaskMode::HOLDING_TRACKED)
                      ? (held_object_.scene_attach_id.empty() ? "object" : held_object_.scene_attach_id)
                      : "held_unknown";
  }

  setState(RobotTaskMode::PLACING);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_task_id_ = genTaskId("place_release");
  }

  if (target_tcp_pose.header.frame_id != "base_link" && !target_tcp_pose.header.frame_id.empty())
  {
    RCLCPP_WARN(LOGGER, "handlePlaceRelease: frame_id '%s', expected base_link",
                target_tcp_pose.header.frame_id.c_str());
  }

  mtc::Task task = place_release_builder_->build(target_tcp_pose, attached_id);
  try
  {
    task.init();
    task.enableIntrospection(true);
    task.introspection().publishTaskDescription();
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = std::string("place_release init: ") + e.what();
    }
    setState(prev_mode);
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "PlaceRelease planning failed (code " << plan_result.val << ")");
    task.explainFailure(std::cout);
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "place_release plan failed";
    }
    setState(prev_mode);
    return false;
  }
  if (task.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "PlaceRelease plan returned no solutions");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "place_release no solutions";
    }
    setState(prev_mode);
    return false;
  }

  moveit_task_constructor_msgs::msg::Solution release_solution_msg;
  task.solutions().front()->toMsg(release_solution_msg, &task.introspection());
  task.introspection().publishSolution(*task.solutions().front());
  if (!solution_executor_->executeSolution(release_solution_msg, wait_for_gripped_fn_))
  {
    RCLCPP_ERROR(LOGGER, "PlaceRelease execution failed");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "place_release execution failed";
    }
    setState(prev_mode);
    return false;
  }

  if (attached_id == "held_unknown" && scene_manager_)
  {
    scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_.valid = false;
  }
  setState(RobotTaskMode::IDLE);
  RCLCPP_INFO(LOGGER, "PlaceRelease finished successfully, state=IDLE");
  return true;
}

bool TaskManager::handleSyncHeldObject(bool set_holding, bool tracked,
                                       const std::string& object_id,
                                       const geometry_msgs::msg::Pose& object_pose,
                                       const geometry_msgs::msg::Pose& tcp_pose,
                                       std::string& out_message)
{
  bool need_attach_held_unknown = false;
  bool need_attach_tracked = false;
  Eigen::Isometry3d tcp_to_object_for_attach = Eigen::Isometry3d::Identity();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (task_mode_ != RobotTaskMode::IDLE && task_mode_ != RobotTaskMode::ERROR)
    {
      out_message = "sync_held_object: only when IDLE or ERROR";
      RCLCPP_WARN(LOGGER, "%s", out_message.c_str());
      return false;
    }
    if (!set_holding)
    {
      out_message = "set_holding=false, no change";
      return true;
    }
    if (tracked)
    {
      Eigen::Isometry3d T_base_tcp = Eigen::Isometry3d::Identity();
      T_base_tcp.translate(Eigen::Vector3d(tcp_pose.position.x, tcp_pose.position.y, tcp_pose.position.z));
      T_base_tcp.rotate(Eigen::Quaterniond(tcp_pose.orientation.w, tcp_pose.orientation.x,
                                           tcp_pose.orientation.y, tcp_pose.orientation.z));
      Eigen::Isometry3d T_base_obj = Eigen::Isometry3d::Identity();
      T_base_obj.translate(Eigen::Vector3d(object_pose.position.x, object_pose.position.y,
                                           object_pose.position.z));
      T_base_obj.rotate(Eigen::Quaterniond(object_pose.orientation.w, object_pose.orientation.x,
                                           object_pose.orientation.y, object_pose.orientation.z));
      Eigen::Isometry3d tcp_to_obj = T_base_tcp.inverse() * T_base_obj;
      held_object_.valid = true;
      held_object_.object_id = object_id.empty() ? "object" : object_id;
      held_object_.scene_attach_id = "held_tracked";
      held_object_.attach_link = "Link6";
      held_object_.object_pose_at_grasp = object_pose;
      held_object_.tcp_pose_at_grasp = tcp_pose;
      held_object_.tcp_to_object = tcp_to_obj;
      task_mode_ = RobotTaskMode::HOLDING_TRACKED;
      out_message = "HOLDING_TRACKED (synced with object_pose + tcp_pose)";
      RCLCPP_INFO(LOGGER, "sync_held_object: %s", out_message.c_str());
      need_attach_tracked = true;
      tcp_to_object_for_attach = tcp_to_obj;
    }
    else
    {
      held_object_.valid = true;
      held_object_.object_id = object_id.empty() ? "unknown" : object_id;
      held_object_.scene_attach_id = "held_unknown";
      held_object_.attach_link = "Link6";
      held_object_.tcp_to_object = Eigen::Isometry3d::Identity();
      task_mode_ = RobotTaskMode::HOLDING_UNTRACKED;
      out_message = "HOLDING_UNTRACKED (only place_release allowed)";
      RCLCPP_INFO(LOGGER, "sync_held_object: %s", out_message.c_str());
      need_attach_held_unknown = true;
    }
  }
  if (scene_manager_ && (need_attach_tracked || need_attach_held_unknown))
  {
    scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
  }
  if (need_attach_tracked && scene_manager_ && !scene_manager_->applyAttachedTrackedObjectToScene(tcp_to_object_for_attach))
  {
    RCLCPP_WARN(LOGGER, "sync_held_object: applyAttachedTrackedObjectToScene failed");
  }
  if (need_attach_held_unknown && scene_manager_ && !scene_manager_->applyAttachedHeldUnknownToScene())
  {
    RCLCPP_WARN(LOGGER, "sync_held_object: applyAttachedHeldUnknownToScene failed");
  }
  return true;
}

bool TaskManager::handleResetHeldObject(std::string& out_message)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (task_mode_ != RobotTaskMode::HOLDING_TRACKED && task_mode_ != RobotTaskMode::HOLDING_UNTRACKED &&
        task_mode_ != RobotTaskMode::IDLE && task_mode_ != RobotTaskMode::ERROR)
    {
      out_message = "reset_held_object: busy (PICKING/PLACING), reject";
      RCLCPP_WARN(LOGGER, "%s", out_message.c_str());
      return false;
    }
    held_object_.valid = false;
    task_mode_ = RobotTaskMode::IDLE;
    out_message = "held object cleared, state=IDLE";
    RCLCPP_INFO(LOGGER, "reset_held_object: %s", out_message.c_str());
  }
  if (scene_manager_)
  {
    scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
    scene_manager_->clearAttachedObjectFromPlanningScene("object");
  }
  return true;
}

RobotTaskMode TaskManager::getMode() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return task_mode_;
}

std::string TaskManager::getTaskId() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_task_id_;
}

std::string TaskManager::getLastError() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_error_;
}

HeldObjectContext TaskManager::getHeldObject() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return held_object_;
}

std::string TaskManager::submitJob(const ManipulationJob& job, std::string* out_reject_reason)
{
  if (policy_.reject_new_jobs_while_busy)
  {
    WorkerStatus s = getWorkerStatus();
    if (s == WorkerStatus::RUNNING_JOB || s == WorkerStatus::RECOVERING)
    {
      const char* reason = "busy and reject_new_jobs_while_busy";
      RCLCPP_WARN(LOGGER, "submitJob rejected: type=%s source=%s reason=%s",
                  jobTypeToCString(job.type),
                  job.source.empty() ? "(none)" : job.source.c_str(), reason);
      if (out_reject_reason)
      {
        *out_reject_reason = reason;
      }
      return "";
    }
  }

  ManipulationJob j = job;
  if (j.job_id.empty())
  {
    j.job_id = genTaskId("job");
  }
  if (j.priority < 0)
  {
    j.priority = getDefaultPriority(j.type);
  }
  if ((j.type == JobType::PICK || j.type == JobType::PLACE) && j.priority > 50)
  {
    RCLCPP_WARN(LOGGER, "submitJob: PICK/PLACE priority=%d > 50 (may outrank recovery jobs), consider lower value",
                j.priority);
  }
  const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  j.created_at_ns = now_ns;

  std::string reject_reason;
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (isDuplicateJob(j, now_ns, &reject_reason))
    {
      RCLCPP_WARN(LOGGER, "submitJob rejected: type=%s source=%s reason=%s",
                  jobTypeToCString(j.type),
                  j.source.empty() ? "(none)" : j.source.c_str(), reject_reason.c_str());
      if (out_reject_reason)
      {
        *out_reject_reason = reject_reason;
      }
      return "";
    }
    last_accepted_type_ = j.type;
    last_accepted_has_pose_ = getJobPoseForDedup(j, &last_accepted_pose_);
    last_accepted_time_ns_ = j.created_at_ns;
  }

  queue_->push(j);
  const std::size_t qsize = queue_->size();
  RCLCPP_INFO(LOGGER,
              "submitJob accepted: job_id=%s type=%s source=%s priority=%d queue_size=%zu",
              j.job_id.c_str(), jobTypeToCString(j.type),
              j.source.empty() ? "(none)" : j.source.c_str(), j.priority, qsize);
  return j.job_id;
}

void TaskManager::startWorker()
{
  if (worker_running_.exchange(true))
  {
    RCLCPP_WARN(LOGGER, "startWorker: already running");
    return;
  }
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_status_ = WorkerStatus::IDLE;
  }
  worker_thread_ = std::thread(&TaskManager::workerLoop, this);
  RCLCPP_INFO(LOGGER, "startWorker: started");
}

void TaskManager::stopWorker()
{
  if (!worker_running_.exchange(false))
  {
    return;
  }
  if (worker_thread_.joinable())
  {
    worker_thread_.join();
  }
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    worker_status_ = WorkerStatus::STOPPED;
    current_job_id_.clear();
    current_job_type_.clear();
    current_job_has_pose_ = false;
  }
  RCLCPP_INFO(LOGGER, "stopWorker: stopped");
}

bool TaskManager::isWorkerRunning() const
{
  return worker_running_.load();
}

WorkerStatus TaskManager::getWorkerStatus() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return worker_status_;
}

std::string TaskManager::getCurrentJobId() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return current_job_id_;
}

std::string TaskManager::getCurrentJobType() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return current_job_type_.empty() ? "NONE" : current_job_type_;
}

std::shared_ptr<TaskQueue> TaskManager::getQueue()
{
  return queue_;
}

void TaskManager::setPolicy(const RuntimePolicy& policy)
{
  policy_ = policy;
}

const RuntimePolicy& TaskManager::getPolicy() const
{
  return policy_;
}

bool TaskManager::cancelJob(const std::string& job_id, std::string* out_message)
{
  if (job_id.empty())
  {
    if (out_message)
    {
      *out_message = "job_id empty";
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (current_job_id_ == job_id)
    {
      if (out_message)
      {
        *out_message = "cannot cancel running job";
      }
      RCLCPP_INFO(LOGGER, "cancelJob: %s cannot cancel (job is running)", job_id.c_str());
      return false;
    }
  }
  ManipulationJob removed_job;
  if (queue_->removeById(job_id, &removed_job))
  {
    const int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    pushExecutionRecordCancelled(removed_job, now_ns);
    if (out_message)
    {
      *out_message = "cancelled";
    }
    RCLCPP_INFO(LOGGER, "cancelJob: %s removed from queue", job_id.c_str());
    return true;
  }
  if (out_message)
  {
    *out_message = "job not found";
  }
  RCLCPP_WARN(LOGGER, "cancelJob: %s not found in queue", job_id.c_str());
  return false;
}

void TaskManager::pushExecutionRecordStart(const ManipulationJob& job, int64_t started_at_ns)
{
  JobExecutionRecordEntry entry;
  entry.job_id = job.job_id;
  entry.job_type = jobTypeToCString(job.type);
  entry.source = job.source;
  entry.result_code = JobResultCode::UNKNOWN;
  entry.created_at_ns = job.created_at_ns;
  entry.started_at_ns = started_at_ns;
  entry.finished_at_ns = 0;
  std::lock_guard<std::mutex> lock(records_mutex_);
  if (recent_records_.size() >= MAX_RECENT_RECORDS)
  {
    recent_records_.pop_front();
  }
  recent_records_.push_back(entry);
}

void TaskManager::updateExecutionRecordFinish(JobResultCode code, const std::string& message, int64_t finished_at_ns)
{
  std::lock_guard<std::mutex> lock(records_mutex_);
  if (recent_records_.empty())
  {
    return;
  }
  recent_records_.back().result_code = code;
  recent_records_.back().message = message;
  recent_records_.back().finished_at_ns = finished_at_ns;
}

void TaskManager::pushExecutionRecordCancelled(const ManipulationJob& job, int64_t finished_at_ns)
{
  JobExecutionRecordEntry entry;
  entry.job_id = job.job_id;
  entry.job_type = jobTypeToCString(job.type);
  entry.source = job.source;
  entry.result_code = JobResultCode::CANCELLED;
  entry.message = "cancelled";
  entry.created_at_ns = job.created_at_ns;
  entry.started_at_ns = 0;
  entry.finished_at_ns = finished_at_ns;
  std::lock_guard<std::mutex> lock(records_mutex_);
  if (recent_records_.size() >= MAX_RECENT_RECORDS)
  {
    recent_records_.pop_front();
  }
  recent_records_.push_back(entry);
}

std::vector<TaskManager::JobExecutionRecordEntry> TaskManager::getRecentRecords(std::size_t max_count) const
{
  std::lock_guard<std::mutex> lock(records_mutex_);
  std::vector<JobExecutionRecordEntry> out;
  const std::size_t n = std::min(max_count, recent_records_.size());
  out.reserve(n);
  for (std::size_t i = 0; i < n; ++i)
  {
    out.push_back(recent_records_[recent_records_.size() - 1 - i]);
  }
  return out;
}

void TaskManager::workerLoop()
{
  const auto poll_timeout = std::chrono::milliseconds(500);
  while (worker_running_.load())
  {
    ManipulationJob job;
    if (!queue_->waitPop(job, poll_timeout))
    {
      continue;
    }
    const int64_t started_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    pushExecutionRecordStart(job, started_ns);
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      worker_status_ = WorkerStatus::RUNNING_JOB;
      current_job_id_ = job.job_id;
      current_job_type_ = jobTypeToCString(job.type);
      current_job_has_pose_ = getJobPoseForDedup(job, &current_job_target_pose_);
    }
    RCLCPP_INFO(LOGGER, "worker: executing job %s type=%s source=%s",
                job.job_id.c_str(), jobTypeToCString(job.type),
                job.source.empty() ? "(none)" : job.source.c_str());
    bool ok = executeJob(job);
    std::string err_msg;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      err_msg = last_error_;
    }
    const int64_t finished_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    updateExecutionRecordFinish(ok ? JobResultCode::SUCCESS : JobResultCode::EXEC_FAILED, err_msg, finished_ns);
    {
      std::lock_guard<std::mutex> lock(worker_mutex_);
      current_job_id_.clear();
      current_job_type_.clear();
      current_job_has_pose_ = false;
      worker_status_ = ok ? WorkerStatus::IDLE : WorkerStatus::ERROR;
    }
    if (!ok)
    {
      RCLCPP_ERROR(LOGGER, "worker: job %s failed", job.job_id.c_str());
      if (policy_.auto_reset_after_execution_failure && recovery_actions_)
      {
        {
          std::lock_guard<std::mutex> lock(worker_mutex_);
          worker_status_ = WorkerStatus::RECOVERING;
        }
        recovery_actions_->resetHeldState();
        recovery_actions_->clearSceneResiduals();
        if (policy_.auto_go_home_after_failure)
        {
          recovery_actions_->goHomeIfSafe();
        }
        std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_status_ = WorkerStatus::IDLE;
      }
    }
  }
}

bool TaskManager::executeJob(const ManipulationJob& job)
{
  switch (job.type)
  {
    case JobType::PICK:
    {
      if (!job.object_pose.has_value())
      {
        RCLCPP_ERROR(LOGGER, "executeJob PICK: missing object_pose");
        return false;
      }
      return handlePick(job.object_pose.value(), job.object_id);
    }
    case JobType::PLACE:
    {
      if (!job.target_pose.has_value())
      {
        RCLCPP_ERROR(LOGGER, "executeJob PLACE: missing target_pose");
        return false;
      }
      return handlePlace(job.target_pose.value());
    }
    case JobType::PLACE_RELEASE:
    {
      if (!job.target_pose.has_value())
      {
        RCLCPP_ERROR(LOGGER, "executeJob PLACE_RELEASE: missing target_pose");
        return false;
      }
      return handlePlaceRelease(job.target_pose.value());
    }
    case JobType::RESET_HELD_OBJECT:
    {
      std::string msg;
      return handleResetHeldObject(msg);
    }
    case JobType::SYNC_HELD_OBJECT:
    {
      std::string msg;
      if (job.tracked)
      {
        if (!job.object_pose.has_value() || !job.tcp_pose.has_value())
        {
          RCLCPP_ERROR(LOGGER, "executeJob SYNC: tracked requires object_pose and tcp_pose");
          return false;
        }
        return handleSyncHeldObject(true, true, job.object_id,
                                    job.object_pose->pose, job.tcp_pose.value(), msg);
      }
      else
      {
        geometry_msgs::msg::Pose empty_pose;
        empty_pose.position.x = 0.0;
        empty_pose.position.y = 0.0;
        empty_pose.position.z = 0.0;
        empty_pose.orientation.w = 1.0;
        empty_pose.orientation.x = 0.0;
        empty_pose.orientation.y = 0.0;
        empty_pose.orientation.z = 0.0;
        return handleSyncHeldObject(true, false, job.object_id, empty_pose, empty_pose, msg);
      }
    }
    default:
      RCLCPP_ERROR(LOGGER, "executeJob: unknown type %d", static_cast<int>(job.type));
      return false;
  }
}

}  // namespace orion_mtc
