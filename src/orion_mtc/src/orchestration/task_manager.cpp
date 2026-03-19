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
#include "orion_mtc/planning/place_generator.hpp"
#include "orion_mtc/core/place_types.hpp"
#include "orion_mtc/strategy/cylinder_side_grasp.hpp"
#include "orion_mtc/planning/cable_side_grasp.hpp"
#include "orion_mtc/planning/cable_segments.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>
#include <vector>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.orchestration");

/* 候选失败原因（用于诊断统计） */
enum class CablePickFailReason
{
  INIT_FAILED,
  PLAN_FAILED,
  EXECUTION_FAILED,
};

/* PICK 任务 MTC 阶段名（缆绳分段侧抓：add_cable_segments、局部 ACM、remove_cable_segments） */
static const std::vector<std::string> PICK_STAGE_NAMES_CABLE_SIDE = {
    "move to ready",
    "add_cable_segments",
    "open hand",
    "allow self-collision (pregrasp)",
    "allow collision (cable local) for pregrasp",
    "move to pregrasp",
    "allow collision (cable local) for approach",
    "approach to grasp (LIN)",
    "close hand",
    "remove_cable_segments",
    "retreat short",
    "lift object",
};

/* PLACE 任务 MTC 阶段名（与 place_task_builder 顺序对应；place_transport_pose 非空时含 move_to_transport） */
static const std::vector<std::string> PLACE_STAGE_NAMES = {
    "allow_collision_hand_object", "move_to_transport", "move_to_pre_place", "allow_collision_object_support",
    "lower_to_place", "open_hand", "detach_object", "allow_collision_object_arm_retreat",
    "retreat_short_clear", "retreat_long_leave", "forbid_collision_hand_object", "forbid_collision_object_support",
    "allow_collision_object_arm", "move_to_ready", "open_hand_ready"
};

/* PLACE_RELEASE 任务 MTC 阶段名（与 place_release_task_builder 顺序对应） */
static const std::vector<std::string> PLACE_RELEASE_STAGE_NAMES = {
    "allow_collision_hand_object", "move_to_pre_place_release", "lower_to_place",
    "open_hand", "detach_object", "allow_collision_object_arm_retreat",
    "retreat_short_clear", "retreat_long_leave", "forbid_collision_hand_object",
    "allow_collision_object_arm", "move_to_ready", "open_hand_ready"
};

/* 去重阈值：位置 1cm，姿态约 5°（quat dot ≥ 0.99），时间窗口 1s */
static constexpr double DEDUP_POS_TOL_M = 0.01;
static constexpr double DEDUP_QUAT_DOT_MIN = 0.99;
static constexpr int64_t DEDUP_TIME_WINDOW_NS = 1000000000;

namespace
{
static constexpr const char* RECON_POSE_TOPIC = "reconstructed_object_pose";
static constexpr const char* RECON_APPROACH_TOPIC = "reconstructed_approach_axis";
}  // namespace

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
  pub_reconstructed_object_pose_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(RECON_POSE_TOPIC, 10);
  pub_reconstructed_approach_axis_ =
      node_->create_publisher<geometry_msgs::msg::Vector3Stamped>(RECON_APPROACH_TOPIC, 10);
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
  if (is_gripper_locked_fn_ && is_gripper_locked_fn_())
  {
    RCLCPP_ERROR(LOGGER, "handlePick: gripper locked (has object), reject (place or open_gripper first)");
    return false;
  }

  setState(RobotTaskMode::PICKING);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_task_id_ = genTaskId("pick");
  }

  /* 支持世界系输入：变换到 base_link 后再规划 */
  geometry_msgs::msg::PoseStamped pose_base = object_pose;
  std::optional<geometry_msgs::msg::Vector3Stamped> axis_stamped =
      get_latest_object_axis_fn_ ? get_latest_object_axis_fn_() : std::nullopt;
  if (pose_base.header.frame_id != "base_link" && transform_to_base_link_fn_)
  {
    geometry_msgs::msg::Vector3Stamped* axis_ptr = axis_stamped.has_value() ? &(*axis_stamped) : nullptr;
    if (transform_to_base_link_fn_(pose_base, axis_ptr))
    {
      RCLCPP_INFO(LOGGER, "handlePick: 已将目标从 %s 变换到 base_link", object_pose.header.frame_id.c_str());
    }
    else
    {
      RCLCPP_WARN(LOGGER, "handlePick: 变换到 base_link 失败，按原 frame_id 使用（可能规划异常）");
    }
  }
  if (pose_base.header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "handlePick: frame_id '%s'，规划使用 base_link 系；若不一致请设置 setTransformToBaseLinkCallback",
                pose_base.header.frame_id.c_str());
  }

  double obj_x = pose_base.pose.position.x;
  double obj_y = pose_base.pose.position.y;
  double obj_z = pose_base.pose.position.z;

  geometry_msgs::msg::Vector3 axis_v;
  axis_v.x = 1.0;
  axis_v.y = 0.0;
  axis_v.z = 0.0;
  if (axis_stamped.has_value())
  {
    axis_v = axis_stamped->vector;
  }
  else if (get_latest_object_axis_fn_)
  {
    std::optional<geometry_msgs::msg::Vector3Stamped> opt = get_latest_object_axis_fn_();
    if (opt.has_value())
    {
      axis_v = opt->vector;
    }
  }
  if (!get_latest_object_axis_fn_ && !axis_stamped.has_value())
  {
    RCLCPP_WARN(LOGGER, "handlePick: no object_axis callback, need cable direction for side grasp");
  }

  const double axis_norm = std::sqrt(axis_v.x * axis_v.x + axis_v.y * axis_v.y + axis_v.z * axis_v.z);
  const bool direction_valid = axis_norm > 0.9;
  if (!direction_valid)
  {
    setStateError("CABLE_SIDE_GRASP: 缆绳轴向无效 (norm <= 0.9)，需提供 object_axis");
    return false;
  }

  /* 缆绳侧向包夹 + 分段碰撞：生成段、多候选逐个尝试，成功即返回 */
  {
    CableDetection cable;
    cable.position = Eigen::Vector3d(obj_x, obj_y, obj_z);
    cable.direction = Eigen::Vector3d(axis_v.x, axis_v.y, axis_v.z);
    if (cable.direction.norm() > 1e-9)
    {
      cable.direction.normalize();
    }
    std::vector<CableSegment> segments = buildCableSegments(
        cable.position, cable.direction,
        config_.cable_grasp.cable_total_length,
        config_.cable_grasp.cable_segment_length,
        config_.cable_grasp.cable_radius);
    std::vector<CableGraspCandidate> candidates =
        generateCableSideGrasps(cable, config_.cable_grasp);
    const std::string plan_frame = "base_link";
    const std::string held_id = object_id.empty() ? "cable" : object_id;
    std::vector<CablePickFailReason> fail_reasons;

    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
      const CableGraspCandidate& cand = candidates[i];
      mtc::Task task = pick_builder_->buildFromCableCandidate(segments, cand, plan_frame);
      try
      {
        task.init();
        task.enableIntrospection(true);
        task.introspection().publishTaskDescription();
      }
      catch (mtc::InitStageException& e)
      {
        RCLCPP_WARN(LOGGER, "handlePick: candidate %zu INIT_FAILED: %s", i, e.what());
        fail_reasons.push_back(CablePickFailReason::INIT_FAILED);
        continue;
      }
      moveit::core::MoveItErrorCode plan_result = task.plan(5);
      if (!plan_result || task.solutions().empty())
      {
        std::ostringstream os;
        task.explainFailure(os);
        RCLCPP_WARN(LOGGER, "handlePick: candidate %zu PLAN_FAILED: %s", i, os.str().c_str());
        fail_reasons.push_back(CablePickFailReason::PLAN_FAILED);
        continue;
      }
      geometry_msgs::msg::Pose object_pose_at_grasp;
      isometryToPose(cand.grasp_pose, object_pose_at_grasp);
      moveit_task_constructor_msgs::msg::Solution solution_msg;
      task.solutions().front()->toMsg(solution_msg, &task.introspection());
      StageReportFn stage_report = nullptr;
      if (stage_report_fn_)
      {
        stage_report = [this](const std::string& jid, const std::string& tt, std::size_t ix,
                             const std::string& name, const std::string& state, const std::string& detail) {
          if (stage_report_fn_)
          {
            stage_report_fn_(jid, tt, ix, name, state, detail);
          }
        };
      }
      HeldObjectContext new_held;
      if (solution_executor_->executePickSolution(
              solution_msg, object_pose_at_grasp,
              held_id,
              task.getRobotModel(), new_held, wait_for_gripped_fn_,
              stage_report, getCurrentJobId(), "PICK", PICK_STAGE_NAMES_CABLE_SIDE))
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        held_object_ = new_held;
        setState(RobotTaskMode::HOLDING_TRACKED);
        if (held_object_state_fn_)
        {
          held_object_state_fn_(getHeldObject());
        }
        RCLCPP_INFO(LOGGER, "handlePick: 缆绳侧抓候选 %zu 成功", i);
        return true;
      }
      RCLCPP_WARN(LOGGER, "handlePick: candidate %zu EXECUTION_FAILED", i);
      fail_reasons.push_back(CablePickFailReason::EXECUTION_FAILED);
    }
    std::ostringstream summary;
    summary << "CABLE_SIDE_GRASP: 所有侧抓候选失败 (共 " << fail_reasons.size() << " 次失败). ";
    int n_init = 0, n_plan = 0, n_exec = 0;
    for (CablePickFailReason r : fail_reasons)
    {
      if (r == CablePickFailReason::INIT_FAILED)
        ++n_init;
      else if (r == CablePickFailReason::PLAN_FAILED)
        ++n_plan;
      else
        ++n_exec;
    }
    summary << "INIT_FAILED=" << n_init << " PLAN_FAILED=" << n_plan << " EXECUTION_FAILED=" << n_exec;
    RCLCPP_ERROR(LOGGER, "handlePick: %s", summary.str().c_str());
    setStateError(summary.str());
  }
  return false;
}

bool TaskManager::retreatToReady()
{
  const std::string arm_group_name = "arm";
  const std::string hand_group_name = "hand";
  const std::string hand_frame = "gripper_tcp";

  mtc::Task task;
  task.stages()->setName("retreat to ready");
  task.loadRobotModel(node_);
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_ready->setGroup(arm_group_name);
  stage_ready->setGoal("ready");
  task.add(std::move(stage_ready));

  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand (ready)", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  try
  {
    task.init();
    task.enableIntrospection(true);
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "retreatToReady init failed: " << e);
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "retreatToReady plan failed (code " << plan_result.val << ")");
    std::ostringstream os;
    task.explainFailure(os);
    RCLCPP_ERROR_STREAM(LOGGER, os.str());
    return false;
  }
  if (task.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "retreatToReady: no solutions");
    return false;
  }

  moveit_task_constructor_msgs::msg::Solution solution_msg;
  task.solutions().front()->toMsg(solution_msg, &task.introspection());
  if (!solution_executor_->executeSolution(solution_msg, wait_for_gripped_fn_))
  {
    RCLCPP_ERROR(LOGGER, "retreatToReady execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "retreatToReady finished");
  return true;
}

bool TaskManager::handleOpenGripper()
{
  const std::string hand_group_name = "hand";
  mtc::Task task;
  task.stages()->setName("open gripper");
  task.loadRobotModel(node_);
  /* CurrentState 来自 joint_states（joystick/编码器），仅对手 group MoveTo，臂关节保持不变 */
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));
  try
  {
    task.init();
    task.enableIntrospection(true);
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "open gripper init failed: " << e);
    return false;
  }
  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "open gripper plan failed (code " << plan_result.val << ")");
    std::ostringstream os;
    task.explainFailure(os);
    RCLCPP_ERROR_STREAM(LOGGER, os.str());
    return false;
  }
  if (task.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "open gripper: no solutions");
    return false;
  }
  moveit_task_constructor_msgs::msg::Solution solution_msg;
  task.solutions().front()->toMsg(solution_msg, &task.introspection());
  if (!solution_executor_->executeSolution(solution_msg, wait_for_gripped_fn_))
  {
    RCLCPP_ERROR(LOGGER, "open gripper execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "open gripper finished");
  return true;
}

bool TaskManager::handleCloseGripper()
{
  const std::string hand_group_name = "hand";
  mtc::Task task;
  task.stages()->setName("close gripper");
  task.loadRobotModel(node_);
  /* CurrentState 来自 joint_states（joystick/编码器），仅对手 group MoveTo，臂关节保持不变 */
  task.add(std::make_unique<mtc::stages::CurrentState>("current"));
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto stage_close = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
  stage_close->setGroup(hand_group_name);
  stage_close->setGoal("close");
  task.add(std::move(stage_close));
  try
  {
    task.init();
    task.enableIntrospection(true);
  }
  catch (mtc::InitStageException& e)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "close gripper init failed: " << e);
    return false;
  }
  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "close gripper plan failed (code " << plan_result.val << ")");
    std::ostringstream os;
    task.explainFailure(os);
    RCLCPP_ERROR_STREAM(LOGGER, os.str());
    return false;
  }
  if (task.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "close gripper: no solutions");
    return false;
  }
  moveit_task_constructor_msgs::msg::Solution solution_msg;
  task.solutions().front()->toMsg(solution_msg, &task.introspection());
  if (!solution_executor_->executeSolution(solution_msg, wait_for_gripped_fn_))
  {
    RCLCPP_ERROR(LOGGER, "close gripper execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "close gripper finished");
  return true;
}

void TaskManager::setPlaceGenerator(PlaceGenerator* place_generator)
{
  place_generator_ = place_generator;
}

void TaskManager::setGripperLockedCallback(std::function<bool()> fn)
{
  is_gripper_locked_fn_ = std::move(fn);
}

void TaskManager::setGetLatestObjectPoseCallback(
    std::function<std::optional<geometry_msgs::msg::PoseStamped>()> fn)
{
  get_latest_object_pose_fn_ = std::move(fn);
}

void TaskManager::setGetLatestObjectAxisCallback(
    std::function<std::optional<geometry_msgs::msg::Vector3Stamped>()> fn)
{
  get_latest_object_axis_fn_ = std::move(fn);
}

void TaskManager::setTransformToBaseLinkCallback(TransformToBaseLinkFn fn)
{
  transform_to_base_link_fn_ = std::move(fn);
}

void TaskManager::setJobEventCallback(JobEventFn fn)
{
  job_event_fn_ = std::move(fn);
}

void TaskManager::setHeldObjectStateCallback(HeldObjectStateFn fn)
{
  held_object_state_fn_ = std::move(fn);
}

void TaskManager::setRecoveryEventCallback(RecoveryEventFn fn)
{
  recovery_event_fn_ = std::move(fn);
}

void TaskManager::setStageReportCallback(StageReportFn fn)
{
  stage_report_fn_ = std::move(fn);
}

std::string TaskManager::getNextJobType() const
{
  ManipulationJob front;
  if (!queue_ || !queue_->peekFront(front))
  {
    return "";
  }
  return jobTypeToCString(front.type);
}

bool TaskManager::handlePlaceSingle(const geometry_msgs::msg::PoseStamped& target_pose)
{
  HeldObjectContext held;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held = held_object_;
  }
  if (!held.valid)
  {
    return false;
  }

  double px = target_pose.pose.position.x;
  double py = target_pose.pose.position.y;
  double pz = target_pose.pose.position.z;
  double qx = target_pose.pose.orientation.x;
  double qy = target_pose.pose.orientation.y;
  double qz = target_pose.pose.orientation.z;
  double qw = target_pose.pose.orientation.w;

  mtc::Task task = place_builder_->build(px, py, pz, qx, qy, qz, qw, held);
  if (task.stages()->numChildren() == 0)
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = "buildPlaceTask: no held context";
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
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = std::string("place init: ") + e.what();
    return false;
  }

  moveit::core::MoveItErrorCode plan_result = task.plan(5);
  if (!plan_result)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Place planning failed (code " << plan_result.val << ")");
    std::ostringstream os;
    task.explainFailure(os);
    RCLCPP_ERROR_STREAM(LOGGER, os.str());
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = "place plan failed";
    return false;
  }
  if (task.solutions().empty())
  {
    RCLCPP_ERROR(LOGGER, "Place plan returned no solutions");
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = "place no solutions";
    return false;
  }

  moveit_task_constructor_msgs::msg::Solution place_solution_msg;
  task.solutions().front()->toMsg(place_solution_msg, &task.introspection());
  task.introspection().publishSolution(*task.solutions().front());
  StageReportFn place_stage_report = nullptr;
  if (stage_report_fn_)
  {
    place_stage_report = [this](const std::string& jid, const std::string& tt, std::size_t ix,
                                const std::string& name, const std::string& state, const std::string& detail) {
      if (stage_report_fn_)
      {
        stage_report_fn_(jid, tt, ix, name, state, detail);
      }
    };
  }
  if (!solution_executor_->executeSolution(place_solution_msg, wait_for_gripped_fn_,
                                           place_stage_report, getCurrentJobId(), "PLACE", PLACE_STAGE_NAMES))
  {
    RCLCPP_ERROR(LOGGER, "Place execution failed");
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_ = "place execution failed";
    return false;
  }

  if (scene_manager_)
  {
    /* 放置成功后兜底清除 attached，再更新 world 位姿，避免 scene 双份 */
    scene_manager_->clearAttachedObjectFromPlanningScene("object");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
    scene_manager_->applyObjectPoseToPlanningScene(px, py, pz, qx, qy, qz, qw);
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_.valid = false;
  }
  setState(RobotTaskMode::IDLE);
  if (held_object_state_fn_)
  {
    held_object_state_fn_(getHeldObject());
  }
  RCLCPP_INFO(LOGGER, "Place finished successfully, state=IDLE");
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

  if (target_pose.header.frame_id != "base_link")
  {
    RCLCPP_WARN(LOGGER, "handlePlace: frame_id '%s', expected base_link", target_pose.header.frame_id.c_str());
  }

  if (place_generator_)
  {
    std::vector<PlaceCandidate> candidates = place_generator_->generate(target_pose, held);
    if (candidates.empty())
    {
      RCLCPP_WARN(LOGGER, "handlePlace: PlaceGenerator returned no candidates");
      setState(RobotTaskMode::HOLDING_TRACKED);
      return false;
    }
    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
      RCLCPP_INFO(LOGGER, "handlePlace: trying candidate %zu/%zu (source=%s)",
                  i + 1, candidates.size(), candidates[i].source.c_str());
      if (handlePlaceSingle(candidates[i].object_pose))
      {
        return true;
      }
    }
    RCLCPP_WARN(LOGGER, "handlePlace: all %zu candidates failed", candidates.size());
    setState(RobotTaskMode::HOLDING_TRACKED);
    return false;
  }

  if (handlePlaceSingle(target_pose))
  {
    return true;
  }
  setState(RobotTaskMode::HOLDING_TRACKED);
  return false;
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
    std::ostringstream os;
    task.explainFailure(os);
    RCLCPP_ERROR_STREAM(LOGGER, os.str());
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
  StageReportFn release_stage_report = nullptr;
  if (stage_report_fn_)
  {
    release_stage_report = [this](const std::string& jid, const std::string& tt, std::size_t ix,
                                 const std::string& name, const std::string& state, const std::string& detail) {
      if (stage_report_fn_)
      {
        stage_report_fn_(jid, tt, ix, name, state, detail);
      }
    };
  }
  if (!solution_executor_->executeSolution(release_solution_msg, wait_for_gripped_fn_,
                                           release_stage_report, getCurrentJobId(), "PLACE_RELEASE", PLACE_RELEASE_STAGE_NAMES))
  {
    RCLCPP_ERROR(LOGGER, "PlaceRelease execution failed");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "place_release execution failed";
    }
    setState(prev_mode);
    return false;
  }

  if (scene_manager_)
  {
    /* 放置成功后兜底清除所有可能残留的 attached */
    scene_manager_->clearAttachedObjectFromPlanningScene("object");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    held_object_.valid = false;
  }
  setState(RobotTaskMode::IDLE);
  if (held_object_state_fn_)
  {
    held_object_state_fn_(getHeldObject());
  }
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
    /* sync(tracked) 时避免 world 残留 "object" 与 attached "held_tracked" 双份 */
    if (need_attach_tracked)
    {
      scene_manager_->removeWorldObject("object");
    }
    scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
  }
  if (need_attach_tracked && scene_manager_
      && !scene_manager_->applyAttachedTrackedObjectToScene(tcp_to_object_for_attach,
                                                            config_.grasp_offset_along_axis))
  {
    RCLCPP_WARN(LOGGER, "sync_held_object: applyAttachedTrackedObjectToScene failed");
  }
  if (need_attach_held_unknown && scene_manager_ && !scene_manager_->applyAttachedHeldUnknownToScene())
  {
    RCLCPP_WARN(LOGGER, "sync_held_object: applyAttachedHeldUnknownToScene failed");
  }
  if (held_object_state_fn_)
  {
    held_object_state_fn_(getHeldObject());
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
  if (held_object_state_fn_)
  {
    held_object_state_fn_(getHeldObject());
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

  if (policy_.reject_new_jobs_while_busy)
  {
    WorkerStatus s = getWorkerStatus();
    if (s == WorkerStatus::RUNNING_JOB || s == WorkerStatus::RECOVERING)
    {
      const char* reason = "busy and reject_new_jobs_while_busy";
      RCLCPP_WARN(LOGGER, "submitJob rejected: type=%s source=%s reason=%s",
                  jobTypeToCString(j.type),
                  j.source.empty() ? "(none)" : j.source.c_str(), reason);
      if (out_reject_reason)
      {
        *out_reject_reason = reason;
      }
      if (job_event_fn_)
      {
        job_event_fn_(j.job_id, jobTypeToCString(j.type), j.source, static_cast<uint32_t>(j.priority),
                      "REJECTED", false, reason, j.created_at_ns, 0, 0);
      }
      return "";
    }
  }

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
      if (job_event_fn_)
      {
        job_event_fn_(j.job_id, jobTypeToCString(j.type), j.source, static_cast<uint32_t>(j.priority),
                      "REJECTED", false, reject_reason, j.created_at_ns, 0, 0);
      }
      return "";
    }
    last_accepted_type_ = j.type;
    last_accepted_has_pose_ = getJobPoseForDedup(j, &last_accepted_pose_);
    last_accepted_time_ns_ = j.created_at_ns;
  }

  queue_->push(j);
  if (job_event_fn_)
  {
    job_event_fn_(j.job_id, jobTypeToCString(j.type), j.source, static_cast<uint32_t>(j.priority),
                  "SUBMITTED", true, "", j.created_at_ns, 0, 0);
  }
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
    if (job_event_fn_)
    {
      job_event_fn_(removed_job.job_id, jobTypeToCString(removed_job.type), removed_job.source,
                    static_cast<uint32_t>(removed_job.priority), "CANCELLED", false, "cancelled",
                    removed_job.created_at_ns, 0, now_ns);
    }
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
    if (job_event_fn_)
    {
      job_event_fn_(job.job_id, jobTypeToCString(job.type), job.source,
                    static_cast<uint32_t>(job.priority), "STARTED", true, "",
                    job.created_at_ns, started_ns, 0);
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
    if (job_event_fn_)
    {
      job_event_fn_(job.job_id, jobTypeToCString(job.type), job.source,
                    static_cast<uint32_t>(job.priority),
                    ok ? "SUCCEEDED" : "FAILED", ok, err_msg,
                    job.created_at_ns, started_ns, finished_ns);
    }
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
        const std::string trigger = "execution_failure";
        bool r1 = recovery_actions_->resetHeldState();
        if (recovery_event_fn_)
        {
          recovery_event_fn_("RESET_HELD", trigger, r1, r1 ? "ok" : "failed");
        }
        bool r2 = recovery_actions_->clearSceneResiduals();
        if (recovery_event_fn_)
        {
          recovery_event_fn_("CLEAR_SCENE", trigger, r2, r2 ? "ok" : "failed");
        }
        bool r3 = true;
        if (policy_.auto_go_home_after_failure)
        {
          r3 = recovery_actions_->goHomeIfSafe();
          if (recovery_event_fn_)
          {
            recovery_event_fn_("GO_HOME", trigger, r3, r3 ? "ok" : "skipped or failed");
          }
        }
        if (recovery_event_fn_)
        {
          recovery_event_fn_("AUTO_RECOVERY", trigger, r1 && r2 && r3, "reset_held + clear_scene + go_home");
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
      /* 目标点仅来自话题：没有话题更新则不规划（无当前目标） */
      if (!get_latest_object_pose_fn_)
      {
        RCLCPP_ERROR(LOGGER, "executeJob PICK: no object_pose topic callback, cannot plan");
        return false;
      }
      std::optional<geometry_msgs::msg::PoseStamped> latest = get_latest_object_pose_fn_();
      if (!latest.has_value())
      {
        RCLCPP_ERROR(LOGGER, "executeJob PICK: no object_pose from topic (no target), skip plan");
        return false;
      }

      /* 缆绳侧向抓取：object_pose/object_axis 由话题提供，规划时用侧抓候选。 */
      double lx = latest->pose.position.x;
      double ly = latest->pose.position.y;
      double lz = latest->pose.position.z;
      RCLCPP_INFO(LOGGER,
                  "executeJob PICK: 感知状态目标点 frame_id=%s x=%.3f y=%.3f z=%.3f (缆绳中心，approach 后 gripper_tcp 将到此点)",
                  latest->header.frame_id.c_str(), lx, ly, lz);
      if (std::abs(lx) < 1e-6 && std::abs(ly) < 1e-6 && std::abs(lz) < 1e-6)
      {
        RCLCPP_WARN(LOGGER, "executeJob PICK: 目标点接近 (0,0,0)，请检查 bridge 是否发布有效 object_pose");
      }
      return handlePick(latest.value(), job.object_id);
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
    case JobType::OPEN_GRIPPER:
      return handleOpenGripper();
    case JobType::CLOSE_GRIPPER:
      return handleCloseGripper();
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
