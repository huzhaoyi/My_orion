/* TaskManager：PICK 侧抓 MTC 构建/规划/执行、任务队列 Worker、持物与 planning scene 对齐 */

#include "orion_mtc/orchestration/task_manager.hpp"
#include "orion_mtc/orchestration/task_queue.hpp"
#include "orion_mtc/orchestration/recovery_actions.hpp"
#include "orion_mtc/planning/pick_task_builder.hpp"
#include "orion_mtc/planning/insert_task_builder.hpp"
#include "orion_mtc/scene/planning_scene_manager.hpp"
#include "orion_mtc/execution/trajectory_executor.hpp"
#include "orion_mtc/execution/solution_executor.hpp"
#include "orion_mtc/core/job_result_code.hpp"
#include "orion_mtc/core/runtime_status.hpp"
#include "orion_mtc/core/constants.hpp"
#include "orion_mtc/core/held_object.hpp"
#include "orion_mtc/core/cable_pick_fail_reason.hpp"
#include "orion_mtc/decision/cable_side_pick_precheck.hpp"
#include "orion_mtc/decision/cylinder_side_grasp.hpp"
#include "orion_mtc/decision/feasibility_checker.hpp"
#include "orion_mtc/orchestration/job_deduplicator.hpp"
#include "orion_mtc/planning/cable_side_grasp.hpp"
#include "orion_mtc/planning/cable_segments.hpp"
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_task_constructor_msgs/msg/solution.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <future>
#include <functional>
#include <optional>
#include <sstream>
#include <vector>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.orchestration");
static constexpr char TARGET_INSERT_HOLES_TOPIC[] = "/manipulator/target_insert_holes_debug";
static constexpr char TARGET_INSERT_HOLE_MARKERS_TOPIC[] = "/manipulator/target_insert_hole_markers";
static constexpr double TARGET_INSERT_HOLE_DIAMETER_M = 0.128;
static constexpr double TARGET_INSERT_HOLE_MARKER_THICKNESS_M = 0.02;

/* PICK 任务 MTC 阶段名（与 MTC 子轨迹顺序对齐：首段为 CurrentState） */
static const std::vector<std::string> PICK_STAGE_NAMES_CABLE_SIDE = {
    "current",
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
    "move to pregrasp (holding)",
    "close hand (at pregrasp)",
};

/* 与缆绳链同构：add peg → open → 自碰 ACM → SerialContainer 内子阶段（与 sub_trajectory 顺序一致） */
static const std::vector<std::string> PICK_STAGE_NAMES_TARGET_SENSOR = {
    "current",
    "move to ready",
    "add targetsensor peg mesh",
    "open hand",
    "allow self-collision (pregrasp)",
    "allow collision (targetsensor peg) for pregrasp",
    "move to pregrasp",
    "allow collision (targetsensor peg) for approach",
    "approach to grasp (LIN)",
    "close hand",
    "remove targetsensor peg mesh",
    "retreat short",
    "move to pregrasp (holding)",
    "close hand (at pregrasp)",
};

struct TargetSensorPickCandidate
{
  geometry_msgs::msg::PoseStamped object_pose;
  int low_z_hard_priority_tier = 0;
  double pregrasp_distance_m = 0.10;
  int pregrasp_priority_rank = 0;
  double approach_sign = -1.0;
  int sign_priority_rank = 0;
  double tool_roll_deg = 0.0;
  int roll_priority_rank = 0;
  int approach_axis_local = 2;
  int axis_priority_rank = 0;
  double down_priority_score = -1.0;
  std::string label;
};

bool buildTargetSensorGoalPoses(const TargetSensorPickCandidate& candidate,
                                double grasp_depth_m,
                                double surface_backoff_m,
                                Eigen::Isometry3d& out_grasp_pose,
                                Eigen::Isometry3d& out_pregrasp_pose)
{
  const geometry_msgs::msg::Pose& pose = candidate.object_pose.pose;
  Eigen::Quaterniond q_object(
      pose.orientation.w,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z);
  if (q_object.norm() < 1e-9)
  {
    q_object = Eigen::Quaterniond::Identity();
  }
  q_object.normalize();
  Eigen::Vector3d local_axis = Eigen::Vector3d::UnitZ();
  if (candidate.approach_axis_local == 0)
  {
    local_axis = Eigen::Vector3d::UnitX();
  }
  else if (candidate.approach_axis_local == 1)
  {
    local_axis = Eigen::Vector3d::UnitY();
  }
  const Eigen::Vector3d n_geom = (q_object * local_axis).normalized();
  const double sign = (candidate.approach_sign < 0.0) ? -1.0 : 1.0;
  const Eigen::Vector3d approach_dir = n_geom * sign;

  Eigen::Vector3d ref = (q_object * Eigen::Vector3d::UnitZ()).normalized();
  Eigen::Vector3d y_axis = ref - ref.dot(approach_dir) * approach_dir;
  if (y_axis.norm() < 1e-3)
  {
    ref = (q_object * Eigen::Vector3d::UnitY()).normalized();
    y_axis = ref - ref.dot(approach_dir) * approach_dir;
  }
  if (y_axis.norm() < 1e-3)
  {
    y_axis = approach_dir.unitOrthogonal();
  }
  y_axis.normalize();
  const Eigen::Vector3d z_axis = approach_dir;
  const Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();
  Eigen::Matrix3d R_tool;
  R_tool.col(0) = x_axis;
  R_tool.col(1) = y_axis;
  R_tool.col(2) = z_axis;

  if (std::abs(candidate.tool_roll_deg) > 1e-6)
  {
    constexpr double PI_D = 3.14159265358979323846;
    const double roll_rad = candidate.tool_roll_deg * PI_D / 180.0;
    const Eigen::Matrix3d R_roll = Eigen::AngleAxisd(roll_rad, approach_dir.normalized()).toRotationMatrix();
    R_tool = R_roll * R_tool;
  }

  const double grasp_depth = std::max(0.0, grasp_depth_m);
  const double surface_backoff = std::max(0.0, surface_backoff_m);
  const Eigen::Vector3d p_object(pose.position.x, pose.position.y, pose.position.z);
  const Eigen::Vector3d p_grasp = p_object - approach_dir * surface_backoff + approach_dir * grasp_depth;

  out_grasp_pose = Eigen::Isometry3d::Identity();
  out_grasp_pose.linear() = R_tool;
  out_grasp_pose.translation() = p_grasp;
  out_pregrasp_pose = Eigen::Isometry3d::Identity();
  out_pregrasp_pose.linear() = R_tool;
  out_pregrasp_pose.translation() = p_grasp - approach_dir * candidate.pregrasp_distance_m;
  return true;
}

bool precheckTargetSensorPickCandidate(const rclcpp::Logger& logger,
                                       std::size_t candidate_index,
                                       const TargetSensorPickCandidate& candidate,
                                       const moveit::core::RobotModelConstPtr& robot_model,
                                       const planning_scene::PlanningScenePtr& scene_for_ik_seed,
                                       const std::string& arm_group_name,
                                       const std::string& hand_frame,
                                       double grasp_depth_m,
                                       double surface_backoff_m,
                                       Eigen::Isometry3d& out_grasp_pose)
{
  Eigen::Isometry3d pregrasp_pose = Eigen::Isometry3d::Identity();
  if (!buildTargetSensorGoalPoses(
          candidate,
          grasp_depth_m,
          surface_backoff_m,
          out_grasp_pose,
          pregrasp_pose))
  {
    return false;
  }
  if (!robot_model)
  {
    return true;
  }
  const moveit::core::JointModelGroup* jmg = robot_model->getJointModelGroup(arm_group_name);
  if (jmg == nullptr)
  {
    RCLCPP_WARN(logger, "handlePick: targetsensor candidate %zu jmg arm missing, skip precheck", candidate_index);
    return true;
  }
  moveit::core::RobotState state(robot_model);
  if (scene_for_ik_seed)
  {
    state = scene_for_ik_seed->getCurrentState();
  }
  else
  {
    state.setToDefaultValues();
  }
  const bool grasp_ik_ok = state.setFromIK(jmg, out_grasp_pose, hand_frame, 0.15);
  if (!grasp_ik_ok)
  {
    RCLCPP_WARN(logger, "handlePick: targetsensor candidate %zu grasp IK fail", candidate_index);
    return false;
  }
  if (scene_for_ik_seed)
  {
    state = scene_for_ik_seed->getCurrentState();
  }
  else
  {
    state.setToDefaultValues();
  }
  const bool pregrasp_ik_ok = state.setFromIK(jmg, pregrasp_pose, hand_frame, 0.15);
  if (!pregrasp_ik_ok)
  {
    RCLCPP_WARN(logger, "handlePick: targetsensor candidate %zu pregrasp IK fail", candidate_index);
    return false;
  }
  state.update();
  if (!state.satisfiesBounds(jmg))
  {
    RCLCPP_WARN(logger, "handlePick: targetsensor candidate %zu pregrasp OUT_OF_BOUNDS", candidate_index);
    return false;
  }
  return true;
}

static int32_t parseTargetSlotFromObjectId(const std::string& object_id)
{
  const std::string key = "slot_";
  const std::size_t pos = object_id.find(key);
  if (pos == std::string::npos)
  {
    return -1;
  }
  std::size_t i = pos + key.size();
  if (i >= object_id.size())
  {
    return -1;
  }
  int32_t slot = 0;
  for (; i < object_id.size(); ++i)
  {
    const char c = object_id[i];
    if (c < '0' || c > '9')
    {
      break;
    }
    slot = static_cast<int32_t>(slot * 10 + static_cast<int32_t>(c - '0'));
  }
  if (slot <= 0)
  {
    return -1;
  }
  return slot;
}

Eigen::Vector3d normalizedInsertAxisLocalFromConfig(const PegInsertConfig& pi)
{
  if (pi.insert_axis_local_xyz.size() != 3u)
  {
    return Eigen::Vector3d::UnitX();
  }
  const Eigen::Vector3d axis_local(pi.insert_axis_local_xyz[0], pi.insert_axis_local_xyz[1],
                                   pi.insert_axis_local_xyz[2]);
  const double n = axis_local.norm();
  if (!std::isfinite(axis_local.x()) || !std::isfinite(axis_local.y()) || !std::isfinite(axis_local.z())
      || n < 1e-9)
  {
    return Eigen::Vector3d::UnitX();
  }
  return axis_local / n;
}

/*
 * TaskManager 构造：保存规划与执行子系统指针，创建 PickTaskBuilder、任务队列、RecoveryActions。
 * wait_for_gripped_fn 在闭合夹爪阶段用于等待夹爪反馈；scene_manager 仅被恢复与持物逻辑使用。
 */
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
  , insert_builder_(std::make_unique<InsertTaskBuilder>(node, config))
  , queue_(std::make_shared<TaskQueue>())
  , recovery_actions_(std::make_unique<RecoveryActions>(scene_manager, this))
{
  auto qos = rclcpp::QoS(1).reliable().transient_local();
  target_insert_holes_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(TARGET_INSERT_HOLES_TOPIC, qos);
  target_insert_hole_markers_pub_ =
      node_->create_publisher<visualization_msgs::msg::MarkerArray>(TARGET_INSERT_HOLE_MARKERS_TOPIC, qos);
  target_insert_hole_debug_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(1000), [this]() { publishTargetInsertHoleDebug(); });
  publishTargetInsertHoleDebug();
}

/* 析构：停止 Worker 线程，避免与正在退出的 ROS 上下文竞态。 */
TaskManager::~TaskManager()
{
  stopWorker();
}

/* 注入审批器指针；handlePick 在 base_link 下会调用 objectPoseWithinWorkspaceHardLimits。nullptr 则跳过工作空间硬校验。 */
void TaskManager::setFeasibilityChecker(FeasibilityChecker* checker)
{
  feasibility_checker_ = checker;
}

/* 设置业务态 task_mode_（加锁）；供抓取成功、恢复、急停等路径统一改写。 */
void TaskManager::setState(RobotTaskMode mode)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  task_mode_ = mode;
}

/* 置 ERROR 态并记录 last_error_，打 ERROR 日志；用于规划/执行/业务规则拒绝等不可恢复路径。 */
void TaskManager::setStateError(const std::string& err)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  task_mode_ = RobotTaskMode::ERROR;
  last_error_ = err;
  RCLCPP_ERROR(LOGGER, "state ERROR: %s", err.c_str());
}

/* 生成唯一任务 id：prefix + 系统纳秒时间戳，用于 pick 与 submitJob 未显式指定 job_id 时。 */
std::string TaskManager::genTaskId(const char* prefix)
{
  auto now = std::chrono::system_clock::now();
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
  std::ostringstream oss;
  oss << prefix << "_" << ns;
  return oss.str();
}

/*
 * 同步抓取主路径（即时调用，非队列）：缆绳侧向多候选 MTC。
 * 流程：急停/持物/ busy / 夹爪 locked 检查 → TF 到 base_link（可选回调）→ feasibility 硬限
 * → 取 object_axis → buildCableSegments + generateCableSideGrasps → 逐候选 precheck → MTC plan
 * → executePickSolution（夹紧检测）→ 成功则 HOLDING_TRACKED；失败可换候选；夹紧失败则 retreatToReady。
 * object_id 空时持物场景标记默认为 "cable"。
 */
bool TaskManager::handlePick(const geometry_msgs::msg::PoseStamped& object_pose,
                             const std::string& object_id,
                             GraspSource grasp_source)
{
  if (estop_requested_.load())
  {
    setStateError("E_STOP");
    estop_requested_.store(false);
    return false;
  }
  RobotTaskMode mode;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    mode = task_mode_;
  }
  if (isHolding(mode))
  {
    RCLCPP_ERROR(LOGGER, "handlePick: already holding, reject (reset_held_object first)");
    return false;
  }
  if (!manipulation_fsm_.canAcceptPick(mode))
  {
    RCLCPP_ERROR(LOGGER, "handlePick: busy (mode not IDLE/ERROR), reject");
    return false;
  }
  if (is_gripper_locked_fn_ && is_gripper_locked_fn_())
  {
    RCLCPP_ERROR(LOGGER, "handlePick: gripper locked (has object), reject (open_gripper first)");
    return false;
  }
  if (feasibility_checker_ != nullptr && feasibility_checker_->inJoyModeSwitchIkCooldown())
  {
    RCLCPP_WARN(LOGGER, "handlePick: rejected (joy auto/manual just switched, skip IK cable precheck)");
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      last_error_ = "JOY_MODE_SWITCH_COOLDOWN";
    }
    return false;
  }

  setState(RobotTaskMode::PICKING);
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_task_id_ = genTaskId("pick");
  }

  /* 支持世界系输入：变换到 base_link 后再规划。规划系必须与机械臂 base_link 一致（URDF 改加 ROV 后根为 world，但 base_link 仍为臂根）。 */
  RCLCPP_INFO(LOGGER,
              "handlePick: 收到 object_pose frame_id=%s pos=(%.4f, %.4f, %.4f) grasp_source=%s "
              "[规划系应为机械臂 base_link]",
              object_pose.header.frame_id.c_str(),
              object_pose.pose.position.x,
              object_pose.pose.position.y,
              object_pose.pose.position.z,
              graspSourceToCString(grasp_source));
  geometry_msgs::msg::PoseStamped pose_base = object_pose;
  const bool use_cable_side_flow =
      (grasp_source == GraspSource::LEGACY || grasp_source == GraspSource::FUSED);
  std::optional<geometry_msgs::msg::Vector3Stamped> axis_stamped;
  if (use_cable_side_flow && grasp_source == GraspSource::FUSED && get_latest_object_axis_fused_fn_)
  {
    axis_stamped = get_latest_object_axis_fused_fn_();
  }
  else if (use_cable_side_flow && grasp_source != GraspSource::FUSED && get_latest_object_axis_legacy_fn_)
  {
    axis_stamped = get_latest_object_axis_legacy_fn_();
  }
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
  if (feasibility_checker_ != nullptr && pose_base.header.frame_id == "base_link")
  {
    std::string ws_reject;
    if (!feasibility_checker_->objectPoseWithinWorkspaceHardLimits(pose_base.pose, ws_reject))
    {
      RCLCPP_WARN(LOGGER, "handlePick: workspace hard reject: %s", ws_reject.c_str());
      setStateError(std::string("WORKSPACE: ") + ws_reject);
      return false;
    }
  }
  else if (feasibility_checker_ != nullptr)
  {
    RCLCPP_WARN(LOGGER, "handlePick: object_pose 未在 base_link（当前 %s），跳过工作空间硬校验",
                pose_base.header.frame_id.c_str());
  }
  RCLCPP_INFO(LOGGER,
              "handlePick: 规划使用 frame=%s target pos=(%.4f, %.4f, %.4f), source=%s",
              pose_base.header.frame_id.c_str(),
              pose_base.pose.position.x, pose_base.pose.position.y, pose_base.pose.position.z,
              graspSourceToCString(grasp_source));

  double obj_x = pose_base.pose.position.x;
  double obj_y = pose_base.pose.position.y;
  double obj_z = pose_base.pose.position.z;

  if (!use_cable_side_flow)
  {
    if (scene_manager_)
    {
      scene_manager_->removeWorldObject(TARGET_SENSOR_PEG_COLLISION_ID);
    }
    const std::string held_id = object_id.empty() ? "targetsensor" : object_id;
    const std::string plan_frame = "base_link";
    std::string last_plan_detail = "";
    moveit::core::RobotModelConstPtr robot_model;
    planning_scene::PlanningScenePtr scene_for_ik_seed;
    const std::string arm_group_name = "arm";
    const std::string hand_frame = "gripper_tcp";
    try
    {
      robot_model_loader::RobotModelLoader loader(node_);
      robot_model = loader.getModel();
      if (robot_model)
      {
        auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        if (client->wait_for_service(std::chrono::milliseconds(300)))
        {
          auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
          req->components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
                                       moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                       moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                       moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
          auto fut = client->async_send_request(req);
          if (fut.wait_for(std::chrono::milliseconds(600)) == std::future_status::ready)
          {
            auto resp = fut.get();
            if (resp)
            {
              scene_for_ik_seed = std::make_shared<planning_scene::PlanningScene>(robot_model);
              scene_for_ik_seed->setPlanningSceneMsg(resp->scene);
            }
          }
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(LOGGER, "handlePick: targetsensor precheck init failed: %s", e.what());
    }

    std::vector<double> pregrasp_candidates;
    for (double pre_m : config_.target_sensor_pick.pregrasp_distances_m)
    {
      if (pre_m > 1e-6)
      {
        pregrasp_candidates.push_back(pre_m);
      }
    }
    if (pregrasp_candidates.empty())
    {
      pregrasp_candidates.push_back(0.10);
    }
    for (double pre_m : config_.target_sensor_pick.fallback_pregrasp_distances_m)
    {
      if (pre_m <= 1e-6)
      {
        continue;
      }
      if (std::find_if(pregrasp_candidates.begin(), pregrasp_candidates.end(),
                       [pre_m](double v) { return std::abs(v - pre_m) < 1e-6; }) == pregrasp_candidates.end())
      {
        pregrasp_candidates.push_back(pre_m);
      }
    }
    std::sort(pregrasp_candidates.begin(), pregrasp_candidates.end());
    if (config_.target_sensor_pick.dynamic_axis_priority_by_z_enable &&
        pose_base.pose.position.z <= config_.target_sensor_pick.dynamic_axis_low_z_threshold_m)
    {
      RCLCPP_INFO(
          LOGGER,
          "handlePick: low target z=%.4f, keep shorter pregrasp first for better reachability",
          pose_base.pose.position.z);
    }
    std::vector<double> sign_candidates;
    for (double sign : config_.target_sensor_pick.approach_sign_candidates)
    {
      sign_candidates.push_back((sign < 0.0) ? -1.0 : 1.0);
    }
    if (sign_candidates.empty())
    {
      const double configured_sign = (config_.target_sensor_pick.approach_normal_sign < 0.0) ? -1.0 : 1.0;
      sign_candidates.push_back(configured_sign);
    }
    std::sort(sign_candidates.begin(), sign_candidates.end());
    sign_candidates.erase(std::unique(sign_candidates.begin(), sign_candidates.end()), sign_candidates.end());
    if (config_.target_sensor_pick.dynamic_sign_priority_enable && !sign_candidates.empty())
    {
      const double preferred_sign =
          (pose_base.pose.position.y < 0.0)
              ? config_.target_sensor_pick.preferred_sign_for_negative_y
              : config_.target_sensor_pick.preferred_sign_for_positive_y;
      auto it = std::find_if(
          sign_candidates.begin(),
          sign_candidates.end(),
          [preferred_sign](double s) { return std::abs(s - preferred_sign) < 1e-6; });
      if (it != sign_candidates.end() && it != sign_candidates.begin())
      {
        const double v = *it;
        sign_candidates.erase(it);
        sign_candidates.insert(sign_candidates.begin(), v);
      }
    }
    std::vector<double> roll_candidates_deg = config_.target_sensor_pick.tool_roll_candidates_deg;
    if (roll_candidates_deg.empty())
    {
      roll_candidates_deg.push_back(0.0);
    }

    const double target_distance_m = std::sqrt(
        pose_base.pose.position.x * pose_base.pose.position.x +
        pose_base.pose.position.y * pose_base.pose.position.y +
        pose_base.pose.position.z * pose_base.pose.position.z);
    const std::vector<int64_t>* configured_axis_order = &config_.target_sensor_pick.approach_axis_local_candidates;
    if (config_.target_sensor_pick.dynamic_axis_priority_enable)
    {
      if (target_distance_m <= config_.target_sensor_pick.dynamic_axis_near_max_distance_m)
      {
        configured_axis_order = &config_.target_sensor_pick.dynamic_axis_near_order;
      }
      else if (target_distance_m <= config_.target_sensor_pick.dynamic_axis_mid_max_distance_m)
      {
        configured_axis_order = &config_.target_sensor_pick.dynamic_axis_mid_order;
      }
      else
      {
        configured_axis_order = &config_.target_sensor_pick.dynamic_axis_far_order;
      }
    }
    if (config_.target_sensor_pick.dynamic_axis_priority_by_z_enable &&
        pose_base.pose.position.z <= config_.target_sensor_pick.dynamic_axis_low_z_threshold_m)
    {
      configured_axis_order = &config_.target_sensor_pick.dynamic_axis_low_z_order;
      RCLCPP_INFO(
          LOGGER,
          "handlePick: low-z axis override z=%.4f threshold=%.4f",
          pose_base.pose.position.z,
          config_.target_sensor_pick.dynamic_axis_low_z_threshold_m);
    }
    std::vector<int> axis_candidates;
    axis_candidates.reserve(configured_axis_order->size());
    for (int64_t axis_raw : *configured_axis_order)
    {
      axis_candidates.push_back(static_cast<int>(axis_raw));
    }
    if (axis_candidates.empty())
    {
      axis_candidates.push_back(config_.target_sensor_pick.approach_axis_local);
    }
    const bool low_z_mode_enabled =
        config_.target_sensor_pick.dynamic_axis_priority_by_z_enable &&
        pose_base.pose.position.z <= config_.target_sensor_pick.dynamic_axis_low_z_threshold_m;
    const int low_z_primary_axis = axis_candidates.empty() ? config_.target_sensor_pick.approach_axis_local
                                                           : axis_candidates.front();
    const std::size_t low_z_short_pre_count = std::min<std::size_t>(4u, pregrasp_candidates.size());
    std::vector<TargetSensorPickCandidate> candidates;
    for (std::size_t axis_rank = 0; axis_rank < axis_candidates.size(); ++axis_rank)
    {
      const int axis_local = axis_candidates[axis_rank];
      for (std::size_t pre_i = 0; pre_i < pregrasp_candidates.size(); ++pre_i)
      {
        const double pre_m = pregrasp_candidates[pre_i];
        for (double sign : sign_candidates)
        {
          int sign_rank = 0;
          for (std::size_t si = 0; si < sign_candidates.size(); ++si)
          {
            if (std::abs(sign_candidates[si] - sign) < 1e-6)
            {
              sign_rank = static_cast<int>(si);
              break;
            }
          }
          for (std::size_t roll_i = 0; roll_i < roll_candidates_deg.size(); ++roll_i)
          {
            const double roll_deg = roll_candidates_deg[roll_i];
            TargetSensorPickCandidate candidate;
            candidate.object_pose = pose_base;
            candidate.low_z_hard_priority_tier = 0;
            candidate.pregrasp_distance_m = pre_m;
            candidate.pregrasp_priority_rank = static_cast<int>(pre_i);
            candidate.approach_sign = sign;
            candidate.sign_priority_rank = sign_rank;
            candidate.tool_roll_deg = roll_deg;
            candidate.roll_priority_rank = static_cast<int>(roll_i);
            candidate.approach_axis_local = axis_local;
            candidate.axis_priority_rank = static_cast<int>(axis_rank);
            Eigen::Quaterniond q_object(
                pose_base.pose.orientation.w,
                pose_base.pose.orientation.x,
                pose_base.pose.orientation.y,
                pose_base.pose.orientation.z);
            if (q_object.norm() < 1e-9)
            {
              q_object = Eigen::Quaterniond::Identity();
            }
            q_object.normalize();
            Eigen::Vector3d local_axis_vec = Eigen::Vector3d::UnitZ();
            if (axis_local == 0)
            {
              local_axis_vec = Eigen::Vector3d::UnitX();
            }
            else if (axis_local == 1)
            {
              local_axis_vec = Eigen::Vector3d::UnitY();
            }
            const Eigen::Vector3d n_geom = (q_object * local_axis_vec).normalized();
            const Eigen::Vector3d approach_dir = n_geom * ((sign < 0.0) ? -1.0 : 1.0);
            const Eigen::Vector3d down_axis = -Eigen::Vector3d::UnitZ();
            candidate.down_priority_score = approach_dir.normalized().dot(down_axis);
            if (low_z_mode_enabled)
            {
              const bool is_short_pregrasp = pre_i < low_z_short_pre_count;
              const bool is_primary_axis = axis_local == low_z_primary_axis;
              if (is_short_pregrasp && is_primary_axis)
              {
                candidate.low_z_hard_priority_tier = 0;
              }
              else if (is_short_pregrasp)
              {
                candidate.low_z_hard_priority_tier = 1;
              }
              else if (is_primary_axis)
              {
                candidate.low_z_hard_priority_tier = 2;
              }
              else
              {
                candidate.low_z_hard_priority_tier = 3;
              }
            }
            std::ostringstream label;
            label << "low_z_tier=" << candidate.low_z_hard_priority_tier
                  << " axis=" << axis_local
                  << " pre=" << pre_m
                  << " sign=" << sign
                  << " roll=" << roll_deg
                  << " down_score=" << candidate.down_priority_score;
            candidate.label = label.str();
            candidates.push_back(candidate);
          }
        }
      }
    }

    if (candidates.empty())
    {
      setStateError("TARGET_SENSOR_PICK: no candidate");
      return false;
    }
    // 优先尝试“更接近竖直下压(-Z)”的候选；同分时保持原配置顺序。
    std::stable_sort(
        candidates.begin(),
        candidates.end(),
        [](const TargetSensorPickCandidate& lhs, const TargetSensorPickCandidate& rhs) {
          if (lhs.low_z_hard_priority_tier != rhs.low_z_hard_priority_tier)
          {
            return lhs.low_z_hard_priority_tier < rhs.low_z_hard_priority_tier;
          }
          if (lhs.axis_priority_rank != rhs.axis_priority_rank)
          {
            return lhs.axis_priority_rank < rhs.axis_priority_rank;
          }
          if (lhs.pregrasp_priority_rank != rhs.pregrasp_priority_rank)
          {
            return lhs.pregrasp_priority_rank < rhs.pregrasp_priority_rank;
          }
          if (lhs.roll_priority_rank != rhs.roll_priority_rank)
          {
            return lhs.roll_priority_rank < rhs.roll_priority_rank;
          }
          if (lhs.sign_priority_rank != rhs.sign_priority_rank)
          {
            return lhs.sign_priority_rank < rhs.sign_priority_rank;
          }
          return lhs.down_priority_score > rhs.down_priority_score;
        });
    if (low_z_mode_enabled)
    {
      RCLCPP_INFO(
          LOGGER,
          "handlePick: low-z hard priority enabled (primary_axis=%d, short_pre_count=%zu)",
          low_z_primary_axis,
          low_z_short_pre_count);
    }

    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
      const TargetSensorPickCandidate& candidate = candidates[i];
      Eigen::Isometry3d grasp_pose = Eigen::Isometry3d::Identity();
      if (!precheckTargetSensorPickCandidate(
              LOGGER, i, candidate, robot_model, scene_for_ik_seed,
              arm_group_name, hand_frame, config_.target_sensor_pick.grasp_depth_m,
              config_.target_sensor_pick.surface_backoff_m,
              grasp_pose))
      {
        continue;
      }

      mtc::Task task = pick_builder_->buildFromTargetSensorPose(
          candidate.object_pose, plan_frame, candidate.pregrasp_distance_m,
          candidate.approach_sign, candidate.tool_roll_deg);
      try
      {
        task.init();
        task.enableIntrospection(true);
        task.introspection().publishTaskDescription();
      }
      catch (mtc::InitStageException& e)
      {
        last_plan_detail = e.what();
        RCLCPP_WARN(LOGGER, "handlePick: targetsensor candidate %zu INIT_FAILED (%s): %s",
                    i, candidate.label.c_str(), e.what());
        continue;
      }

      moveit::core::MoveItErrorCode plan_result = task.plan(5);
      if (!plan_result || task.solutions().empty())
      {
        std::ostringstream os;
        task.explainFailure(os);
        last_plan_detail = os.str();
        RCLCPP_WARN(LOGGER, "handlePick: targetsensor candidate %zu PLAN_FAILED (%s): %s",
                    i, candidate.label.c_str(), last_plan_detail.c_str());
        continue;
      }

      moveit_task_constructor_msgs::msg::Solution solution_msg;
      task.solutions().front()->toMsg(solution_msg, &task.introspection());
      moveit::core::RobotModelConstPtr task_robot_model = task.getRobotModel();
      geometry_msgs::msg::Pose object_pose_at_grasp;
      isometryToPose(grasp_pose, object_pose_at_grasp);
      RCLCPP_INFO(LOGGER, "handlePick: targetsensor candidate %zu selected (%s)",
                  i, candidate.label.c_str());

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
      bool failed_no_grip = false;
      if (solution_executor_->executePickSolution(
              solution_msg, object_pose_at_grasp, held_id, task_robot_model, new_held, wait_for_gripped_fn_,
              stage_report, getCurrentJobId(), "PICK", PICK_STAGE_NAMES_TARGET_SENSOR, {},
              makeEstopAbortFn(), &failed_no_grip))
      {
        HeldObjectContext held_copy;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          held_object_ = new_held;
          held_copy = held_object_;
        }
        setState(RobotTaskMode::HOLDING_TRACKED);
        if (held_object_state_fn_)
        {
          held_object_state_fn_(held_copy);
        }
        RCLCPP_INFO(LOGGER, "handlePick: targetsensor pick success");
        return true;
      }

      if (estop_requested_.load())
      {
        setStateError("E_STOP");
        estop_requested_.store(false);
        return false;
      }
      if (failed_no_grip)
      {
        if (!retreatToReady())
        {
          setStateError("TARGET_SENSOR_PICK: NO_GRIP_AND_RETREAT_FAILED");
        }
        else
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          last_error_ = "TARGET_SENSOR_PICK: no grip, retreated to ready";
          task_mode_ = RobotTaskMode::IDLE;
        }
        return false;
      }
      RCLCPP_WARN(LOGGER, "handlePick: targetsensor candidate %zu EXECUTION_FAILED (%s), try next",
                  i, candidate.label.c_str());
    }

    if (last_plan_detail.empty())
    {
      last_plan_detail = "all candidates failed in precheck/plan/execution";
    }
    RCLCPP_ERROR(LOGGER, "handlePick: targetsensor all candidates failed: %s", last_plan_detail.c_str());
    setStateError("TARGET_SENSOR_PICK: PLAN_FAILED");
    return false;
  }

  geometry_msgs::msg::Vector3 axis_v;
  axis_v.x = 1.0;
  axis_v.y = 0.0;
  axis_v.z = 0.0;
  if (axis_stamped.has_value())
  {
    axis_v = axis_stamped->vector;
  }
  const bool has_axis_cb = (grasp_source == GraspSource::FUSED)
                               ? static_cast<bool>(get_latest_object_axis_fused_fn_)
                               : static_cast<bool>(get_latest_object_axis_legacy_fn_);
  if (!has_axis_cb && !axis_stamped.has_value())
  {
    RCLCPP_WARN(LOGGER, "handlePick: no object_axis callback for grasp_source=%s, need cable direction",
                graspSourceToCString(grasp_source));
  }

  const double axis_norm = std::sqrt(axis_v.x * axis_v.x + axis_v.y * axis_v.y + axis_v.z * axis_v.z);
  const bool direction_valid = axis_norm > 0.9;
  if (!direction_valid)
  {
    setStateError("CABLE_SIDE_GRASP: 缆绳轴向无效 (norm <= 0.9)，需提供 object_axis");
    return false;
  }

  /* 缆绳侧向包夹 + 分段碰撞：多候选按 score 排序；预检/规划失败可换下一候选；夹紧超时则回 ready 不再试 */
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
    std::vector<std::string> cable_world_ids;
    cable_world_ids.reserve(segments.size());
    for (const auto& seg : segments)
    {
      cable_world_ids.push_back(seg.id);
    }
    std::vector<CableGraspCandidate> candidates = generateCableSideGrasps(cable, config_.cable_grasp);
    const std::string plan_frame = "base_link";
    const std::string held_id = object_id.empty() ? "cable" : object_id;
    moveit::core::RobotModelConstPtr robot_model;
    moveit_msgs::msg::PlanningScene scene_base_msg;
    bool has_scene_base_msg = false;
    const std::string arm_group_name = "arm";
    const std::string hand_frame = "gripper_tcp";
    try
    {
      robot_model_loader::RobotModelLoader loader(node_);
      robot_model = loader.getModel();
      if (robot_model)
      {
        auto client = node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
        if (client->wait_for_service(std::chrono::milliseconds(300)))
        {
          auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
          req->components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
                                       moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                       moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
                                       moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
          auto fut = client->async_send_request(req);
          if (fut.wait_for(std::chrono::milliseconds(600)) == std::future_status::ready)
          {
            auto resp = fut.get();
            if (resp)
            {
              scene_base_msg = resp->scene;
              has_scene_base_msg = true;
            }
          }
        }
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN(LOGGER, "handlePick: pregrasp validator init failed: %s", e.what());
    }
    if (robot_model)
    {
      const std::string model_frame = robot_model->getModelFrame();
      RCLCPP_INFO(LOGGER,
                  "handlePick: MoveIt 模型根系=%s，规划/碰撞体使用 plan_frame=%s（应与机械臂 base_link 一致）",
                  model_frame.c_str(), plan_frame.c_str());
      if (model_frame != plan_frame)
      {
        RCLCPP_WARN(LOGGER,
                    "handlePick: 模型根系 %s 与 plan_frame %s 不同；若 URDF 为 world->base_link 则 base_link 与 world 同原点",
                    model_frame.c_str(), plan_frame.c_str());
      }
    }

    planning_scene::PlanningScenePtr scene_for_ik_seed;
    if (robot_model && has_scene_base_msg)
    {
      scene_for_ik_seed = std::make_shared<planning_scene::PlanningScene>(robot_model);
      scene_for_ik_seed->setPlanningSceneMsg(scene_base_msg);
    }

    if (candidates.empty())
    {
      RCLCPP_ERROR(LOGGER, "handlePick: 无侧抓候选");
      setStateError("CABLE_SIDE_GRASP: 无侧抓候选");
      return false;
    }

    for (std::size_t i = 0; i < candidates.size(); ++i)
    {
      if (estop_requested_.load())
      {
        setStateError("E_STOP");
        estop_requested_.store(false);
        return false;
      }
      const CableGraspCandidate& cand = candidates[i];
      const Eigen::Vector3d p_grasp = cand.grasp_pose.translation();
      const Eigen::Quaterniond q_grasp(cand.grasp_pose.linear());
      const Eigen::Vector3d p_pregrasp = cand.pregrasp_pose.translation();
      const Eigen::Quaterniond q_pregrasp(cand.pregrasp_pose.linear());

      RCLCPP_INFO(LOGGER,
                  "handlePick: candidate %zu grasp frame=%s pos=[%.4f %.4f %.4f] quat=[%.4f %.4f %.4f %.4f]",
                  i, plan_frame.c_str(), p_grasp.x(), p_grasp.y(), p_grasp.z(), q_grasp.x(), q_grasp.y(),
                  q_grasp.z(), q_grasp.w());
      RCLCPP_INFO(LOGGER,
                  "handlePick: candidate %zu pregrasp frame=%s pos=[%.4f %.4f %.4f] quat=[%.4f %.4f %.4f %.4f]",
                  i, plan_frame.c_str(), p_pregrasp.x(), p_pregrasp.y(), p_pregrasp.z(), q_pregrasp.x(),
                  q_pregrasp.y(), q_pregrasp.z(), q_pregrasp.w());

      CablePickFailReason pre_reason = CablePickFailReason::NO_IK;
      if (!precheckCableSideGraspCandidate(LOGGER, i, cand, robot_model, scene_for_ik_seed, has_scene_base_msg,
                                           scene_base_msg, segments, plan_frame, arm_group_name, hand_frame,
                                           &pre_reason))
      {
        RCLCPP_WARN(LOGGER, "handlePick: candidate %zu 预检失败 (%s)，试下一候选", i,
                    cablePickFailReasonTag(pre_reason));
        continue;
      }
      mtc::Task task = pick_builder_->buildFromCableCandidate(segments, cand, plan_frame);
      try
      {
        task.init();
        task.enableIntrospection(true);
        task.introspection().publishTaskDescription();
      }
      catch (mtc::InitStageException& e)
      {
        RCLCPP_WARN(LOGGER, "handlePick: candidate %zu INIT_FAILED: %s，试下一候选", i, e.what());
        continue;
      }
      moveit::core::MoveItErrorCode plan_result = task.plan(5);
      if (!plan_result || task.solutions().empty())
      {
        std::ostringstream os;
        task.explainFailure(os);
        RCLCPP_WARN(LOGGER, "handlePick: candidate %zu PLAN_FAILED: %s，试下一候选", i, os.str().c_str());
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
      bool failed_no_grip = false;
      if (solution_executor_->executePickSolution(
              solution_msg, object_pose_at_grasp,
              held_id,
              task.getRobotModel(), new_held, wait_for_gripped_fn_,
              stage_report, getCurrentJobId(), "PICK", PICK_STAGE_NAMES_CABLE_SIDE,
              cable_world_ids,
              makeEstopAbortFn(),
              &failed_no_grip))
      {
        /* setState()/getHeldObject() 均会再锁 state_mutex_，禁止在持锁区内调用 */
        HeldObjectContext held_copy;
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          held_object_ = new_held;
          held_copy = held_object_;
        }
        setState(RobotTaskMode::HOLDING_TRACKED);
        if (held_object_state_fn_)
        {
          held_object_state_fn_(held_copy);
        }
        RCLCPP_INFO(LOGGER, "handlePick: 缆绳侧抓候选 %zu 成功", i);
        return true;
      }
      if (estop_requested_.load())
      {
        setStateError("E_STOP");
        estop_requested_.store(false);
        return false;
      }
      if (failed_no_grip)
      {
        RCLCPP_WARN(LOGGER, "handlePick: 未检测到夹持，回 ready 并结束任务 (candidate %zu)", i);
        if (!retreatToReady())
        {
          setStateError("PICK: 未检测到夹持，且回 ready 失败");
        }
        else
        {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            last_error_ = "PICK: 未检测到夹持，已回 ready";
            task_mode_ = RobotTaskMode::IDLE;
          }
          RCLCPP_WARN(LOGGER, "handlePick: 已回 ready，任务结束");
        }
        return false;
      }
      RCLCPP_WARN(LOGGER, "handlePick: candidate %zu EXECUTION_FAILED，试下一候选", i);
    }
    RCLCPP_ERROR(LOGGER, "handlePick: 所有侧抓候选均失败（预检/规划/执行）");
    setStateError("CABLE_SIDE_GRASP: 所有侧抓候选均失败");
  }
  return false;
}

/*
 * Pilz PTP 回 SRDF 命名状态 "ready"，再插值闭合 hand group（不等待夹紧确认，避免空载超时）。
 * 用于抓取失败恢复、急停后整理姿态；返回 false 表示 init/plan/executeSolution 任一步失败。
 */
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
  auto stage_close = std::make_unique<mtc::stages::MoveTo>("close hand (ready)", interpolation_planner);
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
  // 就绪位闭合不做夹紧确认，避免空载时 wait_for_gripped 超时等待
  if (!solution_executor_->executeSolution(solution_msg, nullptr, nullptr, "", "", {},
                                          makeEstopAbortFn()))
  {
    RCLCPP_ERROR(LOGGER, "retreatToReady execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "retreatToReady finished");
  return true;
}

/*
 * 仅对手 group 规划并执行 MoveTo "open"：CurrentState 锁定当前臂关节（来自 joint_states），
 * 便于手柄/外部已动臂后只张开夹爪。成功则返回 true，内部经 solution_executor 可走 gripped 等待链（若配置）。
 */
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
  if (!solution_executor_->executeSolution(solution_msg, wait_for_gripped_fn_, nullptr, "", "", {},
                                          makeEstopAbortFn()))
  {
    RCLCPP_ERROR(LOGGER, "open gripper execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "open gripper finished");
  return true;
}

/*
 * /left_arm_gripped 话题回调：gripped_value >= 0.5 视为夹紧，直接返回；
 * PICKING 中忽略（避免抓取过程误判）；若当前语义持物且反馈为张开，则清 held_object、置 IDLE、
 *  detached planning scene 中 held_* / object，并通知 held_object_state 订阅方。
 */
void TaskManager::applyGripperFeedbackFromTopic(double gripped_value)
{
  static constexpr double k_locked_threshold = 0.5;
  const bool locked = gripped_value >= k_locked_threshold;
  if (locked)
  {
    return;
  }
  bool need_scene = false;
  bool need_notify = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (task_mode_ == RobotTaskMode::PICKING)
    {
      return;
    }
    if (suppress_ungripped_feedback_)
    {
      RCLCPP_WARN_THROTTLE(LOGGER, *node_->get_clock(), 1500,
                           "applyGripperFeedbackFromTopic: gripped=%.3f ignored while TARGET_INSERT executing",
                           gripped_value);
      return;
    }
    const bool semantically_holding = held_object_.valid || task_mode_ == RobotTaskMode::HOLDING_TRACKED ||
                                      task_mode_ == RobotTaskMode::HOLDING_UNTRACKED;
    if (!semantically_holding)
    {
      return;
    }
    clearHeldObject(held_object_);
    insert_latch_locked_ = false;
    insert_latch_hole_ = -1;
    task_mode_ = RobotTaskMode::IDLE;
    need_scene = true;
    need_notify = true;
    RCLCPP_INFO(LOGGER,
                "applyGripperFeedbackFromTopic: gripped=%.3f (unlocked), cleared held state from topic",
                gripped_value);
  }
  if (need_scene && scene_manager_)
  {
    scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
    scene_manager_->clearAttachedObjectFromPlanningScene("object");
  }
  if (need_notify && held_object_state_fn_)
  {
    held_object_state_fn_(getHeldObject());
  }
}

/*
 * 仅对手 group MoveTo "close"，臂关节保持 CurrentState；用于队列 CLOSE_GRIPPER 与服务触发路径。
 */
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
  if (!solution_executor_->executeSolution(solution_msg, wait_for_gripped_fn_, nullptr, "", "", {},
                                          makeEstopAbortFn()))
  {
    RCLCPP_ERROR(LOGGER, "close gripper execution failed");
    return false;
  }
  RCLCPP_INFO(LOGGER, "close gripper finished");
  return true;
}

/*
 * TargetSensor 插孔：由 InsertTaskBuilder 构图；插入轴为 target_pose 姿态的局部 -Z（竖直孔且单位姿态时等同原 -world Z）。
 */
bool TaskManager::handleTargetInsert(const geometry_msgs::msg::PoseStamped& target_pose, const std::string& object_id)
{
  if (estop_requested_.load())
  {
    setStateError("E_STOP");
    estop_requested_.store(false);
    return false;
  }
  const bool gripper_locked = (is_gripper_locked_fn_ && is_gripper_locked_fn_());
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!held_object_.valid && !isHolding(task_mode_) && !gripper_locked)
    {
      last_error_ = "TARGET_INSERT: no held object";
      RCLCPP_ERROR(LOGGER, "handleTargetInsert: no held object, reject");
      return false;
    }
    if (!held_object_.valid && !isHolding(task_mode_) && gripper_locked)
    {
      RCLCPP_WARN(LOGGER,
                  "handleTargetInsert: allow insert by gripper_locked without held context "
                  "(object may be manually attached)");
    }
    current_task_id_ = genTaskId("insert");
    suppress_ungripped_feedback_ = true;
  }
  struct InsertFeedbackGuard
  {
    TaskManager* self;
    ~InsertFeedbackGuard()
    {
      std::lock_guard<std::mutex> lock(self->state_mutex_);
      self->suppress_ungripped_feedback_ = false;
    }
  } insert_feedback_guard{ this };

  const bool insert_ok = [&]() -> bool
  {
    publishTargetInsertHoleDebug();

    geometry_msgs::msg::PoseStamped pose_base = target_pose;
    const rclcpp::Time stamp_now = node_->now();
    const int32_t requested_hole = parseTargetSlotFromObjectId(object_id);

    const std::string src_frame_before_xform = pose_base.header.frame_id;
    if (pose_base.header.frame_id != "base_link" && transform_to_base_link_fn_)
    {
      const rclcpp::Time stamp_latest_tf(0, 0, node_->get_clock()->get_clock_type());
      pose_base.header.stamp = stamp_latest_tf;
      if (!transform_to_base_link_fn_(pose_base, nullptr))
      {
        RCLCPP_ERROR(LOGGER, "handleTargetInsert: transform target_pose(%s)->base_link failed",
                     src_frame_before_xform.c_str());
        return false;
      }
    }
    else if (pose_base.header.frame_id != "base_link" && !transform_to_base_link_fn_)
    {
      RCLCPP_ERROR(LOGGER,
                   "handleTargetInsert: target_pose(%s) needs transform_to_base_link callback",
                   src_frame_before_xform.c_str());
      return false;
    }
    if (pose_base.header.frame_id != "base_link")
    {
      RCLCPP_ERROR(LOGGER, "handleTargetInsert: target_pose frame must be base_link (got %s)",
                   pose_base.header.frame_id.c_str());
      return false;
    }
    pose_base.header.stamp = stamp_now;
    RCLCPP_INFO(LOGGER,
                "handleTargetInsert: planning target frame=base_link pos=(%.4f, %.4f, %.4f) "
                "quat=(%.4f, %.4f, %.4f, %.4f) object_id=%s",
                pose_base.pose.position.x,
                pose_base.pose.position.y,
                pose_base.pose.position.z,
                pose_base.pose.orientation.x,
                pose_base.pose.orientation.y,
                pose_base.pose.orientation.z,
                pose_base.pose.orientation.w,
                object_id.c_str());

    {
      bool need_pre_extract = false;
      int32_t latched_hole = -1;
      {
        std::lock_guard<std::mutex> lock(state_mutex_);
        need_pre_extract = insert_latch_locked_;
        latched_hole = insert_latch_hole_;
      }
      if (need_pre_extract)
      {
        bool pre_extract_done = false;
        const PegInsertConfig& pi = config_.peg_insert;
        const Eigen::Vector3d axis_local = normalizedInsertAxisLocalFromConfig(pi);
        const Eigen::Quaterniond q_target(
            pose_base.pose.orientation.w,
            pose_base.pose.orientation.x,
            pose_base.pose.orientation.y,
            pose_base.pose.orientation.z);
        Eigen::Vector3d axis_world = q_target * axis_local;
        const double axis_norm = axis_world.norm();
        if (axis_norm < 1e-9)
        {
          RCLCPP_WARN(LOGGER, "handleTargetInsert: pre-extract skipped (invalid insert axis)");
        }
        else
        {
          axis_world /= axis_norm;
          const double extract_distance = std::max(0.0, pi.insert_depth_m);
          if (extract_distance > 1e-6)
          {
            mtc::Task extract_task;
            extract_task.stages()->setName("pre-extract reverse insert");
            extract_task.loadRobotModel(node_);
            const std::string arm_group_name = "arm";
            const std::string hand_frame = "gripper_tcp";
            extract_task.add(std::make_unique<mtc::stages::CurrentState>("current"));
            auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
            cartesian_planner->setMaxVelocityScalingFactor(static_cast<double>(pi.cartesian_velocity_scaling));
            cartesian_planner->setMaxAccelerationScalingFactor(static_cast<double>(pi.cartesian_acceleration_scaling));
            cartesian_planner->setStepSize(static_cast<double>(pi.cartesian_step_size));
            cartesian_planner->setMinFraction(0.90);
            geometry_msgs::msg::Vector3Stamped reverse_axis;
            reverse_axis.header.stamp = stamp_now;
            reverse_axis.header.frame_id = pose_base.header.frame_id;
            reverse_axis.vector.x = axis_world.x();
            reverse_axis.vector.y = axis_world.y();
            reverse_axis.vector.z = axis_world.z();
            auto pre_extract = std::make_unique<mtc::stages::MoveRelative>(
                "pre-extract reverse insert", cartesian_planner);
            pre_extract->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            pre_extract->setGroup(arm_group_name);
            pre_extract->setIKFrame(hand_frame);
            pre_extract->setDirection(reverse_axis);
            pre_extract->setMinMaxDistance(static_cast<float>(extract_distance), static_cast<float>(extract_distance));
            extract_task.add(std::move(pre_extract));
            try
            {
              extract_task.init();
              extract_task.enableIntrospection(true);
            }
            catch (mtc::InitStageException& e)
            {
              RCLCPP_WARN_STREAM(LOGGER, "handleTargetInsert: pre-extract init failed: " << e);
            }
            if (extract_task.solutions().empty())
            {
              const moveit::core::MoveItErrorCode extract_plan_result = extract_task.plan(3);
              if (!extract_plan_result || extract_task.solutions().empty())
              {
                std::ostringstream os;
                extract_task.explainFailure(os);
                RCLCPP_WARN_STREAM(LOGGER, "handleTargetInsert: pre-extract plan failed: " << os.str());
              }
              else
              {
                moveit_task_constructor_msgs::msg::Solution extract_solution_msg;
                extract_task.solutions().front()->toMsg(extract_solution_msg, &extract_task.introspection());
                if (!solution_executor_->executeSolution(
                        extract_solution_msg, wait_for_gripped_fn_, stage_report_fn_, current_task_id_,
                        "TARGET_INSERT", std::vector<std::string>{ "current", "pre-extract reverse insert" },
                        makeEstopAbortFn()))
                {
                  RCLCPP_WARN(LOGGER, "handleTargetInsert: pre-extract execution failed, continue insert");
                }
                else
                {
                  pre_extract_done = true;
                  RCLCPP_INFO(
                      LOGGER,
                      "handleTargetInsert: pre-extract completed (distance=%.3f, locked_hole=%d, target_hole=%d)",
                      extract_distance, static_cast<int>(latched_hole), static_cast<int>(requested_hole));
                }
              }
            }
          }
        }
        if (pre_extract_done)
        {
          std::lock_guard<std::mutex> lock(state_mutex_);
          insert_latch_locked_ = false;
          insert_latch_hole_ = -1;
        }
        else
        {
          RCLCPP_WARN(
              LOGGER,
              "handleTargetInsert: pre-extract not completed, keep insert latch lock "
              "(locked_hole=%d, target_hole=%d)",
              static_cast<int>(latched_hole), static_cast<int>(requested_hole));
        }
      }
    }

    moveit_task_constructor_msgs::msg::Solution solution_msg;
    std::vector<std::string> insert_stage_names;
    auto try_plan_with_pose = [&](const geometry_msgs::msg::PoseStamped& pose_try,
                                  const std::string& plan_tag) -> bool
    {
      InsertTaskBuildResult built_local = insert_builder_->buildTargetInsertTask(pose_try);
      mtc::Task& task_local = built_local.task;
      try
      {
        task_local.init();
        task_local.enableIntrospection(true);
      }
      catch (mtc::InitStageException& e)
      {
        RCLCPP_ERROR_STREAM(LOGGER, "target insert init failed (" << plan_tag << "): " << e);
        return false;
      }
      moveit::core::MoveItErrorCode plan_result_local = task_local.plan(5);
      if (!plan_result_local || task_local.solutions().empty())
      {
        std::ostringstream os;
        task_local.explainFailure(os);
        RCLCPP_WARN_STREAM(LOGGER, "target insert plan failed (" << plan_tag << "): " << os.str());
        return false;
      }
      task_local.solutions().front()->toMsg(solution_msg, &task_local.introspection());
      insert_stage_names = built_local.stage_names;
      return true;
    };

    bool planned = try_plan_with_pose(pose_base, "nominal");
    if (!planned)
    {
      const Eigen::Vector3d axis_local = normalizedInsertAxisLocalFromConfig(config_.peg_insert);
      const Eigen::Quaterniond q_nominal(pose_base.pose.orientation.w,
                                         pose_base.pose.orientation.x,
                                         pose_base.pose.orientation.y,
                                         pose_base.pose.orientation.z);
      Eigen::Vector3d axis_world = q_nominal * axis_local;
      const double axis_norm = axis_world.norm();
      if (axis_norm > 1e-9)
      {
        axis_world /= axis_norm;
        // 圆柱体持物对绕插入轴旋转不敏感：扩展姿态回退候选以提高插孔成功率。
        const std::vector<std::pair<double, std::string>> rotation_candidates = {
          { M_PI, "flip_180_about_insert_axis" },
          { M_PI_2, "flip_90_about_insert_axis" },
          { -M_PI_2, "flip_minus_90_about_insert_axis" },
          { M_PI / 6.0, "roll_30_about_insert_axis" },
          { -M_PI / 6.0, "roll_minus_30_about_insert_axis" },
          { M_PI / 4.0, "roll_45_about_insert_axis" },
          { -M_PI / 4.0, "roll_minus_45_about_insert_axis" },
          { M_PI / 3.0, "roll_60_about_insert_axis" },
          { -M_PI / 3.0, "roll_minus_60_about_insert_axis" },
          { 2.0 * M_PI / 3.0, "roll_120_about_insert_axis" },
          { -2.0 * M_PI / 3.0, "roll_minus_120_about_insert_axis" },
          { 5.0 * M_PI / 6.0, "roll_150_about_insert_axis" },
          { -5.0 * M_PI / 6.0, "roll_minus_150_about_insert_axis" },
        };
        for (const auto& candidate : rotation_candidates)
        {
          const Eigen::Quaterniond q_rot(Eigen::AngleAxisd(candidate.first, axis_world));
          const Eigen::Quaterniond q_try = q_rot * q_nominal;
          geometry_msgs::msg::PoseStamped pose_try = pose_base;
          pose_try.pose.orientation.x = q_try.x();
          pose_try.pose.orientation.y = q_try.y();
          pose_try.pose.orientation.z = q_try.z();
          pose_try.pose.orientation.w = q_try.w();
          planned = try_plan_with_pose(pose_try, candidate.second);
          if (planned)
          {
            RCLCPP_INFO(LOGGER, "handleTargetInsert: planned with orientation fallback %s",
                        candidate.second.c_str());
            break;
          }
        }
      }
    }
    if (!planned)
    {
      RCLCPP_ERROR(LOGGER, "target insert plan failed after orientation fallback");
      return false;
    }

    std::atomic<bool> insert_latch_hit(false);
    std::atomic<bool> stop_latch_monitor(false);
    std::atomic<bool> latch_sampling_enabled(false);
    std::thread latch_monitor_thread;
    static constexpr int k_latch_consecutive_hit_min = 2;
    const auto is_insert_descend_stage = [](const std::string& stage_name) -> bool
    {
      return stage_name == "insert descend" || stage_name.rfind("insert descend segment ", 0) == 0;
    };
    const auto has_any_latch_hit = [&]() -> bool
    {
      if (!get_latest_target_set_fn_)
      {
        return false;
      }
      const std::optional<orion_mtc_msgs::msg::TargetSet> latest = get_latest_target_set_fn_();
      if (!latest.has_value())
      {
        return false;
      }
      for (float v : latest->latches)
      {
        if (std::isfinite(static_cast<double>(v)) && v > 0.5f)
        {
          return true;
        }
      }
      return false;
    };
    const auto mark_insert_latch_locked = [&]()
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      insert_latch_locked_ = true;
      insert_latch_hole_ = requested_hole;
    };
    if (get_latest_target_set_fn_)
    {
      latch_monitor_thread = std::thread([&]()
      {
        int consecutive_hit_count = 0;
        while (!stop_latch_monitor.load())
        {
          if (!insert_latch_hit.load() && latch_sampling_enabled.load())
          {
            if (has_any_latch_hit())
            {
              consecutive_hit_count += 1;
              if (consecutive_hit_count >= k_latch_consecutive_hit_min)
              {
                insert_latch_hit.store(true);
                mark_insert_latch_locked();
                RCLCPP_INFO(LOGGER,
                            "handleTargetInsert: latch detected by monitor during insert descend, "
                            "remaining descend segments will be skipped");
              }
            }
            else
            {
              consecutive_hit_count = 0;
            }
          }
          else
          {
            consecutive_hit_count = 0;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(30));
        }
      });
    }
    struct LatchMonitorGuard
    {
      std::atomic<bool>* stop_flag;
      std::thread* worker;
      ~LatchMonitorGuard()
      {
        if (stop_flag != nullptr)
        {
          stop_flag->store(true);
        }
        if (worker != nullptr && worker->joinable())
        {
          worker->join();
        }
      }
    } latch_monitor_guard{ &stop_latch_monitor, &latch_monitor_thread };

    if (!solution_executor_->executeSolution(
            solution_msg, wait_for_gripped_fn_, stage_report_fn_, current_task_id_, "TARGET_INSERT",
            insert_stage_names, makeEstopAbortFn(),
            [&](std::size_t /*stage_index*/, const std::string& stage_name) -> bool
            {
              const bool is_descend_stage = is_insert_descend_stage(stage_name);
              latch_sampling_enabled.store(is_descend_stage);
              if (insert_latch_hit.load() && is_insert_descend_stage(stage_name))
              {
                return false;
              }
              return true;
            },
            [&](std::size_t /*stage_index*/, const std::string& stage_name)
            {
              const bool is_descend_stage = is_insert_descend_stage(stage_name);
              latch_sampling_enabled.store(false);
              if (!insert_latch_hit.load() && is_descend_stage && has_any_latch_hit())
              {
                RCLCPP_INFO(LOGGER,
                            "handleTargetInsert: latch transient observed after descend stage, "
                            "wait monitor debounce to confirm");
              }
            }))
    {
      RCLCPP_ERROR(LOGGER, "target insert execution failed");
      return false;
    }

    if (get_latest_target_set_fn_)
    {
      bool latch_ok = insert_latch_hit.load();
      const int max_retry = 50;
      float last_latch_max_value = 0.0f;
      bool has_latch_value = false;
      if (!latch_ok)
      {
        for (int i = 0; i < max_retry; ++i)
        {
          const std::optional<orion_mtc_msgs::msg::TargetSet> latest = get_latest_target_set_fn_();
          if (!latest.has_value())
          {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
          }
          for (float latch_value : latest->latches)
          {
            if (!std::isfinite(static_cast<double>(latch_value)))
            {
              continue;
            }
            has_latch_value = true;
            last_latch_max_value = std::max(last_latch_max_value, latch_value);
            if (latch_value > 0.5f)
            {
              latch_ok = true;
              break;
            }
          }
          if (latch_ok)
          {
            break;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
      }
      if (!latch_ok)
      {
        if (has_latch_value)
        {
          RCLCPP_WARN(LOGGER,
                      "handleTargetInsert: latch check failed (object_id=%s, latch_max=%.3f), "
                      "keep previous latch lock state and continue by trajectory result",
                      object_id.c_str(), static_cast<double>(last_latch_max_value));
        }
        else
        {
          RCLCPP_WARN(LOGGER,
                      "handleTargetInsert: latch check failed (object_id=%s, no latch sample), "
                      "keep previous latch lock state and continue by trajectory result",
                      object_id.c_str());
        }
      }
      else
      {
        mark_insert_latch_locked();
        RCLCPP_INFO(LOGGER, "handleTargetInsert: latch check passed (object_id=%s)", object_id.c_str());
      }
    }
    else
    {
      RCLCPP_WARN(LOGGER, "handleTargetInsert: target_set callback not configured, skip latch check");
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      clearHeldObject(held_object_);
      task_mode_ = RobotTaskMode::IDLE;
      last_error_.clear();
    }
    if (scene_manager_)
    {
      scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
      scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
    }
    if (held_object_state_fn_)
    {
      held_object_state_fn_(getHeldObject());
    }
    RCLCPP_INFO(LOGGER, "handleTargetInsert: finished, released object_id=%s", object_id.c_str());
    return true;
  }();

  if (!retreatToReady())
  {
    RCLCPP_ERROR(LOGGER, "handleTargetInsert: failed to return ready after insert flow");
    return false;
  }
  return insert_ok;
}

/* 外部注入「夹爪是否已有物」谓词；为 true 时 handlePick 拒绝，防止重复规划。 */
void TaskManager::setGripperLockedCallback(std::function<bool()> fn)
{
  is_gripper_locked_fn_ = std::move(fn);
}

/* Worker 执行 PICK 时通过此回调取话题最新 object_pose；即时 handlePick 不使用。 */
void TaskManager::setGetLatestObjectPoseCallback(
    std::function<std::optional<geometry_msgs::msg::PoseStamped>()> fn)
{
  get_latest_object_pose_legacy_fn_ = std::move(fn);
}

/* 提供缆绳轴向 Vector3Stamped；handlePick 与 TF 回调配合 transform_to_base_link_fn_ 使用。 */
void TaskManager::setGetLatestObjectAxisCallback(
    std::function<std::optional<geometry_msgs::msg::Vector3Stamped>()> fn)
{
  get_latest_object_axis_legacy_fn_ = std::move(fn);
}

void TaskManager::setGraspChainPerceptionCallbacks(
    std::function<std::optional<geometry_msgs::msg::PoseStamped>()> legacy_pose,
    std::function<std::optional<geometry_msgs::msg::Vector3Stamped>()> legacy_axis,
    std::function<std::optional<geometry_msgs::msg::PoseStamped>()> fused_pose,
    std::function<std::optional<geometry_msgs::msg::Vector3Stamped>()> fused_axis)
{
  get_latest_object_pose_legacy_fn_ = std::move(legacy_pose);
  get_latest_object_axis_legacy_fn_ = std::move(legacy_axis);
  get_latest_object_pose_fused_fn_ = std::move(fused_pose);
  get_latest_object_axis_fused_fn_ = std::move(fused_axis);
}

/* 将 PoseStamped（及可选 Vector3Stamped 轴）从感知系变换到 base_link；失败则保留原 frame_id 并打 WARN。 */
void TaskManager::setTransformToBaseLinkCallback(TransformToBaseLinkFn fn)
{
  transform_to_base_link_fn_ = std::move(fn);
  publishTargetInsertHoleDebug();
}

std::vector<geometry_msgs::msg::PoseStamped> TaskManager::collectTargetInsertHolePosesBaseLink(bool emit_log) const
{
  (void)emit_log;
  std::vector<geometry_msgs::msg::PoseStamped> out;
  return out;
}

void TaskManager::publishTargetInsertHoleDebug()
{
  if (!target_insert_holes_pub_ && !target_insert_hole_markers_pub_)
  {
    return;
  }
  const auto holes = collectTargetInsertHolePosesBaseLink(false);
  geometry_msgs::msg::PoseArray pose_array;
  pose_array.header.frame_id = "base_link";
  pose_array.header.stamp = node_->now();
  pose_array.poses.reserve(holes.size());
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker delete_all;
  delete_all.header = pose_array.header;
  delete_all.ns = "target_insert_holes";
  delete_all.id = 0;
  delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
  markers.markers.push_back(delete_all);
  int32_t marker_id = 1;
  for (std::size_t i = 0; i < holes.size(); ++i)
  {
    pose_array.poses.push_back(holes[i].pose);
    visualization_msgs::msg::Marker m;
    m.header = pose_array.header;
    m.ns = "target_insert_holes";
    m.id = marker_id++;
    m.type = visualization_msgs::msg::Marker::CYLINDER;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = holes[i].pose;
    m.scale.x = TARGET_INSERT_HOLE_DIAMETER_M;
    m.scale.y = TARGET_INSERT_HOLE_DIAMETER_M;
    m.scale.z = TARGET_INSERT_HOLE_MARKER_THICKNESS_M;
    m.color.r = 0.10f;
    m.color.g = 0.75f;
    m.color.b = 0.95f;
    m.color.a = 0.55f;
    markers.markers.push_back(m);
  }
  if (target_insert_holes_pub_)
  {
    target_insert_holes_pub_->publish(pose_array);
  }
  if (target_insert_hole_markers_pub_)
  {
    target_insert_hole_markers_pub_->publish(markers);
  }
}

void TaskManager::setGetLatestTargetSetCallback(
    std::function<std::optional<orion_mtc_msgs::msg::TargetSet>()> fn)
{
  get_latest_target_set_fn_ = std::move(fn);
}

/* 任务生命周期事件（SUBMITTED/STARTED/SUCCEEDED/FAILED/REJECTED/CANCELLED）发布到 /manipulator/job_event。 */
void TaskManager::setJobEventCallback(JobEventFn fn)
{
  job_event_fn_ = std::move(fn);
}

/* 持物上下文变化时回调，用于 /manipulator/held_object_state。 */
void TaskManager::setHeldObjectStateCallback(HeldObjectStateFn fn)
{
  held_object_state_fn_ = std::move(fn);
}

/* 自动恢复各子步骤（清 scene、reset held、go_home）的诊断事件。 */
void TaskManager::setRecoveryEventCallback(RecoveryEventFn fn)
{
  recovery_event_fn_ = std::move(fn);
}

/* MTC 执行过程中按阶段名上报，供 /manipulator/task_stage 与前端展示。 */
void TaskManager::setStageReportCallback(StageReportFn fn)
{
  stage_report_fn_ = std::move(fn);
}

/* 窥视队列队首 job 的类型字符串（不入队）；空队列返回 ""。 */
std::string TaskManager::getNextJobType() const
{
  ManipulationJob front;
  if (!queue_ || !queue_->peekFront(front))
  {
    return "";
  }
  return jobTypeToCString(front.type);
}

/*
 * 持物状态与 planning scene 同步：仅在 IDLE/ERROR 下允许。
 * tracked=true 时需 object_pose 与 tcp_pose，计算 tcp_to_object，挂 held_tracked 并清 world "object"；
 * untracked 挂 held_unknown。set_holding=false 时直接返回成功且不修改（兼容服务语义）。
 */
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
      held_object_.attach_link = "gripper_tcp";
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
      held_object_.attach_link = "gripper_tcp";
      held_object_.tcp_to_object = Eigen::Isometry3d::Identity();
      task_mode_ = RobotTaskMode::HOLDING_UNTRACKED;
      out_message = "HOLDING_UNTRACKED (use reset_held_object or open_gripper to clear)";
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

/*
 * 清除内部 held_object、置 IDLE；非 PICKING 忙碌态才可调用。
 * 同时从 planning scene 移除 held_unknown / held_tracked / object 附着与世界物体。
 */
bool TaskManager::handleResetHeldObject(std::string& out_message)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (task_mode_ != RobotTaskMode::HOLDING_TRACKED && task_mode_ != RobotTaskMode::HOLDING_UNTRACKED &&
        task_mode_ != RobotTaskMode::IDLE && task_mode_ != RobotTaskMode::ERROR)
    {
      out_message = "reset_held_object: busy (PICKING), reject";
      RCLCPP_WARN(LOGGER, "%s", out_message.c_str());
      return false;
    }
    held_object_.valid = false;
    insert_latch_locked_ = false;
    insert_latch_hole_ = -1;
    task_mode_ = RobotTaskMode::IDLE;
    out_message = "held object cleared, state=IDLE";
    RCLCPP_INFO(LOGGER, "reset_held_object: %s", out_message.c_str());
  }
  if (scene_manager_)
  {
    scene_manager_->clearAttachedObjectFromPlanningScene("held_unknown");
    scene_manager_->clearAttachedObjectFromPlanningScene("held_tracked");
    scene_manager_->clearAttachedObjectFromPlanningScene("object");
    scene_manager_->removeWorldObject(TARGET_SENSOR_PEG_COLLISION_ID);
  }
  if (held_object_state_fn_)
  {
    held_object_state_fn_(getHeldObject());
  }
  return true;
}

/* 线程安全读取当前业务态（IDLE/PICKING/HOLDING_/ERROR）。 */
RobotTaskMode TaskManager::getMode() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return task_mode_;
}

/* 当前同步抓取流程的 task id（如 pick_xxx）；队列 job id 另见 worker 字段。 */
std::string TaskManager::getTaskId() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return current_task_id_;
}

/* 最近 setStateError 或业务路径写入的人类可读错误摘要。 */
std::string TaskManager::getLastError() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return last_error_;
}

/* 拷贝返回持物上下文（锁内复制），供服务查询与状态发布。 */
HeldObjectContext TaskManager::getHeldObject() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return held_object_;
}

bool TaskManager::isInsertLatchLocked() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return insert_latch_locked_;
}

int32_t TaskManager::getInsertLatchHole() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return insert_latch_hole_;
}

/*
 * 异步入队：填充 job_id（若空）、默认 priority、created_at_ns；
 * 检查 reject_new_jobs_while_busy、JobDeduplicator 去重；通过则 push 队列并发 SUBMITTED 事件。
 * 拒绝时返回空串并可选写 out_reject_reason。
 */
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
  if (j.type == JobType::PICK && j.priority > 50)
  {
    RCLCPP_WARN(LOGGER, "submitJob: PICK priority=%d > 50 (may outrank recovery jobs), consider lower value",
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

  if (j.type == JobType::PICK && feasibility_checker_ != nullptr &&
      feasibility_checker_->inJoyModeSwitchIkCooldown())
  {
    const char* reason = "joy_mode_switch_ik_cooldown";
    RCLCPP_WARN(LOGGER, "submitJob rejected: PICK reason=%s", reason);
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

  std::string reject_reason;
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (job_deduplicator_.isDuplicate(j, now_ns, worker_status_, current_job_type_,
                                        current_job_has_pose_, current_job_target_pose_, current_job_grasp_source_,
                                        last_accepted_type_,
                                        last_accepted_has_pose_, last_accepted_pose_, last_accepted_grasp_source_,
                                        last_accepted_time_ns_,
                                        &reject_reason))
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
    last_accepted_has_pose_ = JobDeduplicator::getJobPoseForDedup(j, &last_accepted_pose_);
    last_accepted_time_ns_ = j.created_at_ns;
    if (j.type == JobType::PICK)
    {
      last_accepted_grasp_source_ = j.grasp_source;
    }
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

/* 启动后台 worker 线程执行 workerLoop；重复调用打 WARN 且不增线程。 */
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

/* 置 worker_running_ false、join 线程、清 current_job_* 并将 worker_status_ 置 STOPPED。 */
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
    current_job_grasp_source_ = GraspSource::LEGACY;
  }
  RCLCPP_INFO(LOGGER, "stopWorker: stopped");
}

/* 原子读 worker 线程是否在跑（start 后至 exit 前为 true）。 */
bool TaskManager::isWorkerRunning() const
{
  return worker_running_.load();
}

/* Worker 细粒度状态：IDLE/RUNNING_JOB/RECOVERING/ERROR/STOPPED 等，供 get_queue_state。 */
WorkerStatus TaskManager::getWorkerStatus() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return worker_status_;
}

/* Worker 正在执行的 job_id；无任务时为空串。 */
std::string TaskManager::getCurrentJobId() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return current_job_id_;
}

/* 当前执行 job 类型字符串；空时为 "NONE"。 */
std::string TaskManager::getCurrentJobType() const
{
  std::lock_guard<std::mutex> lock(worker_mutex_);
  return current_job_type_.empty() ? "NONE" : current_job_type_;
}

/* 返回共享的任务队列指针（主要用于测试或扩展；正常使用经 submitJob）。 */
std::shared_ptr<TaskQueue> TaskManager::getQueue()
{
  return queue_;
}

/* 更新运行时策略（失败是否自动恢复、是否拒绝忙时入队等），与 MTCConfig 独立。 */
void TaskManager::setPolicy(const RuntimePolicy& policy)
{
  policy_ = policy;
}

/* 只读引用当前 RuntimePolicy。 */
const RuntimePolicy& TaskManager::getPolicy() const
{
  return policy_;
}

/*
 * 取消队列中尚未开始执行的 job；正在执行的 job_id 不可取消。
 * 成功则从队列移除并写 CANCELLED 执行记录与 job_event。
 */
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

/* 环形 recent_records_ 追加一条 STARTED 记录（result 仍为 UNKNOWN，结束由 updateExecutionRecordFinish 写）。 */
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

/* 将最近一条执行记录标记结束：写 result_code、message、finished_at_ns。 */
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

/* 队列入队取消时追加一条独立记录（started_at=0，result=CANCELLED）。 */
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

/* 按时间逆序（最近在前）返回最多 max_count 条执行记录副本。 */
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

/*
 * Worker 主循环：waitPop 任务 → 置 RUNNING_JOB、发 STARTED → executeJob →
 * 成功则 IDLE，失败则 ERROR 并可触发 auto_reset（resetHeld、clearScene、goHome）与 recovery_event。
 */
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
      current_job_has_pose_ = JobDeduplicator::getJobPoseForDedup(job, &current_job_target_pose_);
      current_job_grasp_source_ =
          (job.type == JobType::PICK) ? job.grasp_source : GraspSource::LEGACY;
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
      current_job_grasp_source_ = GraspSource::LEGACY;
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
        /* resetHeldState 已包含场景清理，避免重复 clearSceneResiduals 触发对象不存在噪声。 */
        if (recovery_event_fn_)
        {
          recovery_event_fn_("CLEAR_SCENE", trigger, true, "skipped (included in reset_held)");
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
          recovery_event_fn_("AUTO_RECOVERY", trigger, r1 && r3,
                             "reset_held(includes clear_scene)+go_home");
        }
        std::lock_guard<std::mutex> lock(worker_mutex_);
        worker_status_ = WorkerStatus::IDLE;
      }
    }
  }
}

/* 按 JobType 分派：PICK 优先用 submit_job 传入 object_pose，否则回落最新话题；RESET/OPEN/CLOSE/SYNC 调对应 handle 系列。 */
bool TaskManager::executeJob(const ManipulationJob& job)
{
  switch (job.type)
  {
    case JobType::PICK:
    {
      std::optional<geometry_msgs::msg::PoseStamped> pick_pose;
      if (job.object_pose.has_value())
      {
        pick_pose = job.object_pose.value();
        RCLCPP_INFO(LOGGER,
                    "executeJob PICK: using submit_job object_pose frame_id=%s x=%.3f y=%.3f z=%.3f grasp_source=%s",
                    pick_pose->header.frame_id.c_str(),
                    pick_pose->pose.position.x, pick_pose->pose.position.y, pick_pose->pose.position.z,
                    graspSourceToCString(job.grasp_source));
      }
      else
      {
        std::function<std::optional<geometry_msgs::msg::PoseStamped>()> pose_fn;
        if (job.grasp_source == GraspSource::FUSED)
        {
          pose_fn = get_latest_object_pose_fused_fn_;
        }
        else
        {
          pose_fn = get_latest_object_pose_legacy_fn_;
        }
        if (!pose_fn)
        {
          RCLCPP_ERROR(LOGGER, "executeJob PICK: no object_pose callback for grasp_source=%s, cannot plan",
                       graspSourceToCString(job.grasp_source));
          return false;
        }
        pick_pose = pose_fn();
        if (!pick_pose.has_value())
        {
          RCLCPP_ERROR(LOGGER,
                       "executeJob PICK: no object_pose from topic for grasp_source=%s (no target), skip plan",
                       graspSourceToCString(job.grasp_source));
          return false;
        }
      }

      /* 缆绳侧向抓取：object_pose/object_axis 由对应链话题提供，规划时用侧抓候选。 */
      double lx = pick_pose->pose.position.x;
      double ly = pick_pose->pose.position.y;
      double lz = pick_pose->pose.position.z;
      RCLCPP_INFO(LOGGER,
                  "executeJob PICK: grasp_source=%s 目标点 frame_id=%s x=%.3f y=%.3f z=%.3f "
                  "(缆绳中心，approach 后 gripper_tcp 将到此点)",
                  graspSourceToCString(job.grasp_source), pick_pose->header.frame_id.c_str(), lx, ly, lz);
      if (std::abs(lx) < 1e-6 && std::abs(ly) < 1e-6 && std::abs(lz) < 1e-6)
      {
        RCLCPP_WARN(LOGGER, "executeJob PICK: 目标点接近 (0,0,0)，请检查对应抓取链是否发布有效 object_pose");
      }
      return handlePick(pick_pose.value(), job.object_id, job.grasp_source);
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
    case JobType::TARGET_INSERT:
    {
      if (!job.target_pose.has_value())
      {
        RCLCPP_ERROR(LOGGER, "executeJob TARGET_INSERT: target_pose required");
        return false;
      }
      RCLCPP_INFO(LOGGER,
                  "executeJob TARGET_INSERT: submit target frame=%s pos=(%.4f, %.4f, %.4f) "
                  "quat=(%.4f, %.4f, %.4f, %.4f) object_id=%s",
                  job.target_pose->header.frame_id.c_str(),
                  job.target_pose->pose.position.x,
                  job.target_pose->pose.position.y,
                  job.target_pose->pose.position.z,
                  job.target_pose->pose.orientation.x,
                  job.target_pose->pose.orientation.y,
                  job.target_pose->pose.orientation.z,
                  job.target_pose->pose.orientation.w,
                  job.object_id.c_str());
      return handleTargetInsert(job.target_pose.value(), job.object_id);
    }
    default:
      RCLCPP_ERROR(LOGGER, "executeJob: unknown type %d", static_cast<int>(job.type));
      return false;
  }
}

/* 返回捕获 estop_requested_ 的原子谓词，传入 executePickSolution/executeSolution 以在急停时打断分段执行。 */
std::function<bool()> TaskManager::makeEstopAbortFn() const
{
  return [this]() { return estop_requested_.load(); };
}

/*
 * 急停：置 estop 标志、取消当前 FollowJointTrajectory、清空任务队列（不自动清 latch）。
 * 执行循环内通过 makeEstopAbortFn 感知并中止。
 */
void TaskManager::requestEmergencyStop()
{
  estop_requested_.store(true);
  if (trajectory_executor_)
  {
    trajectory_executor_->cancelOngoingGoals();
  }
  if (queue_)
  {
    queue_->clear();
  }
  RCLCPP_WARN(LOGGER, "requestEmergencyStop: E_STOP, queue cleared, trajectory cancel requested");
}

/* 仅清除 estop_requested_ 原子位，不恢复已清空的队列；供手柄 clear_estop 等。 */
void TaskManager::clearEmergencyStopLatch()
{
  estop_requested_.store(false);
  RCLCPP_INFO(LOGGER, "clearEmergencyStopLatch: estop_requested cleared");
}

/*
 * 服务 go_to_ready：禁止在 PICKING 或 worker RUNNING_JOB 时调用；否则执行 retreatToReady()，
 * 成功则清 last_error、置 IDLE；失败写 out_message。
 */
bool TaskManager::tryGoToReady(std::string& out_message)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (task_mode_ == RobotTaskMode::PICKING)
    {
      out_message = "busy: pick in progress";
      return false;
    }
  }
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_status_ == WorkerStatus::RUNNING_JOB)
    {
      out_message = "busy: worker executing job";
      return false;
    }
  }
  RCLCPP_INFO(LOGGER, "tryGoToReady: start");
  if (!retreatToReady())
  {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      out_message = last_error_.empty() ? "go_to_ready failed" : last_error_;
    }
    return false;
  }
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_error_.clear();
  }
  setState(RobotTaskMode::IDLE);
  out_message = "ok";
  return true;
}

}  // namespace orion_mtc
