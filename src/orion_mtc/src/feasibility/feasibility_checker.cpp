/* 审批模式：几何 + IK + 关节余量，不执行规划 */

#include "orion_mtc/feasibility/feasibility_checker.hpp"
#include "orion_mtc/config/mtc_config.hpp"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <mutex>

namespace orion_mtc
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("orion_mtc.feasibility");

/* 诊断 level：0=info 1=warning 2=error(硬拒绝) */
static const int32_t LEVEL_INFO = 0;
static const int32_t LEVEL_WARNING = 1;
static const int32_t LEVEL_ERROR = 2;

/* severity：0=pass 1=pass_with_warning 2=reject */
static const int32_t SEV_PASS = 0;
static const int32_t SEV_WARNING = 1;
static const int32_t SEV_REJECT = 2;

struct FeasibilityChecker::Impl
{
  rclcpp::Node::SharedPtr node_holder;
  robot_model_loader::RobotModelLoaderPtr loader;
  moveit::core::RobotModelConstPtr robot_model;
  std::string arm_group_name = "arm";
  std::string hand_frame = "Link6";
  std::mutex joint_state_mutex;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_joint_state;
};

FeasibilityChecker::FeasibilityChecker(rclcpp::Node::SharedPtr node)
  : node_(node), impl_(std::make_unique<Impl>())
{
  if (!node_)
  {
    return;
  }
  impl_->node_holder = node_;
  loadParams();
  try
  {
    impl_->loader = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
    impl_->robot_model = impl_->loader->getModel();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "FeasibilityChecker: RobotModelLoader failed: %s", e.what());
    impl_->robot_model.reset();
  }
  std::string joint_state_topic = "joint_states";
  node_->declare_parameter<std::string>("feasibility.joint_state_topic", joint_state_topic);
  node_->get_parameter("feasibility.joint_state_topic", joint_state_topic);
  impl_->sub_joint_state = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(impl_->joint_state_mutex);
        impl_->last_joint_state = msg;
      });
}

FeasibilityChecker::~FeasibilityChecker() = default;

void FeasibilityChecker::setMTCConfig(const MTCConfig* config)
{
  mtc_config_ = config;
}

void FeasibilityChecker::loadParams()
{
  if (!node_)
    return;
  auto decl = [this](const char* name, double def) {
    if (!node_->has_parameter(name))
      node_->declare_parameter<double>(name, def);
  };
  decl("feasibility.max_reach_hard", params_.max_reach_hard);
  decl("feasibility.max_reach_soft", params_.max_reach_soft);
  decl("feasibility.min_reach_safe", params_.min_reach_safe);
  decl("feasibility.z_min", params_.z_min);
  decl("feasibility.z_max", params_.z_max);
  decl("feasibility.joint_margin_warning_rad", params_.joint_margin_warning_rad);
  decl("feasibility.ik_timeout", params_.ik_timeout);
  decl("feasibility.approach_angle_max_deg", params_.approach_angle_max_deg);
  decl("feasibility.suggestion_perturb_xy", params_.suggestion_perturb_xy);
  decl("feasibility.suggestion_perturb_z", params_.suggestion_perturb_z);
  decl("feasibility.suggestion_perturb_yaw_rad", params_.suggestion_perturb_yaw_rad);

  auto get_or = [this](const char* name, double& out, double def) {
    if (node_->has_parameter(name))
      node_->get_parameter(name, out);
    else
      out = def;
  };
  get_or("feasibility.max_reach_hard", params_.max_reach_hard, params_.max_reach_hard);
  get_or("feasibility.max_reach_soft", params_.max_reach_soft, params_.max_reach_soft);
  get_or("feasibility.min_reach_safe", params_.min_reach_safe, params_.min_reach_safe);
  get_or("feasibility.z_min", params_.z_min, params_.z_min);
  get_or("feasibility.z_max", params_.z_max, params_.z_max);
  get_or("feasibility.joint_margin_warning_rad", params_.joint_margin_warning_rad,
         params_.joint_margin_warning_rad);
  get_or("feasibility.ik_timeout", params_.ik_timeout, params_.ik_timeout);
  get_or("feasibility.approach_angle_max_deg", params_.approach_angle_max_deg, params_.approach_angle_max_deg);
  get_or("feasibility.suggestion_perturb_xy", params_.suggestion_perturb_xy, params_.suggestion_perturb_xy);
  get_or("feasibility.suggestion_perturb_z", params_.suggestion_perturb_z, params_.suggestion_perturb_z);
  get_or("feasibility.suggestion_perturb_yaw_rad", params_.suggestion_perturb_yaw_rad,
         params_.suggestion_perturb_yaw_rad);
}

void FeasibilityChecker::addItem(std::vector<orion_mtc_msgs::msg::DiagnosticItem>& items,
                                 const std::string& code, int32_t level, const std::string& message,
                                 const std::string& field, double value, double threshold,
                                 const std::string& suggestion)
{
  orion_mtc_msgs::msg::DiagnosticItem item;
  item.code = code;
  item.level = level;
  item.message = message;
  item.field = field;
  item.value = value;
  item.threshold = threshold;
  item.suggestion = suggestion;
  items.push_back(item);
}

bool FeasibilityChecker::runIkAndJointMargin(const std::string& group_name,
                                             const std::string& link_name,
                                             double px, double py, double pz,
                                             double qx, double qy, double qz, double qw,
                                             std::vector<orion_mtc_msgs::msg::DiagnosticItem>& items)
{
  if (!impl_->robot_model)
  {
    addItem(items, "ROBOT_MODEL_UNAVAILABLE", LEVEL_ERROR,
            "机械臂模型未加载，无法进行 IK 检查", "", 0.0, 0.0, "");
    return false;
  }
  const moveit::core::JointModelGroup* jmg = impl_->robot_model->getJointModelGroup(group_name);
  if (!jmg)
  {
    addItem(items, "GROUP_NOT_FOUND", LEVEL_ERROR,
            "规划组 '" + group_name + "' 不存在", "", 0.0, 0.0, "");
    return false;
  }
  moveit::core::RobotState state(impl_->robot_model);
  {
    std::lock_guard<std::mutex> lock(impl_->joint_state_mutex);
    if (impl_->last_joint_state && !impl_->last_joint_state->name.empty())
    {
      for (size_t i = 0; i < impl_->last_joint_state->name.size(); ++i)
      {
        const std::string& name = impl_->last_joint_state->name[i];
        if (state.getRobotModel()->hasJointModel(name) && i < impl_->last_joint_state->position.size())
        {
          state.setVariablePosition(name, impl_->last_joint_state->position[i]);
        }
      }
      state.update();
    }
    else
    {
      state.setToDefaultValues();
      state.update();
    }
  }
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(px, py, pz);
  Eigen::Quaterniond q(static_cast<double>(qw), static_cast<double>(qx),
                       static_cast<double>(qy), static_cast<double>(qz));
  target.linear() = q.toRotationMatrix();

  bool ik_ok = state.setFromIK(jmg, target, link_name, params_.ik_timeout);
  if (!ik_ok)
  {
    addItem(items, "IK_NO_SOLUTION", LEVEL_ERROR,
            "给定末端姿态下未找到可行 IK 解", "target_pose.orientation", 0.0, 0.0,
            "建议放宽末端姿态约束或调整目标位置");
    return false;
  }
  state.update();
  /* 关节余量检查 */
  const std::vector<std::string>& names = jmg->getActiveJointModelNames();
  double margin_rad = params_.joint_margin_warning_rad;
  for (const std::string& jname : names)
  {
    const moveit::core::JointModel* jm = impl_->robot_model->getJointModel(jname);
    if (!jm || jm->getVariableBounds().empty())
      continue;
    const auto& vb = jm->getVariableBounds()[0];
    if (!vb.position_bounded_)
      continue;
    const std::vector<std::string>& vnames = jm->getVariableNames();
    if (vnames.empty())
      continue;
    double q = state.getVariablePosition(vnames[0]);
    double q_min = vb.min_position_;
    double q_max = vb.max_position_;
    double margin_low = q - q_min;
    double margin_high = q_max - q;
    double margin = std::min(margin_low, margin_high);
    if (margin < 0.0)
    {
      addItem(items, "IK_JOINT_LIMIT_EXCEEDED", LEVEL_ERROR,
              "关节 " + jname + " 超出限位", jname, q, q_max, "");
      return false;
    }
    if (margin < margin_rad)
    {
      addItem(items, "JOINT_NEAR_LIMIT", LEVEL_WARNING,
              "关节 " + jname + " 接近限位，余量 " + std::to_string(static_cast<int>(margin * 57.3)) + "°",
              jname, margin, margin_rad, "建议避免进一步向该方向运动");
    }
  }
  return true;
}

bool FeasibilityChecker::runIkOnly(double px, double py, double pz,
                                  double qx, double qy, double qz, double qw)
{
  if (!impl_->robot_model)
    return false;
  const moveit::core::JointModelGroup* jmg = impl_->robot_model->getJointModelGroup(impl_->arm_group_name);
  if (!jmg)
    return false;
  moveit::core::RobotState state(impl_->robot_model);
  {
    std::lock_guard<std::mutex> lock(impl_->joint_state_mutex);
    if (impl_->last_joint_state && !impl_->last_joint_state->name.empty())
    {
      for (size_t i = 0; i < impl_->last_joint_state->name.size(); ++i)
      {
        const std::string& name = impl_->last_joint_state->name[i];
        if (state.getRobotModel()->hasJointModel(name) && i < impl_->last_joint_state->position.size())
          state.setVariablePosition(name, impl_->last_joint_state->position[i]);
      }
      state.update();
    }
    else
    {
      state.setToDefaultValues();
      state.update();
    }
  }
  Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
  target.translation() = Eigen::Vector3d(px, py, pz);
  Eigen::Quaterniond q(static_cast<double>(qw), static_cast<double>(qx),
                      static_cast<double>(qy), static_cast<double>(qz));
  target.linear() = q.toRotationMatrix();
  if (!state.setFromIK(jmg, target, impl_->hand_frame, params_.ik_timeout))
    return false;
  state.update();
  for (const std::string& jname : jmg->getActiveJointModelNames())
  {
    const moveit::core::JointModel* jm = impl_->robot_model->getJointModel(jname);
    if (!jm || jm->getVariableBounds().empty())
      continue;
    const auto& vb = jm->getVariableBounds()[0];
    if (!vb.position_bounded_)
      continue;
    const std::vector<std::string>& vnames = jm->getVariableNames();
    if (vnames.empty())
      continue;
    double qval = state.getVariablePosition(vnames[0]);
    if (qval < vb.min_position_ || qval > vb.max_position_)
      return false;
  }
  return true;
}

void FeasibilityChecker::trySuggestCorrectionPick(
    const orion_mtc_msgs::srv::CheckPick::Request::SharedPtr req,
    orion_mtc_msgs::srv::CheckPick::Response::SharedPtr res,
    double obj_z, double grasp_z)
{
  double px = req->object_pose.pose.position.x;
  double py = req->object_pose.pose.position.y;
  double pz = obj_z;
  double gz = grasp_z;
  double qx = req->object_pose.pose.orientation.x;
  double qy = req->object_pose.pose.orientation.y;
  double qz = req->object_pose.pose.orientation.z;
  double qw = req->object_pose.pose.orientation.w;
  const double dx = params_.suggestion_perturb_xy;
  const double dz = params_.suggestion_perturb_z;
  const double dyaw = params_.suggestion_perturb_yaw_rad;

  auto inRange = [this](double x, double y, double z) {
    double r = std::sqrt(x * x + y * y + z * z);
    return r >= params_.min_reach_safe && r <= params_.max_reach_hard &&
           z >= params_.z_min && z <= params_.z_max;
  };

  Eigen::Quaterniond q_orig(static_cast<double>(qw), static_cast<double>(qx),
                            static_cast<double>(qy), static_cast<double>(qz));
  std::vector<double> yaw_offsets = { 0.0, dyaw, -dyaw };
  std::vector<std::pair<double, double>> xy_offsets = {
      { 0, 0 }, { -dx, 0 }, { dx, 0 }, { 0, -dx }, { 0, dx },
      { -dx, -dx }, { -dx, dx }, { dx, -dx }, { dx, dx }
  };
  std::vector<double> z_offsets = { 0.0, dz, -dz };

  for (double z_off : z_offsets)
  {
    double gz_try = gz + z_off;
    for (const auto& xy : xy_offsets)
    {
      double px_try = px + xy.first;
      double py_try = py + xy.second;
      if (!inRange(px_try, py_try, gz_try))
        continue;
      for (double yaw_off : yaw_offsets)
      {
        Eigen::AngleAxisd aa(yaw_off, Eigen::Vector3d::UnitZ());
        Eigen::Quaterniond q_try = q_orig * Eigen::Quaterniond(aa);
        if (runIkOnly(px_try, py_try, gz_try, q_try.x(), q_try.y(), q_try.z(), q_try.w()))
        {
          res->best_candidate_pose = req->object_pose;
          res->best_candidate_pose.pose.position.x = px_try;
          res->best_candidate_pose.pose.position.y = py_try;
          res->best_candidate_pose.pose.position.z = pz + z_off;
          res->best_candidate_pose.pose.orientation.x = q_try.x();
          res->best_candidate_pose.pose.orientation.y = q_try.y();
          res->best_candidate_pose.pose.orientation.z = q_try.z();
          res->best_candidate_pose.pose.orientation.w = q_try.w();
          std::string sug = "建议将目标调整至 (x=" + std::to_string(static_cast<int>(px_try * 1000) / 1000.0) +
                           ", y=" + std::to_string(static_cast<int>(py_try * 1000) / 1000.0) +
                           ", z=" + std::to_string(static_cast<int>((pz + z_off) * 1000) / 1000.0) + ") m";
          addItem(res->items, "ALTERNATIVE_POSE_SUGGESTED", LEVEL_INFO, sug, "", 0.0, 0.0, sug);
          return;
        }
      }
    }
  }
}

void FeasibilityChecker::checkPick(const orion_mtc_msgs::srv::CheckPick::Request::SharedPtr req,
                                  orion_mtc_msgs::srv::CheckPick::Response::SharedPtr res)
{
  res->items.clear();
  res->approved = true;
  res->severity = SEV_PASS;
  res->summary = "可执行";

  double px = req->object_pose.pose.position.x;
  double py = req->object_pose.pose.position.y;
  double pz = req->object_pose.pose.position.z;
  double qx = req->object_pose.pose.orientation.x;
  double qy = req->object_pose.pose.orientation.y;
  double qz = req->object_pose.pose.orientation.z;
  double qw = req->object_pose.pose.orientation.w;

  /* Link6 目标：物体位置 + 夹爪偏移（与 pick_task_builder 一致） */
  double gripper_z = mtc_config_ ? mtc_config_->gripper_tip_offset_from_link6_z : 0.028;
  double grasp_z = pz + gripper_z;

  double r = std::sqrt(px * px + py * py + grasp_z * grasp_z);
  if (r > params_.max_reach_hard)
  {
    addItem(res->items, "TARGET_OUT_OF_SAFE_RANGE", LEVEL_ERROR,
            "目标点距离机械臂基座 " + std::to_string(static_cast<int>(r * 100) / 100.0) + " m，超过硬上限 " +
                std::to_string(params_.max_reach_hard) + " m",
            "target_pose.position", r, params_.max_reach_hard, "请将目标移近基座");
    res->approved = false;
    res->severity = SEV_REJECT;
  }
  else if (r > params_.max_reach_soft)
  {
    addItem(res->items, "TARGET_NEAR_REACH_LIMIT", LEVEL_WARNING,
            "目标点距离 " + std::to_string(static_cast<int>(r * 100) / 100.0) + " m，超过推荐范围 " +
                std::to_string(params_.max_reach_soft) + " m",
            "target_pose.position", r, params_.max_reach_soft, "可执行但接近工作边界");
    if (res->severity < SEV_WARNING)
      res->severity = SEV_WARNING;
  }
  if (r < params_.min_reach_safe)
  {
    addItem(res->items, "TARGET_TOO_CLOSE", LEVEL_ERROR,
            "目标点过近基座 " + std::to_string(r) + " m，低于 " + std::to_string(params_.min_reach_safe) + " m",
            "target_pose.position", r, params_.min_reach_safe, "请将目标外移");
    res->approved = false;
    res->severity = SEV_REJECT;
  }
  if (grasp_z < params_.z_min)
  {
    addItem(res->items, "Z_TOO_LOW", LEVEL_ERROR,
            "抓取高度 " + std::to_string(grasp_z) + " m 低于下限 " + std::to_string(params_.z_min) + " m",
            "target_pose.position.z", grasp_z, params_.z_min, "请抬高目标");
    res->approved = false;
    res->severity = SEV_REJECT;
  }
  if (grasp_z > params_.z_max)
  {
    addItem(res->items, "Z_TOO_HIGH", LEVEL_ERROR,
            "抓取高度 " + std::to_string(grasp_z) + " m 超过上限 " + std::to_string(params_.z_max) + " m",
            "target_pose.position.z", grasp_z, params_.z_max, "请降低目标");
    res->approved = false;
    res->severity = SEV_REJECT;
  }

  /* 接近方向：物体姿态 z 轴与竖直向下 (0,0,-1) 夹角 */
  Eigen::Quaterniond q_obj(static_cast<double>(qw), static_cast<double>(qx),
                          static_cast<double>(qy), static_cast<double>(qz));
  Eigen::Vector3d approach_axis = q_obj * Eigen::Vector3d::UnitZ();
  double dot = approach_axis.dot(Eigen::Vector3d(0.0, 0.0, -1.0));
  dot = std::max(-1.0, std::min(1.0, dot));
  double approach_angle_deg = std::acos(dot) * 57.2958;
  if (approach_angle_deg > params_.approach_angle_max_deg)
  {
    addItem(res->items, "BAD_APPROACH_ANGLE", LEVEL_WARNING,
            "接近方向与竖直向下夹角 " + std::to_string(static_cast<int>(approach_angle_deg)) + "°，超过推荐 " +
                std::to_string(static_cast<int>(params_.approach_angle_max_deg)) + "°",
            "target_pose.orientation", approach_angle_deg, params_.approach_angle_max_deg,
            "建议调整物体姿态使夹爪可竖直接近");
    if (res->severity < SEV_WARNING)
      res->severity = SEV_WARNING;
  }

  bool ik_ok = runIkAndJointMargin(impl_->arm_group_name, impl_->hand_frame,
                                  px, py, grasp_z, qx, qy, qz, qw, res->items);
  if (!ik_ok)
  {
    res->approved = false;
    res->severity = SEV_REJECT;
  }
  else if (checkTargetCollision(px, py, grasp_z, qx, qy, qz, qw, res->items))
  {
    res->approved = false;
    res->severity = SEV_REJECT;
  }

  if (res->severity == SEV_REJECT)
    res->summary = "禁止执行：存在硬拒绝项";
  else if (res->severity == SEV_WARNING)
    res->summary = "可执行，但存在风险提示";

  if (res->severity == SEV_REJECT)
    trySuggestCorrectionPick(req, res, pz, grasp_z);
  if (res->best_candidate_pose.header.frame_id.empty())
    res->best_candidate_pose = req->object_pose;
}

void FeasibilityChecker::checkPlace(const orion_mtc_msgs::srv::CheckPlace::Request::SharedPtr req,
                                   orion_mtc_msgs::srv::CheckPlace::Response::SharedPtr res)
{
  res->items.clear();
  res->approved = true;
  res->severity = SEV_PASS;
  res->summary = "可执行";

  if (!req->has_held_object)
  {
    addItem(res->items, "NO_HELD_OBJECT", LEVEL_ERROR,
            "当前未持物，不允许执行放置", "", 0.0, 0.0, "请先执行抓取");
    res->approved = false;
    res->severity = SEV_REJECT;
    res->summary = "禁止执行：未持物";
    return;
  }

  double px = req->place_pose.pose.position.x;
  double py = req->place_pose.pose.position.y;
  double pz = req->place_pose.pose.position.z;
  double qx = req->place_pose.pose.orientation.x;
  double qy = req->place_pose.pose.orientation.y;
  double qz = req->place_pose.pose.orientation.z;
  double qw = req->place_pose.pose.orientation.w;

  /* 放置时 Link6 目标 z 抬高（与 place 任务一致） */
  double gripper_z = mtc_config_ ? mtc_config_->gripper_tip_offset_from_link6_z : 0.028;
  double place_link6_z = pz + gripper_z;

  double r = std::sqrt(px * px + py * py + place_link6_z * place_link6_z);
  if (r > params_.max_reach_hard)
  {
    addItem(res->items, "TARGET_OUT_OF_SAFE_RANGE", LEVEL_ERROR,
            "放置点距离基座 " + std::to_string(r) + " m，超过硬上限 " + std::to_string(params_.max_reach_hard) + " m",
            "place_pose.position", r, params_.max_reach_hard, "请将放置点移近");
    res->approved = false;
    res->severity = SEV_REJECT;
  }
  else if (r > params_.max_reach_soft)
  {
    addItem(res->items, "TARGET_NEAR_REACH_LIMIT", LEVEL_WARNING,
            "放置点距离 " + std::to_string(r) + " m，超过推荐范围", "place_pose.position", r,
            params_.max_reach_soft, "");
    if (res->severity < SEV_WARNING)
      res->severity = SEV_WARNING;
  }
  if (place_link6_z < params_.z_min || place_link6_z > params_.z_max)
  {
    addItem(res->items, "Z_OUT_OF_RANGE", LEVEL_ERROR,
            "放置高度 " + std::to_string(place_link6_z) + " 超出范围 [" +
                std::to_string(params_.z_min) + ", " + std::to_string(params_.z_max) + "]",
            "place_pose.position.z", place_link6_z, params_.z_max, "");
    res->approved = false;
    res->severity = SEV_REJECT;
  }

  bool ik_ok = runIkAndJointMargin(impl_->arm_group_name, impl_->hand_frame,
                                  px, py, place_link6_z, qx, qy, qz, qw, res->items);
  if (!ik_ok)
  {
    res->approved = false;
    res->severity = SEV_REJECT;
  }

  if (res->severity == SEV_REJECT)
    res->summary = "禁止执行：存在硬拒绝项";
  else if (res->severity == SEV_WARNING)
    res->summary = "可执行，但存在风险提示";

  res->adjusted_place_pose = req->place_pose;
}

bool FeasibilityChecker::checkTargetCollision(double px, double py, double pz,
                                              double qx, double qy, double qz, double qw,
                                              std::vector<orion_mtc_msgs::msg::DiagnosticItem>& items)
{
  std::string svc;
  if (!node_->has_parameter("feasibility.get_planning_scene_service"))
    node_->declare_parameter<std::string>("feasibility.get_planning_scene_service", "");
  node_->get_parameter_or("feasibility.get_planning_scene_service", svc, std::string(""));
  if (svc.empty())
    return false;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr client =
      node_->create_client<moveit_msgs::srv::GetPlanningScene>(svc);
  if (!client->wait_for_service(std::chrono::milliseconds(500)))
    return false;
  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
                                   moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
                                   moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
  auto result_future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::milliseconds(800)) !=
      rclcpp::FutureReturnCode::SUCCESS)
    return false;
  auto result = result_future.get();
  if (!result || !impl_->robot_model)
    return false;
  try
  {
    planning_scene::PlanningScenePtr scene = std::make_shared<planning_scene::PlanningScene>(impl_->robot_model);
    scene->setPlanningSceneMsg(result->scene);
    moveit::core::RobotState state(impl_->robot_model);
    {
      std::lock_guard<std::mutex> lock(impl_->joint_state_mutex);
      if (impl_->last_joint_state && !impl_->last_joint_state->name.empty())
      {
        for (size_t i = 0; i < impl_->last_joint_state->name.size(); ++i)
        {
          const std::string& name = impl_->last_joint_state->name[i];
          if (state.getRobotModel()->hasJointModel(name) && i < impl_->last_joint_state->position.size())
            state.setVariablePosition(name, impl_->last_joint_state->position[i]);
        }
        state.update();
      }
      else
      {
        state.setToDefaultValues();
        state.update();
      }
    }
    const moveit::core::JointModelGroup* jmg = impl_->robot_model->getJointModelGroup(impl_->arm_group_name);
    if (!jmg)
      return false;
    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = Eigen::Vector3d(px, py, pz);
    Eigen::Quaterniond q(static_cast<double>(qw), static_cast<double>(qx),
                         static_cast<double>(qy), static_cast<double>(qz));
    target.linear() = q.toRotationMatrix();
    if (!state.setFromIK(jmg, target, impl_->hand_frame, params_.ik_timeout))
      return false;
    state.update();
    scene->setCurrentState(state);
    collision_detection::CollisionResult cresult;
    scene->checkSelfCollision(collision_detection::CollisionRequest(), cresult);
    if (cresult.collision)
    {
      addItem(items, "TARGET_IN_COLLISION", LEVEL_ERROR,
              "目标位姿下机械臂与场景自碰撞或与环境碰撞", "target_pose", 0.0, 0.0, "请调整目标位置或姿态");
      return true;
    }
    collision_detection::CollisionResult cresult_env;
    scene->getCollisionEnv()->checkRobotCollision(collision_detection::CollisionRequest(), cresult_env, state);
    if (cresult_env.collision)
    {
      addItem(items, "TARGET_IN_COLLISION", LEVEL_ERROR,
              "目标位姿下机械臂与环境碰撞", "target_pose", 0.0, 0.0, "请调整目标位置或姿态");
      return true;
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(LOGGER, "checkTargetCollision: %s", e.what());
  }
  return false;
}

void FeasibilityChecker::onJointState(const void*)
{
  /* 已在订阅回调中更新 impl_->last_joint_state */
}

}  // namespace orion_mtc
