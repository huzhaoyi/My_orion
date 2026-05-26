/* PickTaskBuilder：缆绳圆柱分段加入 scene、侧向抓取 MoveRelative / 夹爪阶段构图 */

#include "orion_mtc/planning/pick_task_builder.hpp"
#include "orion_mtc/planning/collision_object_utils.hpp"
#include "orion_mtc/planning/cable_side_grasp.hpp"
#include "orion_mtc/core/constants.hpp"
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Geometry>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

namespace
{
/*
 * TargetSensor 顶抓构姿：gripper_tcp 的 z 为接近方向（沿 peg 轴），
 * y 由物体局部 +Z 在 ⊥z 平面上的投影确定，保证姿态解连续且可复现。
 */
Eigen::Matrix3d targetSensorToolRotation(const Eigen::Quaterniond& q_object,
                                        const Eigen::Vector3d& approach_dir)
{
  const Eigen::Vector3d n = approach_dir.normalized();
  Eigen::Vector3d ref = (q_object * Eigen::Vector3d::UnitZ()).normalized();
  Eigen::Vector3d y_axis = ref - ref.dot(n) * n;
  if (y_axis.norm() < 1e-3)
  {
    ref = (q_object * Eigen::Vector3d::UnitY()).normalized();
    y_axis = ref - ref.dot(n) * n;
  }
  if (y_axis.norm() < 1e-3)
  {
    y_axis = n.unitOrthogonal();
  }
  y_axis.normalize();
  const Eigen::Vector3d z_axis = n;
  const Eigen::Vector3d x_axis = y_axis.cross(z_axis).normalized();
  Eigen::Matrix3d R;
  R.col(0) = x_axis;
  R.col(1) = y_axis;
  R.col(2) = z_axis;
  return R;
}
}  // namespace

/*
 * 保存 node 与 MTCConfig 引用；不拷贝重型资源，由调用方保证 node 存活期覆盖 Task 规划全程。
 */
PickTaskBuilder::PickTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config)
  : node_(node), config_(config)
{
}

/*
 * 从缆段列表与侧向抓取候选组装完整 MTC Task：CurrentState → ready → Add 分段圆柱碰撞体 →
 * 开爪 → ACM 放宽自碰 → 预抓/接近/闭爪 → 清 scene → 短退 → 回到预抓取位 → 再次闭爪。
 * plan_frame 与分段 object header 一致，供 MoveIt 在正确父系下解释 primitive 位姿。
 */
mtc::Task PickTaskBuilder::buildFromCableCandidate(
    const std::vector<CableSegment>& segments,
    const CableGraspCandidate& candidate,
    const std::string& plan_frame)
{
  mtc::Task task;
  task.stages()->setName("orion pick (cable side, segmented)");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "gripper_tcp";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  lin_planner->setPlannerId("LIN");
  lin_planner->setMaxVelocityScalingFactor(config_.cable_grasp.approach_lin_velocity_scaling);
  lin_planner->setMaxAccelerationScalingFactor(config_.cable_grasp.approach_lin_acceleration_scaling);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_ready->setGroup(arm_group_name);
  stage_ready->setGoal("ready");
  task.add(std::move(stage_ready));

  {
    auto stage_add = std::make_unique<mtc::stages::ModifyPlanningScene>("add_cable_segments");
    for (const auto& seg : segments)
    {
      moveit_msgs::msg::CollisionObject obj =
          makeSegmentCollisionObject(seg, plan_frame, moveit_msgs::msg::CollisionObject::ADD);
      obj.header.stamp = node_->now();
      stage_add->addObject(obj);
    }
    task.add(std::move(stage_add));
  }

  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  {
    auto stage_acm = std::make_unique<mtc::stages::ModifyPlanningScene>("allow self-collision (pregrasp)");
    stage_acm->allowCollisions("Link1", std::vector<std::string>{ "Link6", "Link7", "Link8" }, true);
    stage_acm->allowCollisions("Link2", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "arm_base_link" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "Link2" }, true);
    stage_acm->allowCollisions("Link8", std::vector<std::string>{ "arm_base_link" }, true);
    task.add(std::move(stage_acm));
  }

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  /* 仅对抓取局部段放宽末端碰撞（Link1/Link2 不放开） */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (cable local) for pregrasp");
    for (int idx : candidate.local_segment_indices)
    {
      if (idx >= 0 && idx < static_cast<int>(segments.size()))
      {
        stage->allowCollisions(segments[idx].id, CABLE_LOCAL_PREGRASP_ALLOWED_LINKS, true);
      }
    }
    grasp->insert(std::move(stage));
  }

  const rclcpp::Time now = node_->now();
  geometry_msgs::msg::PoseStamped pregrasp_ps = toPoseStamped(candidate.pregrasp_pose, plan_frame, now);
  auto stage_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", ptp_planner);
  stage_pregrasp->setGroup(arm_group_name);
  stage_pregrasp->setGoal(pregrasp_ps);
  stage_pregrasp->setIKFrame(hand_frame);
  grasp->insert(std::move(stage_pregrasp));

  /* approach 阶段继续使用局部段 ACM（同上） */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (cable local) for approach");
    for (int idx : candidate.local_segment_indices)
    {
      if (idx >= 0 && idx < static_cast<int>(segments.size()))
      {
        stage->allowCollisions(segments[idx].id, CABLE_LOCAL_APPROACH_ALLOWED_LINKS, true);
      }
    }
    grasp->insert(std::move(stage));
  }

  geometry_msgs::msg::Vector3Stamped approach_v = toVector3Stamped(candidate.approach_dir, plan_frame, now);
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach to grasp (LIN)", lin_planner);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_approach->setMinMaxDistance(static_cast<float>(candidate.approach_dist),
                                    static_cast<float>(candidate.approach_dist));
  stage_approach->setIKFrame(hand_frame);
  stage_approach->setDirection(approach_v);
  grasp->insert(std::move(stage_approach));

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
  }

  /* 简化版：删除全部缆绳段并标记抓取，不 attach 几何体；executor 根据本阶段设置 held 状态 */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("remove_cable_segments");
    for (const auto& seg : segments)
    {
      stage->removeObject(seg.id);
    }
    grasp->insert(std::move(stage));
  }

  geometry_msgs::msg::Vector3Stamped retreat_v = toVector3Stamped(candidate.retreat_dir, plan_frame, now);
  auto stage_retreat = std::make_unique<mtc::stages::MoveRelative>("retreat short", cartesian_planner);
  stage_retreat->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_retreat->setMinMaxDistance(static_cast<float>(candidate.retreat_dist),
                                   static_cast<float>(candidate.retreat_dist));
  stage_retreat->setIKFrame(hand_frame);
  stage_retreat->setDirection(retreat_v);
  grasp->insert(std::move(stage_retreat));

  pregrasp_ps.header.stamp = node_->now();
  auto stage_pregrasp_holding = std::make_unique<mtc::stages::MoveTo>("move to pregrasp (holding)", ptp_planner);
  stage_pregrasp_holding->setGroup(arm_group_name);
  stage_pregrasp_holding->setGoal(pregrasp_ps);
  stage_pregrasp_holding->setIKFrame(hand_frame);
  grasp->insert(std::move(stage_pregrasp_holding));

  {
    auto stage_close_final = std::make_unique<mtc::stages::MoveTo>("close hand (at pregrasp)", interpolation_planner);
    stage_close_final->setGroup(hand_group_name);
    stage_close_final->setGoal("close");
    grasp->insert(std::move(stage_close_final));
  }

  task.add(std::move(grasp));
  return task;
}

/*
 * TargetSensor 抓取：阶段顺序与缆绳 buildFromCableCandidate 同构（不加缆段）；规划全程 Pilz PTP/LIN。
 * 顶抓定义：接近方向沿物体局部 +X（乘 approach_normal_sign），再沿 -z 退预抓距。
 */
mtc::Task PickTaskBuilder::buildFromTargetSensorPose(
    const geometry_msgs::msg::PoseStamped& object_pose,
    const std::string& plan_frame,
    double pregrasp_distance_m,
    double approach_sign,
    int approach_axis_local,
    double tool_roll_about_approach_deg)
{
  mtc::Task task;
  task.stages()->setName("orion pick (targetsensor)");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "gripper_tcp";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  auto lin_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  lin_planner->setPlannerId("LIN");
  lin_planner->setMaxVelocityScalingFactor(config_.cable_grasp.approach_lin_velocity_scaling);
  lin_planner->setMaxAccelerationScalingFactor(config_.cable_grasp.approach_lin_acceleration_scaling);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_ready->setGroup(arm_group_name);
  stage_ready->setGoal("ready");
  task.add(std::move(stage_ready));

  {
    auto stage_add_peg = std::make_unique<mtc::stages::ModifyPlanningScene>("add targetsensor peg mesh");
    moveit_msgs::msg::CollisionObject peg = makeTargetSensorPegCollisionObject(
        TARGET_SENSOR_PEG_COLLISION_ID, plan_frame, object_pose.pose, moveit_msgs::msg::CollisionObject::ADD);
    peg.header.stamp = node_->now();
    stage_add_peg->addObject(peg);
    task.add(std::move(stage_add_peg));
  }

  auto stage_open = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  stage_open->setGroup(hand_group_name);
  stage_open->setGoal("open");
  task.add(std::move(stage_open));

  {
    auto stage_acm = std::make_unique<mtc::stages::ModifyPlanningScene>("allow self-collision (pregrasp)");
    stage_acm->allowCollisions("Link1", std::vector<std::string>{ "Link6", "Link7", "Link8" }, true);
    stage_acm->allowCollisions("Link2", std::vector<std::string>{ "Link8" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "arm_base_link" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "Link2" }, true);
    stage_acm->allowCollisions("Link8", std::vector<std::string>{ "arm_base_link" }, true);
    task.add(std::move(stage_acm));
  }

  Eigen::Quaterniond q_object(
      object_pose.pose.orientation.w,
      object_pose.pose.orientation.x,
      object_pose.pose.orientation.y,
      object_pose.pose.orientation.z);
  q_object.normalize();
  Eigen::Vector3d local_axis = Eigen::Vector3d::UnitZ();
  if (approach_axis_local == 0)
  {
    local_axis = Eigen::Vector3d::UnitX();
  }
  else if (approach_axis_local == 1)
  {
    local_axis = Eigen::Vector3d::UnitY();
  }
  const Eigen::Vector3d n_geom = (q_object * local_axis).normalized();
  const double sign = (approach_sign < 0.0) ? -1.0 : 1.0;
  const Eigen::Vector3d approach_dir = n_geom * sign;
  Eigen::Matrix3d R_tool = targetSensorToolRotation(q_object, approach_dir);
  if (std::abs(tool_roll_about_approach_deg) > 1e-6)
  {
    constexpr double PI_D = 3.14159265358979323846;
    const double roll_rad = tool_roll_about_approach_deg * PI_D / 180.0;
    const Eigen::Matrix3d R_roll = Eigen::AngleAxisd(roll_rad, approach_dir.normalized()).toRotationMatrix();
    R_tool = R_roll * R_tool;
  }

  const double grasp_depth = std::max(0.0, config_.target_sensor_pick.grasp_depth_m);
  const double surface_backoff = std::max(0.0, config_.target_sensor_pick.surface_backoff_m);
  const Eigen::Vector3d p_object(object_pose.pose.position.x, object_pose.pose.position.y,
                                 object_pose.pose.position.z);
  const Eigen::Vector3d p_grasp = p_object - approach_dir * surface_backoff + approach_dir * grasp_depth;

  Eigen::Isometry3d pregrasp_iso = Eigen::Isometry3d::Identity();
  pregrasp_iso.linear() = R_tool;
  pregrasp_iso.translation() = p_grasp - approach_dir * pregrasp_distance_m;

  const rclcpp::Time now = node_->now();
  geometry_msgs::msg::PoseStamped pregrasp_pose = toPoseStamped(pregrasp_iso, plan_frame, now);

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  {
    auto stage_allow_peg = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (targetsensor peg) for pregrasp");
    stage_allow_peg->allowCollisions(TARGET_SENSOR_PEG_COLLISION_ID, TARGET_SENSOR_PEG_ALLOWED_LINKS, true);
    grasp->insert(std::move(stage_allow_peg));
  }

  auto stage_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", ptp_planner);
  stage_pregrasp->setGroup(arm_group_name);
  stage_pregrasp->setGoal(pregrasp_pose);
  stage_pregrasp->setIKFrame(hand_frame);
  grasp->insert(std::move(stage_pregrasp));

  {
    auto stage_allow_approach = std::make_unique<mtc::stages::ModifyPlanningScene>(
        "allow collision (targetsensor peg) for approach");
    stage_allow_approach->allowCollisions(TARGET_SENSOR_PEG_COLLISION_ID, TARGET_SENSOR_PEG_ALLOWED_LINKS, true);
    grasp->insert(std::move(stage_allow_approach));
  }

  geometry_msgs::msg::Vector3Stamped approach_v;
  approach_v.header.stamp = now;
  approach_v.header.frame_id = plan_frame;
  approach_v.vector.x = approach_dir.x();
  approach_v.vector.y = approach_dir.y();
  approach_v.vector.z = approach_dir.z();
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach to grasp (LIN)", lin_planner);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_approach->setMinMaxDistance(static_cast<float>(pregrasp_distance_m),
                                    static_cast<float>(pregrasp_distance_m));
  stage_approach->setIKFrame(hand_frame);
  stage_approach->setDirection(approach_v);
  grasp->insert(std::move(stage_approach));

  {
    auto stage_close = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage_close->setGroup(hand_group_name);
    stage_close->setGoal("close");
    grasp->insert(std::move(stage_close));
  }

  {
    auto stage_rm_peg = std::make_unique<mtc::stages::ModifyPlanningScene>("remove targetsensor peg mesh");
    stage_rm_peg->removeObject(TARGET_SENSOR_PEG_COLLISION_ID);
    grasp->insert(std::move(stage_rm_peg));
  }

  task.add(std::move(grasp));
  return task;
}

}  // namespace orion_mtc
