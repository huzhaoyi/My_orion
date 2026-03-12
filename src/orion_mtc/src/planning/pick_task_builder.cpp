#include "orion_mtc/planning/pick_task_builder.hpp"
#include "orion_mtc/planning/collision_object_utils.hpp"
#include "orion_mtc/core/constants.hpp"
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mtc = moveit::task_constructor;

namespace orion_mtc
{

PickTaskBuilder::PickTaskBuilder(const rclcpp::Node::SharedPtr& node, const MTCConfig& config)
  : node_(node), config_(config)
{
}

mtc::Task PickTaskBuilder::build(double obj_x, double obj_y, double obj_z,
                                 const geometry_msgs::msg::Quaternion& object_orientation,
                                 const std::string& /*object_id*/)
{
  mtc::Task task;
  task.stages()->setName("orion pick");
  task.loadRobotModel(node_);
  const auto& arm_group_name = "arm";
  const auto& hand_group_name = "hand";
  const auto& hand_frame = "Link6";
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);
  task.stages()->properties().set("group", arm_group_name);
  task.stages()->properties().set("eef", hand_group_name);
  task.stages()->properties().set("ik_frame", hand_frame);

  task.add(std::make_unique<mtc::stages::CurrentState>("current"));

  auto ptp_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "pilz");
  ptp_planner->setPlannerId("PTP");
  /* move to pregrasp 使用 OMPL：可采样多组 IK，比 Pilz 单次 IK 更易在目标点找到解，缓解 NO_IK_SOLUTION */
  auto ompl_planner = std::make_shared<mtc::solvers::PipelinePlanner>(node_, "move_group");
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(1.0);
  cartesian_planner->setMaxAccelerationScalingFactor(1.0);
  cartesian_planner->setStepSize(0.01);

  /* 先 move to ready 再 add object：避免到 ready 的 PTP 路径与物体碰撞（object 尚未在 scene 中） */
  auto stage_ready = std::make_unique<mtc::stages::MoveTo>("move to ready", ptp_planner);
  stage_ready->setGroup(arm_group_name);
  stage_ready->setGoal("ready");
  task.add(std::move(stage_ready));

  {
    geometry_msgs::msg::Pose obj_pose;
    obj_pose.position.x = obj_x;
    obj_pose.position.y = obj_y;
    obj_pose.position.z = obj_z;
    obj_pose.orientation = object_orientation;
    moveit_msgs::msg::CollisionObject object =
        makeTargetCollisionObject("object", obj_pose, moveit_msgs::msg::CollisionObject::ADD);
    object.header.stamp = node_->now();
    auto stage_add = std::make_unique<mtc::stages::ModifyPlanningScene>("add object");
    stage_add->addObject(object);
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
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "base_link" }, true);
    stage_acm->allowCollisions("Link7", std::vector<std::string>{ "Link2" }, true);  // PTP to pregrasp 路径可能 Link7-Link2 贴近
    stage_acm->allowCollisions("Link8", std::vector<std::string>{ "base_link" }, true);
    task.add(std::move(stage_acm));
  }

  auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
  task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
  grasp->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group", "ik_frame" });

  /* 允许 arm 与 object 碰撞后再 move to pregrasp，否则 PTP 路径到物体上方时易报 Link4~7 与 object 碰撞 */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (arm,object) for pregrasp");
    stage->allowCollisions("object", PREGRASP_OBJECT_ALLOWED_LINKS, true);
    grasp->insert(std::move(stage));
  }

  const double approach_max = config_.approach_object_max_dist;
  geometry_msgs::msg::PoseStamped pregrasp;
  pregrasp.header.frame_id = "base_link";
  pregrasp.pose.position.x = obj_x;
  pregrasp.pose.position.y = obj_y;
  pregrasp.pose.position.z = obj_z + approach_max;
  /* 预抓取使用固定“竖直向下”（Link6 的 z 沿 base_link -Z），approach 段沿 hand_frame +Z 下压，提高 IK 成功率；
   * 若用 object_orientation 易在部分目标位姿下报 NO_IK_SOLUTION（Link6 限位/奇异）。 */
  pregrasp.pose.orientation.x = 1.0;
  pregrasp.pose.orientation.y = 0.0;
  pregrasp.pose.orientation.z = 0.0;
  pregrasp.pose.orientation.w = 0.0;
  auto stage_pregrasp = std::make_unique<mtc::stages::MoveTo>("move to pregrasp", ompl_planner);
  stage_pregrasp->setGroup(arm_group_name);
  stage_pregrasp->setGoal(pregrasp);
  stage_pregrasp->setIKFrame(hand_frame);
  grasp->insert(std::move(stage_pregrasp));

  /* 接近/闭合/attach 阶段保持 hand-object 允许（与上阶段重叠，语义明确） */
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
    stage->allowCollisions("object", OBJECT_GRASP_ALLOWED_LINKS, true);
    grasp->insert(std::move(stage));
  }

  /* 接近段：沿 base_link -Z 竖直下压，避免用手系方向导致 IK 多解/步进时姿态漂移产生“旋转卡上去” */
  auto stage_approach = std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
  stage_approach->properties().set("marker_ns", "approach_object");
  stage_approach->properties().set("link", hand_frame);
  stage_approach->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_approach->properties().set("min_fraction", config_.approach_min_fraction);
  stage_approach->setMinMaxDistance(static_cast<float>(config_.approach_object_min_dist),
                                   static_cast<float>(config_.approach_object_max_dist));
  stage_approach->setIKFrame(hand_frame);
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "base_link";
  vec.vector.x = 0.0;
  vec.vector.y = 0.0;
  vec.vector.z = -1.0;  // 竖直向下，纯平动无旋转
  stage_approach->setDirection(vec);
  grasp->insert(std::move(stage_approach));

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision before close");
    stage->allowCollisions("object", OBJECT_GRASP_ALLOWED_LINKS, true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
    stage->setGroup(hand_group_name);
    stage->setGoal("close");
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject("object", hand_frame);
    grasp->insert(std::move(stage));
  }

  if (!config_.support_surface_link.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (object,support)");
    stage->allowCollisions("object", std::vector<std::string>{ config_.support_surface_link }, true);
    grasp->insert(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setMinMaxDistance(static_cast<float>(config_.lift_object_min_dist),
                             static_cast<float>(config_.lift_object_max_dist));
    stage->setIKFrame(hand_frame);
    stage->properties().set("marker_ns", "lift_object");
    geometry_msgs::msg::Vector3Stamped v;
    v.header.frame_id = "base_link";
    v.vector.x = 0.0;
    v.vector.y = 0.0;
    v.vector.z = 1.0;
    stage->setDirection(v);
    grasp->insert(std::move(stage));
  }

  if (!config_.support_surface_link.empty())
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (object,surface)");
    stage->allowCollisions("object", std::vector<std::string>{ config_.support_surface_link }, false);
    grasp->insert(std::move(stage));
  }

  task.add(std::move(grasp));
  return task;
}

}  // namespace orion_mtc
